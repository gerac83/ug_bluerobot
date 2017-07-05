/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 08/12/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Clopema Joint Trajectory Streamer implementation
 */

#include <clopema_controller/ClopemaJointTrajectoryStreamer.h>
#include <industrial_robot_client/utils.h>
#include <std_msgs/Int32.h>

#define ROS_ERROR_RETURN(rtn,...) do {ROS_ERROR(__VA_ARGS__); return(rtn);} while(0)

using industrial::simple_message::SimpleMessage;
using industrial::joint_data::JointData;
using industrial::joint_traj_pt_full::JointTrajPtFull;
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;

ClopemaJointTrajectoryStreamer::ClopemaJointTrajectoryStreamer() :
    synchronize(true),
    as_(node_, "follow_joint_trajectory", false) {
    mutex_connection.reset(new boost::mutex);
    motion_ctrl.set_mutex(mutex_connection);
    streaming_first_received = false;
    streaming_last_time = 0.0;
    streaming_offset = 0;
    if(!industrial_robot_client::joint_trajectory_interface::JointTrajectoryInterface::init("192.168.0.101", 50240)) {
        ROS_ERROR_STREAM("Cannot initialize Joint Trajectory Streamer");
        return;
    }
    srv_power_off = node_.advertiseService("set_drive_power", &ClopemaJointTrajectoryStreamer::cb_set_drive_power, this);
    srv_set_synchro = node_.advertiseService("set_controller_synchronization", &ClopemaJointTrajectoryStreamer::cb_set_synchronization, this);
    pub_queue_cnt = node_.advertise<std_msgs::Int32>("controller_queue_cnt", 1);
    sub_joint_stream = node_.subscribe("joint_stream_command", 1, &ClopemaJointTrajectoryStreamer::cb_joint_stream, this);
    sub_robot_status = node_.subscribe("robot_status", 1, &ClopemaJointTrajectoryStreamer::cb_robot_status, this);
    timer_pub_queue_cnt = node_.createTimer(ros::Duration(0.05), &ClopemaJointTrajectoryStreamer::cb_pub_q_cnt, this);
    this->streaming_thread_ = boost::thread(boost::bind(&ClopemaJointTrajectoryStreamer::streamingThread, this));
    as_.registerPreemptCallback(boost::bind(&ClopemaJointTrajectoryStreamer::cb_as_preempt, this));
    as_.registerGoalCallback(boost::bind(&ClopemaJointTrajectoryStreamer::cb_as_goal, this));
    as_.start();
}

ClopemaJointTrajectoryStreamer::~ClopemaJointTrajectoryStreamer() {
    messages.kill();
    streaming_thread_.join();
}

bool ClopemaJointTrajectoryStreamer::init(SmplMsgConnection* connection) {
    std::map<std::string, double> velocity_limits = get_velocity_limits();
    grouped_joint_names = get_grouped_joints_names();

    if(grouped_joint_names.empty())
        return false;

    bool rtn = true;
    rtn &= industrial_robot_client::joint_trajectory_streamer::JointTrajectoryInterface::init(connection, grouped_joint_names[0], velocity_limits);
    rtn &= motion_ctrl.init(connection, 0);
    return rtn;
}

std::vector<std::vector<std::string> > ClopemaJointTrajectoryStreamer::get_grouped_joints_names() {
    std::vector<std::vector< std::string> > rtn;
    std::vector<std::string> joint_names;
    joint_names.push_back("r1_joint_s");
    joint_names.push_back("r1_joint_l");
    joint_names.push_back("r1_joint_u");
    joint_names.push_back("r1_joint_r");
    joint_names.push_back("r1_joint_b");
    joint_names.push_back("r1_joint_t");
    rtn.push_back(joint_names);
    joint_names.clear();
    joint_names.push_back("r2_joint_s");
    joint_names.push_back("r2_joint_l");
    joint_names.push_back("r2_joint_u");
    joint_names.push_back("r2_joint_r");
    joint_names.push_back("r2_joint_b");
    joint_names.push_back("r2_joint_t");
    rtn.push_back(joint_names);
    joint_names.clear();
    joint_names.push_back("ext_axis");
    rtn.push_back(joint_names);
    return rtn;
}

std::map< std::string, double > ClopemaJointTrajectoryStreamer::get_velocity_limits() {
    std::map<std::string, double> rtn;
    rtn["r1_joint_s"] = 3.49;
    rtn["r1_joint_l"] = 3.49;
    rtn["r1_joint_u"] = 3.84;
    rtn["r1_joint_r"] = 7.16;
    rtn["r1_joint_b"] = 7.16;
    rtn["r1_joint_t"] = 10.65;
    rtn["r2_joint_s"] = 3.84;
    rtn["r2_joint_l"] = 3.49;
    rtn["r2_joint_u"] = 3.84;
    rtn["r2_joint_r"] = 7.16;
    rtn["r2_joint_b"] = 7.16;
    rtn["r2_joint_t"] = 10.65;
    rtn["ext_axis"] = 2.84;
    return rtn;
}

bool ClopemaJointTrajectoryStreamer::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector< SimpleMessage >* msgs) {
    msgs->clear();

    std::vector<std::vector<SimpleMessage> > vec_msgs;
    vec_msgs.resize(grouped_joint_names.size());
    if(vec_msgs.empty()) {
        ROS_WARN_STREAM("gropued joint names have not been filled, cannot convert trajectory to msgs");
        return false;
    }

    for(unsigned int i = 0; i < vec_msgs.size(); ++i) {
        this->all_joint_names_ = grouped_joint_names[i];
        robot_id_ = i; //This is required by the create_message function to assing values to the correct group
        if(!industrial_robot_client::joint_trajectory_streamer::JointTrajectoryInterface::trajectory_to_msgs(traj, &vec_msgs[i])) {
            ROS_WARN_STREAM("Cannot transform trajectory to msgs for group no.: " << i);
            return false;
        }
    }

    int npoints = vec_msgs[0].size();
    for(unsigned int i = 1; i < vec_msgs.size(); ++i) {
        if(npoints != vec_msgs[i].size()) {
            ROS_WARN_STREAM("Number of points is not same for each group cannot mix trajectories for several groups.");
            return false;
        }
    }

    //combine messages for each group to correct order
    for(unsigned int i = 0; i < vec_msgs[0].size(); ++i) {
        for(unsigned int gi = 0; gi < vec_msgs.size(); ++gi) {
            msgs->push_back(vec_msgs[gi].at(i));
        }
    }

    return true;
}

bool ClopemaJointTrajectoryStreamer::add_missed_joints(trajectory_msgs::JointTrajectoryPtr& traj) {
    sensor_msgs::JointState cp = cur_joint_pos_;
    for(unsigned int i = 0; i < grouped_joint_names.size(); ++i) {
        for(unsigned int j = 0; j < grouped_joint_names[i].size(); ++j) {
            std::string jn = grouped_joint_names[i][j];
            if(std::find(traj->joint_names.begin(), traj->joint_names.end(), jn) != traj->joint_names.end()) {
                continue; //is in trajectory
            }

            int n = std::distance(cp.name.begin(), std::find(cp.name.begin(), cp.name.end(), jn));
            if(n >= cp.position.size() || n < 0) {
                ROS_WARN_STREAM("Unknown joint name in trajectory, ignoring... JName:" << jn);
                continue;
            }

            traj->joint_names.push_back(jn);
            double val = cp.position[n];
            for(unsigned int k = 0; k < traj->points.size(); ++k) {
                traj->points[k].positions.push_back(val);
                if(!traj->points[k].velocities.empty())
                    traj->points[k].velocities.push_back(0.0);
                if(!traj->points[k].accelerations.empty())
                    traj->points[k].accelerations.push_back(0.0);
                if(!traj->points[k].effort.empty())
                    traj->points[k].effort.push_back(0.0);
            }
        }
    }
    return true;
}

bool ClopemaJointTrajectoryStreamer::setPowerOff(bool force)  {
    if(force) {
        trajectoryStop();
    }
    bool rtn = motion_ctrl.setTrajMode(false);
    return rtn ;
}

bool ClopemaJointTrajectoryStreamer::send_to_robot(const std::vector<SimpleMessage>& messages) {
    streaming_last_time = 0; //restart offset caused by streaming interface
    streaming_offset = 0;
    streaming_first_received = false;
    if(!motion_ctrl.controllerReady() && !motion_ctrl.setTrajMode(true)) {
        ROS_ERROR_STREAM("Failed to initialize MotoRos motion.  Trajectory ABORTED.  Correct issue and re-send trajectory.");
        return false;
    }
    this->messages.push_vector(messages); //Push vector into the queue - streaming will start automatically
    return true;
}

bool ClopemaJointTrajectoryStreamer::VectorToJointData(const std::vector<double>& vec, JointData& joints) {
    if(vec.size() > joints.getMaxNumJoints())
        ROS_ERROR_RETURN(false, "Failed to copy to JointData.  Len (%d) out of range (0 to %d)", (int)vec.size(), (int)joints.getMaxNumJoints());

    joints.init();
    for(int i = 0; i < vec.size(); ++i)
        joints.setJoint(i, vec[i]);

    return true;
}

bool ClopemaJointTrajectoryStreamer::reconnect(unsigned int n) {
    int connectRetryCount = n;
    if(connectRetryCount-- > 0) { // automatically re-establish connection, if required

        bool connected;
        {
            boost::mutex::scoped_lock sl(*mutex_connection);
            connected = this->connection_->isConnected();
        }
        if(!connected) {
            ROS_INFO("Connecting to robot motion server");
            {
                boost::mutex::scoped_lock sl(*mutex_connection);
                this->connection_->makeConnect();
            }
            ros::Duration(0.250).sleep();  // wait for connection
        }
        {
            boost::mutex::scoped_lock sl(*mutex_connection);
            connected = this->connection_->isConnected();
        }
        if(connected) {
            return true;
        } else if(connectRetryCount <= 0) {
            ROS_ERROR("Timeout connecting to robot controller.  Send new motion command to retry.");
            return false;
        }
    }
    return false;
}


bool ClopemaJointTrajectoryStreamer::send_queue_to_robot(ConcurentQueue<SimpleMessage>& q_send) {
    int count_err = 0;
    while(!q_send.empty()) {
        using namespace industrial::simple_message;
        using motoman::simple_message::motion_reply_message::MotionReplyMessage;
        SimpleMessage tmpMsg, msg, reply;
        if(!q_send.try_pop(tmpMsg)) {
            break; //there is no data in queue, nothing to send
        }

        msg.init(tmpMsg.getMessageType(), CommTypes::SERVICE_REQUEST, ReplyTypes::INVALID, tmpMsg.getData());  // set commType=REQUEST

        bool suc;
        {
            boost::mutex::scoped_lock sl(*mutex_connection);
            suc = this->connection_->sendAndReceiveMsg(msg, reply, false);
        }

        if(!suc) {
            ROS_WARN("Failed sent joint point, will try again");
            q_send.push(tmpMsg);
        } else {
            MotionReplyMessage reply_status;
            if(!reply_status.init(reply)) {
                ROS_ERROR("Aborting trajectory: Unable to parse JointTrajectoryPoint reply");
                count_err++;
                if(count_err > 5) {
                    return false;
                }
                q_send.push(tmpMsg); //send again because is bussy
                ros::Duration(0.005).sleep();
                continue;
            }

            if(reply_status.reply_.getResult() == motoman::simple_message::motion_reply::MotionReplyResults::SUCCESS) {
                count_err = 0;
            } else if(reply_status.reply_.getResult() == motoman::simple_message::motion_reply::MotionReplyResults::BUSY) {
                q_send.push(tmpMsg); //send again because is bussy
            } else {
                ROS_ERROR_STREAM("Aborting Trajectory.  Failed to send point. try: " << count_err << "/" << 5);
                ROS_ERROR_STREAM(reply_status.reply_.getResultString());
                count_err++;
                if(count_err > 5) {
                    return false;
                }
                q_send.push(tmpMsg); //send again because is bussy
                ros::Duration(0.2).sleep();
            }
        }
        ros::Duration(0.005).sleep();
    }
    return true;
}


void ClopemaJointTrajectoryStreamer::streamingThread() {
    while(ros::ok()) {
        q_send.clear(); //will contains N messages where N is number of group
        for(unsigned int i = 0; i < grouped_joint_names.size(); ++i) {
            SimpleMessage tmpMsg;
            if(!messages.wait_and_pop(tmpMsg))
                return;
            q_send.push(tmpMsg);
        }

        if(get_synchronization()) {
            while(ros::ok()) {
                int n;
                bool rdy = motion_ctrl.getSynchro(n);
                if(rdy && n == 0)
                    break;
                ros::Duration(0.005).sleep();
            }
        }

        if(!reconnect(5)) {
            ROS_ERROR_STREAM("Connection to the robot is not possible, stoping streaming, removing all points from queue");
            messages.clear();
            continue;
        }

        ROS_DEBUG_STREAM("Sending queue to the robot");
        if(!send_queue_to_robot(q_send)) {
            messages.clear();
            boost::mutex::scoped_lock l(mutex_as_);
            if(as_.isActive()) {
                as_.setAborted();
            }
            continue;
        }
        ROS_DEBUG_STREAM("Queue sent");
    }
}

void ClopemaJointTrajectoryStreamer::cb_pub_q_cnt(const ros::TimerEvent& event) {
    {
        boost::mutex::scoped_lock sl(*mutex_connection);
        if(!this->connection_->isConnected())
            return;
    }
    int n;
    bool rdy = motion_ctrl.getSynchro(n);
    std_msgs::Int32 qmsg;
    qmsg.data = n;
    pub_queue_cnt.publish(qmsg);
}


void ClopemaJointTrajectoryStreamer::trajectoryStop() {
    messages.clear();
    q_send.clear();
    motion_ctrl.stopTrajectory();
}

bool ClopemaJointTrajectoryStreamer::check_start_state(const trajectory_msgs::JointTrajectory& traj) {
    //Extract current positions of joints in the trajectory
    sensor_msgs::JointState cp_ = cur_joint_pos_;
    std::vector<std::string> cur_name;
    std::vector<double> cur_pos;
    std::vector<std::string> tjn = traj.joint_names;
    for(unsigned int i = 0; i < cp_.name.size(); ++i) {
        std::string cn = cp_.name[i];
        bool traj_contains = std::find(tjn.begin(), tjn.end(), cn) != tjn.end();
        if(traj_contains) {
            cur_name.push_back(cn);
            cur_pos.push_back(cp_.position[i]);
        } else {
            ROS_WARN_STREAM("Joint not in trajectory: " << cn);
        }
    }

    // FS100 requires trajectory start at current position
    namespace IRC_utils = industrial_robot_client::utils;
    if(!IRC_utils::isWithinRange(cur_name, cur_pos, traj.joint_names, traj.points[0].positions, start_pos_tol_)) {
        ROS_WARN_STREAM("Checking start state");
        std::map<std::string, double> cur_map, traj_map;
        industrial_robot_client::utils::toMap(cur_name, cur_pos, cur_map);
        industrial_robot_client::utils::toMap(traj.joint_names, traj.points[0].positions, traj_map);

        if(cur_pos.size() == traj.points[0].positions.size()) {
            for(unsigned int j = 0; j < cur_name.size(); ++j) {
                ROS_INFO_STREAM(cur_name[j] << ": " << cur_map[cur_name[j]] - traj_map[cur_name[j]]);
            }
        }
        ROS_ERROR_RETURN(false, "Validation failed: Trajectory doesn't start at current position.");
    }
    return true;
}


bool ClopemaJointTrajectoryStreamer::is_valid(const trajectory_msgs::JointTrajectory& traj) {
    if(!JointTrajectoryInterface::is_valid(traj))
        return false;

    for(int i = 0; i < traj.points.size(); ++i) {
        const trajectory_msgs::JointTrajectoryPoint& pt = traj.points[i];

        // FS100 requires valid velocity data
        if(pt.velocities.empty())
            ROS_ERROR_RETURN(false, "Validation failed: Missing velocity data for trajectory pt %d", i);
    }

    if((cur_joint_pos_.header.stamp - ros::Time::now()).toSec() > pos_stale_time_)
        ROS_ERROR_RETURN(false, "Validation failed: Can't get current robot position.");


    if(cur_joint_pos_.name.empty()) {
        ROS_ERROR_RETURN(false, "Validation failed: Current position empty ~ is joint state running?");
    }

    return true;
}

// override create_message to generate JointTrajPtFull message (instead of default JointTrajPt)
bool ClopemaJointTrajectoryStreamer::create_message(int seq, const trajectory_msgs::JointTrajectoryPoint& pt, SimpleMessage* msg) {
    JointTrajPtFull msg_data;
    JointData values;

    // copy position data
    if(!pt.positions.empty()) {
        if(VectorToJointData(pt.positions, values))
            msg_data.setPositions(values);
        else
            ROS_ERROR_RETURN(false, "Failed to copy position data to JointTrajPtFullMessage");
    } else
        msg_data.clearPositions();

    // copy velocity data
    if(!pt.velocities.empty()) {
        if(VectorToJointData(pt.velocities, values))
            msg_data.setVelocities(values);
        else
            ROS_ERROR_RETURN(false, "Failed to copy velocity data to JointTrajPtFullMessage");
    } else
        msg_data.clearVelocities();

    // copy acceleration data
    if(!pt.accelerations.empty()) {
        if(VectorToJointData(pt.accelerations, values))
            msg_data.setAccelerations(values);
        else
            ROS_ERROR_RETURN(false, "Failed to copy acceleration data to JointTrajPtFullMessage");
    } else
        msg_data.clearAccelerations();

    // copy scalar data
    msg_data.setRobotID(robot_id_);
    msg_data.setSequence(seq + streaming_offset);
    msg_data.setTime(pt.time_from_start.toSec() + streaming_last_time);

    // convert to message
    JointTrajPtFullMessage jtpf_msg;
    jtpf_msg.init(msg_data);

    return jtpf_msg.toRequest(*msg);  // assume "request" COMM_TYPE for now
}

bool ClopemaJointTrajectoryStreamer::get_synchronization() {
    boost::mutex::scoped_lock sl(mutex_synchronize);
    return synchronize;
}

void ClopemaJointTrajectoryStreamer::set_synchronization(bool enable) {
    boost::mutex::scoped_lock sl(mutex_synchronize);
    synchronize = enable;
}

bool ClopemaJointTrajectoryStreamer::cb_set_drive_power(industrial_msgs::SetDrivePowerRequest& req, industrial_msgs::SetDrivePowerResponse& res) {
    if(req.drive_power)
        this->motion_ctrl.setTrajMode(true);
    else
        setPowerOff(false);

    streaming_first_received = false;
    return true;
}

bool ClopemaJointTrajectoryStreamer::cb_set_synchronization(clopema_drivers_msgs::SetSynchronizationRequest& req, clopema_drivers_msgs::SetSynchronizationResponse& res) {
    set_synchronization(req.enable);
    return true;
}

void ClopemaJointTrajectoryStreamer::cb_joint_stream(const trajectory_msgs::JointTrajectoryConstPtr& msg) {
    if(msg->points.size() != 1) {
        ROS_WARN_STREAM("Joint streaming interface expect just one point in the trajectory - if you see this message then use jointTrajectoryCB instead of point streaming, it is safer!!!");
        return;
    }

    trajectory_msgs::JointTrajectoryPtr traj(new trajectory_msgs::JointTrajectory);
    *traj = *msg;
    if(!add_missed_joints(traj))
        return;

    std::vector<SimpleMessage> new_traj_msgs;
    if(!trajectory_to_msgs(traj, &new_traj_msgs))
        return;

    if(!streaming_first_received) {
        if(!check_start_state(*traj)) {
            ROS_WARN_STREAM("check start state failed");
            return;
        }
        if(!send_to_robot(new_traj_msgs)) { //send command messages to robot
            ROS_WARN_STREAM("Cannot send first point to robot");
            return;
        }
        streaming_first_received = true;
    } else {
        messages.push_vector(new_traj_msgs);
    }

    streaming_offset += new_traj_msgs.size();
    streaming_last_time += traj->points.back().time_from_start.toSec();
}

void ClopemaJointTrajectoryStreamer::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr& msg) {
    if(msg->points.empty()) {
        trajectoryStop();
        return;
    }

    trajectory_msgs::JointTrajectoryPtr traj(new trajectory_msgs::JointTrajectory);
    *traj = *msg;

    if(!add_missed_joints(traj))
        return;

    std::vector<SimpleMessage> new_traj_msgs;
    if(!trajectory_to_msgs(traj, &new_traj_msgs))
        return;
    if(!check_start_state(*traj)) {
        ROS_WARN_STREAM("check start state failed");
        return;
    }
    if(!send_to_robot(new_traj_msgs)) //send command messages to robot
        ROS_WARN_STREAM("Cannot send trajectory to robot");
}

void ClopemaJointTrajectoryStreamer::cb_as_goal() {
    boost::mutex::scoped_lock l(mutex_as_);
    if(as_.isActive()) {
        return;
    }

    control_msgs::FollowJointTrajectoryGoalConstPtr goal;
    goal = as_.acceptNewGoal();
    if(goal->trajectory.points.empty()) {
        as_.setAborted();
        return;
    }

    trajectory_msgs::JointTrajectoryPtr traj(new trajectory_msgs::JointTrajectory);
    *traj = goal->trajectory;

    if(!add_missed_joints(traj)) {
        as_.setAborted();
        return;
    }

    std::vector<SimpleMessage> new_traj_msgs;
    if(!trajectory_to_msgs(traj, &new_traj_msgs)) {
        as_.setAborted();
        return;
    }
    if(!check_start_state(*traj)) {
        ROS_WARN_STREAM("check start state failed");
        as_.setAborted();
        return;
    }
    if(!send_to_robot(new_traj_msgs)) { //send command messages to robot
        ROS_WARN_STREAM("Cannot send trajectory to robot");
        as_.setAborted();
        return;
    }
}

void ClopemaJointTrajectoryStreamer::cb_as_preempt() {
    boost::mutex::scoped_lock l(mutex_as_);
    trajectoryStop();
    as_.setPreempted();
}

void ClopemaJointTrajectoryStreamer::cb_robot_status(const industrial_msgs::RobotStatusConstPtr& rs) {
    boost::mutex::scoped_lock lock(mutex_as_, boost::try_to_lock);
    if(!lock)
        return;

    if(rs->in_motion.val)
        return;

    if(!messages.empty())
        return;
    if(!q_send.empty())
        return;

    int n;
    bool rdy = motion_ctrl.getSynchro(n);
    if(!rdy && n == 0)
        return;

    if(!as_.isActive())
        return;

    as_.setSucceeded();
}
