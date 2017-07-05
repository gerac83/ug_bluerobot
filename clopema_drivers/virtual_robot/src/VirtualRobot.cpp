#include <virtual_robot/VirtualRobot.h>
#include <sensor_msgs/JointState.h>
#include <industrial_msgs/RobotStatus.h>

VirtualRobot::VirtualRobot() :
    node_("~"),
    as_(node_, "follow_joint_trajectory", false) {

    if(!node_.getParam("joint_names", joint_names)) {
        ROS_ERROR_STREAM("Joint Names are not specified, stoping");
        return;
    }
    node_.getParam("default_positions", joint_positions);
    joint_positions.resize(joint_names.size(), 0.0); //To ensure same size of the vectors

    timer_next_state = node_.createTimer(ros::Duration(robot_period), &VirtualRobot::cb_next_state, this);

    std::string set_power_down_service;
    if(node_.getParam("set_power_down_service", set_power_down_service)) {
        if(!set_power_down_service.empty()) {
            ROS_INFO_STREAM("Set power down service allowed: " << set_power_down_service);
            ser_set_drive_power = node_.advertiseService(set_power_down_service, &VirtualRobot::cb_set_drive_power, this);
        }
    }

    std::string synchronization_service;
    if(node_.getParam("synchronization_service", synchronization_service)) {
        if(!synchronization_service.empty()) {
            ROS_INFO_STREAM("Syncronization service allowed: " << synchronization_service);
            ser_synchronization = node_.advertiseService(synchronization_service, &VirtualRobot::cb_set_synchronization, this);
        }
    }

    bool allow_joint_states, allow_robot_status, allow_path_actionlib, allow_point_streaming, allow_path_service, allow_path_topic, allow_stop_motion;
    node_.param("allow_joint_states", allow_joint_states, true);
    node_.param("allow_robot_status", allow_robot_status, true);
    node_.param("allow_point_streaming", allow_point_streaming, true);
    node_.param("allow_path_actionlib", allow_path_actionlib, true);
    node_.param("allow_path_service", allow_path_service, true);
    node_.param("allow_stop_motion", allow_stop_motion, true);
    node_.param("allow_path_topic", allow_path_topic, true);

    if(allow_path_topic) {
        ROS_INFO_STREAM("Path topic allowed");
        sub_trajectory = node_.subscribe("joint_path_command", 1, &VirtualRobot::cb_joint_trajectory, this);
    }

    if(allow_stop_motion) {
        ROS_INFO_STREAM("Stop motion service allowed");
        ser_stop_motion = node_.advertiseService("stop_motion", &VirtualRobot::cb_stop_motion, this);
    }

    if(allow_path_service) {
        ROS_INFO_STREAM("Path trajectory allowed");
        ser_trajectory = node_.advertiseService("joint_path_command", &VirtualRobot::cb_joint_trajectory, this);
    }

    if(allow_joint_states) {
        ROS_INFO_STREAM("Joint states publisher allowed");
        timer_joint_state = node_.createTimer(ros::Duration(0.05), &VirtualRobot::cb_publish_joint_state, this);
        pub_joint_state = node_.advertise<sensor_msgs::JointState>("joint_states", 1);
    }

    if(allow_robot_status) {
        ROS_INFO_STREAM("Robot status publisher allowed");
        timer_robot_status = node_.createTimer(ros::Duration(0.05), &VirtualRobot::cb_publish_robot_status, this);
        pub_robot_status = node_.advertise<industrial_msgs::RobotStatus>("robot_status", 1);
    }

    if(allow_point_streaming) {
        ROS_INFO_STREAM("Point streaming interface allowed");
        sub_joint_stream = node_.subscribe("joint_stream_command", 1, &VirtualRobot::cb_joint_stream, this);
    }

    if(allow_path_actionlib) {
        ROS_INFO_STREAM("Actionlib interface allowed");
        as_.registerPreemptCallback(boost::bind(&VirtualRobot::cb_as_preempt, this));
        as_.registerGoalCallback(boost::bind(&VirtualRobot::cb_as_goal, this));
        as_.start();
    }

}

void VirtualRobot::cb_publish_joint_state(const ros::TimerEvent& te) {
    sensor_msgs::JointState jmsg;
    jmsg.header.stamp = ros::Time::now();
    jmsg.name = joint_names;
    jmsg.position = joint_positions;
    jmsg.velocity.resize(joint_names.size(), 0.0);
    jmsg.effort.resize(joint_names.size(), 0.0);
    pub_joint_state.publish(jmsg);
}

void VirtualRobot::cb_publish_robot_status(const ros::TimerEvent& te) {
    industrial_msgs::RobotStatus rsmsg;
    rsmsg.header.stamp = ros::Time::now();
    rsmsg.mode.val = 2;
    rsmsg.e_stopped.val = 0;
    rsmsg.in_error.val = 0;
    rsmsg.drives_powered.val = drive_powered ? 1 : 0;
    rsmsg.motion_possible.val = drive_powered ? 1 : 0;
    rsmsg.in_motion.val = queue.empty() ? 0 : 1;
    pub_robot_status.publish(rsmsg);
}

void VirtualRobot::cb_next_state(const ros::TimerEvent& te) {
    if(!queue.try_pop(joint_positions)) {
        if(as_.isActive()) {
            as_.setSucceeded();
        }
    }
}

trajectory_msgs::JointTrajectoryPoint VirtualRobot::complete_point(const trajectory_msgs::JointTrajectoryPoint& p, std::vector< std::string > pnames) {
    trajectory_msgs::JointTrajectoryPoint pout;
    for(unsigned int i = 0; i < joint_names.size(); ++i) {
        int k = std::find(pnames.begin(), pnames.end(), joint_names[i]) - pnames.begin();
        if(k >= pnames.size() || k < 0) {
            pout.positions.push_back(joint_positions[i]);
        } else {
            pout.positions.push_back(p.positions[k]);
        }
    }
    pout.time_from_start = p.time_from_start;

    return pout;
}


bool VirtualRobot::send_trajectory_to_robot(const trajectory_msgs::JointTrajectory& traj) {
    using namespace trajectory_msgs;

    if(traj.points.size() == 1) {
        JointTrajectoryPoint p = complete_point(traj.points[0], traj.joint_names);
//         int number_of_steps = ceil((p.time_from_start).toSec() / robot_period);
//         for(unsigned int i = 0; i <= number_of_steps; ++i) {
            queue.push(p.positions);
//         }
        drive_powered = true;
    }

    for(unsigned int i = 1; i < traj.points.size(); ++i) { //for each point
        JointTrajectoryPoint p_last = complete_point(traj.points[i - 1], traj.joint_names);
        JointTrajectoryPoint p = complete_point(traj.points[i], traj.joint_names);


        std::vector<double> jdiff(p.positions.size(), 0.0);
        //compute differences
        for(unsigned int j = 0; j < p.positions.size(); ++j) {
            jdiff[j] = p.positions[j] - p_last.positions[j];
        }

        int number_of_steps = ceil((p.time_from_start - p_last.time_from_start).toSec() / robot_period);
        if(number_of_steps < 1) {
            ROS_WARN_STREAM("Time of trajectory points have not been set correctly");
            ROS_DEBUG_STREAM("Time of current point: " << p.time_from_start.toSec());
            ROS_DEBUG_STREAM("Time of previous point: " << p_last.time_from_start.toSec());
            return false;
        }

        std::vector<double> jstate(p.positions.size(), 0.0);
        for(unsigned int k = 1; k <= number_of_steps; ++k) {
            for(unsigned int j = 0; j < p.positions.size(); ++j) {
                jstate[j] = p_last.positions[j] +  k * jdiff[j] / number_of_steps;
            }
            queue.push(jstate);
            drive_powered = true;
        }
    }
    return true;
}


void VirtualRobot::cb_as_goal() {
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

    if(!send_trajectory_to_robot(*traj))
        as_.setAborted();
}

void VirtualRobot::cb_as_preempt() {
    queue.clear();
    as_.setPreempted();
}

bool VirtualRobot::cb_set_drive_power(industrial_msgs::SetDrivePowerRequest& req, industrial_msgs::SetDrivePowerResponse& res) {
    drive_powered = req.drive_power;
    return true;
}

bool VirtualRobot::cb_set_synchronization(clopema_drivers_msgs::SetSynchronizationRequest& req, clopema_drivers_msgs::SetSynchronizationResponse& res) {
    return true;
}

void VirtualRobot::cb_joint_stream(const trajectory_msgs::JointTrajectoryConstPtr& msg) {
    if(!send_trajectory_to_robot(*msg)) {
        ROS_ERROR_STREAM("Cannot execute trajectory");
    }
}

void VirtualRobot::cb_joint_trajectory(const trajectory_msgs::JointTrajectoryConstPtr& msg) {
    if(!send_trajectory_to_robot(*msg)) {
        ROS_ERROR_STREAM("Cannot execute trajectory");
    }
}

bool VirtualRobot::cb_joint_trajectory(industrial_msgs::CmdJointTrajectoryRequest& req, industrial_msgs::CmdJointTrajectoryResponse& res) {
    if(!send_trajectory_to_robot(req.trajectory)) {
        ROS_ERROR_STREAM("Cannot execute trajectory");
    }
    return true;
}

bool VirtualRobot::cb_stop_motion(industrial_msgs::StopMotionRequest& req, industrial_msgs::StopMotionResponse& res) {
    queue.clear();
    return true;
}
