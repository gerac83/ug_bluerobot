/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 08/12/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: ClopemaJointTrajectoryStreamer class - streaming points synchronelly to the controller.
 */

#ifndef CLOPEMAJOINTTRAJECTORYSTREAMER_H
#define CLOPEMAJOINTTRAJECTORYSTREAMER_H

#include <clopema_controller/MotomanMotionCtrl.h>

#include <simple_message/simple_message.h>
#include <simple_message/messages/joint_traj_pt_full_message.h>
#include <motoman_driver/industrial_robot_client/joint_trajectory_streamer.h>
#include <motoman_driver/simple_message/motoman_motion_reply_message.h>
#include <clopema_controller/ConcurentQueue.h>
#include <industrial_msgs/SetDrivePower.h>
#include <clopema_drivers_msgs/SetSynchronization.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <industrial_msgs/RobotStatus.h>

using motoman::motion_ctrl::MotomanMotionCtrl;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;

class ClopemaJointTrajectoryStreamer : public industrial_robot_client::joint_trajectory_interface::JointTrajectoryInterface {
public:
    ClopemaJointTrajectoryStreamer();
    ~ClopemaJointTrajectoryStreamer();

    /** \brief Initialize robot connection using specified method.
      * \details Will initialize all internal variables (grouped_joint_names, velocity limits, motion_ctrl) -
      *          Nothing is extracted from URDF you can overwrite get_grouped_joints_names or get_velocity_limits if you want.
      * \param connection new robot-connection instance (ALREADY INITIALIZED).
      * \return true on success, false otherwise */
    virtual bool init(SmplMsgConnection* connection);

    /** \brief Set trajectory mode to false which will cause the servo power off
      * \param force whether stop trajectory before setPowerOff */
    bool setPowerOff(bool force = false);
    
    /** \brief Stop current trajectory */
    void trajectoryStop();

protected:
    /** \brief Get grouped joint names i.e. vector of vector of joint names
     *  \details For example: {{r1_j1, r1_j2, ..., r1_jn}, {r2_j1, ..., r2_jn}, {station_1}}
     *           Is called during initialization. */
    virtual std::vector<std::vector<std::string> > get_grouped_joints_names();

    /** \brief Get map of maximum velocities for each joint in grouped joint_names
      * \details Is called during initialization */
    virtual std::map<std::string, double> get_velocity_limits();

    /** \brief Convert trajectory to msgs based on grouped_joint_names
     *  \details The result order (for three groups) will be: g1-point1;g2-point1;g3-point1;g1-point2;g2-point2;g3-point2;... */
    virtual bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<SimpleMessage>* msgs);

    /** \brief Callback for joint trajectory - in addition to parent add non filled joints into the trajectory based on start state
      * \details Adding not filled points will allow execution of non full trajectory (for example only rotating one axis) */
    virtual void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr& msg);

    /** \brief Send messages into the controller via separate streaming thread */
    virtual bool send_to_robot(const std::vector<SimpleMessage>& messages);

    /** \brief Create SimpleMessage for sending to the robot
      * \param[in] seq sequence # of this point in the overall trajectory
      * \param[in] pt  trajectory point data
      * \param[out] msg message for sending to robot
      * \return true on success, false otherwise */
    virtual bool create_message(int seq, const trajectory_msgs::JointTrajectoryPoint& pt, SimpleMessage* msg);
    
    /** \brief Check that start state is equal to current state */
    bool check_start_state(const trajectory_msgs::JointTrajectory& traj);
    
    /** \brief Check current trajectory for validity */
    bool is_valid(const trajectory_msgs::JointTrajectory& traj);
    
    /** \brief Convert vector of doubles to the joint data */
    static bool VectorToJointData(const std::vector<double>& vec, industrial::joint_data::JointData& joints);

    /** \brief Streaming thread which send data from queue to the controller */
    virtual void streamingThread();
    
    /** \brief Try to reconnect n times 
      * \return true if connected, false otherwise */
    bool reconnect(unsigned int n);
    
    /** \brief Set synchronization 
     *  \param \enable if true synchronization will be used for motion controll */
    void set_synchronization(bool enable = true);
    
    /** \brief Get value of synchronization variable 
     *  \return true if synchronization is required */
    bool get_synchronization();
    
    /** \brief Send q_send to the robot in 'parallel' i.e. if first group is busy send to the sencond etc.
     *  \details If busy si returned for message it is pushed to the queue end ~ i.e. order is not mandatory
     *           Will block until all points are send or until error
     *  \return true if all points were sucessfully send to the robot */
    bool send_queue_to_robot(ConcurentQueue<SimpleMessage>& q_send);
    
    /** \brief Add missed joints into the trajectory based on current state */
    bool add_missed_joints(trajectory_msgs::JointTrajectoryPtr& traj);
    
    /** \brief Set drive power callback */
    bool cb_set_drive_power(industrial_msgs::SetDrivePowerRequest& req, industrial_msgs::SetDrivePowerResponse& res);
    
    /** \brief Set synchronization callback */
    bool cb_set_synchronization(clopema_drivers_msgs::SetSynchronizationRequest& req, clopema_drivers_msgs::SetSynchronizationResponse& res);
    
    /** \brief Publish queue count from the controller */
    void cb_pub_q_cnt(const ros::TimerEvent& event);
    
    /** \brief Callback of the point streaming interface, assume just one point in trajectory */
    void cb_joint_stream(const trajectory_msgs::JointTrajectoryConstPtr& msg);
    
    /** \brief Callback for goals for actionlib server */
    void cb_as_goal();
    
    /** \brief Callback for preemption for actionlib server */
    void cb_as_preempt();
    
    /** \brief Callback for robot state - used for in_motion controlling for actionlib server */
    void cb_robot_status(const industrial_msgs::RobotStatusConstPtr& rs);

protected:
    static constexpr double pos_stale_time_ = 1.0;  // max time since last "current position" update, for validation (sec)
    static constexpr double start_pos_tol_  = 4e-2; // max difference btwn start & current position, for validation (rad)

    ConcurentQueue<SimpleMessage> messages;
    ConcurentQueue<SimpleMessage> q_send; //queue (SET) which is sent directly to the robot nomatter in what order, must be global in case of trajStop
    motoman::motion_ctrl::MotomanMotionCtrl motion_ctrl; //motman specific controlling
    std::vector<std::vector<std::string> > grouped_joint_names; // joints in groups
    int robot_id_;
    
    boost::thread streaming_thread_;
    boost::shared_ptr<boost::mutex> mutex_connection;
    
    bool synchronize; //protected (mutex) variable to share synchronization status
    boost::mutex mutex_synchronize;
    
    ros::ServiceServer srv_set_synchro;
    ros::ServiceServer srv_power_off;
    
    bool streaming_first_received;
    int streaming_offset;
    double streaming_last_time;
    
    ros::Timer timer_pub_queue_cnt;
    ros::Publisher pub_queue_cnt;
    ros::Subscriber sub_joint_stream;
    ros::Subscriber sub_robot_status;
    
    boost::mutex mutex_as_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    
};

#endif // CLOPEMAJOINTTRAJECTORYSTREAMER_H
