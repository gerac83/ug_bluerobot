/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 10/13/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Virtual Robot Definition provides interface for robot
 */

#ifndef VIRTUAL_ROBOT_H
#define VIRTUAL_ROBOT_H

#include <ros/ros.h>
#include <virtual_robot/ConcurentQueue.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <industrial_msgs/SetDrivePower.h>
#include <clopema_drivers_msgs/SetSynchronization.h>
#include <actionlib/server/simple_action_server.h>
#include <industrial_msgs/CmdJointTrajectory.h>
#include <industrial_msgs/StopMotion.h>

class VirtualRobot {
    public:
        VirtualRobot();

        /** \brief Publish current joint state */
        void cb_publish_joint_state(const ros::TimerEvent& te);

        /** \brief Publish robot status */
        void cb_publish_robot_status(const ros::TimerEvent& te);

        /** \brief Apply next robot state from the queue */
        void cb_next_state(const ros::TimerEvent& te);

        /** \brief Interpolate trajectory based on time parametrization and add it to the execution queue */
        bool send_trajectory_to_robot(const trajectory_msgs::JointTrajectory& traj);

        /** \brief Add all joint names to the trajectory point */
        trajectory_msgs::JointTrajectoryPoint complete_point(const trajectory_msgs::JointTrajectoryPoint& p, std::vector<std::string> pnames);

        /** \brief Set drive power callback */
        bool cb_set_drive_power(industrial_msgs::SetDrivePowerRequest& req, industrial_msgs::SetDrivePowerResponse& res);

        /** \brief Set synchronization callback */
        bool cb_set_synchronization(clopema_drivers_msgs::SetSynchronizationRequest& req, clopema_drivers_msgs::SetSynchronizationResponse& res);

        /** \brief Callback of the point streaming interface, assume just one point in trajectory */
        void cb_joint_stream(const trajectory_msgs::JointTrajectoryConstPtr& msg);

        /** \brief Callback for goals for actionlib server */
        void cb_as_goal();

        /** \brief Callback for preemption for actionlib server */
        void cb_as_preempt();

        /** \brief Callback for joint trajectory - in addition to parent add non filled joints into the trajectory based on start state
        *   \details Adding not filled points will allow execution of non full trajectory (for example only rotating one axis) */
        void cb_joint_trajectory(const trajectory_msgs::JointTrajectoryConstPtr& msg);
        
        /** \brief Service callback to execute trajectory */
        bool cb_joint_trajectory(industrial_msgs::CmdJointTrajectoryRequest& req, industrial_msgs::CmdJointTrajectoryResponse& res);
        
        /** \brief Stop motion service callback function */
        bool cb_stop_motion(industrial_msgs::StopMotionRequest& req, industrial_msgs::StopMotionResponse& res);

    private:
        ros::NodeHandle node_;
        std::vector<std::string> joint_names;
        std::vector<double> joint_positions;
        static const double precision = 1e-3;
        static const double robot_period = 0.01;

        ros::Timer timer_joint_state, timer_robot_status, timer_next_state;
        ros::Publisher pub_joint_state, pub_robot_status;
        ros::ServiceServer ser_set_drive_power, ser_synchronization, ser_trajectory, ser_stop_motion;
        ros::Subscriber sub_joint_stream, sub_trajectory;

        ConcurentQueue<std::vector<double> > queue;

        actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
        bool drive_powered;
};

#endif //VIRTUAL_ROBOT
