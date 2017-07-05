/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 10/16/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Virtual Gripper provide standard action lib interface in order to be controllable via moveit or other higher level stuff.
 *             Moreover, the joint state of the gripper is published 0-closed/1-open.
 */

#ifndef VIRTUALGRIPPER_H
#define VIRTUALGRIPPER_H

#include <ros/ros.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/server/simple_action_server.h>

class VirtualGripper {
    public:
        VirtualGripper();
    
        /** \brief Execute goal required by the client */
        void cb_execute_goal(const control_msgs::GripperCommandGoalConstPtr &goal);
        
        /** \brief Timer for joint_state publishing */
        void cb_timer_joint_states(const ros::TimerEvent& te);
        
    private:
        ros::NodeHandle node_;
        std::string joint_name;
        double joint_position;
        
        ros::Publisher pub_joint_states;
        ros::Timer timer_joint_states;
        actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;
        
};

#endif // VIRTUALGRIPPER_H
