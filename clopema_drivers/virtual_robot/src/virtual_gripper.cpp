/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 10/16/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Virtual gripper node which provides ros interface to virtual gripper
 */

#include <ros/ros.h>
#include <virtual_robot/VirtualGripper.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "virtual_gripper");
    ros::NodeHandle node("~");
    
    VirtualGripper vg;
    
    
    ros::spin();
    return 0;
}
