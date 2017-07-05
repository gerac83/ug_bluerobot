/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 10/13/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Clopema Virtual Robot controller
 */

#include <ros/ros.h>
#include <virtual_robot/VirtualRobot.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "virtual_robot");
    ros::NodeHandle node("~");
    
    VirtualRobot vb;
    ros::spin();
    
    return 0;
}
