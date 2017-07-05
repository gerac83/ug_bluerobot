/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 07/24/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Clopema Joint Streamer Node implementation
 */

#include <ros/ros.h>
#include <clopema_controller/ClopemaJointTrajectoryStreamer.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "clopema_joint_streamer");
    ros::NodeHandle node("~");
    
    ClopemaJointTrajectoryStreamer jts;
    jts.run();

    return 0;
}
