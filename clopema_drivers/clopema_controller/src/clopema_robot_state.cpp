/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 07/23/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Clopema Robot State Publisher
 */

#include <ros/ros.h>
#include <clopema_controller/ClopemaJointRelayHandler.h>
#include <clopema_controller/ClopemaRobotStateInterface.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "clopema_robot_state");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    industrial_robot_client::robot_state_interface::ClopemaRobotStateInterface rsi;
    if(!rsi.init("192.168.0.101", 50241)) {
        ROS_ERROR_STREAM("Cannot initialize controller");
        return -1;
    }
    
    rsi.run();
    ros::waitForShutdown();
    
    return 0;
}
