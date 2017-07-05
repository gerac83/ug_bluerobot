/**
 * Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
 *
 * Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
 * Institute:   Czech Technical University in Prague
 * Created on:  Oct 17, 2013
 */

#include "ros/ros.h"
#include "clopema_robot/robot_commander.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grippers_demo", ros::init_options::AnonymousName);
  clopema_robot::ClopemaRobotCommander robot("arms");
  ros::Rate r(2);
  while (ros::ok())
  {
      robot.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_OPEN);
      robot.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_OPEN, true);
      r.sleep();
      robot.setGripperState(clopema_robot::ACTION_R1_GRIPPER, clopema_robot::GRIPPER_CLOSE);
      robot.setGripperState(clopema_robot::ACTION_R2_GRIPPER, clopema_robot::GRIPPER_CLOSE, true);
      r.sleep();
  }
}
