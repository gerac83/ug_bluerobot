/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 10/10/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Test that robot is ready for movement. Driver is expected to be started in advance by:
 *             roslaunch clopema_controller start_controller.launch
 *             catkin_make run_tests_clopema_controller_gtest_robot_state_ready
 * 
 *             What is tested:
 *                      1) Correct size of JointState msg
 *                      2) RobotStatus values - i.e. e-stop is not pressed, mode value, etc.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <industrial_msgs/RobotStatus.h>

TEST(RobotStateReady, JointStateSize) {
    sensor_msgs::JointStateConstPtr jmsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(5.0));
    ASSERT_TRUE(jmsg);
    EXPECT_EQ(jmsg->name.size(), 13);
    EXPECT_EQ(jmsg->position.size(), 13);
}

TEST(RobotStateReady, RobotStatusValues) {
    industrial_msgs::RobotStatusConstPtr rsmsg = ros::topic::waitForMessage<industrial_msgs::RobotStatus>("/clopema_controller/robot_status", ros::Duration(5.0));
    ASSERT_TRUE(rsmsg);
    EXPECT_FALSE(rsmsg->e_stopped.val);
    EXPECT_FALSE(rsmsg->in_error.val);
    EXPECT_FALSE(rsmsg->in_motion.val);
    EXPECT_EQ(rsmsg->mode.val, 2);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gtest_controller_actionlib");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
