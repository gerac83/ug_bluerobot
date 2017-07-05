/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 10/10/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Test virtual robot interface. In order to test to pass you need to launch virtual_robot_test.launch in advance.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

TEST(VirtualRobot, JointNamesFromParam) {
    using namespace sensor_msgs;
    JointStateConstPtr jmsg = ros::topic::waitForMessage<JointState>("/virtual_gripper/joint_states", ros::Duration(5.0));
    ASSERT_TRUE(jmsg);
    ASSERT_EQ(jmsg->name.size(), 1);
    ASSERT_EQ(jmsg->name[0], "joint");
}

TEST(VirtualRobot, DefaultValuesFromParam) {
    using namespace sensor_msgs;
    JointStateConstPtr jmsg = ros::topic::waitForMessage<JointState>("/virtual_gripper/joint_states", ros::Duration(5.0));
    ASSERT_TRUE(jmsg);
    ASSERT_EQ(jmsg->position.size(), 1);
    EXPECT_DOUBLE_EQ(jmsg->position[0], 0.5);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rostest_virtual_gripper");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

