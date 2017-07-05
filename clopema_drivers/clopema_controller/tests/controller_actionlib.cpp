/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 10/10/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Test ActionLib interface provided by clopema_controller package.
 *             Controller must be started in advance and robot will move during the test!
 *
 *             roslaunch clopema_controller start_controller.launch
 *             catkin_make run_tests_clopema_controller_gtest_controller_actionlib
 *
 *             Following tests are performing:
 *                      1) MoveTAxis - moveing with r2_joint_t by +PI/16 and back again
 *                      2) MoveFromBadState - this is expected not to move with robot because start state is wrong
 *
 *    Warning: Robot is going to move during the test (T-axis of the second robot only)
 */

#include <gtest/gtest.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <industrial_msgs/SetDrivePower.h>

TEST(ControllerActionlib, MoveTAxis) {
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("/clopema_controller/follow_joint_trajectory");
    bool server_connected = client.waitForServer(ros::Duration(5.0));
    ASSERT_TRUE(server_connected);

    sensor_msgs::JointStateConstPtr jmsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(5.0));
    ASSERT_TRUE(jmsg);
    unsigned int k = std::distance(jmsg->name.begin(), std::find(jmsg->name.begin(), jmsg->name.end(), "r2_joint_t"));
    ASSERT_TRUE(k >= 0);
    ASSERT_TRUE(k < jmsg->name.size());

    double t_value = jmsg->position[k];

    control_msgs::FollowJointTrajectoryGoal g;
    g.trajectory.joint_names.push_back("r2_joint_t");

    trajectory_msgs::JointTrajectoryPoint p;
    p.accelerations.push_back(0.0);
    p.velocities.push_back(0.0);
    p.positions.push_back(0.0);

    p.positions[0] = t_value;
    p.time_from_start = ros::Duration(0.0);
    g.trajectory.points.push_back(p);

    p.positions[0] = t_value - M_PI_4 / 4;
    p.time_from_start = ros::Duration(1.0);
    g.trajectory.points.push_back(p);
    client.sendGoal(g);

    bool res = client.waitForResult(ros::Duration(5.0));
    ASSERT_TRUE(res);
    ASSERT_TRUE(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

    g.trajectory.points.clear();
    p.positions[0] = t_value - M_PI_4 / 4;
    p.time_from_start = ros::Duration(0.0);
    g.trajectory.points.push_back(p);

    p.positions[0] = t_value;
    p.time_from_start = ros::Duration(1.0);
    g.trajectory.points.push_back(p);

    client.sendGoal(g);
    res = client.waitForResult(ros::Duration(5.0));
    ASSERT_TRUE(res);
    ASSERT_TRUE(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    
    industrial_msgs::SetDrivePower srv;
    srv.request.drive_power = false;
    EXPECT_TRUE(ros::service::call("/clopema_controller/set_drive_power", srv));
}

TEST(ControllerActionlib, MoveFromBadState) {
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("/clopema_controller/follow_joint_trajectory");
    bool server_connected = client.waitForServer(ros::Duration(5.0));
    ASSERT_TRUE(server_connected);

    sensor_msgs::JointStateConstPtr jmsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(5.0));
    ASSERT_TRUE(jmsg);
    unsigned int k = std::distance(jmsg->name.begin(), std::find(jmsg->name.begin(), jmsg->name.end(), "r2_joint_t"));
    ASSERT_TRUE(k >= 0);
    ASSERT_TRUE(k < jmsg->name.size());

    double t_value = jmsg->position[k] + 1.0;

    control_msgs::FollowJointTrajectoryGoal g;
    g.trajectory.joint_names.push_back("r2_joint_t");

    trajectory_msgs::JointTrajectoryPoint p;
    p.accelerations.push_back(0.0);
    p.velocities.push_back(0.0);
    p.positions.push_back(0.0);

    p.positions[0] = t_value;
    p.time_from_start = ros::Duration(0.0);
    g.trajectory.points.push_back(p);

    p.positions[0] = t_value - M_PI_4 / 4;
    p.time_from_start = ros::Duration(1.0);
    g.trajectory.points.push_back(p);
    client.sendGoal(g);

    bool res = client.waitForResult(ros::Duration(5.0));
    ASSERT_TRUE(res);
    ASSERT_TRUE(client.getState() == actionlib::SimpleClientGoalState::ABORTED);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gtest_controller_actionlib");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
