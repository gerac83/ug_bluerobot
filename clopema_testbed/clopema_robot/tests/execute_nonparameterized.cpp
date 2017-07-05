/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 10/17/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Try execute nonparametrized trajectory.
 */

#include <gtest/gtest.h>
#include <clopema_robot/robot_commander.h>

TEST(ExecuteNonParametrized, ExecuteNonParametrized) {
    clopema_robot::ClopemaRobotCommander crc("arms");
    clopema_robot::ClopemaRobotCommander::Plan p;
    
    ASSERT_TRUE(crc.move_to_named_target(clopema_robot::NAMED_TARGET_ARMS_HOME));
    ASSERT_TRUE(crc.setJointValueTarget("r2_joint_t", 1.0));
    ASSERT_TRUE(crc.plan(p));
    for(unsigned int i=0; i<p.trajectory_.joint_trajectory.points.size(); ++i ) {
        p.trajectory_.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
    }
    ASSERT_TRUE(crc.execute(p));
    crc.set_start_state_from_traj(p.trajectory_.joint_trajectory);
    ASSERT_TRUE(crc.move_to_named_target(clopema_robot::NAMED_TARGET_ARMS_HOME));
    crc.setServoPowerOff();
}

TEST(ExecuteNonParametrized, ExecuteNonParametrizedNotAllowed) {
    clopema_robot::ClopemaRobotCommander crc("arms");
    clopema_robot::ClopemaRobotCommander::Plan p;
    crc.overwrite_time_parameterization = false;
    
    ASSERT_TRUE(crc.setJointValueTarget("r2_joint_t", 1.0));
    ASSERT_TRUE(crc.plan(p));
    for(unsigned int i=0; i<p.trajectory_.joint_trajectory.points.size(); ++i ) {
        p.trajectory_.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
    }
    ASSERT_FALSE(crc.execute(p));
    crc.setServoPowerOff();
}

TEST(ExecuteNonParametrized, ExecuteParametrizedFromIKT) {
    using namespace Eigen;
    clopema_robot::ClopemaRobotCommander crc("arms");
    
    robot_state::RobotState rs(*crc.getCurrentState());
    ASSERT_TRUE(crc.move_to_named_target(clopema_robot::NAMED_TARGET_ARMS_HOME));
    Affine3d pose_ = rs.getGlobalLinkTransform("xtion1_link");
    pose_.translation()[2] += 0.1;
    ASSERT_TRUE(rs.setFromIK(rs.getJointModelGroup("r1_xtion"), pose_));
    ASSERT_TRUE(crc.setJointValueTarget(rs));

    clopema_robot::ClopemaRobotCommander::Plan p;
    ASSERT_TRUE(crc.plan(p));
    ASSERT_TRUE(crc.execute(p));
    crc.set_start_state_from_traj(p.trajectory_.joint_trajectory);
    ASSERT_TRUE(crc.move_to_named_target(clopema_robot::NAMED_TARGET_ARMS_HOME));
    crc.setServoPowerOff();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "execute_nonparametrized");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
