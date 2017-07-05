#include <clopema_robot/robot_commander.h>
#include <gtest/gtest.h>
#include <fstream>

TEST(PlanFromStartPos, PlanFromStartPos) {
    clopema_robot::ClopemaRobotCommander crc("r2_arm");
    clopema_robot::ClopemaRobotCommander::Plan plan;
    
    std::vector<std::string> planners = crc.get_planner_types();
    for (unsigned int i = 0; i < planners.size(); i++) {
        crc.setPlannerId(planners[i]);
        robot_state::RobotState rs(*crc.getCurrentState());
        rs.setToDefaultValues();
               
        rs.setVariablePosition("r2_joint_s", 0.5);
        crc.setStartState(rs);
        crc.setNamedTarget(clopema_robot::NAMED_TARGET_ARMS_HOME);
            
        ASSERT_TRUE(crc.plan(plan));
        
        ASSERT_TRUE(plan.trajectory_.joint_trajectory.points.size() > 0);
        ASSERT_TRUE(plan.trajectory_.joint_trajectory.joint_names.size() == 6);
        
        std::string filename = (boost::format("/tmp/rostests%d.log") % i ).str();
        std::ofstream f(filename.c_str());
        f << (plan.trajectory_.joint_trajectory) << std::endl;
        f.close();
        
        trajectory_msgs::JointTrajectoryPoint point = plan.trajectory_.joint_trajectory.points[0];
        for(int i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); i++)
        {
            EXPECT_NEAR(point.positions[i], *rs.getJointPositions(plan.trajectory_.joint_trajectory.joint_names[i]), 0.01);
        }
    }
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gtest_plan_from_start_pos");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
