#include "clopema_libs/RobotMovingUtils.h"
#include "clopema_robot/robot_commander.h"
#include "eigen_conversions/eigen_msg.h"
#include "clopema_libs/clopema_pose_generator.h"
#include "math.h"

using namespace moveit::planning_interface;

/* ------------------------------------------------------------------------- */
bool RobotMovingUtils::move_external_axis_to_table(std::string table)
{
    clopema_robot::ClopemaRobotCommander ext("ext");
    if(table == "t3")
        ext.setNamedTarget("ext_minus_90");
    else if(table == "t1")
        ext.setNamedTarget("ext_plus_90");
    else if(table == "t2")
        ext.setNamedTarget("home_ext");

    return ext.move();
}

/* ------------------------------------------------------------------------- */
bool RobotMovingUtils::move_external_axis_home()
{
    clopema_robot::ClopemaRobotCommander ext("ext");
    ext.setNamedTarget("ext_home");
    return ext.move();
}

/* ------------------------------------------------------------------------- */
bool RobotMovingUtils::move_arms_home()
{
    clopema_robot::ClopemaRobotCommander arms("arms");
    arms.setNamedTarget(clopema_robot::NAMED_TARGET_ARMS_HOME);
    return arms.move();
}

bool RobotMovingUtils::move_all_home()
{
    move_arms_home();
    move_external_axis_home();
}

/* ------------------------------------------------------------------------- */
bool RobotMovingUtils::open_gripper(std::string arm)
{
    clopema_robot::ClopemaRobotCommander crc(arm + "_arm");
    return crc.execute_traj(crc.gripper_trajectory(clopema_robot::GRIPPER_OPEN));
}

/* ------------------------------------------------------------------------- */
bool RobotMovingUtils::close_gripper(std::string arm)
{
    clopema_robot::ClopemaRobotCommander crc(arm + "_arm");
    return crc.execute_traj(crc.gripper_trajectory(clopema_robot::GRIPPER_CLOSE));
}

/* ------------------------------------------------------------------------- */
bool RobotMovingUtils::open_grippers()
{
    clopema_robot::ClopemaRobotCommander crc("arms");
    return crc.execute_traj(crc.gripper_trajectory(clopema_robot::GRIPPER_OPEN));
}

/* ------------------------------------------------------------------------- */
bool RobotMovingUtils::close_grippers()
{
    clopema_robot::ClopemaRobotCommander crc("arms");
    return crc.execute_traj(crc.gripper_trajectory(clopema_robot::GRIPPER_CLOSE));
}

/* ------------------------------------------------------------------------- */
void RobotMovingUtils::servo_off()
{
   clopema_robot::ClopemaRobotCommander crc("arms");
   crc.setServoPowerOff();
}

/* ------------------------------------------------------------------------- */
bool RobotMovingUtils::move_xtion1_above_t3()
{
    move_external_axis_home();
    clopema_robot::ClopemaRobotCommander g("arms");
    g.setStartStateToCurrentState();                             
    return g.move_to_named_target(clopema_robot::NAMED_TARGET_ARMS_XTION1_ABOVE_T3);
}

/* ------------------------------------------------------------------------- */
bool RobotMovingUtils::target_xtion_to_clockwise_table(std::string table, std::string arm)
{
        if(table == "t3")
        {
            move_external_axis_home();
        }
        if(table == "t4")
        {
            move_external_axis_to_table("t3");
        }
    
        clopema_robot::ClopemaRobotCommander g(arm + "_arm");
        geometry_msgs::Pose table_pose;
        robot_state::RobotStatePtr state = g.getCurrentState();
        tf::poseEigenToMsg(state->getGlobalLinkTransform(table + "_desk"), table_pose);

        geometry_msgs::Pose xtion_new_pose;
        xtion_new_pose.position.z = 0.7;

        if(table == "t3")
        {
            table_pose.position.x += 0.55;
            xtion_new_pose.position.x -= 0.25;
        }
        
        if(table == "t4")
        {
            table_pose.position.y -= 0.35;
            xtion_new_pose.position.y += 0.15;
        }
        
        xtion_new_pose = RobotMovingUtils::create_oriented_pose(xtion_new_pose, table_pose);

        g.setJointValueTarget(xtion_new_pose, arm + "_ee");

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if(g.plan(plan))
        {
            g.execute(plan);
            return true;
        }
    return false;
}

/* ------------------------------------------------------------------------- */
geometry_msgs::Pose RobotMovingUtils::create_oriented_pose(geometry_msgs::Pose pose, geometry_msgs::Pose target)
{
    Eigen::Affine3d target_eig, pose_eig;
    tf::poseMsgToEigen(target, target_eig);
    tf::poseMsgToEigen(pose, pose_eig);
    Eigen::Affine3d pose_in_base_eig = target_eig * pose_eig;

    geometry_msgs::Pose pose_in_base;
    tf::poseEigenToMsg(pose_in_base_eig, pose_in_base);
    
    return  clopema_pose_generator::orient_pose(pose_in_base.position, target.position);
}
