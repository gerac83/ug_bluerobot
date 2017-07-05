#ifndef ROBOT_MOVING_UTILS_H
#define ROBOT_MOVING_UTILS_H

#include <string>
#include <geometry_msgs/Pose.h>

class RobotMovingUtils
{
public:
    /// Move external axis of the robot ahead to given table
    static bool move_external_axis_to_table(std::string table);

    /// Move external axis to the home position
    static bool move_external_axis_home();

    /// Move arms to the home position
    static bool move_arms_home();
    
    /// Move all to the home position
    static bool move_all_home();
    
    /// Open gripper of selected arm
    static bool open_gripper(std::string arm);
    
    /// Close gripper of selected arm
    static bool close_gripper(std::string arm);
    
    /// Open both grippers
    static bool open_grippers();
    
    /// Close_both_grippers
    static bool close_grippers();
    
    /// Set the robot servo off
    static void servo_off();
    
    static bool move_xtion1_above_t3();
    
    static bool target_xtion_to_clockwise_table(std::string table, std::string arm);
    
    static geometry_msgs::Pose create_oriented_pose(geometry_msgs::Pose pose, geometry_msgs::Pose target);
    
};

#endif
