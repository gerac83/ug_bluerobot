#!/usr/bin/env python

import geometry_msgs.msg
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface


def check_plan_is_valid(plan):
    """
    Checks if current plan contains a valid trajectory
    """
    return (plan is not None and len(plan.joint_trajectory.points) > 0)

rospy.init_node("clopema_shadow_example", anonymous=True)

# Initializing the Arm commander
arm_commander = MoveGroupCommander("r1_arm_and_manipulator")
print arm_commander.get_planning_frame()

# Setting this you can control the speed of the movements planned to the arm
# arm_commander.set_max_velocity_scaling_factor(0.5)

pose = geometry_msgs.msg.Pose()
pose.position.x = -0.069
pose.position.y = -1.218
pose.position.z = 1.050

pose.orientation.x = -0.70711
pose.orientation.y = 0.70711
pose.orientation.z = 0.0
pose.orientation.w = 0.0


arm_goal_pose = arm_commander.get_current_pose()
arm_commander.set_start_state_to_current_state()
arm_commander.set_pose_target(arm_goal_pose)
plan = arm_commander.plan()

if check_plan_is_valid(plan):
    arm_commander.execute(plan)
else:
    rospy.logerr("Not valid plan found for specified arm pose")
    quit()