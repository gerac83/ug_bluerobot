#!/usr/bin/env python

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
arm_commander.set_planner_id("RRTConnectkConfigDefault")

joint_state = {'r1_joint_u': -0.33792722047743184,
               'r1_joint_t': -0.581361684344759,
               'r1_joint_s': -0.5812563802049505,
               'r1_joint_r': 3.475027978193726e-06,
               'r1_joint_b': 0.602388912655484,
               'r1_joint_l': 0.26450211798139023}

arm_commander.set_start_state_to_current_state()
arm_commander.set_joint_value_target(joint_state)
plan = arm_commander.plan()

if check_plan_is_valid(plan):
    arm_commander.execute(plan)
    rospy.loginfo("Plan executed")
else:
    rospy.logerr("Not valid plan found for specified arm pose")
    quit()