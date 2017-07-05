#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  November 7, 2014

# ---- Doc ------------------------------------------------------------------- #

"""Demo speed

Usage:
    demo_speed.py [options]

Options:
    -h, --help          Show this help message

"""

# ---- Import ---------------------------------------------------------------- #

import rospy
import itertools
from docopt import docopt
from clopema_robot import ClopemaRobotCommander, RobotState
from clopema_libs.pose import make_pose
from clopema_robot.speed import modify_speed
# ---- Constants ------------------------------------------------------------- #

node_name = "demo_speed"
node_anonymous = True
ik_link = 'r1_ee'
frame_id = 'base_link'

# ---- Main ------------------------------------------------------------------ #

def main():
    rospy.init_node(node_name, anonymous=node_anonymous)

    p1 = make_pose(-0.1,-0.8,1.4,1,1,0,0,frame_id=frame_id)
    p2 = make_pose(-0.5,-0.8,1.4,1,1,0,0,frame_id=frame_id)
    speeds = [0.1,0.2,0.5,1,1.2,1.5]

    robot = ClopemaRobotCommander('arms')
    robot.overwrite_time_parameterization = False

    state = RobotState.from_robot_state_msg(robot.get_current_state())
    state.update_from_pose(p2, ik_link_id=ik_link)

    robot.set_joint_value_target(state.joint_state)
    traj = robot.plan()
    robot.execute(traj)

    for speed, pose in zip(speeds, itertools.cycle([p1,p2])):
        rospy.sleep(0.5)
        robot.set_start_state_to_current_state()
        state = RobotState.from_robot_state_msg(robot.get_current_state())
        state.update_from_pose(pose, ik_link_id=ik_link)

        robot.set_joint_value_target(state.joint_state)
        traj = robot.plan()
        time_before = traj.joint_trajectory.points[-1].time_from_start.to_sec()
        traj = modify_speed(traj, speed)
        time_after = traj.joint_trajectory.points[-1].time_from_start.to_sec()
        traj_len = len(traj.joint_trajectory.points)

        print "Speed:", speed, "time before:", time_before, "after:", time_after, "length:", traj_len

        robot.execute(traj)

def show_pose(pose):
    x,y,z = (pose.position.x, pose.position.y, pose.position.z)
    qx,qy,qz,qw = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return "%f,%f,%f,%f,%f,%f,%f" % (x,y,z,qx,qy,qz,qw)


if __name__ == '__main__':
    main()
