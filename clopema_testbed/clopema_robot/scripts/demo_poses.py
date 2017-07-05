#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  November 7, 2014

# ---- Doc ------------------------------------------------------------------- #

"""Demo poses

Usage:
    demo_poses.py [options]

Options:
    -h, --help          Show this help message

"""

# ---- Import ---------------------------------------------------------------- #

import rospy
from docopt import docopt
from clopema_robot import ClopemaRobotCommander, RobotState
from clopema_libs.pose import make_pose
# ---- Constants ------------------------------------------------------------- #

node_name = "demo_poses"
node_anonymous = True
ik_link = 'r1_ee'
frame_id = 'base_link'

# ---- Main ------------------------------------------------------------------ #

def main():
    rospy.init_node(node_name, anonymous=node_anonymous)

    poses = []
    poses.append(make_pose(-0.3,-0.8,1.4,1,1,0,0,frame_id=frame_id))
    poses.append(make_pose(-0.3,-0.8,1.4,0,1,0,0,frame_id=frame_id))
    poses.append(make_pose(-0.3,-0.8,1.4,1,0,0,0,frame_id=frame_id))
    poses.append(make_pose(-0.3,-0.8,1.4,-1,0,0,0,frame_id=frame_id))
    poses.append(make_pose(-0.3,-0.8,1.4,1,0,1,0,frame_id=frame_id))
    poses.append(make_pose(-0.3,-0.8,1.4,1,1,1,1,frame_id=frame_id))
    poses.append(make_pose(-0.3,-0.8,1.4,1,0,0,1,frame_id=frame_id))
    poses.append(make_pose(-0.3,-0.8,1.4,1,1,0,0,frame_id=frame_id))

    robot = ClopemaRobotCommander('arms')

    for pose in poses:
        state = RobotState.from_robot_state_msg(robot.get_current_state())
        state.update_from_pose(pose, ik_link_id=ik_link)

        robot.set_joint_value_target(state.joint_state)
        traj = robot.plan()

        robot.execute(traj)


if __name__ == '__main__':
    main()
