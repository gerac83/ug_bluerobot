#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 5, 2014

# ---- Doc ------------------------------------------------------------------- #

"""
Usage:
    demo_parallel_move.py [options]

Options:
    -h, --help          Show this help

"""

# ---- Imports --------------------------------------------------------------- #

import numpy as np
import rospy

from docopt import docopt
from clopema_robot import ClopemaRobotCommander, RobotState
from clopema_robot.parallel import parallel_move
from clopema_robot import services
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

# ---- Constants ------------------------------------------------------------- #

node_name = 'demo_parallel_move'
frame_id  = 'base_link'
ik_link_1 = 'r1_ee'
ik_link_2 = 'r2_ee'

# ---- Main ------------------------------------------------------------------ #

def main():
    opt = docopt(__doc__)

    rospy.init_node(node_name)
    p1 = make_pose(-0.3,-0.8,1.4,0,1,0,0,frame_id=frame_id)
    p2 = make_pose(0.3,-0.8,1.4,0,1,0,0,frame_id=frame_id)

    poses = []
    poses.append(make_pose(-0.4,-0.8,1.4,0,1,0,0))
    poses.append(make_pose(-0.4,-0.9,1.4,0,1,0,0))
    poses.append(make_pose(-0.4,-0.9,1.3,0,1,0,0))
    poses.append(make_pose(-0.3,-0.8,1.4,0,1,0,0))
    poses.append(make_pose(-0.3,-0.8,1.4,0.1,0.9,0,0))
    poses.append(make_pose(-0.3,-0.8,1.4,0.29,0.95,-0.072,-0.022))
    poses.append(make_pose(-0.3,-0.8,1.2,0.29,0.95,-0.072,-0.022))
    poses.append(make_pose(-0.2,-0.6,1.2,0.29,0.95,-0.072,-0.022))
    poses.append(make_pose(-0.3,-0.8,1.4,0,1,0,0))
    poses.append(make_pose(-0.3,-0.8,1.4,0,0.98,0,0.2))
    poses.append(make_pose(-0.3,-0.8,1.4,0,0.99,0,-0.1))
    poses.append(make_pose(-0.3,-0.8,1.4,0,1,0,0))

    # Go into start position
    robot = ClopemaRobotCommander('arms')
    state = RobotState.from_robot_state_msg(robot.get_current_state())
    state.update_from_pose(p1, ik_link_1)
    state.update_from_pose(p2, ik_link_2)

    robot.set_joint_value_target(state.joint_state)
    traj = robot.plan()
    robot.execute(traj)

    state = RobotState.from_robot_state_msg(robot.get_current_state())
    full_poses = parallel_move(state, poses)

    res = services.compute_cartesian_path_dual(state, (ik_link_1, ik_link_2), full_poses, jump_threshold=1.5)

    if res.error_code.val == res.error_code.SUCCESS and res.fraction == 1.0:
        traj = res.solution
        robot.execute(traj)
    else:
        rospy.logerr("Unable to plan Cartesian path, error_code: %d, fraction: %f", res.error_code.val, res.fraction)

def make_pose(x,y,z,qx,qy,qz,qw, frame_id=None):
    pose = Pose(Point(x,y,z),Quaternion(qx,qy,qz,qw))
    if frame_id:
        pose = PoseStamped(pose=pose)
        pose.header.frame_id = frame_id
    return pose

if __name__ == '__main__':
    main()
