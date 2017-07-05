#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Apr 1, 2014

"""Generate calibration trajectory

Usage:
    gen_calib_trajectory.py [options] <output_file>
    gen_calib_trajectory.py -h | --help

Options:
    -x NUM, --xtion NUM         Xtion number (1 or 2) [default: 1]
    -f FRAME, --frame FRAME     Frame of the calibration target
    -d, --debug                 Display debug information
    -h, --help                  Display this message
"""

# ---- Imports --------------------------------------------------------------- #

import rospy
import sys
import math
import numpy as np

from clopema_libs import docopt
from clopema_libs.pose import orient_pose
from clopema_libs.tf_utils import get_tf2_buffer
from clopema_robot import ClopemaRobotCommander
from clopema_libs.io import write_msgs

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from moveit_commander.exception import MoveItCommanderException

from copy import deepcopy
from math import pi

# ---- Constants ------------------------------------------------------------- #

NODE_NAME = "gen_calib_trajectory"
NODE_ANONYMOUS = True
ROBOT_GROUP = "xtions"
ROBOT_ID = 1
BASE_FRAME_ID = "base_link"

L1_X_RANGE = (-0.3, 0.4)
L1_Y_RANGE = (-0.3, 0.4)
L1_Z_RANGE = (0.65, 1.0)
L1_X_STEP = 0.2
L1_Y_STEP = 0.2
L1_Z_STEP = 0.2

L2_X_RANGE = (-0.6, 0.7)
L2_Y_RANGE = (-0.6, 0.7)
L2_Z_RANGE = (1.1, 1.5)
L2_X_STEP = 0.4
L2_Y_STEP = 0.4
L2_Z_STEP = 0.2


# ---- Main ------------------------------------------------------------------ #

def main():
    # Process command line arguments
    argv = rospy.myargv(argv=sys.argv)
    opt  = docopt(__doc__, argv = argv[1:])

    opt_debug = opt['--debug']
    opt_xtion = int(opt['--xtion'])
    opt_target_frame = opt['--frame']
    opt_output = opt['<output_file>']

    if opt_xtion == 1:
        opt_other_s_link = "r2_joint_s"
        opt_other_s_val = pi / 2
        opt_ee_link = "xtion1_link_ee"
        opt_group = "r1_xtion"
    elif opt_xtion == 2:
        opt_other_s_link = "r1_joint_s"
        opt_other_s_val = -pi / 2
        opt_ee_link = "xtion1_link_ee"
        opt_group = "r2_xtion"
    else:
        rospy.logerr("Xtion number is not valid: %d", opt_xtion)
        return

    if opt_debug:
        print opt

    # Initialise node
    rospy.init_node(NODE_NAME, anonymous=NODE_ANONYMOUS)

    # Initialise tf2 buffer
    tfb = get_tf2_buffer()

    # Initialise robot commander
    robot = ClopemaRobotCommander(opt_group)

    # Get start position, all zeros, but exte rotated towards calibration table,
    # and S of the other arm rotated -90 degrees.
    current_state = robot.get_current_state()
    start_state = deepcopy(current_state)
    start_state.joint_state.position = [0] * len(start_state.joint_state.name)
    start_state.joint_state.position[start_state.joint_state.name.index(opt_other_s_link)] = opt_other_s_val
    start_state.joint_state.position[start_state.joint_state.name.index("ext_axis")] = base_rotation("base_link", opt_target_frame, tfb)

    if opt_debug:
        print start_state

    # Generate poses
    header = Header()
    header.frame_id = opt_target_frame

    pose_list = []
    for x,y,z in xyzrange(L1_X_RANGE, L1_Y_RANGE, L1_Z_RANGE, L1_X_STEP, L1_Y_STEP, L1_Z_STEP):
        p = make_pose(x, y, z, 0, 1, 0, 0)
        pose_list.append(PoseStamped(header, p))
        pose_list.append(PoseStamped(header, orient_pose(p)))

    for x,y,z in xyzrange(L2_X_RANGE, L2_Y_RANGE, L2_Z_RANGE, L2_X_STEP, L2_Y_STEP, L2_Z_STEP):
        p = make_pose(x, y, z, 0, 1, 0, 0)
        pose_list.append(PoseStamped(header, p))
        pose_list.append(PoseStamped(header, orient_pose(p)))

    # TODO: Sort poses
    # Plan throught poses
    trajs = plan_through_poses(robot, start_state, pose_list, opt_ee_link)

    # Print info
    print "# generated poses: %d" % len(pose_list)
    print "# trajectories:    %d" % len(trajs)

    # Wite trajectories
    write_msgs(trajs, opt_output)


# ---- Helper functions ------------------------------------------------------ #

def plan_through_poses(robot, start_state, poses, ee_link):
    trajs = []
    for pose in poses:
        robot.set_start_state(start_state)
        try:
            robot.set_joint_value_target(pose, ee_link)
        except MoveItCommanderException:
            rospy.logwarn("Unable to set target from IK.")
            continue

        traj = robot.plan()
        if traj is not None and len(traj.joint_trajectory.points) > 0:
            trajs.append(make_full_trajectory(traj, start_state, robot.get_dx100_joints()))
            start_state = get_final_robot_state(start_state, traj)
    return trajs


def make_full_trajectory(traj, robot_state, active_joints):

    # Get all joints that should be appedned
    joint_names = deepcopy(traj.joint_trajectory.joint_names)
    joint_values = []
    for name, value in zip(robot_state.joint_state.name, robot_state.joint_state.position):
        if name not in traj.joint_trajectory.joint_names and name in active_joints:
            joint_names.append(name)
            joint_values.append(value)

    # Set new joint names
    traj.joint_trajectory.joint_names = joint_names

    # Appedn all points
    for i in range(0,len(traj.joint_trajectory.points)):
        traj.joint_trajectory.points[i].positions = list(traj.joint_trajectory.points[i].positions) + joint_values
        traj.joint_trajectory.points[i].velocities = list(traj.joint_trajectory.points[i].velocities) + [0] * len(joint_values)
        traj.joint_trajectory.points[i].accelerations = list(traj.joint_trajectory.points[i].accelerations) + [0] * len(joint_values)

    return traj


def get_final_robot_state(robot_state, traj):
    final_state = deepcopy(robot_state)

    for k,v in zip(traj.joint_trajectory.joint_names, traj.joint_trajectory.points[-1].positions):
        final_state.joint_state.position[final_state.joint_state.name.index(k)] = v
    return final_state


def xyzrange(x_range, y_range, z_range, x_step, y_step, z_step):
    for x in np.arange(x_range[0], x_range[1], x_step):
        for y in np.arange(y_range[0], y_range[1], y_step):
            for z in np.arange(z_range[0], z_range[1], z_step):
                yield (x,y,z)


def make_pose(x, y, z, qx=0, qy=0, qz=0, qw=1):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


def base_rotation(source_frame, target_frame, tfb):
    # Get the transforamtion from baselink to frame
    transform = tfb.lookup_transform(source_frame, target_frame, rospy.Time(0))

    x = transform.transform.translation.x
    y = transform.transform.translation.y
    z = transform.transform.translation.z

    trans = [x,y,z]

    # Compute dot product
    d = sum([a * b for (a,b) in zip([0,-1],trans)])
    d = d / math.sqrt(sum([a ** 2 for a in trans[0:2]]))

    return math.acos(d)


if __name__ == "__main__":
    main()
