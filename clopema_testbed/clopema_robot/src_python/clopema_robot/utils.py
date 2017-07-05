# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 22, 2014

# ---- Imports --------------------------------------------------------------- #

import rospy
import numpy as np

from moveit_commander import RobotCommander
from clopema_libs.pose import pose2affine, affine2pose



# ---- Functions ------------------------------------------------------------- #

def fix_link_for_ik(pose, link):
    target_link = link
    if link.startswith("r1"):
        target_link = "r1_tip_link"
    elif link.startswith("r2"):
        target_link = "r2_tip_link"
    elif link.startswith("xtion1"):
        target_link = "xtion1_link_ee"
    elif link.startswith("xtion2"):
        target_link = "xtion2_link_ee"

    if not link == target_link:
        return target_link, convert_link(pose, link, target_link)
    else:
        return target_link, pose


def convert_link(pose, source_link, target_link):
    """Convert link to another link.

    Note: The transformation between source and target link should be
          fixed, but it is not checked.

    Arguments:
        pose         -- Pose to be transformed.
        source_link  -- Link of the pose.
        target_link  -- Link where the pose should be tranformed.

    Returns:
        Pose for the target_link so that the source_link is in the orignal
        pose.
    """
    robot = RobotCommander()
    Ts = pose2affine(robot.get_link(source_link).pose().pose)
    Tt = pose2affine(robot.get_link(target_link).pose().pose)
    Tp = pose2affine(pose)

    T  = np.dot(np.linalg.inv(Tt), Ts)
    p  = affine2pose(np.dot(Tp, np.linalg.inv(T)))
    return p

def print_joint_bounds(robot=None):
    if not robot:
        robot = RobotCommander()
    print 'NAME'.center(16), 'MIN'.center(10), 'MAX'.center(10)

    for name in robot.get_joint_names():
        joint = robot.get_joint(name)
        bounds = joint.bounds()

        if bounds:
            print '%15s % 10.3f % 10.3f' % (name, bounds[0], bounds[1])


def get_moving_joints(joint_traj_msg):
    """Given joint trajectory message return list of moving joints.

    INPUT
        joint_traj_msg      [JointTrajectory]

    OUTPUT
        moving              [List of Strings]
    """
    traj = np.array([p.positions for p in joint_traj_msg.points])
    max_diff = np.max(np.diff(traj, axis=0), axis=0)

    moving = []
    for i in np.nonzero(max_diff)[0]:
        moving.append(joint_traj_msg.names[i])
    return moving


def get_group_for_joint(joint_names, robot=None, smallest=False, groups=None):
    """Given list of joints find the most appropriate group.

    INPUT
        link_ids            [List of Strings]
        robot               [RobotCommander]
        smallest            [Bool]

    OUTPUT
        group               [String]
    """
    if not robot:
        robot = RobotCommander()

    if not groups:
        groups = robot.get_group_names()

    group = None
    n = np.Inf if smallest else 0
    for g in groups:
        names = robot.get_joint_names(g)
        if all_in(joint_names, names):
            if smallest and len(names) < n:
                n = len(names)
                group = g
            elif len(names) > n:
                n = len(names)
                group = g
    return group


def all_in(A, B):
    """Given list A and list B return true if all items in A are also in B."""
    for a in A:
        if not a in B:
            return False
    return True

