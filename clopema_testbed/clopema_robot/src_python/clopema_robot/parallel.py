# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 31, 2013

import numpy as np
import rospy
from clopema_robot.services import get_position_fk
from clopema_libs.pose import affine2pose, pose2affine


def parallel_move(start_state, poses):
    """
    """
    master_ik_link = 'r1_ee'
    slave_ik_link = 'r2_ee'

    # Get pose mirror_ik_link_id in ik_link_id
    res = get_position_fk(start_state, [master_ik_link, slave_ik_link])

    # Check error code
    if res.error_code.val is not res.error_code.SUCCESS:
        rospy.logerr("Unable to compute FK, error code: %d", res.error_code.val)
        return None

    master_pose = res.pose_stamped[res.fk_link_names.index(master_ik_link)].pose
    slave_pose = res.pose_stamped[res.fk_link_names.index(slave_ik_link)].pose

    slave_poses = parallel_poses(master_pose, slave_pose, poses)

    return poses, slave_poses


def parallel_poses(master_pose, slave_pose, poses):
    """
    """
    # Get the transformation between poses
    master_T = pose2affine(master_pose)
    slave_T = pose2affine(slave_pose)
    T = np.dot(np.linalg.inv(master_T), slave_T)

    # Clone poses
    slave_poses = []
    for pose in poses:
        t = pose2affine(pose)
        slave_poses.append(affine2pose(np.dot(t,T)))

    return slave_poses

