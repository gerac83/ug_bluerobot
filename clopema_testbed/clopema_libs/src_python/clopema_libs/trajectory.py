# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Apr 4, 2014

"""Library of helper functions for trajectory processing.

"""


def get_moving_joints(traj, threshold=0.01):
    """Get joint names that are moving.

    INPUT
        traj            [moveit_msgs/RobotTrajectory]
                        Trajectory to be tested

        threshold       [Float, default=0.01]
                        When the difference of two sucessive joint position
                        exceeds this limit, the joint is recognised as moving.

    OUTPUT
        moving_joints   [List of Strings]
                        List of moving joints
    """

    # Length of the trajectory
    N = len(traj.joint_trajectory.points)

    # Check each joint separatedly
    moving_joints = set()
    joint_names = traj.joint_trajectory.joint_names
    p_prev = traj.joint_trajectory.points[0]

    for i in range(1, N):
        p = traj.joint_trajectory.points[i]
        for i, joint_name in enumerate(joint_names):
            if abs(p_prev.positions[i] - p.positions[i]) > threshold:
                moving_joints.add(joint_name)
    return list(moving_joints)
