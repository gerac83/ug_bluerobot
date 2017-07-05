# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 24, 2014

# ---- Imports --------------------------------------------------------------- #

import numpy as np

# ---- Constants ------------------------------------------------------------- #


def modify_speed(traj, factor):
    N = len(traj.joint_trajectory.points)
    M = len(traj.joint_trajectory.joint_names)
    for i in range(0,N):
        traj.joint_trajectory.points[i].time_from_start = traj.joint_trajectory.points[i].time_from_start/factor
        for j in range(0,M):
            traj.joint_trajectory.points[i].velocities = list(traj.joint_trajectory.points[i].velocities)
            traj.joint_trajectory.points[i].velocities[j] = traj.joint_trajectory.points[i].velocities[j] * factor
    return traj
