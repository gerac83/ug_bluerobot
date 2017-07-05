#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 19, 2013

import numpy as np


def get_orientation(pA=None, pB=None, A=None, B=None, v0=np.array([0,0,-1])):
    """Get orientation from A to B

    Arguments:
        pA, pB      : {Pose} whole pose
        A, B        : {numpy array, 1x3} position vector

        One from pA and A and, pB and B must be given.

    Returns:
        quaternion  : {numpy array}
        pose        : {Pose}
    """
    # Check input
    assert(pA is not None or A is not None)
    assert(pB is not None or B is not None)

    if A is None:
        A = np.array([pA.position.x, pA.position.y, pA.position.z])
    if B is None:
        B = np.array([pB.position.x, pB.position.y, pB.position.z])

    # Compute vector between
    v = A - B
    v_ = v / np.linalg.norm(v)
    v0_ = v0 / np.linalg.norm(v)
    angle = np.arccos(np.dot(v_, v0_))
    axis = np.cross(v_, v0_)

    if np.all(axis == 0):
        axis = np.array([-1, 0, 0])
