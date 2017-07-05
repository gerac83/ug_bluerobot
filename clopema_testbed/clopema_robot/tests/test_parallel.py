#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 4, 2014

# ---- Imports -------------------------------------------------------------- #

import numpy as np
import unittest
from clopema_robot import parallel
from clopema_libs.pose import affine2pose, pose2affine
from geometry_msgs.msg import Pose, Point, Quaternion

# ---- Constants ------------------------------------------------------------ #

PKG = 'clopema_robot'


# ---- Tests ---------------------------------------------------------------- #

class TestParallel(unittest.TestCase):

    def setUp(self):
        pass

    def test_parallel_poses(self):

        p1 = Pose(Point(0,0,0), Quaternion(0,0,1,0))
        p2 = Pose(Point(0,0,1), Quaternion(0,0,1,0))

        ps = []
        for i in np.arange(0,10,0.5):
            ps.append(Pose(Point(0,i,0), Quaternion(0,0,1,0)))

        ps_bar = parallel.parallel_poses(p1,p2,ps)

        master_T = pose2affine(p1)
        slave_T = pose2affine(p2)
        T = np.dot(slave_T, np.linalg.inv(master_T))

        for p1_, p2_ in zip(ps, ps_bar):
            T1 = pose2affine(p1_)
            T2 = pose2affine(p2_)
            TX = np.dot(T2, np.linalg.inv(T1))
            self.assertTrue(np.allclose(T,TX), "Not Same: \n" + str(T) + "\n" + str(TX))

# ---- Main ----------------------------------------------------------------- #

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_parallel', TestParallel)
