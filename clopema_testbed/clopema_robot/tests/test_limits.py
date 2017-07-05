#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 23, 2014

# ---- Imports -------------------------------------------------------------- #

import rospy
import unittest
from moveit_commander import RobotCommander

# ---- Constants ------------------------------------------------------------ #

PKG = 'clopema_robot'


# ---- Tests ---------------------------------------------------------------- #

class TestLimits(unittest.TestCase):

    def setUp(self):
        self.robot = RobotCommander()

        self.bounds = list()
        for name in self.robot.get_joint_names():
            joint = self.robot.get_joint(name)
            bounds = joint.bounds()
            if bounds:
                self.bounds.append((name, bounds))

    def test_min_max(self):
        """Test whether the min is actually lower than max."""
        failed = list()
        for name, bounds in self.bounds:
            if bounds[0] > bounds[1]:
                failed.append(name)
        self.assertFalse(failed, "Bounds are not valid for: %s" % " ".join(failed))

# ---- Main ----------------------------------------------------------------- #

if __name__ == '__main__':
    import rostest
    rospy.logwarn("test_min_max")
    rostest.rosrun(PKG, 'test_limits', TestLimits)
