#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 31, 2014

# ---- Imports -------------------------------------------------------------- #

import unittest
import rospy
from clopema_robot import ClopemaRobotCommander


# ---- Constants ------------------------------------------------------------ #

PKG = 'clopema_robot'


# ---- Tests ---------------------------------------------------------------- #

class TestAsyncExecute(unittest.TestCase):

    def setUp(self):
        self.commander = ClopemaRobotCommander("arms")
        self.commander.set_start_state_to_current_state()
        self.commander.set_named_target("home_arms")
        self.commander.go(wait=True)

        self.commander.set_joint_value_target({'r1_joint_t':1.0})
        self.traj = self.commander.plan()

    def test_async(self):
        """Test whether the async execute returns """
        self.commander.async_execute(self.traj)
        state1 = self.commander.get_current_state()
        rospy.sleep(0.1)
        state2 = self.commander.get_current_state()
        a = state1.joint_state.position[state1.joint_state.name.index('r1_joint_t')]
        b = state2.joint_state.position[state2.joint_state.name.index('r1_joint_t')]
        self.assertNotEqual(a,b)

    def test_stop(self):
        """
        Test whether is possible to stop the execution of a trajectory in the
        middle of execution.
        """
        self.commander.async_execute(self.traj)
        self.commander.stop()
        rospy.sleep(0.1)
        state1 = self.commander.get_current_state()
        rospy.sleep(0.1)
        state2 = self.commander.get_current_state()
        a = state1.joint_state.position[state1.joint_state.name.index('r1_joint_t')]
        b = state2.joint_state.position[state2.joint_state.name.index('r1_joint_t')]
        self.assertEqual(a,b)
        self.assertNotEqual(a, 1.0)
        self.assertNotEqual(a, 0.0)

# ---- Main ----------------------------------------------------------------- #

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'async_execute', TestAsyncExecute)
