"""
function_states.py

This module contain states that are used to transform user data.

Libor Wagner on February 27, 2013
"""

import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach

from smach import State

class FunctionApplicationState(State):
    """General function application on user data state."""

    def __init__(self, fun, outcomes = ['succeeded'], input_keys = [], output_keys = []):
        State.__init__(self, outcomes, input_keys = input_keys, output_keys = output_keys)

        self._fun = fun

    def execute(self, ud):
        outcome = self._fun(ud)
        return outcome
