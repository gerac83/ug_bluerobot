"""
buffer_state.py


Libor Wagner on September  6, 2013
"""
PACKAGE = 'clopema_smach'

import roslib
roslib.load_manifest(PACKAGE)

import rospy
from smach import State


class BufferState(State):

    def __init__(self, repeat=False):
        self.repeat = repeat
        self.counter = 0

        State.__init__(self, outcomes = ['succeeded', 'finished'])
        self.register_input_keys(['list'])
        self.register_output_keys(['item', 'counter'])

    def execute(self, ud):
        # Reset counter if loop is set
        if self.counter >= len(ud.list) and self.repeat:
            self.counter = 0

        if self.counter >= len(ud.list):
            return 'finished'
        else:
            ud.item = ud.list[self.counter]
            self.counter += 1
            ud.counter = self.counter
        return 'succeeded'
