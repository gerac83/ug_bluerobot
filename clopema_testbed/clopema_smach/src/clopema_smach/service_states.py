"""
service_states.py

Libor Wagner on February  1, 2013
"""

import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach

from smach import State

__all__ = ['WaitForServiceState']

class WaitForServiceState(State):
    """
    WaitForServiceState is state that waits for services to become available.
    When timeout parameter is set, and exceeded the state outcome is aborted,
    succeeded otherwise.
    """

    def __init__(self, services, timeout = None):
        State.__init__(self, ['succeeded', 'aborted'])
        self._services = services
        self._timeout = timeout

    def execute(self, ud):
        failed = []
        for service in self.services:
            try:
                rospy.wait_for_service(self._service, timeout = self._timeout)
            except ROSException, e:
                failed.append(service)

        if len(failed) > 0:
            return 'aborted'
        else:
            return 'succeeded'


