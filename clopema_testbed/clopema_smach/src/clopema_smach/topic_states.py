"""
topic_state.py

Topic state waits for message on certain topic.

Libor Wagner on February 20, 2013
"""

import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach
import smach_ros

import threading
import traceback

from smach               import State

class TopicState(smach.State):
    """State that waits for a message on topic."""

    def __init__(self, topic, msg_type, user_cb, output_keys = [], input_keys =[], timeout = None):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], output_keys = output_keys, input_keys = input_keys)

        self._topic        = topic
        self._msg_type     = msg_type
        self._user_cb      = user_cb
        self._timeout      = timeout
        self._aborted      = True

        self._trigger_cond = threading.Condition()

    def execute(self, userdata):
        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._cb, callback_args=userdata)

        self._trigger_cond.acquire()
        self._trigger_cond.wait(self._timeout)
        self._trigger_cond.release()

        self._sub.unregister()

        if self._aborted:
            return 'aborted'

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'succeeded'

    def _cb(self, msg, ud):
        try:
            self._aborted = False
            cont = self._user_cb(msg, ud)
            if not cont:
                self._trigger_cond.acquire()
                self._trigger_cond.notify()
                self._trigger_cond.release()
        except Exception, e:
            rospy.logerr("Error thrown while executing condition callback %s: %s" % (str(self._user_cb), str(e)))
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()
            self._aborted = True

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()

