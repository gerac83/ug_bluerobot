#!/usr/bin/env python

"""
write.py

Simple state to write msg on some topic to a file.

Libor Wagner on April 19, 2013
"""

# Constants
PACKAGE_NAME = "clopema_smach"

# Imports
import roslib; roslib.load_manifest(PACKAGE_NAME)
import rospy
import tf
import threading

from clopema_utilities.write import write_msg, write_tf
from smach import State


__all__ = []

class WrongInitialisationErorr(Exception):

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class WriteMsgState(State):
    """State that writes an arbitrary mesage to file. The only limitation is
    that the message type must be supported by the clopema_utilities.write
    package."""

    def __init__(self, topic = None, ttype = None, name_template = None, timeout = None, drop = 0, n = 1):
        """Initialise WriteMsgState.

        :topic: Topic name (optional, msg is expected as an input key if not set)
        :ttype: Type of the topic (required when topic is set)
        :name_template: Template name (optional, filename is expected as an input key if not set)
        """
        State.__init__(self, outcomes = ["succeeded", "aborted", "preempted"])

        # Register msg as input key if topic is not set
        if topic == None:
            self.register_input_keys(['msg'])

        # Register filename as input key if name_template is not set
        if name_template == None:
            self.register_input_keys(['path'])

        # Check whether is everything set properly
        if (not topic == None) and (ttype == None):
            raise WrongInitialisationErorr("If topic is set the ttype must set also!")

        self._topic = topic
        self._ttype = ttype
        self._timeout = timeout
        self._name_template = name_template
        self._drop = drop
        self._n = n
        self._trigger_cond = threading.Condition()

    def _input_cb(self, msg, path):

        if self._dropped >= self._drop:
            write_msg(msg, path)
            self._sub.unregister()
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()
        else:
            self._dropped += 1


    def execute(self, ud):
        """Execute the WriteMsgState."""

        # Get filename either from user-data or using template
        if self._name_template == None:
            filename = ud.path
        else:
            filename = self._name_template % (self._n)
            self._n += 1

        # Get message either from user-data or directly from topic
        if self._topic == None:
            msg = ud.msg
            write_msg(msg, filename)
        else:
            self._dropped = 0
            self._sub = rospy.Subscriber(self._topic, self._ttype, self._input_cb, callback_args=filename)
            self._trigger_cond.acquire()
            self._trigger_cond.wait(self._timeout)
            self._trigger_cond.release()

        return "succeeded"

class WriteTFState(State):

    def __init__(self, source_frame = None, target_frame = None, tf_listener = None, name_template = 'tf%03d.txt', n = 1):
        State.__init__(self, outcomes = ["succeeded", "aborted", "preempted"])

        if target_frame == None:
            self.register_input_keys(['target_frame'])

        if source_frame == None:
            self.register_input_keys(['source_frame'])

        if name_template == None:
            self.register_input_keys(['path'])

        self._target_frame = target_frame
        self._source_frame = source_frame

        if tf_listener == None:
            self._tf_listener = tf.TransformListener()
        else:
            self._tf_listener = tf_listener

        self._name_template = name_template
        self._n = n

    def execute(self, ud):
        """Execute the WriteTFState"""

        if self._target_frame == None:
            target_frame = ud.target_frame
        else:
            target_frame = self._target_frame

        if self._source_frame == None:
            source_frame = ud.source_frame
        else:
            source_frame = self._source_frame

        if self._name_template == None:
            path = ud.path
        else:
            path = self._name_template % self._n
            self._n += 1

        # Get translation and rotation
        now = rospy.Time.now()
        self._tf_listener.waitForTransform(source_frame, target_frame, now, rospy.Duration(4.0))
        (trans,rot) = self._tf_listener.lookupTransform(source_frame, target_frame, now)

        # Finally save to file
        write_tf(trans, rot, path)

        return "succeeded"
