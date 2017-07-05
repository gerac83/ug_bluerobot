"""
utility_states.py



Libor Wagner on February 18, 2013
"""

import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach
import subprocess
import signal
import time

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from smach import State
from smach_ros import ServiceState
#from clopema_utilities.srv import SaveFile, SaveFileRequest

class PoseBufferState(State):
    """Pose buffer state, each time execute is called next pose taken from pose
    buffer and stored in the user data."""

    def __init__(self, publish_markers = False):
        State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['poses'], output_keys = ['pose', 'n'])

        self._marker_pub = None
        self._n_markers_published = 0
        self._poses = None
        self._n = 0

        # Publish poses if requested
        if publish_markers:
            self._marker_pub = rospy.Publisher('pose_buffer', MarkerArray)


    def execute(self, ud):

        if not  self._poses == ud.poses.poses:
            self._poses = ud.poses.poses
            self._n = 0

        # Abort if no more poses
        if len(self._poses) <= self._n:
            return 'aborted'

        # Publish markers if required
        if self._marker_pub:
            self.publish_markers(ud.poses)

        # Set pose to user data
        msg = PoseStamped()
        msg.header = ud.poses.header
        msg.pose = self._poses[self._n]
        ud.n = self._n
        ud.pose = msg
        self._n += 1


        # Retunr succeeded
        return 'succeeded'

    def publish_markers(self, pose_array):
        """ublish current set of markers and highlight the one which will be
        used."""

        marker_array = MarkerArray()
        for i, pose in zip(range(0,len(pose_array.poses)), pose_array.poses):
            marker = Marker()
            marker.header = pose_array.header
            marker.ns = "pose_buffer"
            marker.action = Marker.ADD
            marker.type = Marker.CUBE
            marker.id = i
            marker.pose = pose_array.poses[i]

            if (i == self._n):
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            elif (i < self._n):
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.color.a = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0

            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01

            marker.lifetime = rospy.Duration()
            marker_array.markers.append(marker)

        # Remove excess markers
        if self._n_markers_published > (i + 1):
            for j in range(i+1, self._n_markers_published):
                marker = Marker()
                marker.ns = "pose_buffer"
                marker.id = j
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)

        self._n_markers_published = (i + 1)

        self._marker_pub.publish(marker_array)

class CounterState(State):

    def __init__(self, ub = None):
        State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['n'], output_keys = ['n'])
        self._ub = ub


    def execute(self, ud):
        ud.n += 1

        if ud.n >= self._ub:
            return "aborted"
        else:
            return 'succeeded'

class PauseState(State):

    def __init__(self, sec):
        State.__init__(self, outcomes = ['succeeded'])
        self._sec = sec

    def execute(self, ud):
        time.sleep(self._sec)
        return 'succeeded'

class SaveFileState(ServiceState):

    def __init__(self, service, name_template = "capture%03d.png"):
        ServiceState.__init__(self, service, SaveFile, request_cb = self._request_cb, response_cb = self._response_cb)
        self._name_template = name_template
        self._n = 0

    def _request_cb(self, ud, request):
        self._n += 1

        req = SaveFileRequest()
        req.path = self._name_template % self._n

        return req

    def _response_cb(self, ud, response):
        print response

