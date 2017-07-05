"""
perception_grasp_states.py

State related to grasp point detection.

Libor Wagner on February 20, 2013
"""

import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach

from topic_states      import TopicState
from utility_sm        import gensm_withlaunch
from geometry_msgs.msg import PoseArray

class WaitGraspCandidatesState(TopicState):
    """Wait for grasp point candidates on given topic"""
    def __init__(self, topic):
        TopicState.__init__(self, topic, PoseArray, self.input_cb, output_keys = ['pose_array'])

    def input_cb(self, msg, ud):
        if len(msg.poses) > 0:
            msg.poses = sorted(msg.poses, cmp = lambda x,y: cmp(x.position.z, y.position.z), reverse = True)
            ud.pose_array = msg
            return False
        else:
            return True


