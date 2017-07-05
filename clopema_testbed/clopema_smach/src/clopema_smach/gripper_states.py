import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach

from smach               import Concurrence
from smach_ros           import ServiceState

__all__ = ['GripperState']

class GripperState(ServiceState):
    """
    GipperState is a SMACH state that uses service open and close gripper.
    """

    def __init__(self, robot_arm, set_open):
        raise NotImplementedError()