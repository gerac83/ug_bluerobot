# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 20, 2014

"""Collection of helpers for interactive mode such as IPython.
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def init_node(name='anonymous'):
    rospy.init_node(name, anonymous=True)

def pose_from_xyzrpy(x,y,z,rr,pp,yy,frame_id='base_link'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    [qx,qy,qz,qw] = quaternion_from_euler(rr,pp,yy)

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose



