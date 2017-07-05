# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Jan 6, 2014

import rospy
import tf2_ros

tfb = None
tfl = None


def get_tf2_buffer():
    """Get a TF2 Buffer, ensure that there is only one buffer in a application.

    Returns:
        tfb         : {tf2_ros.Buffer}
    """

    global tfb, tfl

    # Create new buffer and listener if not allready created
    if tfb is None:
        tfb = tf2_ros.Buffer(cache_time = rospy.Duration(5.0))
        tfl = tf2_ros.TransformListener(tfb)
        # Wait for cache to fill up
        rospy.sleep(1)

    return tfb


def transform_msg_to_T(transform_msg):
    """Convert transform_msg to transformation matrix.

    INPUT
        transform_msg       [geometry_msgs/Transform]

    OUTPUT
        T                   [4x4 array]
    """

    qx = transform_msg.rotation.x
    qy = transform_msg.rotation.y
    qz = transform_msg.rotation.z
    qw = transform_msg.rotation.w

    x = transform_msg.translation.x
    y = transform_msg.translation.y
    z = transform_msg.translation.z

    T = [[1.0 - 2.0 * qy * qy - 2 * qz * qz, 2.0 * qx * qy - 2.0 * qz * qw, 2.0 * qx * qz + 2.0 * qy * qw, x],
         [2.0 * qx * qy + 2.0 * qz * qw, 1.0 - 2.0 * qx * qx - 2.0 * qz * qz, 2.0 * qy * qz - 2.0 * qx * qw, y],
         [2.0 * qx * qz - 2.0 * qy * qw, 2.0 * qy * qz + 2.0 * qx * qw, 1.0 - 2.0 * qx * qx - 2.0 * qy * qy, z],
         [0,0,0,1]]

    return T
