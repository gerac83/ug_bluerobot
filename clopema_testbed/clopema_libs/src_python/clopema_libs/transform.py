# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 19, 2013

import numpy as np
import rospy

from clopema_libs.geometry import q2mat
from geometry_msgs.msg import TransformStamped


def transform2tq(transform):
    """Converts transform message to translation vector and quaternion.

    Arguments:
        transform   : {geometry_msgs.Transform message}

    Returns:
        t           : {numpy array} Translation vector (x,y,z)
        q           : {numpy array} Quaternion (x,y,z,w)

    Example:
    >>> from geometry_msgs.msg import Transform, Vector3, Quaternion
    >>> msg = Transform(Vector3(1,2,3), Quaternion(4,5,6,7))
    >>> t,q = transform2tq(msg)
    >>> t
    array([1, 2, 3])
    >>> q
    array([4, 5, 6, 7])
    """
    translation = transform.translation
    t = np.array([translation.x, translation.y, translation.z])
    rotation = transform.rotation
    q = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
    return t,q


def transform2T(transform):
    """Converts transform message to transformation matrix.

    Arguments:
        transform   : {geometry_msgs.Transform message}

    Returns:
        T           : {numpy matrix}

    Example:
    >>> from geometry_msgs.msg import Transform, Vector3, Quaternion
    >>> msg = Transform(Vector3(1,0,0), Quaternion(0,1,0,0))
    >>> transform2T(msg)
    matrix([[-1.,  0.,  0.,  1.],
            [ 0.,  1.,  0.,  0.],
            [ 0.,  0., -1.,  0.],
            [ 0.,  0.,  0.,  1.]])
    """
    t,q = transform2tq(transform)
    T = np.vstack((np.hstack((q2mat(q), t.reshape(3,1))), [0,0,0,1]))
    return T


def pose2transform(p, name):
    """Convert Pose message to transform

    Arguments:
        p       : {geometry_msgs/PoseStamped message}
        name    : {string}
                  Child frame name (id)

    Returns:
        t       : {geometry_msgs/TransformStamped message}
    """
    t = TransformStamped()
    t.header = p.header
    t.transform.translation = p.pose.position
    t.transform.rotation = p.pose.orientation
    t.child_frame_id = name
    return t


def frame2point(frame_id, tfb, base_id='base_link', stamp=None, homegenous=False):
    """Converts frame name to 3d point in a base frame.

    Arguments:
        frame_id    : {string}
        tfb         : {tf2_ros.Buffer}
        base_id     : {string} optional, default: base_link
        stamp       : {rospy.Time} optional, defautl: rospy.Time(0)
        homegenous  : {bool} optional, default: False

    Returns
        p           : {3x1 or 4x1 numpy array}
    """
    if stamp is None:
        stamp = rospy.Time(0)
    tform = tfb.lookup_transform(base_id, frame_id, stamp, timeout=rospy.Duration(10))
    if homegenous:
        p = np.array([tform.transform.translation.x, tform.transform.translation.y, tform.transform.translation.z, 1]).reshape(4,1)
    else:
        p = np.array([tform.transform.translation.x, tform.transform.translation.y, tform.transform.translation.z]).reshape(3,1)
    return p


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
