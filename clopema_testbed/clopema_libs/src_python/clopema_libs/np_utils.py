# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Jan 13, 2014

import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

bridge = None


def imgmsg_to_np(msg, encoding=None):
    """Converts ROS Image message to numpy array.

    Input:
        msg         : {sensor_msgs/Image}
                    Input image message.
        encoding    : {String, optional}
                    Desired encoding of the output image. If ommited the format
                    is left as it is.

    Output:
        image   : {(N,M,D) numpy array}
                  Output numpy array.

    Note that this function uses persistent CvBridge class.
    """
    global bridge

    if bridge is None:
        bridge = CvBridge()

    if encoding is None:
        return np.asarray(bridge.imgmsg_to_cv(msg, msg.encoding))
    else:
        return np.asarray(bridge.imgmsg_to_cv(msg, encoding))


def np_to_imgmsg(image, encoding='passthrough'):
    """Convert numpy array to ROS Image message.

    Input
        image       : {numpy array}
                      Input image array
        encoding    : {String, default: passthrough}
                      Desired encoding

    Output:
        msg         : {sensor_msgs/Image}
                      ROS Image message
    """
    global bridge

    if bridge is None:
        bridge = CvBridge()

    return bridge.cv2_to_imgmsg(image, encoding)


def points_to_np(points, dim=3, dtype=np.double):
    """Convert list of ROS points to numpy array.

    Input:
        points  : {geometry_msgs/Point[]}

    Output:
        mat     : {(D,N) numpy array}
    """
    N = len(points)
    assert(dim <= 3 and dim > 0)

    mat = np.zeros((dim,N), dtype=dtype)
    for i in range(N):
        mat[0,i] = points[i].x
        if dim > 1:
            mat[1,i] = points[i].y
        if dim > 2:
            mat[2,i] = points[i].z

    return mat


def np_to_points(mat):
    """Convert column vector of numpy array to list of points.

    Input:
        mat     : {(D,N) numpy array}

    Output:
        points  : {geometry_msgs/Point[]}
    """
    D,N = mat.shape

    assert(D <= 3 and D > 0)
    points = list()
    for i in range(N):
        p = Point()
        p.x = mat[0,i]
        if D > 1:
            p.y = mat[1,i]
        if D > 2:
            p.z = mat[2,i]
        points.append(p)
    return points
