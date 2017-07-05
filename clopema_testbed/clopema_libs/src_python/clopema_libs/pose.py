# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 20, 2013

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion, Point
from std_msgs.msg import Header
from clopema_libs.geometry import mat2q, q2mat
from moveit_commander import conversions


def pose2affine(pose):
    """Convert pose to affine transform.

    Example:
    >>> p = conversions.list_to_pose([1,2,3,0,1,0,0])
    >>> pose2affine(p)
    array([[-1.,  0.,  0.,  1.],
           [ 0.,  1.,  0.,  2.],
           [ 0.,  0., -1.,  3.],
           [ 0.,  0.,  0.,  1.]])
    """
    lst = conversions.pose_to_list(pose)
    pos = np.array(lst[:3])
    rot = np.array(lst[3:])

    T = np.zeros((4,4))
    T[np.ix_([0,1,2],[0,1,2])] = q2mat(rot)
    T[np.ix_([0,1,2],[3])] = pos.reshape(3,1)
    T[np.ix_([3], [0,1,2,3])] = [0,0,0,1]

    return T


def affine2pose(T):
    """Convert affine transformation to Pose.

    Example:
    >>> T = np.array([[-1,0,0,1],[0,1,0,2],[0,0,-1,3],[0,0,0,1]])
    >>> conversions.pose_to_list(affine2pose(T))
    [1.0, 2.0, 3.0, 0.0, 1.0, 0.0, 0.0]
    """
    q = mat2q(T[0:3][:,0:3])
    t = T[np.ix_([0,1,2],[3])]

    return conversions.list_to_pose(np.append(t,q))


def qt2pose(q,t):
    """Convert quaternion and translation to pose message

    Arguments:
        q   : {1x4 array like}
        t   : {1x3 array like}

    Returns
        p   : {geometry_msgs/Pose}

    Example:
    >>> t = [1,2,3]
    >>> q = [4,5,6,7]
    >>> q_,t_ = pose2qt(qt2pose(q,t))
    >>> np.all(t == t_)
    True
    >>> np.all(q == q_)
    True
    """
    return Pose(Vector3(t[0], t[1], t[2]), Quaternion(q[0],q[1],q[2],q[3]))


def pose2qt(p):
    """Convert pose to quaternion and translation

    Arguments:
        p   : {geometry_msgs/Pose}

    Returns
        q   : {1x4 numpy array}
        t   : {1x3 numpy array}

    Example:
    >>> t = [1,2,3]
    >>> q = [4,5,6,7]
    >>> q_,t_ = pose2qt(qt2pose(q,t))
    >>> np.all(t == t_)
    True
    >>> np.all(q == q_)
    True
    """
    q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    t = np.array([p.position.x, p.position.y, p.position.z])
    return q,t

def make_pose(x,y,z,qx,qy,qz,qw, frame_id=None):
    """Make Pose or PoseStamped message

    Arguments:
        x,y,z           position
        qx,qy,qz,qw     orientation

    Returns
        pose            Pose if no frame_id is given, PoseStamped otherwise.
    """
    pose = Pose(Point(x,y,z),Quaternion(qx,qy,qz,qw))
    if frame_id:
        pose = PoseStamped(pose=pose)
        pose.header.frame_id = frame_id
    return pose

def pose_stamped(pose, seq=0, frame_id="base_link", stamp=rospy.Time(0)):
    """Make PoseStamped from Pose

    Arguments:
        pose        : {geometry_msgs.Pose}
        frame_id    : {string}
        stamp       : {rospy.Time}

    Returns:
        pose_staped : {geometry_msgs.PoseStamped}

    Example:
    >>> from geometry_msgs.msg import Pose, Vector3, Quaternion
    >>> msg = pose_stamped(Pose(Vector3(1,2,3), Quaternion(4,5,6,7)))
    >>> msg # doctest: +NORMALIZE_WHITESPACE
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: base_link
    pose:
      position:
        x: 1
        y: 2
        z: 3
      orientation:
        x: 4
        y: 5
        z: 6
        w: 7
    >>> msg = pose_stamped(Pose(Vector3(1,2,3), Quaternion(4,5,6,7)), seq=1, frame_id="test", stamp=rospy.Time(1))
    >>> msg # doctest: +NORMALIZE_WHITESPACE
    header:
      seq: 1
      stamp:
        secs: 1
        nsecs: 0
      frame_id: test
    pose:
      position:
        x: 1
        y: 2
        z: 3
      orientation:
        x: 4
        y: 5
        z: 6
        w: 7
    """
    return PoseStamped(Header(seq=seq, frame_id=frame_id, stamp=stamp), pose)


def orient_pose(pose, target = np.array([0,0,0])):
    """Orinter pose towards target

    Arguments:
        pose        : {geometry_msgs/Pose}
        [target]    : {1x3 numpy array}

    Returns:
        pose        : {geometry_msgs/Pose}

    Example:
    TBD
    """
    _,t = pose2qt(pose)
    R = _direction(t, target)
    q = mat2q(R)
    q = q / np.linalg.norm(q)

    return qt2pose(q,t)


def _direction(A, B):
    """Compute coordinate system so that the Z axis is the AB vector

    Arguments:
        A,B     : {1x3 numpy array}

    Returns:
        R       : {3x3 numpy array}
                  Coordinate system where the Z axis (last column) points in the
                  AB direction.

    Example:
    >>> A = np.array([0,0,1])
    >>> B = np.array([0,0,0])
    >>> _direction(A,B)
    array([[ 1.,  0.,  0.],
           [ 0., -1.,  0.],
           [ 0.,  0., -1.]])
    >>> A = np.array([0,0,0])
    >>> B = np.array([0,0,1])
    >>> _direction(A,B)
    array([[ 1.,  0.,  0.],
           [ 0.,  1.,  0.],
           [ 0.,  0.,  1.]])
    """
    # Z axis of the new coordiante system
    Z = B - A
    Z = Z / np.linalg.norm(Z)

    # X axis of the new coordiante system
    X = np.cross(Z, [0,0,1])

    # Singularity check
    if np.all(X == 0):
        X = np.array([1,0,0])
    else:
        X = X / np.linalg.norm(X)

    # Y axis of the new coordinate system
    Y = np.cross(Z,X)
    Y = Y / np.linalg.norm(Y)

    # Coordinate system
    R = np.hstack((X.reshape(3,1),Y.reshape(3,1),Z.reshape(3,1)))

    return R


def gen_pose_grid_target(x_range, y_range, z_range, target=np.array([0,0,0])):
    """Generate grid of poses with z axis pointing to the given target"""
    poses = []

    for z in z_range:
        for x in x_range:
            for y in y_range:
                pose = orient_pose(Pose(position=Vector3(x,y,z)), target)
                poses.append(pose)

    return poses


def gen_pose_grid(x_range, y_range, z_range, orientation = Quaternion(0,1,0,0)):
    """Generate grid of poses with fixed orientation"""
    poses = []

    for x in x_range:
        for y in y_range:
            for z in z_range:
                pose = Pose(position=Vector3(x,y,z), orientation=orientation)
                poses.append(pose)
    return poses


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
