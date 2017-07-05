# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 26, 2013

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped as PoseStampedMsg


class PoseStamped(PoseStampedMsg):
    """Wrapper for PoseStamed message with improved interface and conversions."""

    def __init__(self, header=Header(), pose=Pose()):
        PoseStampedMsg.__init__(self)
        self.pose = pose
        self.header = header

    @staticmethod
    def from_msg(msg):
        """New PoseStamped from PoseStamped message.

        Arguments:
            msg         : {geometry_msgs/PoseStamped}

        Returns:
            pose        : {PoseStamped}

        Examples:
        >>> pose = PoseStamped.from_msg(PoseStampedMsg(Header(), Pose()))
        >>> pose.pose.position.x
        0.0
        """
        return PoseStamped(msg.header, msg.pose)

    @staticmethod
    def from_pose_msg(msg, seq=0, stamp=rospy.Time(0), frame_id='', header=None):
        """New PoseStamped from Pose message.

        Arguments:
            msg         : {geometry_msgs/Pose}
            seq         : {integer} optional
            stamp       : {rospy.Time} optional
            frame_id    : {string} optional
            header      : {std_msgs/Header} optional
                          If header is given the seq, stamp and frame_id
                          parameters are ignored.

        Returns:
            pose        : {PoseStamped}

        Examples:

        """
        if header is not None:
            return PoseStamped(header=header, pose=msg)
        else:
            return PoseStamped(header=Header(seq=seq, stamp=stamp, frame_id=frame_id), pose=msg)

    @staticmethod
    def from_lists(position=[0,0,0], orientation=[0,0,0,0], header=Header()):
        """New PoseStamped from position and orientation as lists.

        Arguments:
            position    : {array like} optional
                          Position in form [x,y,z]
            orientation : {array like} optional
                          Orientation as an quaternion [x,y,z,w]
            header      : {std_msgs/Header} optional

        Returns:
            pose        : {PoseStamped}

        Examples:
        >>> p = PoseStamped.from_lists([1,2,3],[4,5,6,7], Header(0, rospy.Time(0), "base_link"))
        >>> p  # doctest: +NORMALIZE_WHITESPACE
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
        """
        p = Pose()
        p.position.x = position[0]
        p.position.y = position[1]
        p.position.z = position[2]
        p.orientation.x = orientation[0]
        p.orientation.y = orientation[1]
        p.orientation.z = orientation[2]
        p.orientation.w = orientation[3]

        return PoseStamped(pose=p, header=header)


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
