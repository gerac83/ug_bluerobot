# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 20, 2014

# ---- Imports --------------------------------------------------------------- #

import rospy
import tf

# ---- Constants ------------------------------------------------------------- #

node_name = "demo_cartesian_path"
node_anonymous = True

# ---- Implementation -------------------------------------------------------- #

def publish_poses(poses, names=None, frame_id="base_link"):
    br = tf.TransformBroadcaster()

    if not names:
        rospy.logdebug("No names set, creating default.")
        names = ["p_%02d" % i for i in range(0,len(poses))]

    for p,n in zip(poses, names):
        pos = [p.position.x, p.position.y, p.position.z]
        ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        br.sendTransform(pos, ori, rospy.Time.now(), n, frame_id)

def publish_stamped_poses(poses, names=None):
    br = tf.TransformBroadcaster()

    if not names:
        rospy.logdebug("No names set, creating default.")
        names = ["p_%02d" % i for i in range(0,len(poses))]

    for p,n in zip(poses, names):
        p = p.pose
        pos = [p.position.x, p.position.y, p.position.z]
        ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        br.sendTransform(pos, ori, p.header.stamp, n, p.header.frame_id)
