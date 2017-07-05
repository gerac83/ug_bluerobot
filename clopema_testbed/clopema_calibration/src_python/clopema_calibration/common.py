# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Dec 4, 2013

import math

def write_frame_calibration(T, frame_id, path):
    """Writes frame calibration into file.

    Arguments:
        T           : {(4,4) numpy array}
                      Transformation from parent to frame_id.
        frame_id    : {string}
        path        : {string}
    """
    f = open(path, 'w')
    print >>f, "2"
    print >>f, frame_id
    for r in T:
        for v in r:
            print >>f, "%3.10f" % v,
        print >>f

    f.close()


def lookup_rotation(source_frame, target_frame, tf_listener = None):
    """
    Compute rotation angle for the base station towards given frame.

    @param target_frame: Target frame
    @param tf_listener: Transform Listener (optional)
    @returns: Angle in radians
    """

    # Check the tf_listener and create new one if None
    if tf_listener is None:
        tf_listener = tf.TransformListener()

    # Get the transforamtion from baselink to frame
    (trans,rot) = tf_listener.lookupTransform(source_frame, target_frame, rospy.Time(0))

    # Compute dot product
    d = sum([a * b for (a,b) in zip([0,-1],trans)])
    d = d / math.sqrt(sum([a ** 2 for a in trans[0:2]]))

    return math.acos(d)

