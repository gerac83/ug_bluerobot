#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Dec 2, 2013


"""Table calibration script.


Usage:
    table_calibration.py [options]
    table_calibration.py -h | --help

Options:
    -t <table>, --table <table>     Which table to check [default: 2]
    -i <image>, --image <image>     Which image topic to use [default: image]
    -c <camera>, --camera <camera>  Which camera info topic to use [default: camera_info]
    -x <xtion>, --xtion <xtion>
    -h, --help                      Display help message

Advanced options:
    --search-size NUM               Size of the search rectangle for each corners
    --depth-margin NUM              Margin of the rectangle for plane fitting
"""

import rospy
import tf2_ros
import numpy as np
import os

from clopema_libs import docopt
from clopema_libs.transform import frame2point
from clopema_calibration.table_calibration import TableCalibration
from clopema_calibration import write_frame_calibration
from clopema_robot import get_table_frames, get_xtion_val


if __name__ == '__main__':
    rospy.init_node('table_calibration_check', anonymous=True)

    args = docopt(__doc__, argv=rospy.myargv()[1:])

    if args['--xtion'] is not None:
        n = int(args['--xtion'])
        image_topic_name = get_xtion_val(n, 'rgb_image')
        camera_info_topic_name = get_xtion_val(n, 'rgb_info')
        depth_image_topic_name = get_xtion_val(n, 'depth_image')
    else:
        image_topic_name = args['--image']
        camera_info_topic_name = args['--camera']

    tfb = tf2_ros.Buffer(cache_time = rospy.Duration(5.0))
    tfl = tf2_ros.TransformListener(tfb)
    rospy.sleep(1)  # Wait for cache to fill up

    frame_ids = get_table_frames(int(args['--table']))

    # Get corners
    corners = np.hstack([frame2point(f, tfb, homegenous=True) for f in frame_ids[1:]])

    tcc = TableCalibration(corners=corners, tfb=tfb)
    tcc.image_topic_name = image_topic_name
    tcc.camera_info_topic_name = camera_info_topic_name
    tcc.dept_image_topic_name = depth_image_topic_name

    if args['--search-size'] is not None:
        tcc.corners_search_size = int(args['--search-size'])

    if args['--depth-margin'] is not None:
        tcc.depth_margin = int(args['--depth-margin'])

    tcc.start()

    # Output path
    import roslib.packages
    import os.path
    pkg = roslib.packages.get_pkg_dir('clopema_description')
    output_path = os.path.join(pkg, 'calibration_' + os.environ['CLOPEMA_PARTNER'], frame_ids[0] + '.txt')

    print "===================================================================="
    print "Move the robot using teach pendant of RViz into position so that all "
    print "table corners are visible, check whether the calibration is taking"
    print "the right corners, then press ctrl+c to stop and save the calibration"
    print "data in %s" % output_path

    rospy.spin()

    if tcc.T.size == 16:
        print
        print tcc.tform
        write_frame_calibration(tcc.tform, frame_ids[0], output_path)
