#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Jul 23, 2014

"""Test script for dotmatrix service.

Usage:
    test_dotmatrix_srv.py [options] <image>

Options:
    --repeat            Repeat the call
    --debug             Run in debug mode.
"""

import rospy
import sys
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from clopema_libs import docopt
from clopema_libs.np_utils import np_to_imgmsg
from clopema_calibration.srv import DetectDotmatrix
from scipy.misc import imread
import numpy as np
import threading

# ---- Constants

dotmatrix_service = '/dotmatrix/detect'


def main():

    # Process command line arguments
    opt = docopt(__doc__)
    file_name = opt['<image>']
    repeat = opt['--repeat']

    # Read the image
    image = imread(file_name)
    image = image.astype('float32')

    # RGB to gray-scale
    if len(image.shape) > 2 and image.shape[2] == 3:
        image_gray = np.dot(image[...,:3], [0.299, 0.587, 0.144])
    else:
        image_gray = image

    # Resample to 0-255
    image_gray = image_gray / np.max(image_gray) * 255
    image3 = np.atleast_3d(image_gray)
    image3 = image3.astype('uint8')
    image_msg = np_to_imgmsg(image3)
    print 'Image shape: %s dtype: %s, message enc: %s' % (str(image.shape), str(image.dtype), image_msg.encoding)

    # Initialize ros
    rospy.init_node("pubimage", anonymous=True)

    # Wait for service to be ready
    rospy.loginfo("Waiting for %s service to be ready ...", dotmatrix_service)
    rospy.wait_for_service(dotmatrix_service)

    # Register and call the service
    rospy.loginfo("Service %s is ready, issuing request ...", dotmatrix_service)
    srv = rospy.ServiceProxy(dotmatrix_service, DetectDotmatrix)

    try:
        x,y = call_service(srv, image_msg)

        fig = plt.figure()
        ax = fig.add_subplot(111)
        points, = ax.plot(x, y, 'xr')
        ax.imshow(image_gray)
        plt.draw()

        if repeat:
            t = threading.Thread(target=update_plot,args=(srv,image_msg, points,))
            t.daemon = True
            t.start()

        plt.show()
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))
        sys.exit(1)


def call_service(srv, image_msg):
    res = srv(image_msg)
    x = [d.imcol for d in res.dotmatrix.dots]
    y = [d.imrow for d in res.dotmatrix.dots]
    return x, y


def update_plot(srv, image_msg, points):
    while True:
        x,y = call_service(srv, image_msg)
        points.set_xdata(x)
        points.set_ydata(y)
        plt.draw()


if __name__ == '__main__':
    main()
