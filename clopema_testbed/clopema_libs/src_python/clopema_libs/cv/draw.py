# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 28, 2013

import cv2
import numpy as np


def draw_points(image, points, radius=3, color=(0,0,255)):
    """Draw points to on a image.

    Arguments:
        image       : {}
        points      : {2xN array like}
        radius      : {float} optional, default: 3
        color       : {tuple(integer, integer, integer)} optional, default: (0,0,255)
    """

    for p in points.T:
        cv2.circle(image, tuple(np.int0(p)), radius, color, -1)
