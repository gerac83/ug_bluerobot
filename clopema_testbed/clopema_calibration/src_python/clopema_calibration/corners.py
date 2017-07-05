# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 4, 2013

import numpy as np
import cv2


def refine_corners(gray, corners, size, scale):
    """Refine corners location.

    Arguments:
        gray            : {numpy array} Gray scale image
        corners         : {2xN numpy array} Corners as column vectors
        search_size     : {int} Size of the search rectangle
        search_scale    : {int} Scale of the image for search

    Returns:
        refined         : {2xN numpy array} Refined corners as column vectors
        mask            : {1xN array} Mask of the refined corners
    """

    # Fix shape of the corners array
    corners = corners.reshape(2,-1)

    # Resize the image
    gray_ = cv2.resize(gray, (0,0), fx=scale, fy=scale)

    # Mask the corners
    mask = [False] * corners.shape[1]

    # Refine corners one by one
    refined = corners.T.copy().astype(np.float32)
    for i, c in enumerate(corners.T):
        if c[0] > 0 and c[0] < gray.shape[1] and c[1] > 0 and c[1] < gray.shape[0]:
            c_ = _refine_corner(gray_, c * scale, size)
            if c_ is not None:
                refined[i,:] = c_ / scale
                mask[i] = True

    refined_ = refined[np.nonzero(mask)[0], :]
    cv2.cornerSubPix(gray, refined_, (11,11),(-1,-1),(cv2.cv.CV_TERMCRIT_ITER | cv2.cv.CV_TERMCRIT_EPS, 10, 0.01))
    refined[np.nonzero(mask)[0], :] = refined_

    return refined.T,mask


def _refine_corner(gray, corner, size, maxCorners=1, qualityLevel=0.01, minDistance=100, useHarrisDetector=True):
    """refine single corner location.

    Arguments:
        gray    -- Gray scale image.
        corner  -- Corner rough location.
        size    -- Size of the searched neighbourhood.
    """

    # Rough corner position
    x,y = np.int0(corner)

    # Create mask for the corner neighbourhood
    mask = np.zeros(gray.shape[:2],np.uint8)
    mask[max(y - size,0):min(y + size,mask.shape[0]),max(x - size,0):min(x + size,mask.shape[1])] = 255

    # Find corner feature in that neighbourhood
    corners = cv2.goodFeaturesToTrack(gray,maxCorners,qualityLevel,minDistance, useHarrisDetector = useHarrisDetector, mask = mask)

    return corners
