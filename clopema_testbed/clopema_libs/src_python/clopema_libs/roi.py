# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Dec 3, 2013

import numpy as np
import cv2


def extract_poly(image, poly):
    """Extract polygonial region of interest.

    Arguments:
        image       : {NxMxD numpy array}
        poly        : {2xK numpy array} optional

    Note: either poly or mask should be specified.

    Returns:
        data        : {DxH numpy array}
    """
    # Create mask
    mask = create_mask(image.shape[:2], poly)

    return extract_mask(image, mask)


def extract_mask(image, mask):
    """Extract mask from the image.

    Arguments:
        image       : {NxMxD numpy array}
        mask        : {NxM numpy array} optional

    Returns:
        data        : {DxH numpy array}
    """
    # Convert mask to 1D array
    mask = mask.reshape(-1)

    if len(image.shape) == 2:
        rgb = image.reshape(-1)
        rgb_ = rgb[np.nonzero(mask)[0]]

    elif len(image.shape) == 3:
        rgb = image.reshape(-1,image.shape[2]).T
        rgb_ = rgb[:, np.nonzero(mask)[0]]

    return rgb_


def insert_mask(image, data, mask):
    """Insert data into image using mask.

    Arguments:
        image       : {NxMxD numpy array}
        data        : {DxH numpy array}
        mask        : {NxM numpy array} optional

    Returns:
        image       : {NxMxD numpy array}
    """
    if len(image.shape) == 2:
        rgb = image.reshape(-1)
        rgb[np.nonzero(mask.reshape(-1))[0]] = data
        rgb = rgb.reshape(image.shape)

    elif len(image.shape) == 3:
        rgb = image.reshape(-1,image.shape[2]).T
        rgb[:, np.nonzero(mask.reshape(-1))[0]] = data
        rgb = rgb.T.reshape(image.shape)
    return rgb


def create_mask(shape, roi):
    """Create of given shape based on given roi.

    Arguments:
        shape       : {tuple, int}
        roi         : {(2,K)}

    Returns:
        mask        : {((N,M) numpy array)}
    """
    mask = np.zeros(shape, np.int8)
    cv2.fillPoly(mask, [np.int0(roi.T)], 255)

    return mask
