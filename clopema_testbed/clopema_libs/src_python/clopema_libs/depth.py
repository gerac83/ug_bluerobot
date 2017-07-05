# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 23, 2013

import numpy as np


def d2X(P, d):
    """Project depth image to point cloud

    Arguments:
        P   : {3x4 numpy array} Projection matrix
        d   : {HxW numpy array} Depth image

    Returns:
        X   : {HxWx3 numpy array} Organised point cloud

    Example:
    >>> d = np.array([[0,1,2],[3,4,5],[6,7,8]])
    >>> P = np.array([[1,0,1,0],[0,1,1,0],[0,0,1,0]])
    >>> d2X(P,d) # doctest: +NORMALIZE_WHITESPACE
    array([[[ 0,  0,  0],
            [ 0, -1,  1],
            [ 2, -2,  2]],
           [[-3,  0,  3],
            [ 0,  0,  4],
            [ 5,  0,  5]],
           [[-6,  6,  6],
            [ 0,  7,  7],
            [ 8,  8,  8]]])
    """
    w,h = d.shape
    u,v = np.meshgrid(np.arange(0,h), np.arange(0,w))

    x = ((u - P[0,2]) / P[0,0]) * d
    y = ((v - P[1,2]) / P[1,1]) * d
    z = d

    return np.dstack((x,y,z))


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
