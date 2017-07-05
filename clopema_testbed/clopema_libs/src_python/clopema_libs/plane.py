# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 29, 2013

import numpy as np


def plane_dist(points, plane):
    """Compute point plane distance.

    Arguments:
        points      : {Nx3 numpy array}
        plane       : {numpy array}

    Returns:
        distance    : {numpy array}

    Example:
    >>> p = np.array([0,0,1,0])
    >>> ps = np.array([[0,0,1],[1,1,-1]])
    >>> plane_dist(ps, p)
    array([1, 1])
    """
    dst = np.dot(points, plane[:-1])
    dst += plane[-1]
    return np.absolute(dst)


def plane_fit_recursive(points, indexes=None, threshold=0.01, max_iter=10):
    """Recursively fit plane to a set of points.

    Arguments:
    points      -- Set of points given as a columns of a numpy array.
    indexes     -- Initial set of indexes to be used in the computation.
    threshold   -- Inlier threshold.
    max_iter    -- Max number of iterations.

    Returns:
    plane       -- Plane equation.
    """
    plane = None
    for i in range(0,max_iter):

        # Select inliers
        if indexes is not None:
            inliers = points[:, indexes]
        else:
            inliers = points

        # Fit plane to inliers
        plane = plane_fit_svd(inliers)

        # Get inliers indexes for the next iteration
        d  = plane_dist(points, plane)
        indexes = np.nonzero(d < threshold)[0]

    return plane


def plane_fit_svd(points):
    """Fit plane to a set of 3d points using SVD.

    Arguments:
    points      -- Set of points given as a columns of a numpy array.

    Returns:
    plane       -- Plane equation.
    """
    [rows,cols] = points.shape
    p = (np.ones((rows,1)))
    AB = np.hstack([points,p])
    [u, d, v] = np.linalg.svd(AB,0)
    B = v[-1,:]
    return B


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
