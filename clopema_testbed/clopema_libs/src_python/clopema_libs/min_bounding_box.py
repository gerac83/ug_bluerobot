import numpy as np

from numpy import *
from convex_hull import convex_hull


def min_bounding_box(points):
    """Find min area bounding box.

    Use:
        bb = min_bounding_box(points)

    Input:
        points          : {(2,N) numpy array}
                          Set of points

    Output:
        bb              : {(2,4) numpy array}
                          Min-area bounding box

    """
    # Compute convex hull
    CH = convex_hull(points)
    CH = CH[:, r_[:CH.shape[1],0]]

    # Compute all angles
    E = diff(CH, axis=1)
    T = arctan2(E[1,:], E[0,:])
    T = unique(T % (pi / 2.0))

    # Compute all possible rotations
    R = cos(kron(ones((2,2)),T).reshape(2 * T.size,2, order='F') + kron(ones((T.size,1)),np.array([[0, -pi], [pi, 0]]) / 2.0))
    RCH = dot(R, CH)

    bsize = (np.max(RCH, axis=1) - np.min(RCH, axis=1))
    area  = prod(bsize.reshape(2,bsize.size / 2, order='F'), axis=0)

    # find minimal area, thus the index of the angle in T
    i = np.argmin(area)

    # compute the bound (min and max) on the rotated frame
    Rf    = R[2 * i + [0,1],:]   # rotated frame
    bound = dot(Rf, CH)                # project CH on the rotated frame
    bmin  = np.min(bound, axis=1)
    bmax  = np.max(bound, axis=1)

    # compute the corner of the bounding box
    Rf = Rf.T
    bb = np.zeros((2,4))
    bb[:,3] = dot(bmax[0], Rf[:,0]) + dot(bmin[1], Rf[:,1])
    bb[:,0] = dot(bmin[0], Rf[:,0]) + dot(bmin[1], Rf[:,1])
    bb[:,1] = dot(bmin[0], Rf[:,0]) + dot(bmax[1], Rf[:,1])
    bb[:,2] = dot(bmax[0], Rf[:,0]) + dot(bmax[1], Rf[:,1])

    return bb
