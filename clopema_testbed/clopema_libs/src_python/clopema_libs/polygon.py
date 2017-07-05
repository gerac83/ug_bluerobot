# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 19, 2013

import numpy as np
from intersection import _intersection_parameters


def poly_resize(poly, val):
    """Resize polygon by value.

    Arguments:
        poly        : {(2,N) numpy array}
        val         : {float}

    Returns:
        poly        : {(2,N) numpy array}

    Examples:
    >>> p1 = np.array([[0.,0.],[2.,0.]]).T
    >>> p2 = poly_resize(p1, 0.)
    >>> (p1 == p2).all()
    True
    >>> poly_resize(p1, 0.5)
    array([[ 0.5,  1.5],
           [ 0. ,  0. ]])
    """

    # Compute centroind
    c = np.mean(poly, axis=1)

    poly_ = np.zeros(poly.shape)
    for i, p in enumerate(poly.T):
        v = c - p
        n = np.linalg.norm(v)

        if n < val:
            poly_[:,i] = c
        else:
            poly_[:,i] = p + (v / n) * val

    return poly_


def poly_scale(poly, scale):
    """Resize polygon by value.

    Arguments:
        poly        : {(2,N) numpy array}
        scale       : {float}

    Returns:
        poly        : {(2,N) numpy array}

    Examples:
    >>> p1 = np.array([[0,0],[2,2]]).T
    >>> p2 = poly_scale(p1, 1)
    >>> (p1 == p2).all()
    True
    >>> poly_scale(p1, 0.5)
    array([[ 0.5,  1.5],
           [ 0.5,  1.5]])
    """

    # Compute centroind
    c = np.mean(poly, axis=1)

    poly_ = np.zeros(poly.shape)
    for i, p in enumerate(poly.T):
        v = p - c
        poly_[:,i] = c + v * scale

    return poly_


def poly_to_segments(poly):
    """Convert polygon to list of segments.

    Input:
        poly        : {(2,N) numpy array}

    Output:
        segments    : {(4,N) numpy array}

    Example:
    >>> poly = np.array([[0,1,-2,-1],[0,1,3,2]])
    >>> segm = poly_to_segments(poly)
    >>> segm
    array([[ 0.,  1., -2., -1.],
           [ 0.,  1.,  3.,  2.],
           [ 1., -2., -1.,  0.],
           [ 1.,  3.,  2.,  0.]])
    """
    N    = poly.shape[1]
    idx2 = np.r_[1:N, 0]

    segm = np.empty((4, N))
    segm[[0,1], :] = poly
    segm[[2,3], :] = poly[:,idx2]
    return segm


def is_point_inside_poly(poly, p):
    """Check whether p is inside polygon.

    Input:
        poly        : {(2,N) numpy array}
        p           : {(2,1) numpy array}

    Ouptut:
        inside      : {bool}

    Example:
    >>> poly = np.array([[0,1,-2,-1],[0,1,3,2]])
    >>> is_point_inside_poly(poly, np.array([[0,1]]).T)
    True
    >>> is_point_inside_poly(poly, np.array([[2,2]]).T)
    False
    """
    segms = poly_to_segments(poly)
    p_ = p + np.array([[1],[0]])

    n = 0
    for segm in segms.T:
        t,_,s,_,parallel = _intersection_parameters(p, p_, segm[0:2].reshape(2,1), segm[2:4].reshape(2,1))
        if not parallel and t > 0 and s >= 0 and s < 1:
            n += 1
    return n % 2 != 0


def area_of_polygon(poly):
    """Compute area of a polygon.

    Input:
        poly        : {(2,N) numpy array}
                      Polygon as column vectors in clockwise order.

    Output:
        area        : {float}
                      Area of the polygon.

    Example:
    >>> P = np.array([[0,1,1,0],[0,0,1,1]])
    >>> area_of_polygon(P)
    1.0
    """
    N   = poly.shape[1]
    Q0s = poly
    Q1s = poly[:, np.r_[1:poly.shape[1],0]]

    area = 0
    for i in range(N):
        x1 = Q0s[0,i]
        x2 = Q0s[1,i]
        y1 = Q1s[0,i]
        y2 = Q1s[1,i]
        area += (x1 * y2) - (x2 * y1)

    return np.abs(area / 2.0)


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
