# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 19, 2013

import numpy as np
from scipy.linalg import rq


def pix2ray(u, P):
    """Project rectified pixel to a 3d ray.

    Arguments:
        u       : {(2,N) numpy array}
                  One or more image pixels as a column vectors.
        P       : {(3,4) array like}

    Returns:
        x       : {(3,N) numpy array}
                  Unit vectors passing from camera center (0,0,0) throught the
                  coresponding rectified point in a image plane.

    Examples:
    >>> P = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
    >>> u = np.array([[0,1,0],[0,0,1]])
    >>> np.allclose(u, point2pix(pix2ray(u, P), P))
    True
    """
    x = np.zeros((3, u.shape[1]))
    x[0,:] = (u[0,:] - P[0,2]) / P[0,0]
    x[1,:] = (u[1,:] - P[1,2]) / P[1,1]
    x[2,:] = 1

    n = np.sqrt(np.sum(x ** 2, axis=0))
    x = x / np.tile(n, (3,1))

    return x


def point2pix(x, P):
    """Project point(s) in 3D to camera plane.

    Arguments:
        x       : {(3 or 4,N) numpy array}
                  3D points or homegenous 3D points as a column vectors.
        P       : {(3,4) array like}
                  Projection matrix

    Returns:
        u       : {(2,N) numpy array}
                  Points in the image plane.

    Examples:
    >>> P = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
    >>> u = np.array([[0,1,0],[0,0,1]])
    >>> np.allclose(u, point2pix(pix2ray(u, P), P))
    True
    """
    if x.shape[0] == 3:
        x = np.vstack((x, [1] * x.shape[1]))

    u = np.dot(P, x)
    u = u / u[2]
    u = np.array(u[:2, :])
    return u


def P2KRt(P):
    """Decomposition of projection matrix into K,R,t.

    Example:
    >>> K = np.matrix([[1,0,50],[0,2,50],[0,0,1]])
    >>> R = np.matrix([[-1,0,0],[0,1,0],[0,0,-1]])
    >>> t = np.matrix([10,-10,0])
    >>> P = K*np.hstack((R,t.T))
    >>> K_,R_,t_ = P2KRt(P)
    >>> np.all(K_ == K)
    True
    >>> np.all(R_ == R)
    True
    >>> np.all(t_ == t)
    True
    """
    K,R = rq(P[:,0:3])

    # Fix signs
    T = np.diag(np.sign(np.diag(K)))
    K = np.dot(K,T)
    R = np.dot(T,R)

    t = np.linalg.inv(K) * P[:,3]
    return K,R,t.T


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
