# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 19, 2013

import numpy as np
import scipy
import scipy.linalg
from tf.transformations import quaternion_from_matrix


def plane_line_intersection(plane, p0, v):
    """Compute plane line intersection.

    Arguments:
    plane   -- A plane coefficients (a,b,c,d) where  ax + by + cz + d = 0
    p0      -- Point on a line.
    v       -- Direction vector of a line.

    Example:
    >>> plane = np.array([0, 0, 1, -1])
    >>> p0    = np.array([0,0,0])
    >>> v     = np.array([0,0,1])
    >>> plane_line_intersection(plane, p0, v)
    array([0, 0, 1])
    """

    # Compute dot product
    d = np.dot(plane[0:3], v)

    # Check for perpendicularity
    if d == 0:
        return None

    s = -(np.dot(plane[0:3], p0) + plane[3]) / d

    return p0 + s * v


def skew(v):
    """Create a skew matrix for a given vector.

    Example:
    >>> v = np.array([1,2,3])
    >>> skew(v)
    array([[ 0, -3,  2],
           [ 3,  0, -1],
           [-2,  1,  0]])
    """
    if len(v) == 4:
        v = v[:3] / v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T


def vrrotvec(v1, v2):
    """Computer ration axis and angle between two vectors.

    Arguments:
    v1,v2   -- Vector (numpy array of shape (3))

    Example:
    >>> v1 = np.array([0,0,1])
    >>> v2 = np.array([1,0,0])
    >>> vrrotvec(v1, v2)
    array([ 0.        ,  1.        ,  0.        ,  1.57079633])
    """
    # First normalize
    v1_ = v1 / np.linalg.norm(v1)
    v2_ = v2 / np.linalg.norm(v2)

    # Compute roation axis
    v = np.cross(v1_,v2_)
    v = v / np.linalg.norm(v)

    # Compute angle
    a = np.arccos(np.dot(v1_, v2_))

    return np.append(v,a)


def vrrotvec2mat(r):
    """Convert rotation axis and anglo to rotation matrix.

    Example:
    >>> v1 = np.array([0,0,1])
    >>> v2 = np.array([1,0,0])
    >>> r  = vrrotvec(v1, v2)
    >>> np.around(vrrotvec2mat(r), decimals=5)
    array([[ 0.,  0.,  1.],
           [ 0.,  1.,  0.],
           [-1.,  0.,  0.]])
    """
    # get skew matrix
    s = skew(r[:3])

    # Rodrigues formula for the rotation matrix
    R = np.eye(3) + np.sin(r[3]) * s + (1 - np.cos(r[3])) * np.dot(s,s)

    return R


def normal_from3(p1, p2, p3):
    """Compute normal vector from three points.

    Example:
    >>> p1 = np.array([0,0,0]).reshape(3,1)
    >>> p2 = np.array([1,0,0]).reshape(3,1)
    >>> p3 = np.array([0,1,0]).reshape(3,1)
    >>> n  = normal_from3(p1,p2,p3)
    >>> n / np.linalg.norm(n)
    array([[ 0.],
           [ 0.],
           [ 1.]])
    """
    # Compute vectors
    v1 = p2 - p1
    v2 = p3 - p1

    # Compute cross product
    v  = np.cross(v1.reshape(3), v2.reshape(3))

    return v.reshape(3,1)


def q2mat(q):
    """Convert quaternion to rotation matrix.

    Arguments:
        q   : {numpy array} Quaternion (x,y,z,w)

    Returns:
        R   : {numpy matrix} Rotation matrix

    Example:
    >>> q = np.array([ 0.,  1.,  0.,  0.])
    >>> q2mat(q)
    matrix([[-1.,  0.,  0.],
            [ 0.,  1.,  0.],
            [ 0.,  0., -1.]])
    """
    q_ = q / np.linalg.norm(q)
    qx, qy, qz, qw = q_

    m = np.matrix([[1.0 - 2.0 * qy * qy - 2.0 * qz * qz, 2.0 * qx * qy - 2.0 * qz * qw, 2.0 * qx * qz + 2.0 * qy * qw],
                   [2.0 * qx * qy + 2.0 * qz * qw, 1.0 - 2.0 * qx * qx - 2.0 * qz * qz, 2.0 * qy * qz - 2.0 * qx * qw],
                   [2.0 * qx * qz - 2.0 * qy * qw, 2.0 * qy * qz + 2.0 * qx * qw, 1.0 - 2.0 * qx * qx - 2.0 * qy * qy]])

    return m


def mat2q(m):
    """Compute quaternion from rotation matrix.

    Example:
    >>> m1 = np.array([[-1, 0, 0],[0,1,0],[0,0,-1]]) # 180 deg rotation around y
    >>> q = mat2q(m1)
    >>> np.allclose(q, [ 0.,  1.,  0.,  0.])
    True

    >>> m2 = np.array([[0, 1, 0],[1,0,0],[0,0,-1]])
    >>> q = mat2q(m2)
    >>> np.allclose(q, [ 0.70711, 0.70711, 0, 0])
    True
    """
    M = np.hstack((np.vstack((m, [0,0,0])), [[0],[0],[0],[1]]))
    q = quaternion_from_matrix(M)
    return q


def q2aa(q):
    """Quaternion to axis angle

    Arguments:
        q   : {1x4 array like} (x,y,z,w)

    Returns:
        aa  : {1x4 numpy array} (x,y,z,a)

    Example:
    >>> q = [0,1,0,0]
    >>> q2aa(q) # doctest: +NORMALIZE_WHITESPACE
    array([ 0.        ,  1.        ,  0.        ,  3.14159265])
    """
    q2 = q[3] ** 2
    a = 2 * np.arccos(q[3])
    if a == 0:
        return np.array([0,0,1,a])
    else:
        x = q[0] / np.sqrt(1 - q2)
        y = q[1] / np.sqrt(1 - q2)
        z = q[2] / np.sqrt(1 - q2)
        return np.array([x,y,z,a])


def aa2q(aa):
    """Axis angle to quaternion

    Arguments:
        aa  : {1x4 array like}

    Returns:
        aa  : {1x4 numpy array} (x,y,z,a)

    Example:
    >>> q = [0,1,0,0]
    >>> q_ = aa2q(q2aa(q))
    >>> np.allclose(q, q_)
    True
    """
    qx = aa[0] * np.sin(aa[3] / 2)
    qy = aa[1] * np.sin(aa[3] / 2)
    qz = aa[2] * np.sin(aa[3] / 2)
    qw = np.cos(aa[3] / 2)
    return np.array([qx,qy,qz,qw])


def homegenous(a):
    """Convert to homegenous coordiantes

    Arguments:
        a       : {NxM numpy array}

    Returns:
        b       : {N+1xM numpy array}

    Examples:
    >>> a = np.array([[1,2,3],[4,5,6]])
    >>> b = homegenous(a)
    >>> np.all(b[:2,:] == a)
    True
    >>> np.all(b[2,:] == 1)
    True
    """
    return np.vstack((a, [1] * a.shape[1]))


def null(A, eps=1e-15):
    """Compute null space Ab = 0

    Arguments:
        A       : {ndarray}
        eps     : {float} optional, defauls: 1e-15

    Returns:
        b       : {ndarray}

    Examples:
    >>> A = np.matrix(np.random.rand(3,3))
    >>> k = null(A)
    >>> np.allclose(A * k, 0)
    True
    """
    u, s, vh = scipy.linalg.svd(A)
    null_mask = (s <= eps)
    null_space = scipy.compress(null_mask, vh, axis=0)
    return scipy.transpose(null_space)


def get_perspective_transform(a, b):
    """Compute perspective transform from four corespondences.

    Arguments:
        a       : {2x4 numpy array}
        b       : {2x4 numpy array}

    Returns:
        H       : {3x3 numpy array}
                  Plane projection matrix so that [b 1]' = H * [a 1]' up to scale.

    Examples:
    >>> a = np.array([[0,1,1,0],[0,0,1,1]])
    >>> b = np.array([[0,1,1,0.5],[0,0,1,1]])
    >>> H = get_perspective_transform(a,b)
    >>> b_ = H * homegenous(a)
    >>> b_ = b_ / b_[2]
    >>> np.allclose(homegenous(b), b_)
    True
    """
    # Construct matrix A of equations
    A = np.zeros((8,9))
    for i in range(0,4):
        j = i * 2
        k = j + 1
        A[j, :] = [a[0,i], a[1,i], 1, 0, 0, 0, -a[0,i] * b[0,i], -b[0,i] * a[1,i], -b[0,i]]
        A[k, :] = [0, 0, 0, a[0,i], a[1,i], 1, -a[0,i] * b[1,i], -a[1,i] * b[1,i], -b[1,i]]

    A = np.matrix(A)
    A = np.vstack((A, [0] * A.shape[1]))
    H = np.matrix(null(A).reshape(3,3))
    H = H / H[2,2]
    return H


def project_point_on_line(P0, P1, Q):
    """Compute projection of a point on a line.

    Input:
        P0,P1       : {(2,1) numpy array}
                      Points defining a line.
        Q           : {(2,1) numpy array}
                      Point to be mirrored.

    Output:
        Q_          : {(2,1) numpy array}
                      Projection of a point p to a line L

    Example:
    >>> P0 = np.array([[1,1]]).T
    >>> P1 = np.array([[2,1]]).T
    >>> Q  = np.array([[3,2]]).T
    >>> X  = project_point_on_line(P0,P1,Q)
    >>> np.allclose(X, np.array([[3,1]]).T)
    True
    """
    v = P1 - P0
    tQ = np.dot(v.T, Q - P0) / np.dot(v.T,v)
    return tQ * v + P0


def mirror_point_along_line(P0, P1, Q):
    """Compute mirror point along a line.

    Input:
        P0,P1       : {(2,1) numpy array}
                      Points defining a line.
        Q           : {(2,1) numpy array}
                      Point to be mirrored.

    Output:
        Q_          : {(2,1), numpy array}
                      Possition of the mirrored point.

    Example:
    >>> P0 = np.array([[1,1]]).T
    >>> P1 = np.array([[2,1]]).T
    >>> Q  = np.array([[3,2]]).T
    >>> X  = mirror_point_along_line(P0,P1,Q)
    >>> np.allclose(X, np.array([[3,0]]).T)
    True
    """
    Q1 = project_point_on_line(P0, P1, Q)
    v  = Q1 - Q
    return 2 * v + Q


if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
