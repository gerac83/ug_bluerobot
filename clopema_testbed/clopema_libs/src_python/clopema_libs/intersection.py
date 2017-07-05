# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Dec 11, 2013

# Inspired by the following article:
# http://geomalgorithms.com/a05-_intersect-1.html


def _intersection_parameters(P0, P1, Q0, Q1):
    """Compute intersection parameters.

    Input:
        P0, P1      : {(2,1) numpy array}
                      First line as two points.
        Q0, Q1      : {(2,1) numpy array}
                      Second line as two points.

    Output:
        s_I, u      : {float}
                      Parametric description of the intersection point:
                      P0 + s_I * u
        t_I, v      : {float}
                      Parametric description of the intersection point:
                      Q0 + t_I * v

    """
    u = P1 - P0
    v = Q1 - Q0
    w = P0 - Q0

    p = u[0] * v[1] - u[1] * v[0]

    if p == 0:
        s_I = None
        t_I = None
    else:
        s_I = (v[1] * w[0] - v[0] * w[1]) / (v[0] * u[1] - v[1] * u[0])
        t_I = (u[0] * w[1] - u[1] * w[0]) / (u[0] * v[1] - u[1] * v[0])

    return s_I, u, t_I, v, p == 0


def line_line_intersection(P0, P1, Q0, Q1):
    """Find intersection of two lines defined as couple of points.

    Input:
        P0, P1      : {(2,1) numpy array}
                      First line as two points.
        Q0, Q1      : {(2,1) numpy array}
                      Second line as two points.

    Output:
        X           : {(2,1) numpy array}
                      Intersection point os P and Q line or None if no such
                      exists.
    """
    s_I, u, _, _, p = _intersection_parameters(P0,P1,Q0,Q1)
    if p:
        return None
    else:
        return P0 + s_I * u


def line_segment_intersection(P0, P1, Q0, Q1):
    """Find intersection of a line and line segment.


    Input:
        P0, P1      : {(2,1) numpy array}
                      First line as two points.
        Q0, Q1      : {(2,1) numpy array}
                      Second line as two points.

    Output:
        X           : {(2,1) numpy array}
                      Intersection point liing on the P line and between Q0 and
                      Q1 on the Q line or None if no such exist.

    """
    s_I, u, t_I, v, p = _intersection_parameters(P0,P1,Q0,Q1)
    X = None

    if t_I >= 0 and t_I <= 1 and not p:
        X = P0 + s_I * u
    return X


def segment_segment_intersection(P0, P1, Q0, Q1):
    """Find intersection of two line segments defined as couple of points.

    Input:
        P0, P1      : {(2,1) numpy array}
                      First line as two points.
        Q0, Q1      : {(2,1) numpy array}
                      Second line as two points.

    Output:
        X           : {(2,1) numpy array}
                      Intersection point liing between P0 and P1 on line P and
                      between Q0 and Q1 on line Q or None if no such exist.
    """
    s_I, u, t_I, v, p = _intersection_parameters(P0,P1,Q0,Q1)
    X = None

    if s_I >= 0 and s_I <= 1 and t_I >= 0 and t_I <= 1 and not p:
        X = P0 + s_I * u
    return X

if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
