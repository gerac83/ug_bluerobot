# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 27, 2013

"""Trajectory input and output functions.

Examples:
    >>> s = '''time_from_start,test1,test2,test3
    ... 0.1,0.1,0.2,0.3
    ... 0.2,0.2,0.3,0.4'''
    >>> s == trajectory_to_str(trajectory_from_str(s))
    True
    >>> s = '''time_from_start,test1,test2,test3
    ... 0.1,0.1,0.2,0.3
    ... 0.2,0.2,0.3,0.4
    ... ---
    ... time_from_start,test1,test2,test3
    ... 0.1,0.1,0.2,0.3
    ... 0.2,0.2,0.3,0.4
    ... ---
    ... time_from_start,test1,test2,test3
    ... 0.1,0.1,0.2,0.3
    ... 0.2,0.2,0.3,0.4'''
    >>> s == trajectories_to_str(trajectories_from_str(s))
    True
"""

import StringIO
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def trajectory_to_str(joint_trajectory, delimeter=','):
    """Convert trajectory to redable string.

    Arguments:
        joint_trajectory    : {trajectory_msgs/JointTrajectory}
        delimeter           : {string} optional

    Returns:
        string              : {string}

    Example:
    >>> jt = JointTrajectory()
    >>> jt.joint_names = ['test1', 'test2', 'test3']
    >>> p = JointTrajectoryPoint()
    >>> p.positions = [0.1, 0.2, 0.3]
    >>> p.time_from_start = 0.1
    >>> jt.points.append(p)
    >>> p = JointTrajectoryPoint()
    >>> p.positions = [0.2, 0.3, 0.4]
    >>> p.time_from_start = 0.2
    >>> jt.points.append(p)
    >>> print trajectory_to_str(jt) # doctest: +NORMALIZE_WHITESPACE
    time_from_start,test1,test2,test3
    0.1,0.1,0.2,0.3
    0.2,0.2,0.3,0.4
    """
    output = StringIO.StringIO()

    print >>output, "time_from_start," + delimeter.join(joint_trajectory.joint_names)
    for p in joint_trajectory.points:
        print >>output, str(p.time_from_start) + "," + delimeter.join([str(v) for v in p.positions])

    val = output.getvalue()
    output.close()
    return val[:-1]


def trajectories_to_str(joint_trajectories, d1=',', d2="---"):
    """Convert multiple trajectories to string.

    Arguments:
        joint_trajectory    : {list of trajectory_msgs/JointTrajectory}
        d1                  : {string} optional
                              Delimeter of values
        d2                  : {string} optional
                              Delimeter of trajectories

    Returns:
        string              : {string}

    Examples:
    >>> jt = JointTrajectory()
    >>> jt.joint_names = ['test1', 'test2', 'test3']
    >>> p = JointTrajectoryPoint()
    >>> p.positions = [0.1, 0.2, 0.3]
    >>> p.time_from_start = 0.1
    >>> jt.points.append(p)
    >>> p = JointTrajectoryPoint()
    >>> p.positions = [0.2, 0.3, 0.4]
    >>> p.time_from_start = 0.2
    >>> jt.points.append(p)
    >>> print trajectories_to_str([jt,jt,jt]) # doctest: +NORMALIZE_WHITESPACE
    time_from_start,test1,test2,test3
    0.1,0.1,0.2,0.3
    0.2,0.2,0.3,0.4
    ---
    time_from_start,test1,test2,test3
    0.1,0.1,0.2,0.3
    0.2,0.2,0.3,0.4
    ---
    time_from_start,test1,test2,test3
    0.1,0.1,0.2,0.3
    0.2,0.2,0.3,0.4
    """
    d2 = '\n' + d2 + '\n'
    return d2.join(trajectory_to_str(t) for t in joint_trajectories)


def trajectory_from_str(string, delimeter=','):
    """Parse trajectory from string.

    Arguments:
        string              : {string}
        delimeter           : {string} optional

    Returns:
        joint_trajectory    : {trajectory_msgs/JointTrajectory}

    Examples:
    >>> s = '''time_from_start,test1,test2,test3
    ... 0.1, 0.1,0.2,0.3
    ... 0.2, 0.2,0.3,0.4'''
    >>> trajectory_from_str(s)  # doctest: +NORMALIZE_WHITESPACE
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    joint_names: ['test1', 'test2', 'test3']
    points:
      -
        positions: [0.1, 0.2, 0.3]
        velocities: []
        accelerations: []
        effort: []
        time_from_start: 0.1
      -
        positions: [0.2, 0.3, 0.4]
        velocities: []
        accelerations: []
        effort: []
        time_from_start: 0.2
    """
    lines = string.replace(" ","").split('\n')
    jt = JointTrajectory()

    # Parse header
    jt.joint_names = lines[0].split(delimeter)[1:]

    for s in lines[1:]:
        p = JointTrajectoryPoint()
        ss = s.split(delimeter)
        p.positions = [float(v) for v in ss[1:]]
        p.time_from_start = float(ss[0])
        jt.points.append(p)

    return jt

def trajectories_from_str(string, d1=',', d2='---'):
    """Convert strin to multiple trajectories.

    Arguments:
        string              : {string}
        d1                  : {string} optional
                              Delimeter of values
        d2                  : {string} optional
                              Delimeter of trajectories

    Returns:
        joint_trajectory    : {list of trajectory_msgs/JointTrajectory}
    Examples:
    >>> s = '''time_from_start,test1,test2,test3
    ... 0.1,0.1,0.2,0.3
    ... 0.2,0.2,0.3,0.4
    ... ---
    ... time_from_start,test1,test2,test3
    ... 0.1,0.1,0.2,0.3
    ... 0.2,0.2,0.3,0.4
    ... ---
    ... time_from_start,test1,test2,test3
    ... 0.1,0.1,0.2,0.3
    ... 0.2,0.2,0.3,0.4'''
    >>> trajs = trajectories_from_str(s)
    >>> len(trajs) == 3
    True
    """
    t_strings = string.replace(" ","").split('\n' + d2 + '\n')
    ts = list()
    for t_string in t_strings:
        ts.append(trajectory_from_str(t_string, d1))
    return ts

if __name__ == '__main__':
    import doctest
    f,_ = doctest.testmod()
    print "Failed" if f > 0 else "OK"
