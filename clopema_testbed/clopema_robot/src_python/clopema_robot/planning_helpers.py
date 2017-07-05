# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 18, 2013

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


def map_ik(robot, poses=None, link=None, links=None):
    """Map inverse kinematic to list of poses.

    Arguments:
        robot   : {ClopemaRobotCommander}
        poses   : {Iterable of Pose}
        link    : {String}
        links   : {List of string}

        Either link or links must be given.

    Returns:
        joint_states : {List of JointState}
        no_failed : {Integer} A number of failed IK computations.
    """
    # Input assumptions
    assert(link is not None or links is not None)

    N = len(poses)

    if links is None:
        links = [link] * N

    # Compute ik for each pose
    f = 0
    joint_states = list()
    for p,l in zip(poses, links):
        rs = robot.get_ik(p, l)
        if rs is None:
            f += 1
        else:
            joint_states.append(rs.joint_state)

    return joint_states, f


def plan_through_joint_states(robot, joint_states):
    """Plan list of trajectories through joint states.

    Arguments:
        robot           : {ClopemaRobotCommander}
        joint_states    : {Iterable of JointState messages}

    Returns:
        trjactories     : {List of RobotTrajectory messages}
        failed_no       : {Integer} Number of failed trajectories
    """

    f = 0
    N = len(joint_states)
    trajectories = list()
    jsA = joint_states[0]
    for i in range(1, N):
        jsB = joint_states[i]
        rt = robot.plan_between_joint_states(jsA,jsB)
        if rt is not None:
            trajectories.append(rt)
            jsA = JointState()
            jsA.name = rt.joint_trajectory.joint_names
            jsA.position = rt.joint_trajectory.points[-1].positions
        else:
            f += 1

    return trajectories, f


def plan_through_poses(robot, poses, link=None,links=None):
    """Plan list of trajectories through list of poses.

    Arguments:
        robot   : {ClopemaRobotCommander}
        poses   : {Interable of PoseStamed messages}
        link    : {string}
        links   : {Iterable of strings}

    Returns:
        trajectories : {List of RobotTrajectory}
        ik_fail_no   : {Integer} Number of failed IK queries
        plan_fail_no : {Integer} Number of failed plan queries
    """

    joint_states, ik_f = map_ik(robot=robot, poses=poses, link=link, links=links)
    if len(joint_states) <= 0:
        return [], ik_f, -1

    trajectories, plan_f = plan_through_joint_states(robot, joint_states)
    if len(trajectories) <= 0:
        return [], ik_f, plan_f

    return trajectories, ik_f, plan_f
