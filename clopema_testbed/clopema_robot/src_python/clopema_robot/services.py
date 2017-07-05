# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Apr 2, 2014


# ---- Imports --------------------------------------------------------------- #

import rospy
import utils

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import Constraints
from clopema_robot.group import infer_group_from_ee
from clopema_moveit.srv import TrajCollision, TrajCollisionRequest
from clopema_robot.srv import TimeParametrization, TimeParametrizationRequest
from industrial_msgs.srv import SetDrivePower, SetDrivePowerRequest
from clopema_robot.srv import SetSpeed, SetSpeedRequest
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from clopema_moveit.srv import GetCartesianPathDual, GetCartesianPathDualRequest
from copy import deepcopy


# ---- Constants ------------------------------------------------------------- #

SRV_GET_POSITION_IK_SERVICE = "/compute_ik"
SRV_GET_POSITION_FK = "/compute_fk"
SRV_CHECK_TRAJECTORY_SERVICE = "/check_trajectory"
SRV_ADD_TIME_PARAMETRISATION = "/move_group/add_time_parametrization"
SRV_SET_DRIVE_POWER = "/clopema_controller/set_drive_power"
SRV_SET_ROBOT_SPEED = "/move_group/set_robot_speed"
SRV_EXECUTE = "/execute_kinematic_path"
SRV_GET_CARTESIAN_PATH_DUAL = "/compute_cartesian_path_dual"


# ---- Implementations ------------------------------------------------------- #

def get_position_ik(robot_state, ik_link_name, pose_stamped, constraints=Constraints(), avoid_collisions=True, timeout=rospy.Time(1), persistent=False, wait_timeout=None):
    """Call position IK service.

    INPUT (see moveit_msgs/PositionIKRequest for details)
        robot_state         [moveit_msgs/RobotState]
        ik_link_name        [String, default=""]
        pose_stamped        [geometry_msgs/PoseStamped, default=None]
        constraints         [moveit_msgs/Constraints, default=None]
        avoid_collisions    [Bool, default=True]
        timeout             [Float, default=1.0]

        persistent          [Bool, default=False]
                            Set to True if calling this function many times.

        wait_timeout        [float, default=None]
                            Time to wait for the service.

    OUTPUT
        response            [moveit_msgs/GetPositionIKResponse]
    """

    ik_srv = get_service_proxy(SRV_GET_POSITION_IK_SERVICE, GetPositionIK, persistent, wait_timeout)

    # Make a copy
    pose_stamped = deepcopy(pose_stamped)

    # Fix the link if differ from default
    new_link_name, new_pose = utils.fix_link_for_ik(pose_stamped.pose, ik_link_name)
    if not new_link_name == ik_link_name:
        rospy.logdebug("Link %s is not IK link, was converted to %s", ik_link_name, new_link_name)
        ik_link_name = new_link_name
        pose_stamped.pose = new_pose

    req = GetPositionIKRequest()
    req.ik_request.group_name = infer_group_from_ee(ik_link_name)
    req.ik_request.robot_state = robot_state
    req.ik_request.constraints = constraints
    req.ik_request.avoid_collisions = avoid_collisions
    req.ik_request.ik_link_name = ik_link_name
    req.ik_request.pose_stamped = pose_stamped
    req.ik_request.ik_link_names = []
    req.ik_request.pose_stamped_vector = []
    req.ik_request.timeout = timeout
    req.ik_request.attempts = 1

    res = ik_srv(req)
    return res


def add_time_parametrisation(traj, start_state, group, persistent=False, wait_timeout=None):
    """Call add time parametrisation service

    INPUT
        traj                [moveit_msgs/RobotTrajectory]
        start_state         [moveit_msgs/RobotState]
        group               [String]

    OUTPUT
        traj                [moveit_msgs/RobotTrajectory]
    """

    time_param_srv = get_service_proxy(SRV_ADD_TIME_PARAMETRISATION, TimeParametrization, persistent, wait_timeout)

    req = TimeParametrizationRequest()
    req.trajectory = traj
    req.start_state = start_state
    req.group = group

    res = time_param_srv(req)
    return res


def check_trajectory(start_state, trajectory, enable_collision_list=[], persistent=False, wait_timeout=None):
    """Call check trajectory service

    INPUT
        start_state             [moveit_msgs/RobotState]
        trajectory              [moveit_msgs/RobotTrajectory]
        enable_collision_list   [Tuple of Strings]

        persistent          [Bool, default=False]
                            Set to True if calling this function many times.

        wait_timeout        [Float, default=None]
                            Time to wait for the service.
    OUTPUT
        valid               [Bool]
                            True if the trajectory is collision free, False
                            otherwise.
    """

    srv_proxy = get_service_proxy(SRV_CHECK_TRAJECTORY_SERVICE, TrajCollision, persistent, wait_timeout)

    req = TrajCollisionRequest()
    req.start_state = start_state
    req.rtraj = trajectory

    if len(enable_collision_list) > 0:
        collision_link_1, collision_link_2 = zip(*enable_collision_list)
        req.enable_collision_1 = collision_link_1
        req.enable_collision_2 = collision_link_2

    res = srv_proxy(req)

    return res.valid


def set_drive_power(power, persistent=False, wait_timeout=None):
    """Call set drive power service

    INPUT
        power           [Bool]
        persistent      [Bool, default=False]
        wait_timeout    [Float, default=None]

    OUTPUT
        response        [SetDrivePowerResponse]
    """
    srv_proxy = get_service_proxy(SRV_SET_DRIVE_POWER, SetDrivePower, persistent, wait_timeout)
    res = srv_proxy(SetDrivePowerRequest(power))
    return res


def set_robot_speed(speed, persistent=False, wait_timeout=None):
    """Call set robot speed service

    INPUT
        speed           [Float, range: 0.01 - 0.2]
        persistent      [Bool, default=False]
        wait_timeout    [Float, default=None]

    OUTPUT
        response        [SetSpeedResponse]
    """
    rospy.logwarn("Set robot speed is deprecated, use time parametrisation instead.")
    srv_proxy = get_service_proxy(SRV_SET_ROBOT_SPEED, SetSpeed, persistent, wait_timeout)
    res = srv_proxy(SetSpeedRequest(speed=speed))
    return res


def execute(trajectory,wait=True,persistent=False,wait_timeout=None):
    """Call set robot speed service

    INPUT
        trajectory      [RobotTrajectory]
        wait            [Bool]
        persistent      [Bool, default=False]
        wait_timeout    [Float, default=None]

    OUTPUT
        response        [SetSpeedResponse]
    """
    srv_proxy = get_service_proxy(SRV_EXECUTE, ExecuteKnownTrajectory, persistent, wait_timeout)
    req = ExecuteKnownTrajectoryRequest()
    req.trajectory = trajectory
    req.wait_for_execution = wait
    res = srv_proxy(req)
    return res

def get_position_fk(robot_state,fk_link_names,persistent=False,wait_timeout=None):
    """Call compute forward kinematics service

    INPUT
        robot_state     [RobotState message]
        fk_link_names   [List of Strings]
        persistent      [Bool, default=False]
        wait_timeout    [Float, default=None]

    OUTPUT
        response        [GetPositionFKResponse]
    """
    srv_proxy = get_service_proxy(SRV_GET_POSITION_FK, GetPositionFK, persistent, wait_timeout)

    req = GetPositionFKRequest()
    req.robot_state = robot_state
    req.fk_link_names = fk_link_names

    res = srv_proxy(req)
    return res


def compute_cartesian_path_dual(start_state,
                            links,
                            poses,
                            frame_id='base_link',
                            max_step=0.01,
                            jump_threshold=1.2,
                            avoid_collisions=True,
                            path_constraints=Constraints(),
                            persistent=False,
                            wait_timeout=None):

    srv_proxy = get_service_proxy(SRV_GET_CARTESIAN_PATH_DUAL,
                                  GetCartesianPathDual,
                                  persistent,
                                  wait_timeout)

    req = GetCartesianPathDualRequest()
    req.header.frame_id = frame_id
    req.start_state = start_state
    req.group_name = 'arms'
    req.link_name_1 = links[0]
    req.link_name_2 = links[1]
    req.waypoints_1 = poses[0]
    req.waypoints_2 = poses[1]
    req.max_step    = max_step
    req.jump_threshold = jump_threshold
    req.avoid_collisions = avoid_collisions
    req.path_constraints = path_constraints

    res = srv_proxy(req)
    return res

# ---- Helper functions ------------------------------------------------------ #

services = dict()


def get_service_proxy(service_name, service_type, persistent, wait_timeout):
    if persistent:
        if service_name not in services:
            rospy.wait_for_service(service_name, wait_timeout)
            services[service_name] = rospy.ServiceProxy(service_name, service_type)
        srv = services[service_name]
    else:
        rospy.wait_for_service(service_name, wait_timeout)
        srv = rospy.ServiceProxy(service_name, service_type)
    return srv





