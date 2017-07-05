#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 15, 2013

from moveit_msgs.msg import RobotState as RobotStateMsg
from services import get_position_ik
from moveit_msgs.msg import MoveItErrorCodes

class IKError(Exception):

    def __init__(self, error_code):
        self.error_code = error_code

    def __str__(self):
        return "IK Error Code: %d" % self.error_code


class RobotState(RobotStateMsg):

    def __init__(self, **kwargs):
        RobotStateMsg.__init__(self, **kwargs)

    @staticmethod
    def from_robot_state_msg(msg):
        rs = RobotState()
        rs.set_from_robot_state_msg(msg)
        return rs

    @staticmethod
    def from_joint_State_msg(msg):
        rs = RobotState()
        rs.set_from_joint_state_msg(msg)
        return rs

    def reset(self):
        RobotStateMsg.__init__(self)

    def set_from_robot_state_msg(self, msg):
        self.joint_state = msg.joint_state
        self.multi_dof_joint_state = msg.multi_dof_joint_state
        self.attached_collision_objects = msg.attached_collision_objects
        self.is_diff = msg.is_diff

    def set_from_joint_state_msg(self, msg):
        self.reset()
        self.joint_state = msg

    def set_from_dict(self, ddict):
        self.reset()
        for k,v in ddict.iteritems():
            self.set_joint(k,v)

    def set_joint(self, name, position, velocity=0, effort=0):
        try:
            i = self.joint_state.name.index(name)
            self.joint_state.position[i] = position
        except ValueError:
            self.joint_state.name.append(name)
            self.joint_state.position.append(position)

    def update_from_robot_state_msg(self, msg):
        raise NotImplementedError()

    def update_from_joint_state_msg(self, msg):
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            self.set_joint(name, pos, vel, eff)

    def update_from_dict(self, ddict):
        for name,position in ddict.iteritems():
            self.set_joint(name,position)

    def update_from_pose(self, pose_stamped, ik_link_id):
        res = get_position_ik(self, ik_link_name=ik_link_id, pose_stamped=pose_stamped)

        if not res.error_code.val == MoveItErrorCodes.SUCCESS:
            raise IKError(res.error_code.val)
        else:
            self.set_from_robot_state_msg(res.solution)

