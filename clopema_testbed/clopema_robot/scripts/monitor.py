#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Dec 4, 2013

import urwid
import rospy
import math
import numpy as np
from clopema_robot import ClopemaRobotCommander


def jmsg2dict(msg):
    return dict(zip(msg.name, msg.position))


def rad2deg(rad):
    return (rad / (2 * math.pi)) * 360


class Monitor(object):

    palette = [('body','black','light gray', 'standout'),
               ('header','black','light gray', 'standout'),
               ('footer','black','light gray', 'standout'),
               ('warn','dark red','light gray', 'standout')]

    refresh = 0.1
    values  = dict()

    def __init__(self, robot, r1_frame_id="r1_tip_link", r2_frame_id="r2_tip_link"):
        self.robot = robot
        self.r1_frame_id = r1_frame_id
        self.r2_frame_id = r2_frame_id
        text_header = [(u"press q to quit")]
        footer = urwid.AttrMap(urwid.Text(text_header), 'footer')

        self.body = urwid.Filler(footer, 'top')
        self.view = urwid.Frame(self.body, footer=footer)
        self.loop = urwid.MainLoop(self.view, self.palette, unhandled_input=self.exit_on_q)

        self.update_alarm = self.loop.set_alarm_in(0, self.update)
        self.loop.run()

    def update(self, loop=None, user_data=None):
        state = self.robot.get_current_state()
        r1_pose = self.robot.get_current_pose(self.r1_frame_id).pose
        r2_pose = self.robot.get_current_pose(self.r2_frame_id).pose
        st = jmsg2dict(state.joint_state)

        s = list()
        s += ["             R1                       R2 \n"]
        s += ["      [rad]     [deg]          [rad]     [deg] \n"]
        r1_val = st["r1_joint_s"]
        r2_val = st["r2_joint_s"]
        s += [" S: ", self.format_val("r1_joint_s_rad",r1_val), "   ", self.format_val("r1_joint_s_deg",rad2deg(r1_val)), "     S: ", self.format_val("r2_joint_s_rad",r2_val), "   ", self.format_val("r2_joint_s_deg",rad2deg(r2_val)), '\n']
        r1_val = st["r1_joint_l"]
        r2_val = st["r2_joint_l"]
        s += [" L: ", self.format_val("r1_joint_l_rad",r1_val), "   ", self.format_val("r1_joint_l_deg",rad2deg(r1_val)), "     L: ", self.format_val("r2_joint_l_rad",r2_val), "   ", self.format_val("r2_joint_l_deg",rad2deg(r2_val)), '\n']
        r1_val = st["r1_joint_u"]
        r2_val = st["r2_joint_u"]
        s += [" U: ", self.format_val("r1_joint_u_rad",r1_val), "   ", self.format_val("r1_joint_u_deg",rad2deg(r1_val)), "     U: ", self.format_val("r2_joint_u_rad",r2_val), "   ", self.format_val("r2_joint_u_deg",rad2deg(r2_val)), '\n']
        r1_val = st["r1_joint_r"]
        r2_val = st["r2_joint_r"]
        s += [" R: ", self.format_val("r1_joint_r_rad",r1_val), "   ", self.format_val("r1_joint_r_deg",rad2deg(r1_val)), "     R: ", self.format_val("r2_joint_r_rad",r2_val), "   ", self.format_val("r2_joint_r_deg",rad2deg(r2_val)), '\n']
        r1_val = st["r1_joint_b"]
        r2_val = st["r2_joint_b"]
        s += [" B: ", self.format_val("r1_joint_b_rad",r1_val), "   ", self.format_val("r1_joint_b_deg",rad2deg(r1_val)), "     B: ", self.format_val("r2_joint_b_rad",r2_val), "   ", self.format_val("r2_joint_b_deg",rad2deg(r2_val)), '\n']
        r1_val = st["r1_joint_t"]
        r2_val = st["r2_joint_t"]
        s += [" T: ", self.format_val("r1_joint_t_rad",r1_val), "   ", self.format_val("r1_joint_t_deg",rad2deg(r1_val)), "     T: ", self.format_val("r2_joint_t_rad",r2_val), "   ", self.format_val("r2_joint_t_deg",rad2deg(r2_val)), '\n']
        s += ["\n\n"]
        s += ["    ", self.r1_frame_id, "      ", self.r2_frame_id, "\n"]
        s += [" X: ", self.format_val("r1_x",r1_pose.position.x), "   X: ", self.format_val("r2_x",r2_pose.position.x), '\n']
        s += [" Y: ", self.format_val("r1_y",r1_pose.position.y), "   Y: ", self.format_val("r2_y",r2_pose.position.y), '\n']
        s += [" Z: ", self.format_val("r1_z",r1_pose.position.z), "   Z: ", self.format_val("r2_z",r2_pose.position.z), '\n']
        s += ["QX: ", self.format_val("r1_qx",r1_pose.orientation.x), "  QX: ", self.format_val("r2_qx",r2_pose.orientation.x), '\n']
        s += ["QY: ", self.format_val("r1_qy",r1_pose.orientation.y), "  QY: ", self.format_val("r2_qy",r2_pose.orientation.y), '\n']
        s += ["QZ: ", self.format_val("r1_qz",r1_pose.orientation.z), "  QZ: ", self.format_val("r2_qz",r2_pose.orientation.z), '\n']
        s += ["QW: ", self.format_val("r1_qw",r1_pose.orientation.w), "  QW: ", self.format_val("r2_qw",r2_pose.orientation.w), '\n']

        text = urwid.AttrMap(urwid.Text(s), 'body')
        self.view.set_body(urwid.Filler(text, 'top'))
        self.update_alarm = self.loop.set_alarm_in(self.refresh, self.update)

    def exit_on_q(self, input):
        if input in ('q', 'Q'):
            raise urwid.ExitMainLoop()

    def format_val(self, name, value):
        if name in self.values:
            rec = self.values[name]

            if np.allclose(rec[0], value):
                s = ("body","% -8.3f" % value)
            else:
                s = ("warn", "% -8.3f" % value)

            if rec[1] > 2 / self.refresh:
                self.values[name] = (value, 0)
            else:
                self.values[name] = (rec[0], rec[1] + 1)
        else:
            s = ("body", "% -8.3f" % value)
            self.values[name] = (value, 0)
        return s

if __name__ == "__main__":
    rospy.init_node("robot_monitor", anonymous=True)
    robot = ClopemaRobotCommander('arms')
    monitor = Monitor(robot)
