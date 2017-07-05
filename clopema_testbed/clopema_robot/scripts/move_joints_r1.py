#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 15, 2013

"""
Usage:
    move_joints_r1.py [options] [--] S [L] [U] [R] [B] [T]

Options:
    -s SPEED, --speed SPEED     Speed of the movement.
    -f, --sof                   Stop servo at the end.


The S,L,U,R,B,T arguments are joint radiuses in degrees it can also be '-' and
that means that the joint stays the same.

The '--' is required if you are using negative numbers.
"""

from math import radians, degrees

from clopema_libs import docopt
from clopema_libs.ui import ask
from clopema_robot import ClopemaRobotCommander


if __name__ == '__main__':
    args = docopt(__doc__, options_first=True)

    robot = ClopemaRobotCommander("r1_arm")

    if args['S'] is not None and args['S'] is not '-':
        robot.set_joint_value_target("r1_joint_s", radians(float(args['S'])))

    if args['L'] is not None and args['L'] is not '-':
        robot.set_joint_value_target("r1_joint_l", radians(float(args['L'])))

    if args['U'] is not None and args['U'] is not '-':
        robot.set_joint_value_target("r1_joint_u", radians(float(args['U'])))

    if args['R'] is not None and args['R'] is not '-':
        robot.set_joint_value_target("r1_joint_r", radians(float(args['R'])))

    if args['B'] is not None and args['B'] is not '-':
        robot.set_joint_value_target("r1_joint_b", radians(float(args['B'])))

    if args['T'] is not None and args['T'] is not '-':
        robot.set_joint_value_target("r1_joint_t", radians(float(args['T'])))

    traj = robot.plan()

    # Print some info
    print "From:   ", " ".join(["% +3.2f" % degrees(r) for r in traj.joint_trajectory.points[0].positions])
    print "To:     ", " ".join(["% +3.2f" % degrees(r) for r in traj.joint_trajectory.points[-1].positions])
    print "Length: ", len(traj.joint_trajectory.points)

    if args['--speed'] is not None:
        print "Speed   ", "%+3.2f" % float(args['--speed'])
    else:
        print "Speed   ", "%+3.2f" % robot.get_robot_speed()

    r = ask("Execute trajectory [Y,n]? ", ['y','n'], 'y')
    if r == 'y':
        if args['--speed'] is not None:
            robot.set_robot_speed(float(args['--speed']))

        robot.go()

        if args['--sof']:
            robot.set_servo_power_off(True)
