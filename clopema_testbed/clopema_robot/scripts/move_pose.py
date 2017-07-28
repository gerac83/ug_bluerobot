#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 15, 2013

"""
Usage:
    move_pose_r2.py [options] [--] X Y Z QX QY QZ QW

Options:
    -s SPEED, --speed SPEED     Speed of the movement.
    -l LINK, --link LINK        End effector link [default: r1_ee]
    -c, --cartesian             Plan in cartesian space
    -f, --sof                   Stop servo at the end.
    --no-check                  Do not check collisions.
    --eef-step STEP             Step of the end effector [default: 0.01].

The '--' is required if you are using negative numbers.
"""

import rospy
from clopema_libs import docopt
from clopema_libs.ui import ask
from clopema_robot import ClopemaRobotCommander
from copy import deepcopy


if __name__ == '__main__':
    rospy.init_node("move_pose", anonymous=True)
    args = docopt(__doc__, options_first=True)

    group = ClopemaRobotCommander.get_group_for_ee(args['--link'])
    robot = ClopemaRobotCommander(group)
    link_id = args['--link']
    link  = robot.robot.get_link(link_id)
    pose  = link.pose().pose
    pose_ = deepcopy(pose)
    eef_step = float(args['--eef-step'])

    # Print info
    print "group: ", group
    print "link_id: ", link_id
    print "eef_step: ", eef_step

    if args['X'] is not None and args['X'] is not '-':
        pose_.position.x = float(args['X'])
    if args['Y'] is not None and args['Y'] is not '-':
        pose_.position.y = float(args['Y'])
    if args['Z'] is not None and args['Z'] is not '-':
        pose_.position.z = float(args['Z'])
    if args['QX'] is not None and args['QX'] is not '-':
        pose_.orientation.x = float(args['QX'])
    if args['QY'] is not None and args['QY'] is not '-':
        pose_.orientation.y = float(args['QY'])
    if args['QZ'] is not None and args['QZ'] is not '-':
        pose_.orientation.z = float(args['QZ'])
    if args['QW'] is not None and args['QW'] is not '-':
        pose_.orientation.w = float(args['QW'])

    if args['--cartesian']:
        (traj,fraction) = robot.compute_cartesian_path([pose_], link_id, eef_step=eef_step, avoid_collisions=(not args['--no-check']))
        print fraction
    else:
        robot.set_joint_value_target(pose_, link_id)
        traj = robot.plan()

    # Print some info
    print "From:     ", " pos: %+3.2f %+3.2f %+3.2f, ori: %+3.2f %+3.2f %+3.2f %+3.2f" % (pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    print "To:      ", " pos: %+3.2f %+3.2f %+3.2f, ori: %+3.2f %+3.2f %+3.2f %+3.2f" % (pose_.position.x, pose_.position.y, pose_.position.z, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w)
    print "Length:   ", len(traj.joint_trajectory.points)

    if args['--speed'] is not None:
        print "Speed   ", "%+3.2f" % float(args['--speed'])
    else:
        print "Speed   ", "%+3.2f" % robot.get_robot_speed()

    if len(traj.joint_trajectory.points) <= 0:
        print "No valid trajectory found please check the robot output."
    else:
        r = ask("Execute trajectory? ", {'Y':'Yes','n':'No'})
        if r == 'y':
#            if args['--speed'] is not None:
#                robot.set_robot_speed(float(args['--speed']))

            robot.execute(traj)

            if args['--sof']:
                robot.set_servo_power_off(True)
