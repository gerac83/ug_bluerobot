#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  March 18, 2014

"""Execute robot trajectory.

Usage:
    execute_trajectory.py [options] <file_name>
    execute_trajectory.py -h | --help

Options:
    --speed SPEED   Robot speed
    --no-sof        Do not turn servo off at the end
    --force         Do not ask for anything
    -d, --debug     Run in debug mode
"""

import rospy
from clopema_libs import docopt
from clopema_libs.io import read_msgs
from clopema_libs.ui import ask
from clopema_robot import ClopemaRobotCommander
from moveit_msgs.msg import RobotTrajectory


_NODE_NAME = "execute_trajectory_cmd"
_DEBUG = False


if __name__ == "__main__":

    # Parse command line
    args = docopt(__doc__)
    _DEBUG = args['--debug']

    # Print all arguments if debug
    if _DEBUG:
        print args

    # Initialize node
    rospy.init_node(_NODE_NAME, anonymous=True)

    # Initialize robot
    robot = ClopemaRobotCommander("arms")

    trajs = read_msgs(args['<file_name>'], RobotTrajectory)

    # Check speed
    if args['--speed'] is not None:
        print "Speed   ", "%+3.2f" % float(args['--speed'])
    else:
        print "Speed   ", "%+3.2f" % robot.get_robot_speed()

    if not args['--force']:
        r = ask("Execute trajectory? ", {'Y':'Yes','n':'No'})
    else:
        r = 'y'

    if r == 'y':
        if args['--speed'] is not None:
            robot.set_robot_speed(float(args['--speed']))

        for traj in trajs:
            robot.execute(traj)

    if not args['--no-sof']:
        robot.set_servo_power_off(True)
