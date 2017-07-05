#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Dec 12, 2013

"""Controll clopema grippers.

Usage:
    gripper.py -r N -s S
    gripper.py -h

Options:
    -r N, --robot N         Robot number i.e. 1 or 2.
    -s S, --state S         Desired state of the gripper 0 closed and 1 openned.
    -h, --help              Display this message.
"""

import rospy
import clopema_robot as robot
from clopema_libs import docopt


if __name__ == '__main__':
    rospy.init_node("move_pose", anonymous=True)
    args = docopt(__doc__, options_first=True)

    N = int(args['--robot'])
    S = int(args['--state'])

    actions = {1:robot.ACTION_R1_GRIPPER, 2: robot.ACTION_R2_GRIPPER}
    robot.ClopemaRobotCommander.set_gripper_state(actions[N], S)
