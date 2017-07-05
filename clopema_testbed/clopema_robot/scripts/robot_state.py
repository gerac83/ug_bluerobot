#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Jan 16, 2014

"""Save and execute robot state.

Usage:
    robot_state.py [options] save <file_name>
    robot_state.py [options] exec <file_name>
    robot_state.py -h | --help

Options:
    -d, --debug         Debug mode.
    --speed SPEED       Robot speed.
    --sof               Turn servo off at the end.
    --force             Do not ask for permission.
"""

import rospy
import yaml
import genpy
import numpy as np
from clopema_libs import docopt
from clopema_libs.ui import ask
from clopema_robot import ClopemaRobotCommander

from sensor_msgs.msg import JointState

_NODE_NAME = "robot_state_cmd"
_DEBUG = False


def save_cmd(filename):
    robot = ClopemaRobotCommander("arms")
    rs = robot.get_current_state()
    js = rs.joint_state

    with open(filename, "w") as f:
        f.write(str(js))


def get_values(ddict, keys):
    values = []
    for k in keys:
        values.append(ddict[k])
    return values


def has_arms(rs, js):

    rs_dict = dict(zip(rs.joint_state.name, rs.joint_state.position))

    name = list(js.name)
    position = list(js.position)

    if "ext_axis" in name:
        i = name.index("ext_axis")
        del name[i]
        del position[i]
    js_p = np.array(position)
    rs_p = np.array(get_values(rs_dict, name))

    print " ".join(["%10s" % v for v in name])
    print "Start state:  ", ", ".join(["% -6.4f" % v for v in js_p])
    print "Target state: ", ", ".join(["% -6.4f" % v for v in rs_p])
    print "Diff:         ", ", ".join(["% -6.4f" % v for v in np.abs(rs_p - js_p)])

    return not np.allclose(js_p, rs_p, atol=2e-4)


def has_ext(rs, js):
    rs_dict = dict(zip(rs.joint_state.name, rs.joint_state.position))
    js_dict = dict(zip(js.name, js.position))

    if "ext_axis" in js_dict:
        print rs_dict["ext_axis"], js_dict["ext_axis"]
        return not np.allclose(rs_dict["ext_axis"], js_dict["ext_axis"])
    else:
        return False


def exec_cmd(filename):
    with open(filename, "r") as f:
        data = yaml.load_all(f)
        s = next(data)
    js = JointState()
    genpy.message.fill_message_args(js, s)
    robot = ClopemaRobotCommander("arms")
    rs = robot.get_current_state()

    if has_arms(rs, js):
        robot = ClopemaRobotCommander("arms")

        robot.set_joint_value_target(js)
        traj = robot.plan()
        if args['--speed'] is not None:
            print "Speed   ", "%+3.2f" % float(args['--speed'])
        else:
            print "Speed   ", "%+3.2f" % robot.get_robot_speed()

        if len(traj.joint_trajectory.points) <= 0:
            print "No valid trajectory found please check the robot output."
        else:
            r = ask("Execute trajectory? ", {'Y':'Yes','n':'No'})
            if r == 'y':
                if args['--speed'] is not None:
                    robot.set_robot_speed(float(args['--speed']))

                robot.execute(traj)

    if has_ext(rs, js):
        robot = ClopemaRobotCommander("ext")

        robot.set_joint_value_target(js)
        traj = robot.plan()
        if args['--speed'] is not None:
            print "Speed   ", "%+3.2f" % float(args['--speed'])
        else:
            print "Speed   ", "%+3.2f" % robot.get_robot_speed()

        if len(traj.joint_trajectory.points) <= 0:
            print "No valid trajectory found please check the robot output."
        else:
            r = ask("Execute trajectory? ", {'Y':'Yes','n':'No'})
            if r == 'y':
                if args['--speed'] is not None:
                    robot.set_robot_speed(float(args['--speed']))

                robot.execute(traj)

    if args['--sof']:
        robot.set_servo_power_off(True)


if __name__ == '__main__':

    # Parse command line
    args = docopt(__doc__)
    _DEBUG = args['--debug']

    if _DEBUG:
        print args

    # Initialize node
    rospy.init_node(_NODE_NAME, anonymous=True)

    # Initialize robot

    # Call apropriate model
    if args['save']:
        save_cmd(args['<file_name>'])
    elif args['exec']:
        exec_cmd(args['<file_name>'])

