#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Apr 1, 2014

"""
Usage:
    camera_calib_capture_v2.py [options] <trajectory_file>
    camera_calib_capture_v2.py -h | --help

Options:
    -d, --debug             Show debug information
    -h, --help              Show this help message
    -o DIR, --output DIR    Output directory
    -x, ID, --xtion ID      Xtion ID either 1 or 2, infered from the trajectory
                            if ommited.
    -f, --force             Overwrite existing output directory.
    --no-confirm            Run without asking
"""

# ---- Imports --------------------------------------------------------------- #

import rospy
import sys

from sensor_msgs.msg import Image

from clopema_libs.measure import Measure
from clopema_libs import docopt
from clopema_libs.io import read_msgs, write_msg, write_image_msg, prepare_output_directory, prepare_unique_directory
from clopema_libs.tf_utils import get_tf2_buffer, transform_msg_to_T
from clopema_libs.trajectory import get_moving_joints
from clopema_robot import ClopemaRobotCommander
from clopema_robot.xtion import get_xtion
from clopema_libs.topic import TopicBuffer
from moveit_msgs.msg import RobotTrajectory

# ---- Constants ------------------------------------------------------------- #

NODE_NAME = "camera_calib_capture"
NODE_ANONYMOUS = True
ROBOT_GROUP = "arms"
PRE_CAPTURE_PAUSE = 3


# ---- Main ------------------------------------------------------------------ #

def main():
    # Process cammand line arguments
    argv = rospy.myargv(argv=sys.argv)
    opt  = docopt(__doc__, argv = argv[1:])

    opt_debug = opt['--debug']
    opt_trajectory_file = opt['<trajectory_file>']
    opt_force = opt['--force']
    opt_no_confirm = opt['--no-confirm']
    opt_xtion = opt['--xtion']
    opt_output = opt['--output']

    if opt_debug:
        print opt

    # Initialise node
    rospy.init_node(NODE_NAME, anonymous=NODE_ANONYMOUS)

    # Initialise robot
    robot = ClopemaRobotCommander(ROBOT_GROUP)

    # Initialise tf buffer
    tfb = get_tf2_buffer()

    # Load trajectories
    trajs = read_msgs(opt_trajectory_file, RobotTrajectory)

    # Infer what xtion we are calibrating
    if opt_xtion is None:
        xtion = get_xtion(infer_xtion(trajs))
    else:
        xtion = get_xtion(int(opt_xtion))

    # Check output directory
    if opt_output is not None:
        output_path = opt_output
        if not prepare_output_directory(output_path, opt_force):
            rospy.logerr("Output directory is not empty, try adding -f if you want to overwrite it.")
            return
    else:
        output_path = prepare_unique_directory("%s_calib" % xtion.name)

    # Initialise measurement
    calib = CameraCalib(robot, tfb, xtion, output_path)
    calib.confirm = not opt_no_confirm

    # Run calibration capture
    calib.run(trajs)

    # Turn servo off
    robot.set_servo_power_off()


# ---- CameraCalib class ----------------------------------------------------- #

class CameraCalib(Measure):

    def __init__(self, robot, tfb, xtion, output_path):
        Measure.__init__(self, robot)
        self.xtion = xtion
        self.output_path = output_path
        self.image_buff = TopicBuffer(self.xtion.rgb_topic_name, Image)
        self.tfb = tfb
        self.source_frame_id = xtion.calib_source_link_id
        self.target_frame_id = xtion.calib_target_link_id

    def measure(self, n):
        # Sleep for some time, to be sure that the robot is stopped
        rospy.sleep(PRE_CAPTURE_PAUSE)
        time = rospy.Time.now()
        # Save image from camera
        image_msg = self.image_buff.get_newer(time)
        write_image_msg(image_msg, self.output_path + "/%03d_RGB.png" % (n + 1))

        # Save joint state
        robot_state = self.robot.get_current_state()
        write_msg(robot_state.joint_state, self.output_path + "/%03d_J.txt" % (n + 1))

        # Save transformation
        transform = self.tfb.lookup_transform(self.source_frame_id, self.target_frame_id, time)
        write_transform_msg(transform.transform, self.output_path + "/%03d_T.txt" % (n + 1))


# ---- Helper functions ------------------------------------------------------ #

def write_transform_msg(transform_msg, file_path):
    """Save translatin and rotation to a given file."""
    T = transform_msg_to_T(transform_msg)
    with open(file_path, 'w') as f:
        for row in T:
            f.write("%f,%f,%f,%f\n" % (row[0], row[1], row[2], row[3]))


def infer_xtion(trajs):
    traj = trajs[0]
    moving_joints = get_moving_joints(traj)
    if moving_joints[0].startswith("r1"):
        return 1
    else:
        return 2


if __name__ == "__main__":
    main()
