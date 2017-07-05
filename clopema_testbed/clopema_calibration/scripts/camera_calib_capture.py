#!/usr/bin/env python
"""
Camera calibration capture script.

Usage:
    camera_calib_capture.py  -l LINK -t FRAME -i TOPIC -o PATH [options]
    camera_calib_capture.py  -h | --help

Options:
    -l LINK, --ik-link LINK         Link for inverse kinematic
    -t FRAME, --target-frame FRAME  Frame id of the calibtation target
    -o PATH, --output-path PATH     Output path
    -i TOPIC, --image-topic TOPIC   Image topic
    -f, --force                     Force rewrite output directory
    -h, --help                      Show this message

Advanced options:
    --radius RADIUS                 Comma separated list of radius values for
                                    calibration pose calculation
                                    [default: 0.8,1.0,1.2,1.4]
    --roll ROLL                     Comma separated list of roll vaules in deg.
                                    [default: 5,20,30]
    --pitch PITCH                   Comma separated list of pitch values in deg.
                                    [default: 0,120,240]
    --no-ext-axis                   Do not rate the external axis towards target
                                    frame.
    --no-move-away                  Do not move the another arm away.
"""

# Libor Wagner on August 22, 2013

PACKAGE_NAME = 'clopema_calibration'
NODE_NAME    = 'xtion_calib_capture'

import roslib
roslib.load_manifest(PACKAGE_NAME)

import rospy
import sys
import tf
import os
import copy
import math
import clopema_calibration.calib_utils as tools

from clopema_utilities import docopt
from smach import Sequence
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from clopema_utilities.pose import gen_pose_sphere_target

from clopema_smach.write import WriteMsgState, WriteTFState
from clopema_smach import gensm_plan_vis_exec, Plan2ToJointsState, PauseState
from clopema_smach.msg import MA1400JointState
from clopema_smach import PlanExtAxisState


def gen_sm_rotate_towards(target_frame, tfl):
    sm = gensm_plan_vis_exec(PlanExtAxisState())
    sm.userdata.position = tools.lookup_rotation('base_link', target_frame, tfl)
    return sm


def gen_sm_move_away(robot, target_frame, tf_listener):
    """Compose get in start position state machine."""
    sm = gensm_plan_vis_exec(Plan2ToJointsState(), input_keys=['goal_r1', 'goal_r2'])
    joints = MA1400JointState()
    if robot == 2:
        sm.userdata.goal_r2 = copy.deepcopy(joints)
        joints.s = -math.pi / 2
        sm.userdata.goal_r1 = copy.deepcopy(joints)
    else:
        sm.userdata.goal_r1 = copy.deepcopy(joints)
        joints.s = math.pi / 2
        sm.userdata.goal_r2 = copy.deepcopy(joints)
    return sm


def gen_sm_capture_rgb(image_topic, output_dir, arm, tfl):
    """Compose the capture RGB state machine."""
    sm = Sequence(outcomes=['succeeded', 'preempted', 'aborted'], connector_outcome='succeeded')

    write_rgb = WriteMsgState(image_topic, ttype=Image, name_template=output_dir + "/%03d_RGB.png", drop=10)
    write_Jrgb = tools.WriteJointState(name_template=output_dir + "/%03d_J.txt")
    write_Trgb = WriteTFState("/r%d_link_1" % arm, "r%d_link_6" % arm, tfl, name_template=output_dir + "/%03d_T.txt")

    with sm:
        Sequence.add("PAUSE", PauseState(2))
        Sequence.add("CAPTURE_RGB", write_rgb)
        Sequence.add("SAVE_Jrgb", write_Jrgb)
        Sequence.add("SAVE_Trgb", write_Trgb)

    return sm


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    argv = rospy.myargv(argv=sys.argv)
    opt  = docopt(__doc__, argv = argv[1:])
    print opt

    # Copy parameters
    target_frame    = opt['--target-frame']
    output_path     = opt['--output-path']
    image_topic     = opt['--image-topic']
    ik_link_name    = opt['--ik-link']
    group_name      = tools.group_by_link[ik_link_name]
    arm             = tools.robot_by_link[ik_link_name]
    force_rewrite   = opt['--force']
    no_ext_axis     = opt['--no-ext-axis']
    no_move_away    = opt['--no-move-away']
    radius          = tools.parse_list(opt['--radius'], ',')
    roll            = tools.parse_list(opt['--roll'], ',')
    pitch           = tools.parse_list(opt['--pitch'], ',')

    # Check calibration target frame
    tfl = tf.TransformListener()
    tfb = tf.TransformBroadcaster()
    rospy.sleep(1)
    if not tfl.frameExists(target_frame):
        rospy.logfatal('Calibration target frame "%s" does not exist!', target_frame)
        sys.exit(1)

    # Check output path
    if os.path.isdir(output_path):
        if force_rewrite:
            import shutil
            shutil.rmtree(output_path)
        else:
            rospy.logfatal('Directory: %s: exist and is not empty', output_path)
            sys.exit(1)

    os.makedirs(output_path)
    rospy.loginfo("Empty directory created: %s", output_path)

    # Rotate towards target frame
    if not no_ext_axis:
        sm = gen_sm_rotate_towards(target_frame, tfl)
        outcome = sm.execute()
        rospy.sleep(1)

        if outcome is not 'succeeded':
            rospy.logfatal("Unable to get into start position!")
            exit(1)

    # Move another arm away
    if not no_move_away:
        sm = gen_sm_move_away(arm, target_frame, tfl)
        outcome = sm.execute()
        rospy.sleep(1)

        if outcome is not 'succeeded':
            rospy.logfatal("Unable to get into start position!")
            exit(1)

    # Compute poses
    pa = PoseArray()
    pa.header.frame_id = target_frame
    pa.poses = gen_pose_sphere_target(radius, roll, pitch)
    tools.publish_poses(pa, tfb)

    # Compute trajectories
    trajs = tools.plan_through_posearray(group_name, pa, ik_link_name)

    # RGB capture state machine
    traj = tools.plan_to_trajectory(group_name, trajs[0])
    capture = gen_sm_capture_rgb(image_topic, output_path, arm, tfl)
    capture_rgb = tools.gen_sm_capture(capture, confirm = True)

    capture_rgb.userdata.trajectories = [traj] + trajs
    outcome = capture_rgb.execute()

    if outcome is not 'succeeded':
        rospy.logfatal("State machine capture_rgb failed!!")
        exit(1)
