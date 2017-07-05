#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  November 19, 2014

# ---- Doc ------------------------------------------------------------------- #

"""Demo poses

Usage:
    demo_polish.py [options]

Options:
    -h, --help          Show this help message

"""

# ---- Import ---------------------------------------------------------------- #

import rospy
import numpy as np
import itertools
from docopt import docopt
from clopema_robot import ClopemaRobotCommander, RobotState
from clopema_libs.pose import make_pose
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from tf.transformations import quaternion_from_euler
from clopema_libs.pose import pose_stamped
from clopema_libs.rviz import publish_poses
from copy import deepcopy
from clopema_robot import services


# ---- Constants ------------------------------------------------------------- #

node_name = "demo_polish"
node_anonymous = True

no_candidates = 10
angle_limit = np.pi/4
table_ofset = 0.1
r1_ik_link = "r1_ee_par"
r2_ik_link = "r2_ee_par"
frame_id = "t3_desk"
above_table = 0.00
p0 = Point(0.3,0, above_table)
p1 = Point(0.1,0, above_table)
tilt = np.pi/20

# ---- Main ------------------------------------------------------------------ #

def main():
    # Initialize node
    rospy.init_node(node_name, anonymous=node_anonymous)

    # Initialize robot commander
    arms = ClopemaRobotCommander("arms")

    # Move to start position
    move_to_start_state()

    # Compute polish move
    traj = compute_polish_move(p0, p1, length=0.2, frame_id=frame_id)

def move_to_start_state():
    ext = ClopemaRobotCommander("ext")
    ext.set_named_target("ext_minus_90")
    traj = ext.plan()
    ext.execute(traj)

def compute_polish_move(hold_point, start_point, length=0.1, frame_id="base_link"):

    # Compute the third point
    p0 = point_to_np(hold_point)
    p1 = point_to_np(start_point)
    v  = p1 - p0
    v  = v / np.linalg.norm(v)
    p2 = p1 + length * v
    rospy.loginfo("p0=%s, p1=%s, p2=%s", str(p0), str(p1), str(p2))

    # Get the vector angle
    a = np.arctan2(v[1], v[0])
    rospy.loginfo("Angle: [%f, %f]", a-angle_limit, a+angle_limit)

    # Generate poses
    poses0 = generate_poses(p0, 0, 2*np.pi, no_candidates)

    poses1 = generate_poses(p1, a-angle_limit, a+angle_limit, no_candidates)
    poses2 = generate_poses(p2, a-angle_limit, a+angle_limit, no_candidates)

    candidates = [(p[0],p[1][0],p[1][1]) for p in itertools.product(poses0, zip(poses1, poses2))]

    arms = ClopemaRobotCommander("arms")
    state = RobotState.from_robot_state_msg(arms.get_current_state())

    trajs = []
    for p0,p1,p2 in candidates:
        trajs = []
        state0 = deepcopy(state)

        # Point for the first arm
        r1_p03 = deepcopy(p0)
        r1_p03.position.z = r1_p03.position.z + table_ofset
        r1_p12 = deepcopy(p0)

        # Points for the second arm
        r2_p0 = deepcopy(p1)
        r2_p0.position.z = r2_p0.position.z + table_ofset
        r2_p1 = deepcopy(p1)
        r2_p2 = deepcopy(p2)
        r2_p3 = deepcopy(p2)
        r2_p3.position.z = r2_p3.position.z + table_ofset

        try:
            state0.update_from_pose(pose_stamped(r1_p03, frame_id=frame_id), ik_link_id=r1_ik_link)
        except Exception as e:
            rospy.logwarn("Arm: %s, pose: %s: %s", str(r1_ik_link), show_pose(r1_p03), str(e))
            continue
        try:
            state0.update_from_pose(pose_stamped(r2_p0, frame_id=frame_id), ik_link_id=r2_ik_link)
        except Exception as e:
            rospy.logwarn("Arm: %s, pose: %s: %s", str(r2_ik_link), show_pose(r2_p0), str(e))
            continue

        arms.set_start_state_to_current_state()
        arms.set_joint_value_target(state0.joint_state)
        traj = arms.plan()
        trajs.append(traj)

        if not traj:
            rospy.logwarn("Unable to plan to initial position.")
            continue

        arms.set_start_state(state0)
        arms.set_pose_reference_frame(frame_id)
        wp_1 = [r1_p12,r1_p12,r1_p03]
        wp_2 = [r2_p1, r2_p2, r2_p3]
        traj, fraction = arms.compute_cartesian_path(zip(wp_1, wp_2), (r1_ik_link, r2_ik_link), jump_threshold=2, avoid_collisions=False)

        if fraction < 1:
            rospy.logwarn("Unable to interpolate polish move")
            continue

        enable_collisions = [("t3_desk","r1_gripper"), ("t3_desk", "r2_gripper")]
        if not services.check_trajectory(state0, traj, enable_collisions, wait_timeout=1):
            rospy.logwarn("Found trajectory is in collision!")
            continue

        trajs.append(traj)
        break

    print len(trajs)
    arms.set_robot_speed(0.05)
    arms.execute(trajs[0])
    arms.execute(trajs[1])



def generate_poses(p, a_from, a_to, candidates):
    return [Pose(Point(*p),Quaternion(*quaternion_from_euler(0,np.pi/2 + tilt,angle))) for angle in np.linspace(a_from,a_to,candidates)]

def point_to_np(p):
    return np.array([p.x,p.y,p.z])

def np_to_point(p):
    return Point(p[0], p[1], p[2])

def show_pose(pose):
    p = pose.position
    o = pose.orientation
    return "P:[%f,%f,%f] O:[%f,%f,%f,%f]" % (p.x,p.y,p.z,o.x,o.y,o.z,o.w)

def show_trajectory(traj):
    s = ''
    for point in traj.joint_trajectory.points:
        s += ' '.join(["%-3.3f" % f for f in point.positions]) + "\n"
    return s

if __name__ == '__main__':
    main()
