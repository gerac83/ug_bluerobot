#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 21, 2014


# ---- Imports --------------------------------------------------------------- #

import rospy
import numpy as np

from clopema_robot import ClopemaRobotCommander
from clopema_robot import RobotState
from clopema_libs.ihelpers import pose_from_xyzrpy

# ---- Constants ------------------------------------------------------------- #


# ---- Main ------------------------------------------------------------------ #

def main():
    rospy.init_node("test_planning_and_execution", anonymous=True)
    robot = ClopemaRobotCommander("arms")
    robot.set_start_state_to_current_state()

    # Get initial robot state
    state = RobotState.from_robot_state_msg(robot.get_current_state())

    # Set position of both arms
    state.update_from_pose(pose_from_xyzrpy(0.3,-1.0,1.5, np.pi/2,0,np.pi/2), ik_link_id="r2_ee")

    # Set as target
    robot.set_joint_value_target(state.joint_state)

    # Plan trajectory
    traj = robot.plan()

    # Execute trajectory
    robot.execute(traj)

if __name__ == '__main__':
    main()
