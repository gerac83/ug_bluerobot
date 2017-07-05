#!/usr/bin/env python

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 21, 2014


# ---- Imports --------------------------------------------------------------- #

import rospy

from clopema_robot.services import *

# ---- Main ------------------------------------------------------------------ #

def main():
    rospy.init_node("check_services", anonymous=True)
    all_srv = [SRV_GET_POSITION_IK_SERVICE, SRV_CHECK_TRAJECTORY_SERVICE, SRV_ADD_TIME_PARAMETRISATION,
               SRV_SET_DRIVE_POWER, SRV_SET_ROBOT_SPEED]
    for srv in all_srv:
        ok = True
        try:
            rospy.wait_for_service(srv, timeout=1.0)
        except ROSException as e:
            ok = False
        print srv, "--", "SUCCESS" if ok else "FAILURE"

if __name__ == '__main__':
    main()
