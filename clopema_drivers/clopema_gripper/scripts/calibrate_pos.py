#!/usr/bin/env python

import rospy
import sys

from clopema_gripper.srv import CalibratePosition, CalibratePositionRequest
from clopema_gripper.msg import SkinData

SRV_NAME_FORMAT = "/r%d_gripper/CalibratePosition"
TOPIC_ROBOT_SKIN = "/clopema_gripper/SkinDataPeriodic"

TIMEOUT = 10
SLEEP = 3

def call_calibrate_position(num):
    rospy.loginfo("Calibrating R%d gripper ...", num)
    rospy.wait_for_service(SRV_NAME_FORMAT % num, timeout=TIMEOUT)
    srv = rospy.ServiceProxy(SRV_NAME_FORMAT % num, CalibratePosition)
    req = CalibratePositionRequest()
    srv.call(req)
    rospy.loginfo("R%d gripper is now calibrated.", num)

def main():
    rospy.init_node("calibrate_pos",anonymous=True)
    rospy.wait_for_message(TOPIC_ROBOT_SKIN, SkinData)
    for i in [int(num) for num in  sys.argv[1:] if not num.startswith("__")]:
        call_calibrate_position(i)
        rospy.sleep(SLEEP)


if __name__ == "__main__":
    main()
