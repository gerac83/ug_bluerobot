#include <ros/ros.h>
#include <stdint.h>
#include <clopema_gripper/CalibratePosition.h>
#include <clopema_gripper/CalibrateForce.h>
#include <clopema_gripper/CalibrateAmbientProximity.h>
#include <clopema_gripper/GetDirection.h>
#include <clopema_gripper/GetFrequency.h>
#include <clopema_gripper/GetGripperState.h>
#include <clopema_gripper/GetMod.h>
#include <clopema_gripper/GetPosition.h>
#include <clopema_gripper/MoveAbsolute.h>
#include <clopema_gripper/MoveAbsolutePercentage.h>
#include <clopema_gripper/MoveRelative.h>
#include <clopema_gripper/MoveRelativePercentage.h>
#include <clopema_gripper/SetDirection.h>
#include <clopema_gripper/SetFrequency.h>
#include <clopema_gripper/SetMod.h>
#include <clopema_gripper/SetGripperState.h>
#include <clopema_gripper/MotorPosition.h>
#include <clopema_gripper/MotorStatus.h>

bool pos_calib = false;
bool force_calib = false;
bool prox_calib = false;

void statCallBack(const clopema_gripper::MotorStatus::ConstPtr& msg)
{
	pos_calib = msg->position_calibrated;
	force_calib = msg->force_calibrated;
	prox_calib = msg->ambient_and_proximity_calibrated;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "StepperServerCalibTest");
	ros::NodeHandle node;

	ros::Subscriber sub2 = node.subscribe("r1_gripper/MotorStatus", 1000, statCallBack);


	ros::ServiceClient pos_calib_client = node.serviceClient<clopema_gripper::CalibratePosition>("r1_gripper/CalibratePosition");
	ros::ServiceClient force_calib_client = node.serviceClient<clopema_gripper::CalibrateForce>("r1_gripper/CalibrateForce");
	ros::ServiceClient prox_calib_client = node.serviceClient<clopema_gripper::CalibrateAmbientProximity>("r1_gripper/CalibratePosition");
	clopema_gripper::CalibratePosition pos_calib_srv;
	clopema_gripper::CalibrateForce force_calib_srv;
	clopema_gripper::CalibrateAmbientProximity prox_calib_srv;

	ros::Rate r(10);

	pos_calib_client.call(pos_calib_srv);
	while (ros::ok() && !pos_calib)
	{
		ros::spinOnce();
		r.sleep();
	}

	force_calib_client.call(force_calib_srv);
	while (ros::ok() && !force_calib)
	{
		ros::spinOnce();
		r.sleep();
	}

	prox_calib_client.call(prox_calib_srv);
	while (ros::ok() && !prox_calib)
	{
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
