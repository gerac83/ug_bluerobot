#include <ros/ros.h>
#include <stdint.h>
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

void posCallBack(const clopema_gripper::MotorPosition::ConstPtr& msg)
{
	ROS_INFO("@%f", msg->motorPosition);
}

void statCallBack(const clopema_gripper::MotorStatus::ConstPtr& msg)
{
	ROS_INFO("(MotorStatus) %9s %9s %12s %14s %s",
			msg->opening?"Opening":"", msg->closing?"Closing":"",
			msg->position_calibrated?"Pos-Calibed":"",
			msg->force_calibrated?"Force-Calibed":"",
			msg->ambient_and_proximity_calibrated?"Amb-Prox-Calibed":"");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "StepperServerTest");
	ros::NodeHandle node;

	ros::Subscriber sub1 = node.subscribe("r1_gripper/MotorPosition", 1000, posCallBack);
	ros::Subscriber sub2 = node.subscribe("r1_gripper/MotorStatus", 1000, statCallBack);


	ros::ServiceClient client = node.serviceClient<clopema_gripper::GetPosition>("r1_gripper/GetPosition");
	ros::ServiceClient client1 = node.serviceClient<clopema_gripper::GetDirection>("r1_gripper/GetDirection");
	ros::ServiceClient client2 = node.serviceClient<clopema_gripper::GetFrequency>("r1_gripper/GetFrequency");
	ros::ServiceClient client3 = node.serviceClient<clopema_gripper::GetGripperState>("r1_gripper/GetGripperState");
	ros::ServiceClient client4 = node.serviceClient<clopema_gripper::GetMod>("r1_gripper/GetMod");
	ros::ServiceClient client5 = node.serviceClient<clopema_gripper::MoveAbsolute>("r1_gripper/MoveAbsolute");
	ros::ServiceClient client6 = node.serviceClient<clopema_gripper::MoveAbsolutePercentage>("r1_gripper/MoveAbsolutePercentage");
	clopema_gripper::GetPosition srv;
	clopema_gripper::GetDirection srv1;
	clopema_gripper::GetFrequency srv2;
	clopema_gripper::GetGripperState srv3;
	clopema_gripper::GetMod srv4;
	clopema_gripper::MoveAbsolute  srv5;
	clopema_gripper::MoveAbsolutePercentage srv6;

	srv6.request.movement = 80.00;

	ros::Rate r(3);
	while (ros::ok())
	{

		srv6.request.movement = 100 - srv6.request.movement;

		client.call(srv);
		ROS_INFO("current position: %f",srv.response.motorPosition);
		client1.call(srv1);
		ROS_INFO("direction: %u",srv1.response.direction.open_close);
		client2.call(srv2);
		ROS_INFO("frequency: %u",srv2.response.frequency);
		client3.call(srv3);
		ROS_INFO("open? %u",srv3.response.open);
		client4.call(srv4);
		ROS_INFO("mod: %u",srv4.response.mod);

		client6.call(srv6);
		//client6.call(srv6);
		ros::spinOnce();

		r.sleep();
	}

	return 0;
}
