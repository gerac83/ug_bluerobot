#include <clopema_gripper_motor.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/GripperCommandAction.h>
#include <clopema_gripper/CalibratePosition.h>
#include <clopema_gripper/CalibrateForce.h>
#include <clopema_gripper/CalibrateAmbientProximity.h>
#include <clopema_gripper/GetDirection.h>
#include <clopema_gripper/GetFrequency.h>
#include <clopema_gripper/GetGripperState.h>
#include <clopema_gripper/GetMod.h>
#include <clopema_gripper/GetPosition.h>
#include <clopema_gripper/Stop.h>
#include <clopema_gripper/MoveAbsolute.h>
#include <clopema_gripper/MoveAbsolutePercentage.h>
#include <clopema_gripper/MoveAbsoluteUpdate.h>
#include <clopema_gripper/MoveAbsoluteUpdatePercentage.h>
#include <clopema_gripper/MoveRelative.h>
#include <clopema_gripper/MoveRelativePercentage.h>
#include <clopema_gripper/ForcePercentage.h>
#include <clopema_gripper/SetDirection.h>
#include <clopema_gripper/SetFrequency.h>
#include <clopema_gripper/SetMod.h>
#include <clopema_gripper/SetGripperState.h>
#include <clopema_gripper/MotorPosition.h>
#include <clopema_gripper/MotorStatus.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>

#define PUBLISH_FREQUENCY 30

static gripper_motor *bus = NULL;

class gripper_command_action_server_wrapper
{
private:
	actionlib::SimpleActionServer<control_msgs::GripperCommandAction> action_server;
public:
	gripper_command_action_server_wrapper(ros::NodeHandle &node, const std::string &name):
		action_server(node, name, boost::bind(&gripper_command_action_server_wrapper::execute, this, _1), false)
	{
		action_server.start();
	}
	~gripper_command_action_server_wrapper() {}

	void execute(const control_msgs::GripperCommandGoalConstPtr &goal);
};

bool calibratePosition(clopema_gripper::CalibratePosition::Request &req, clopema_gripper::CalibratePosition::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_CALIB_POS);
	return true;
}

bool calibrateForce(clopema_gripper::CalibrateForce::Request &req, clopema_gripper::CalibrateForce::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_CALIB_FORCE);
	return true;
}

bool calibrateAmbientProximity(clopema_gripper::CalibrateAmbientProximity::Request &req, clopema_gripper::CalibrateAmbientProximity::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_CALIB_AMB_PROX);
	return true;
}

bool getDirection(clopema_gripper::GetDirection::Request &req, clopema_gripper::GetDirection::Response &res)
{
	if (!bus)
		return false;
	res.direction.open_close = bus->execute(MOTOR_CMD_GET_DIR)?
		(uint8_t)clopema_gripper::Direction::DIR_CLOSE:
		(uint8_t)clopema_gripper::Direction::DIR_OPEN;
	return true;
}

bool getFrequency(clopema_gripper::GetFrequency::Request &req, clopema_gripper::GetFrequency::Response &res)
{
	if (!bus)
		return false;
	res.frequency = bus->execute(MOTOR_CMD_GET_FREQ);
	return true;
}

bool getGripperState(clopema_gripper::GetGripperState::Request &req, clopema_gripper::GetGripperState::Response &res)
{
	if (!bus)
		return false;
	uint16_t x = bus->execute(MOTOR_CMD_GET_POSITION);
	res.open = (x ? 0 : 1);
	return true;
}

bool getMod(clopema_gripper::GetMod::Request &req, clopema_gripper::GetMod::Response &res)
{
	if (!bus)
		return false;
	res.mod = bus->execute(MOTOR_CMD_GET_MOD);
	return true;
}

bool getPosition(clopema_gripper::GetPosition::Request &req, clopema_gripper::GetPosition::Response &res)
{
	if (!bus)
		return false;
	res.motorPosition = bus->execute(MOTOR_CMD_GET_POSITION);
	return true;
}

bool MoveAbsolute(clopema_gripper::MoveAbsolute::Request &req, clopema_gripper::MoveAbsolute::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_MOV_ABS, req.movement);
	return true;
}

bool MoveAbsolutePercentage(clopema_gripper::MoveAbsolutePercentage::Request &req, clopema_gripper::MoveAbsolutePercentage::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_MOV_ABS_PER, req.movement);
	return true;
}

bool MoveAbsoluteUpdate(clopema_gripper::MoveAbsoluteUpdate::Request &req, clopema_gripper::MoveAbsoluteUpdate::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_MOV_ABS_UPD, req.movement);
	return true;
}

bool MoveAbsoluteUpdatePercentage(clopema_gripper::MoveAbsoluteUpdatePercentage::Request &req, clopema_gripper::MoveAbsoluteUpdatePercentage::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_MOV_ABS_UPD_PER, req.movement);
	return true;
}

bool MoveRelative(clopema_gripper::MoveRelative::Request &req, clopema_gripper::MoveRelative::Response &res)
{
	if (!bus)
		return false;
	motor_cmd_mov cmd = (req.direction.open_close == clopema_gripper::Direction::DIR_OPEN?MOTOR_CMD_MOV_REL_O:MOTOR_CMD_MOV_REL_C);
	bus->execute(cmd, req.movement);
	return true;
}

bool MoveRelativePercentage(clopema_gripper::MoveRelativePercentage::Request &req, clopema_gripper::MoveRelativePercentage::Response &res)
{
	if (!bus)
		return false;
	motor_cmd_mov_per cmd = (req.direction.open_close == clopema_gripper::Direction::DIR_OPEN?MOTOR_CMD_MOV_REL_O_PER:MOTOR_CMD_MOV_REL_C_PER);
	bus->execute(cmd, req.movement);
	return true;
}

bool ForcePercentage(clopema_gripper::ForcePercentage::Request &req, clopema_gripper::ForcePercentage::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_MOV_FORCE_PER, req.force);
	return true;
}

bool setDirection(clopema_gripper::SetDirection::Request &req, clopema_gripper::SetDirection::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_SET_DIR, req.direction.open_close);
	return true;
}

bool setFrequency(clopema_gripper::SetFrequency::Request &req, clopema_gripper::SetFrequency::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_SET_FREQ, req.frequency);
	return true;
}

bool setMod(clopema_gripper::SetMod::Request &req, clopema_gripper::SetMod::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_SET_MOD, req.mod);
	return true;
}

bool setGripperState(clopema_gripper::SetGripperState::Request &req, clopema_gripper::SetGripperState::Response &res)
{
	if (!bus)
		return false;
	uint16_t x = bus->execute(MOTOR_CMD_GET_POSITION);

	res.prev = (x ? 0 : 1);

	if (req.open)
		bus->execute(MOTOR_CMD_MOV_OPEN);
	else
		bus->execute(MOTOR_CMD_MOV_CLOSE);
	return true;
}

bool stop(clopema_gripper::Stop::Request &req, clopema_gripper::Stop::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MOTOR_CMD_MOV_STOP);
	return true;
}

void gripper_command_action_server_wrapper::execute(const control_msgs::GripperCommandGoalConstPtr &goal)
{
	bool success = true;
	float prev_position = -100;
	float goal_position = goal->command.position * 100;

	if (goal_position < 0)
		goal_position = 0;
	if (goal_position > 100)
		goal_position = 100;

	/* first, send the command and calculate the expected result */
	if (goal_position < 1e-3)
		/* close command */
		bus->execute(MOTOR_CMD_MOV_CLOSE);
	else if (goal_position > 100 - 1e-3)
		/* open command */
		bus->execute(MOTOR_CMD_MOV_OPEN);
	else
		bus->execute(MOTOR_CMD_MOV_ABS_UPD_PER, goal_position);

	ros::Rate r(5);

	while (true)
	{
#define PRECISION 1
		/* check to see if reached goal or not moving anymore */
		float cur_position = bus->last_position();
		float dist_goal = cur_position - goal_position;
		float dist_prev = cur_position - prev_position;
		if ((dist_goal > -PRECISION && dist_goal < PRECISION) ||
			(dist_prev > -PRECISION && dist_prev < PRECISION
				 && !(bus->last_status() & (MOTOR_STATUS_OPENING | MOTOR_STATUS_CLOSING))))
			break;

		prev_position = cur_position;

		/* check to see if preempted */
		if (action_server.isPreemptRequested() || !ros::ok())
		{
			action_server.setPreempted();
			success = false;
			break;
		}

		/* give feedback */
		control_msgs::GripperCommandFeedback feedback;
		feedback.position = cur_position;
		feedback.effort = 0;
		feedback.stalled = false;
		feedback.reached_goal = false;

		action_server.publishFeedback(feedback);

		r.sleep();
	}

	if (success)
	{
		control_msgs::GripperCommandResult result;
		result.position = bus->last_position();
		result.effort = 0;
		result.stalled = false;
		result.reached_goal = success;

                ros::Duration(0.25).sleep(); //WAIT because gripper report close even when is still moving.
		action_server.setSucceeded(result);
	}
}

void publish_messages(ros::Publisher &pubPos, ros::Publisher &pubStat, ros::Publisher &pub_pos, unsigned int sequence, std::string &name)
{
	clopema_gripper::MotorPosition pos;

	pos.motorPosition = bus->last_position(&pos.motorPositionSteps);
	pubPos.publish(pos);

	clopema_gripper::MotorStatus stat;
	uint8_t st;

	st = bus->last_status();
	stat.opening = st & MOTOR_STATUS_OPENING;
	stat.closing = st & MOTOR_STATUS_CLOSING;
	stat.position_calibrated = st & MOTOR_STATUS_POS_CALIB;
	stat.force_calibrated = st & MOTOR_STATUS_FORCE_CALIB;
	stat.ambient_and_proximity_calibrated = st & MOTOR_STATUS_AMBPROX_CALIB;
	pubStat.publish(stat);

	sensor_msgs::JointState state;
	state.header.seq = sequence;
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	state.header.stamp.sec = ts.tv_sec;
	state.header.stamp.nsec = ts.tv_nsec;
	state.header.frame_id = "0";
	state.name.push_back(name);
	state.position.push_back(1 - pos.motorPosition / 100.0f);
	pub_pos.publish(state);
}

void print_help(const char *prog)
{
	std::cout << "Usage: " << prog << " [--help] [--limb L]" << std::endl;
	std::cout << std::endl;
	std::cout << "    --limb L        The limb this motor server controls" << std::endl;
	std::cout << "                    Give 1 for the right limb and 2 for the left one" << std::endl;
}

int parse_args(int argc, char **argv)
{
	int limb = -1;
	bool got_limb = false;
	for (int i = 1; i < argc; ++i)
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
		{
			print_help(argv[0]);
			return -1;
		}
		else if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--limb") == 0)
		{
			if (i + 1 < argc)
			{
				++i;
				if (sscanf(argv[i], "%d", &limb) != 1 || (limb != 1 && limb != 2))
					print_help(argv[0]);
				else
					got_limb = true;
			}
			else
				print_help(argv[0]);
		}
	if (!got_limb)
		print_help(argv[0]);

	return limb;
}

int main(int argc, char **argv)
{
	int limb;

	limb = parse_args(argc, argv);
	if (limb < 1)
		return 0;

	urt_init();

	std::string name;
	std::ostringstream sout;
	sout << "r" << limb << "_gripper";

	name = sout.str();

	bus = new gripper_motor(limb == 1?MOTOR_LIMB_RIGHT:MOTOR_LIMB_LEFT);

	ros::init(argc, argv, name + "_motor");
	ROS_INFO("Starting");
	ros::NodeHandle node;

	ros::ServiceServer serviceCalibPos = node.advertiseService(name + "/CalibratePosition", calibratePosition);
	ros::ServiceServer serviceCalibForce = node.advertiseService(name + "/CalibrateForce", calibrateForce);
	ros::ServiceServer serviceCalibProx = node.advertiseService(name + "/CalibrateAmbientProximity", calibrateAmbientProximity);
	ros::ServiceServer serviceGetDir = node.advertiseService(name + "/GetDirection", getDirection);
	ros::ServiceServer serviceGetFreq = node.advertiseService(name + "/GetFrequency", getFrequency);
	ros::ServiceServer serviceGetGrSt = node.advertiseService(name + "/GetGripperState", getGripperState);
	ros::ServiceServer serviceGetMod = node.advertiseService(name + "/GetMod", getMod);
	ros::ServiceServer serviceGetPos = node.advertiseService(name + "/GetPosition", getPosition);
	ros::ServiceServer serviceMovAbs = node.advertiseService(name + "/MoveAbsolute", MoveAbsolute);
	ros::ServiceServer serviceMovAbsP = node.advertiseService(name + "/MoveAbsolutePercentage", MoveAbsolutePercentage);
	ros::ServiceServer serviceMovAbsU = node.advertiseService(name + "/MoveAbsoluteUpdate", MoveAbsoluteUpdate);
	ros::ServiceServer serviceMovAbsUP = node.advertiseService(name + "/MoveAbsoluteUpdatePercentage", MoveAbsoluteUpdatePercentage);
	ros::ServiceServer serviceMovRel = node.advertiseService(name + "/MoveRelative", MoveRelative);
	ros::ServiceServer serviceMovRelP = node.advertiseService(name + "/MoveRelativePercentage", MoveRelativePercentage);
	ros::ServiceServer serviceForceP = node.advertiseService(name + "/ForcePercentage", ForcePercentage);
	ros::ServiceServer serviceSetDir = node.advertiseService(name + "/SetDirection", setDirection);
	ros::ServiceServer serviceSetFreq = node.advertiseService(name + "/SetFrequency", setFrequency);
	ros::ServiceServer serviceSetMod = node.advertiseService(name + "/SetMod", setMod);
	ros::ServiceServer serviceSetGrSt = node.advertiseService(name + "/SetGripperState", setGripperState);
	ros::ServiceServer serviceStop = node.advertiseService(name + "/Stop", stop);
	ros::Publisher pubPos = node.advertise<clopema_gripper::MotorPosition>(name + "/MotorPosition", 1000);
	ros::Publisher pubStat = node.advertise<clopema_gripper::MotorStatus>(name + "/MotorStatus", 1000);

	std::ostringstream jout;
	jout << "r" << limb << "_joint_grip";
	ros::Publisher pub_pos = node.advertise<sensor_msgs::JointState>("/joint_states", 1000);
	std::string joint_state = jout.str();

	gripper_command_action_server_wrapper *asw = new gripper_command_action_server_wrapper(node, name + "/command");

	clopema_gripper::MotorPosition msg;

	ROS_INFO("Starting to spin");

	ros::Rate publish_rate(PUBLISH_FREQUENCY);

	unsigned int seq = 0;
	while(ros::ok())
	{
		ros::spinOnce();

		publish_messages(pubPos, pubStat, pub_pos, seq, joint_state);

		publish_rate.sleep();
	}

	delete asw;
	delete bus;

	urt_exit();

	return 0;
}
