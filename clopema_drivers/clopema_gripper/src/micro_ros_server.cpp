#include <clopema_gripper_micro.h>
#include <ros/ros.h>
#include <clopema_gripper/StartMicro.h>
#include <clopema_gripper/StopMicro.h>
#include <audio_common_msgs/AudioData.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>

#define PUBLISH_FREQUENCY 30

static gripper_micro *bus = NULL;
uint32_t last_seq = 0xFFFFFFFF;

bool startMicro(clopema_gripper::StartMicro::Request &req, clopema_gripper::StartMicro::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MICROPHONE_CMD_START);
	last_seq = 0xFFFFFFFF;
	return true;
}

bool stopMicro(clopema_gripper::StopMicro::Request &req, clopema_gripper::StopMicro::Response &res)
{
	if (!bus)
		return false;
	bus->execute(MICROPHONE_CMD_STOP);
	return true;
}

void publish_messages(ros::Publisher &pubData)
{
	audio_common_msgs::AudioData frame;
	uint8_t d[MAX_AUDIO_FRAME_LENGTH];
	uint32_t seq;

	if (bus->next_data(seq, d))
		return;

	if (last_seq == 0xFFFFFFFF)
		last_seq = seq;
	else if (((last_seq + 1) & 0x00FFFFFF) != seq)
		ROS_WARN("Some data seem to have been skipped (last seq: %u, current seq: %u)", last_seq, seq);

	last_seq = seq;
	frame.data.insert(frame.data.begin(), d, d + MAX_AUDIO_FRAME_LENGTH);

	pubData.publish(frame);
}

void print_help(const char *prog)
{
	std::cout << "Usage: " << prog << " [--help] [--limb L]" << std::endl;
	std::cout << std::endl;
	std::cout << "    --limb L        The limb this micro server controls" << std::endl;
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

	bus = new gripper_micro(limb == 1?MICRO_LIMB_RIGHT:MICRO_LIMB_LEFT);

	ros::init(argc, argv, name + "_micro");
	ROS_INFO("Starting");
	ros::NodeHandle node;

	ros::ServiceServer serviceStartMicro = node.advertiseService(name + "/StartMicro", startMicro);
	ros::ServiceServer serviceStopMicro = node.advertiseService(name + "/StopMicro", stopMicro);
	ros::Publisher pubData = node.advertise<audio_common_msgs::AudioData>(name + "/MicroData", 1000);

	bus->start();

	ROS_INFO("Starting to spin");

	ros::Rate publish_rate(PUBLISH_FREQUENCY);

	while(ros::ok())
	{
		ros::spinOnce();

		publish_messages(pubData);

		publish_rate.sleep();
	}

	delete bus;

	urt_exit();

	return 0;
}
