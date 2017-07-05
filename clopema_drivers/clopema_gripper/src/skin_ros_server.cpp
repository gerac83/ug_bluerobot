/*
 * Copyright (C) 2013-2014  Maclab, Università di Genova
 *
 * This file is part of CloPeMa Gripper Module.
 *
 * CloPeMa Gripper Module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * CloPeMa Gripper Module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CloPeMa Gripper Module.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <clopema_gripper/SkinInit.h>
#include <clopema_gripper/SkinRead.h>
#include <clopema_gripper/SkinData.h>
#include <clopema_gripper_skin.h>
#include <list>
#ifndef NDEBUG
# include <cstdio>
#endif
#include <signal.h>
#include <string>

static volatile sig_atomic_t _must_exit = 0;

void sighandler(int signum)
{
	_must_exit = 1;
}

class server
{
private:
	/* skin */
	skin_object skin;
	urt_sem *sem;

	/* data transfers */
	ros::Publisher pub;
	ros::ServiceServer srv;

	bool init_responder(clopema_gripper::SkinInit::Request &req, clopema_gripper::SkinInit::Response &res);
	bool serve(clopema_gripper::SkinRead::Request &req, clopema_gripper::SkinRead::Response &res);

	void fill_data(clopema_gripper::SkinData &d);

public:
	ros::ServiceServer init_srv;

	int startup(std::string calibration_file, bool fixed_file, const char *shmem_name, const char *mutex_name);
	void publish();

	server(ros::NodeHandle *n);
	~server();
};

server::server(ros::NodeHandle *n)
{
	urt_init();
	sem = urt_sem_new(1);
	if (sem == NULL)
		ROS_WARN("failed to acquire real-time semaphore.  Reading unsynchronized");
	init_srv = n->advertiseService("clopema_gripper/SkinInit", &server::init_responder, this);
	srv = n->advertiseService("clopema_gripper/SkinDataSporadic", &server::serve, this);
	pub = n->advertise<clopema_gripper::SkinData>("clopema_gripper/SkinDataPeriodic", 10);
}

server::~server()
{
	urt_sem_delete(sem);
}

int server::startup(std::string calibration_file, bool fixed_file, const char *shmem_name, const char *mutex_name)
{
	std::string cf = calibration_file + (fixed_file?"":"clbr");
	int ret = skin.load(cf.c_str(), shmem_name, mutex_name);
	if (ret && ret != SKIN_NO_MEM && !fixed_file)
		ret = skin.load((calibration_file + "clbr-2").c_str(), shmem_name, mutex_name);
	switch (ret)
	{
	case SKIN_NO_MEM:
		ROS_ERROR("Failed to initialize server (out of memory)");
		break;
	case SKIN_NO_FILE:
		ROS_ERROR("Failed to initialize server (missing cailbration file)");
		break;
	case SKIN_FILE_INVALID:
		ROS_ERROR("Failed to initialize server (invalid calibration file)");
		break;
	case SKIN_SUCCESS:
		if (sem)
			if (skin.reader()->register_user(sem) != SKIN_SUCCESS)
			{
				ROS_ERROR("Failed to register for notifications.  Reading without synchronization");
				urt_sem_delete(sem);
				sem = NULL;
			}
		ret = skin.reader()->start();
		if (ret != SKIN_SUCCESS)
		{
			ROS_ERROR("rosskin server: could not start reader");
			return false;
		}
		ROS_INFO("rosskin server: started acquisition");
		return 0;
	default:
		ROS_ERROR("Failed to initialize server (error code: %d) (did you start the driver?)", ret);
		break;
	}

	return -1;
}

bool server::init_responder(clopema_gripper::SkinInit::Request &req, clopema_gripper::SkinInit::Response &res)
{
	res.sensor_types_count = skin.p_sensor_types_count;
	res.sensors_count = skin.p_sensors_count;
	res.sensor_neighbors_count = skin.p_sensor_neighbors_count;

	/* packed helper arrays */
	res.sensor_neighbors.reserve(skin.p_sensor_neighbors_count);
	for (uint32_t i = 0, size = skin.p_sensor_neighbors_count; i < size; ++i)
		res.sensor_neighbors.push_back(skin.p_sensor_neighbors[i]);

	/* sensor types */
	for (uint32_t i = 0, size = skin.p_sensor_types_count; i < size; ++i)
	{
		res.sensor_type_sensors_begins.push_back(skin.p_sensor_types[i].sensors_begin);
		res.sensor_type_sensors_ends.push_back(skin.p_sensor_types[i].sensors_end);
	}

	/* sensors */
	res.sensor_relative_positions.reserve(skin.p_sensors_count * 3);
	res.sensor_relative_orientations.reserve(skin.p_sensors_count * 3);
	res.sensor_flattened_positions.reserve(skin.p_sensors_count * 2);
	res.sensor_neighbors_begins.reserve(skin.p_sensors_count);
	res.sensor_neighbors_counts.reserve(skin.p_sensors_count);
	res.sensor_robot_links.reserve(skin.p_sensors_count);
	for (skin_sensor_id i = 0, size = skin.p_sensors_count; i < size; ++i)
	{
		skin_sensor *s = &skin.p_sensors[i];
		res.sensor_relative_positions.push_back(s->relative_position[0]);
		res.sensor_relative_positions.push_back(s->relative_position[1]);
		res.sensor_relative_positions.push_back(s->relative_position[2]);
		res.sensor_relative_orientations.push_back(s->relative_orientation[0]);
		res.sensor_relative_orientations.push_back(s->relative_orientation[1]);
		res.sensor_relative_orientations.push_back(s->relative_orientation[2]);
		res.sensor_flattened_positions.push_back(s->flattened_position[0]);
		res.sensor_flattened_positions.push_back(s->flattened_position[1]);
		res.sensor_radii.push_back(s->radius);
		res.sensor_neighbors_begins.push_back(s->neighbors - skin.p_sensor_neighbors);
		res.sensor_neighbors_counts.push_back(s->neighbors_count);
		res.sensor_robot_links.push_back(s->robot_link);
	}

	return true;
}

void server::publish()
{
	clopema_gripper::SkinData d;

#define MIN_DELAY 15000000
	int ret;
	if (sem == NULL || (ret = skin.reader()->wait_read(sem, MIN_DELAY)) == SKIN_SUCCESS)
	{
		if (_must_exit || !ros::ok())
			return;
		fill_data(d);
		pub.publish(d);
	}
	if (sem == NULL || ret != SKIN_SUCCESS)
		ros::Duration(MIN_DELAY / 1000000000.0).sleep();
#undef MIN_DELAY
}

bool server::serve(clopema_gripper::SkinRead::Request &req, clopema_gripper::SkinRead::Response &res)
{
	fill_data(res.d);
	return true;
}

void server::fill_data(clopema_gripper::SkinData &d)
{
	d.sensor_responses.reserve(skin.p_sensors_count);
	d.sensor_receive_times.reserve(skin.p_sensors_count);

	for (skin_sensor_id i = 0, size = skin.p_sensors_count; i < size; ++i)
	{
		skin_sensor_response response;
		urt_time receive_time;

		response = skin.p_sensors[i].get_response(&receive_time);
		d.sensor_responses.push_back(response);
		d.sensor_receive_times.push_back(receive_time);
	}
}

int main(int argc, char **argv)
{
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = sighandler;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGSEGV, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGQUIT, &sa, NULL);
	sigaction(SIGUSR1, &sa, NULL);
	sigaction(SIGUSR2, &sa, NULL);

	std::string calibration_file;
	bool calibration_file_given = false;
	const char *home_env = getenv("HOME");
	std::string home;
	if (home_env == NULL)
		home = ".";
	else
		home = std::string(home_env) + "/.clopema_view";

	calibration_file = home + "/data/";

	const char *shmem_name = NULL, *mutex_name = NULL;

	for (int i = 1; i < argc; ++i)
	{
		if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
		{
			std::cout << "Usage: " << argv[0] << " [--help] [--calibrate file] [--shmem name] [--mutex name]" << std::endl << std::endl;
			return 0;
		}
		else if ((strcmp(argv[i], "--calibrate") == 0 || strcmp(argv[i], "-c") == 0) && i + 1 != argc)
		{
			calibration_file_given = true;
			calibration_file = argv[++i];
		}
		else if ((strcmp(argv[i], "--shmem") == 0 || strcmp(argv[i], "-s") == 0) && i + 1 != argc)
			shmem_name = argv[++i];
		else if ((strcmp(argv[i], "--mutex") == 0 || strcmp(argv[i], "-m") == 0) && i + 1 != argc)
			mutex_name = argv[++i];
	}

	ros::init(argc, argv, "rosskin_server");
	if (!ros::ok())
		return EXIT_FAILURE;

	ros::NodeHandle n;
	server *s = new server(&n);
	if (s->startup(calibration_file, calibration_file_given, shmem_name, mutex_name))
		return EXIT_FAILURE;

	ROS_INFO("rosskin server: up and running");

	while (!_must_exit && ros::ok())
	{
		s->publish();	/* publish has a timed wait, so this is not a busy loop */
		ros::spinOnce();
	}

	delete s;

	if (urt_is_rt_context())
		urt_exit();

	return 0;
}
