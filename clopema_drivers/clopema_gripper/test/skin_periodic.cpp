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

#include <rosskin.h>
#include <iostream>
#include <signal.h>

static volatile sig_atomic_t _must_exit = 0;

void sighandler(int signum)
{
	_must_exit = 1;
}

static void _got_values(clopema_gripper::Connector &con, void *d)
{
	skin_sensor_response response;
	urt_time receive_time;

	response = con.skin.sensors()[0].get_response(&receive_time);
	std::cout << "Sensor 0: " << response << " @" << receive_time << std::endl;
}

int main(int argc, char **argv)
{
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = sighandler;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGQUIT, &sa, NULL);
	sigaction(SIGUSR1, &sa, NULL);
	sigaction(SIGUSR2, &sa, NULL);

	ros::init(argc, argv, "test_periodic", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	ros::NodeHandle n;

	clopema_gripper::Connector con;
	con.load(n, true, _got_values, NULL);

	std::cout << "Getting values" << std::endl;
	for (int i = 0; i < 5 && !_must_exit; ++i)
	{
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	if (!_must_exit)
	{
		std::cout << "Pausing" << std::endl;
		con.pause();
	}
	if (!_must_exit)
		ros::Duration(3.0).sleep();
	if (!_must_exit)
	{
		std::cout << "Resuming" << std::endl;
		con.resume();
	}

	ros::Rate r(100);
	while (ros::ok() && !_must_exit)
	{
		ros::spinOnce();
		r.sleep();
	}

	con.unload();
	ros::shutdown();
	return 0;
}
