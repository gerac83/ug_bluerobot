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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_sporadic", ros::init_options::AnonymousName);
	ros::NodeHandle n;

	clopema_gripper::Connector con;
	con.load(n);

	con.request();
	std::cout << "Sensor 0: " << con.skin.sensors()[0].get_response() << std::endl;

	return 0;
}
