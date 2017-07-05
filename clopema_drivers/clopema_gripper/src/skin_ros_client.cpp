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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <clopema_gripper/SkinInit.h>
#include <clopema_gripper/SkinRead.h>
#include <clopema_gripper/SkinData.h>
#include <clopema_gripper_rosskin.h>
#include <iostream>

static void _fill_data(const clopema_gripper::SkinData &d, clopema_gripper::Connector &con)
{
	skin_sensor_size s_count = con.skin.p_sensors_count;

	if (d.sensor_responses.size() != s_count)
	{
		ROS_WARN("rosskin client: mismatch in sensors count between init and data");
		if (d.sensor_responses.size() < s_count)
			s_count = d.sensor_responses.size();
	}
	if (d.sensor_responses.size() != d.sensor_receive_times.size())
	{
		ROS_WARN("rosskin client: mismatch in sensors count between data's responses and receive times");
		if (d.sensor_receive_times.size() < s_count)
			s_count = d.sensor_receive_times.size();
	}

	for (skin_sensor_id i = 0; i < s_count; ++i)
	{
		con.skin.p_sensors[i].response = d.sensor_responses[i];
		con.skin.p_sensors[i].receive_time = d.sensor_receive_times[i];
	}
}

class just_for_ros_callback
{
public:
	clopema_gripper::Connector *con;
	void receive(const clopema_gripper::SkinData::ConstPtr &d);
};

void just_for_ros_callback::receive(const clopema_gripper::SkinData::ConstPtr &d)
{
	_fill_data(*d, *con);
	if (con->callback)
		con->callback(*con, con->user_data);
}

int clopema_gripper::Connector::load(ros::NodeHandle &n, bool periodic, clopema_gripper::Connector::callback_func cb, void *data)
{
	ros::ServiceClient srv = n.serviceClient<clopema_gripper::SkinInit>("clopema_gripper/SkinInit");
	clopema_gripper::SkinInit s;

	/* unload the skin and unsubscribe in case of double load */
	unload();

	if (!srv.call(s))
	{
		ROS_ERROR("rosskin client: could not contact rosskin server");
		return -1;
	}

	loaded = true;

	skin.p_sensor_types_count = s.response.sensor_types_count;
	skin.p_sensors_count = s.response.sensors_count;
	skin.p_sensor_neighbors_count = s.response.sensor_neighbors_count;

	skin.p_sensor_types = (skin_sensor_type *)malloc(skin.p_sensor_types_count * sizeof(*skin.p_sensor_types));
	skin.p_sensors = (skin_sensor *)malloc(skin.p_sensors_count * sizeof(*skin.p_sensors));
	if (skin.p_sensor_neighbors_count > 0)
		skin.p_sensor_neighbors = (skin_sensor_id *)malloc(skin.p_sensor_neighbors_count * sizeof(*skin.p_sensor_neighbors));

	if ((skin.p_sensor_neighbors_count > 0 && skin.p_sensor_neighbors == NULL) || skin.p_sensor_types == NULL || skin.p_sensors == NULL)
	{
		ROS_ERROR("rosskin client: out of memory");
		unload();
		return -1;
	}

	/* packed helper arrays */
	for (uint32_t i = 0, size = s.response.sensor_neighbors.size(); i < size; ++i)
		skin.p_sensor_neighbors[i] = s.response.sensor_neighbors[i];

	/* sensor types */
	for (skin_sensor_type_id i = 0, size = skin.p_sensor_types_count; i < size; ++i)
	{
		skin_sensor_type *st = &skin.p_sensor_types[i];
		st->id = i;
		st->sensors_begin = s.response.sensor_type_sensors_begins[i];
		st->sensors_end = s.response.sensor_type_sensors_ends[i];

		for (skin_sensor_id j = st->sensors_begin; j < st->sensors_end; ++j)
			skin.p_sensors[j].type = i;
	}

	/* sensors */
	for (skin_sensor_id i = 0, size = skin.p_sensors_count; i < size; ++i)
	{
		skin_sensor *sn = &skin.p_sensors[i];
		sn->relative_position[0] = s.response.sensor_relative_positions[3 * i];
		sn->relative_position[1] = s.response.sensor_relative_positions[3 * i + 1];
		sn->relative_position[2] = s.response.sensor_relative_positions[3 * i + 2];
		sn->relative_orientation[0] = s.response.sensor_relative_orientations[3 * i];
		sn->relative_orientation[1] = s.response.sensor_relative_orientations[3 * i + 1];
		sn->relative_orientation[2] = s.response.sensor_relative_orientations[3 * i + 2];
		sn->flattened_position[0] = s.response.sensor_flattened_positions[2 * i];
		sn->flattened_position[1] = s.response.sensor_flattened_positions[2 * i + 1];
		sn->radius = s.response.sensor_radii[i];
		sn->neighbors = skin.p_sensor_neighbors + s.response.sensor_neighbors_begins[i];
		sn->neighbors_count = s.response.sensor_neighbors_counts[i];
		sn->robot_link = s.response.sensor_robot_links[i];

		sn->id = i;
		sn->p_object = &skin;
	}

	/* start the acquisition */
	if (!periodic)
		req_srv = n.serviceClient<clopema_gripper::SkinRead>("clopema_gripper/SkinDataSporadic");
	else
	{
		internal = new just_for_ros_callback;
		if (internal == NULL)
		{
			ROS_ERROR("rosskin client: out of memory");
			unload();
			return -1;
		}
		((just_for_ros_callback *)internal)->con = this;
		callback = cb;
		user_data = data;
		sub = n.subscribe("clopema_gripper/SkinDataPeriodic", 1, &just_for_ros_callback::receive, (just_for_ros_callback *)internal);
	}

	ROS_INFO("rosskin client: up and running");

	return 0;
}

void clopema_gripper::Connector::unload()
{
	if (!loaded)
		return;

	if (ros::isShuttingDown())
		std::cerr << "rosskin client: I cannot guarantee a clean exit if ROS is shutting down." << std::endl
			  << "              Make sure you handle the signal yourself and call unload() before ros::shutdown()" << std::endl;
	skin.unload();
	cmd_srv = ros::ServiceClient();
	sub = ros::Subscriber();
	req_srv = ros::ServiceClient();
	if (internal)
		delete (just_for_ros_callback *)internal;
	callback = NULL;
	user_data = NULL;
	internal = NULL;
	loaded = false;
}

int clopema_gripper::Connector::request()
{
	clopema_gripper::SkinRead r;

	if (!req_srv.call(r))
	{
		ROS_WARN("rosskin client: could not request acquisition");
		return -1;
	}

	_fill_data(r.response.d, *this);

	return 0;
}
