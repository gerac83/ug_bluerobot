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

#ifndef CLOPEMA_GRIPPER_ROSSKIN_H
#define CLOPEMA_GRIPPER_ROSSKIN_H

#include <ros/ros.h>
#include <clopema_gripper_skin.h>

namespace clopema_gripper
{
class Connector
{
public:
	/* callback to be called if periodic. */
	typedef void (*callback_func)(clopema_gripper::Connector &, void *);

	skin_object skin;

	/*
	 * load the skin and start acquisition.
	 *
	 * If periodic is false, then the read is sporadic.  If not, then asks the server for
	 * a publisher that publishes data as soon as it gets them.
	 *
	 * If periodic, a callback would be registered to be called everytime a message is
	 * received.  This callback is passed this object (to access `skin`) as well as
	 * a user defined `data` pointer.
	 *
	 * NOTE: if you wait for ROS to shutdown with a signal, then it would not be possible
	 * to cleanly detach from the server in `unload` until ROS implements an on_shutdown
	 * callback.  Therefore, make sure you handle the signals yourself and call `unload`
	 * before ros::shutdown()
	 */
	int load(ros::NodeHandle &n, bool periodic = false, callback_func cb = NULL, void *data = NULL);
	void unload();

	/*
	 * request a sporadic read
	 *
	 * Works only if loaded in sporadic mode.  In this case, the function blocks until
	 * a response is received (or failed).  Once done, the `skin` can be accessed to
	 * retrieve the sensor values.
	 */
	int request();

	Connector(): callback(NULL), user_data(NULL), loaded(false), internal(NULL) {}
	~Connector() { unload(); }

	callback_func callback;
	void *user_data;
private:
	bool loaded;
	ros::ServiceClient cmd_srv;
	ros::Subscriber sub;
	ros::ServiceClient req_srv;
	void *internal;
};
}

#endif
