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

#ifndef CLOPEMA_GRIPPER_MOTOR_H
#define CLOPEMA_GRIPPER_MOTOR_H

#include <urt.h>
#include <clopema_gripper_driver.h>

enum motor_limb
{
	MOTOR_LIMB_LEFT = 0,
	MOTOR_LIMB_RIGHT = 1,
};

#define MOTOR_STATUS_OPENING 0x01
#define MOTOR_STATUS_CLOSING 0x20
#define MOTOR_STATUS_POS_CALIB 0x02
#define MOTOR_STATUS_FORCE_CALIB 0x04
#define MOTOR_STATUS_AMBPROX_CALIB 0x08
#define MOTOR_STATUS_CALIB_END 0x10

class gripper_motor
{
	public:
		gripper_motor(motor_limb l, const char *shmem_name = NULL, const char *shmutex_name = NULL);
		~gripper_motor();

		int32_t execute(motor_cmd_calib cmd);
		int32_t execute(motor_cmd_mov cmd, uint16_t data);
		int32_t execute(motor_cmd_mov_per cmd, float data);
		int32_t execute(motor_cmd_mov_bulk cmd);
		int32_t execute(motor_cmd_set cmd, uint16_t data);
		int32_t execute(motor_cmd_get cmd);

		float last_position(uint16_t *in_steps = NULL);
		uint8_t last_status();
	private:
		motor_limb limb;
		int32_t send(uint8_t cmd, uint16_t payload);

		struct motor_shm_data *motor_data;
		urt_mutex *motor_data_mutex;
};

#endif
