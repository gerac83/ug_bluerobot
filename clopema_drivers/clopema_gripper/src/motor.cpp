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
#include "clopema_gripper_motor.h"

gripper_motor::gripper_motor(motor_limb l, const char *shmem_name, const char *shmutex_name): limb(l)
{
	motor_data = (struct motor_shm_data *)urt_shmem_attach(shmem_name?shmem_name:MOTOR_SHMEM);
	motor_data_mutex = urt_shmutex_attach(shmutex_name?shmutex_name:MOTOR_SHMUTEX);

	if (motor_data == NULL || motor_data_mutex == NULL)
	{
		urt_shmem_detach(motor_data);
		urt_shmutex_detach(motor_data_mutex);
		motor_data = NULL;
		motor_data_mutex = NULL;
		urt_err("failed to attach to driver; is it running?\n");
		exit(EXIT_FAILURE);
	}
}

gripper_motor::~gripper_motor()
{
	urt_shmem_detach(motor_data);
	urt_shmutex_detach(motor_data_mutex);
}

int32_t gripper_motor::execute(motor_cmd_calib cmd)
{
	return send(cmd, 0);
}

int32_t gripper_motor::execute(motor_cmd_mov cmd, uint16_t data)
{
	return send(cmd, data);
}

int32_t gripper_motor::execute(motor_cmd_mov_per cmd, float data)
{
	return send(cmd, static_cast<uint16_t>(data*100));
}

int32_t gripper_motor::execute(motor_cmd_mov_bulk cmd)
{
	return send(cmd, 0);
}

int32_t gripper_motor::execute(motor_cmd_set cmd, uint16_t data)
{
	return send(cmd, data);
}

int32_t gripper_motor::execute(motor_cmd_get cmd)
{
	int timeout = 100;
	int ret;
	struct motor_shm_data *md = &motor_data[limb];

	if (send(cmd, 0))
		return -1;

	/* wait for response */
	while (!md->request_serviced && --timeout)
		usleep(10000);

	if (timeout <= 0)
		return -1;

	urt_mutex_lock(motor_data_mutex);

	switch (cmd)
	{
	case MOTOR_CMD_GET_FREQ:
		ret = md->frequency;
		break;
	case MOTOR_CMD_GET_DIR:
		ret = md->direction;
		break;
	case MOTOR_CMD_GET_MOD:
		ret = md->modality;
		break;
	case MOTOR_CMD_GET_POSITION:
		ret = md->position;
		break;
	default:
		break;
	}

	md->request_serviced = false;

	urt_mutex_unlock(motor_data_mutex);

	return ret;
}

float gripper_motor::last_position(uint16_t *in_steps)
{
	int32_t last_position;
	struct motor_shm_data *md = &motor_data[limb];

	urt_mutex_lock(motor_data_mutex);
	last_position = md->position;
	if (in_steps)
		*in_steps = md->position_steps;
	urt_mutex_unlock(motor_data_mutex);

	return last_position / 100.0f;
}

uint8_t gripper_motor::last_status()
{
	uint8_t last_status;
	struct motor_shm_data *md = &motor_data[limb];

	urt_mutex_lock(motor_data_mutex);
	last_status = md->status;
	urt_mutex_unlock(motor_data_mutex);

	return last_status;
}

int32_t gripper_motor::send(uint8_t cmd, uint16_t payload)
{
	int timeout = 100;
	struct motor_shm_data *md = &motor_data[limb];

	/* wait for previous message to be sent */
	while (md->new_command && --timeout)
		usleep(10000);

	if (timeout <= 0)
		return -1;

	urt_mutex_lock(motor_data_mutex);
	md->request_serviced = false;
	md->command = cmd;
	md->payload = payload;
	md->new_command = true;
	urt_mutex_unlock(motor_data_mutex);

	return 0;
}
