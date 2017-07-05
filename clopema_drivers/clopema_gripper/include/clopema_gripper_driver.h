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

#ifndef CLOPEMA_GRIPPER_DRIVER_H
#define CLOPEMA_GRIPPER_DRIVER_H

#define				SKIN_SHMEM			"SKNSHM"
#define				SKIN_SHRWL			"SKNRWL"
#define				GRIPPER_SHMEM			"GRPSHM"
#define				MOTOR_SHMEM			"MTRSHM"
#define				MOTOR_SHMUTEX			"MTRMTX"
#define				MICRO_SHMEM			"MICSHM"
#define				MICRO_SHMUTEX			"MICMTX"

#define				MAX_MODULES			16
#define				MAX_SENSORS_PER_MODULE		8
#define				MAX_AUDIO_FRAME_LENGTH		1280
#define				MAX_AUDIO_FRAME_BUFFER		10

/* shared memory containing data regarding tactile sensors */
struct skin_shm_data
{
	long long		received[MAX_MODULES * MAX_SENSORS_PER_MODULE];		/* time of last acquisition of taxel */
	uint16_t		responses[MAX_MODULES * MAX_SENSORS_PER_MODULE];	/* data itself */
	uint16_t		taxels_count;
	bool			users_must_stop;		/* whether user program must stop */
	bool			initializing;			/* whether the driver is still initializing */
	long long		last_update;			/* time of last update to the data */
};

struct microphone_shm_data
{
	/* data to be sent */
	uint8_t			command;			/* command to be sent */
	bool			new_command;			/* whether there is new command to send */

	/* retrieved data */
	struct microphone_shm_data_buffer
	{
		uint32_t		sequence_number;
		uint8_t			data[MAX_AUDIO_FRAME_LENGTH];
		long long		last_update;
	} buffers[MAX_AUDIO_FRAME_BUFFER];
};

enum microphone_cmd
{
	MICROPHONE_CMD_START		= 0x20,
	MICROPHONE_CMD_STOP		= 0x21,
};

/* shared memory containing data regarding gripper states */
struct gripper_shm_data
{
	uint16_t		proximity;			/* measured value of proximity sensor */
	uint16_t		light;				/* measured value of light sensor */
	uint16_t		magnetic_encoder;		/* measured position reported by magnetic encoder */
	uint16_t		holding_cloth;			/* whether cloth is gripped */
	long long		last_update;			/* time of last update to the data */
};

enum motor_cmd_calib
{
	MOTOR_CMD_CALIB_POS		= 0x0E,
	MOTOR_CMD_CALIB_FORCE		= 0x11,
	MOTOR_CMD_CALIB_AMB_PROX	= 0x12,

	/* not an actual motor command, but there to fix driver state in case something went wrong */
	MOTOR_CMD_CALIB_FORCE_RESET	= 0xFF,
};

enum motor_cmd_mov
{
	MOTOR_CMD_MOV_ABS		= 0x01,
	MOTOR_CMD_MOV_REL_O		= 0x03,
	MOTOR_CMD_MOV_REL_C		= 0x04,
	MOTOR_CMD_MOV_ABS_UPD		= 0x16,
};

enum motor_cmd_mov_per
{
	MOTOR_CMD_MOV_ABS_PER		= 0x02,
	MOTOR_CMD_MOV_REL_O_PER		= 0x05,
	MOTOR_CMD_MOV_REL_C_PER		= 0x06,
	MOTOR_CMD_MOV_ABS_UPD_PER	= 0x17,
	MOTOR_CMD_MOV_FORCE_PER		= 0x15,
};

enum motor_cmd_mov_bulk
{
	MOTOR_CMD_MOV_STOP		= 0x14,
	MOTOR_CMD_MOV_OPEN		= 0x0F,
	MOTOR_CMD_MOV_CLOSE		= 0x13,
};

enum motor_cmd_set
{
	MOTOR_CMD_SET_FREQ		= MOTOR_CMD_MOV_REL_C_PER + 1,
	MOTOR_CMD_SET_DIR,
	MOTOR_CMD_SET_MOD
};

enum motor_cmd_get
{
	MOTOR_CMD_GET_FREQ		= MOTOR_CMD_SET_MOD + 1,
	MOTOR_CMD_GET_DIR,
	MOTOR_CMD_GET_MOD,
	MOTOR_CMD_GET_POSITION
};

/* shared memory containing data regarding gripper motor */
struct motor_shm_data
{
	/* data to be sent */
	uint8_t			command;			/* command to be sent */
	bool			new_command;			/* whether there is new command to send */
	uint16_t		payload;			/* payload of the command */

	/* retrieved data */
	uint16_t		position;			/* motor position (close = 100% = 10000, open = 0%) */
	uint16_t		position_steps;			/* specific motor steps */
	uint16_t		frequency;			/* steps per second */
	uint8_t			status;				/* status of the gripper */
	bool			direction;			/* current direction of movement */
	uint8_t			modality;			/* modality of the stepper motor */
	bool			request_serviced;		/* sets to true if any other values are updated */
};

#endif
