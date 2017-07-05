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

#include <libpcan.h>
#define URT_LOG_PREFIX "clopema gripper driver: "
#include <fcntl.h>
#include <urt.h>
#include "clopema_gripper_driver.h"

#define MODULE_ID_BITS 4
#define MODULE_COUNT (1 << MODULE_ID_BITS)
#define MAX_DELAY 20000000ll

/*
 * TO_U16 takes two bytes, first and second.  Based on whether the IHB sends little-endian or big-endian,
 * the low/high byte could be either of them.  Currently, the IHB sends little-endian for skin and big-endian for motor
 */
#define TO_U16_LE(first, second) ((second) << 8 | (first))
#define TO_U16_BE(first, second) ((first) << 8 | (second))
#define TO_U24_BE(first, second, third) ((first) << 16 | (second) << 8 | (third))

#define COMPONENT_DEBUG 0x0
#define COMPONENT_MOTOR 0x1
#define COMPONENT_SKIN 0x2
#define COMPONENT_GRIPPER 0x3
#define COMPONENT_COMMAND 0x1

#define COMPONENT(id) ((id) >> 9 & 0x3)
#define COMMAND_ID_FOR(lr) (COMPONENT_COMMAND << 9 | (lr))
#define LEFT_OR_RIGHT(id) ((id) & 1)

#define SKIN_COMPONENT(id) ((id) >> 4 & 0x1F)
#define SKIN_IS_TAXEL(id) (SKIN_COMPONENT(id) == 0)
#define SKIN_IS_MICROPHONE(id) (SKIN_COMPONENT(id) == 1)

#define MODULE_ID(id) (((id) << 2 & 0x04) | ((id) >> 2 & 0x03))
#define MODULE_PACKET(id) ((id) >> 1 & 1)

#define MICROPHONE_MESSAGE_TYPE(id) ((id) >> 2 & 0x3)
#define MICROPHONE_IS_START(id) (MICROPHONE_MESSAGE_TYPE(id) == 0)
#define MICROPHONE_IS_DATA(id) (MICROPHONE_MESSAGE_TYPE(id) == 1)

URT_MODULE_LICENSE("GPL");
URT_MODULE_AUTHOR("Shahbaz Youssefi");

static int device = -1;
static int frequency = 40;
static int net_scan_tries = 10;
static char skin_shmem_name[URT_NAME_LEN + 1] = SKIN_SHMEM;
static char skin_shrwl_name[URT_NAME_LEN + 1] = SKIN_SHRWL;
static char motor_shmem_name[URT_NAME_LEN + 1] = MOTOR_SHMEM;
static char motor_shmutex_name[URT_NAME_LEN + 1] = MOTOR_SHMUTEX;
static char gripper_shmem_name[URT_NAME_LEN + 1] = GRIPPER_SHMEM;
static char micro_shmem_name[URT_NAME_LEN + 1] = MICRO_SHMEM;
static char micro_shmutex_name[URT_NAME_LEN + 1] = MICRO_SHMUTEX;
URT_MODULE_PARAM_START()
URT_MODULE_PARAM(device, int, "The index of pcan PCI card (0, 1, 2, ...).  Set to -1 for autodetect.\n"
		"\t\t\tDefault value is -1")
URT_MODULE_PARAM(frequency, int, "The frequency of data acquisition.\n"
		"\t\t\tDefault value is 25 (Hz)")
URT_MODULE_PARAM(net_scan_tries, int, "The number of tries to scan the network and detect the modules.\n"
		"\t\t\tDefault value is 10")
URT_MODULE_PARAM_ARRAY(skin_shmem_name, char, NULL, "URT name for skin shared memory.  Default value is \"SKNSHM\"")
URT_MODULE_PARAM_ARRAY(skin_shrwl_name, char, NULL, "URT name for skin shared rwlock.  Default value is \"SKINRWL\"")
URT_MODULE_PARAM_ARRAY(motor_shmem_name, char, NULL, "URT name for motor shared memory.  Default value is \"MTRSHM\"")
URT_MODULE_PARAM_ARRAY(motor_shmutex_name, char, NULL, "URT name for motor shared mutex.  Default value is \"MTRMTX\"")
URT_MODULE_PARAM_ARRAY(gripper_shmem_name, char, NULL, "URT name for gripper shared memory.  Default value is \"GRPSHM\"")
URT_MODULE_PARAM_ARRAY(micro_shmem_name, char, NULL, "URT name for micro shared memory.  Default value is \"MICSHM\"")
URT_MODULE_PARAM_ARRAY(micro_shmutex_name, char, NULL, "URT name for micro shared mutex.  Default value is \"MICMTX\"")
URT_MODULE_PARAM_END()

struct ihb_data
{
	HANDLE can_handle;
	bool writer_running, can_running;
	volatile sig_atomic_t stop;

	/* motor data */
	struct motor_shm_data *motor_data;
	urt_mutex *motor_data_mutex;
	bool calibrating[2];

	/* skin data */
	struct skin_shm_data *taxel_data;
	urt_rwlock *data_rwl;
	urt_mutex *skin_data_mutex;
	uint16_t module_map[MODULE_COUNT];	/*
						 * module id received from hardware (16 bits) is used as index to this array
						 * the value it contains would be id in taxel_responses in skin_shm_data
						 */
	uint16_t module_raw_data[MODULE_COUNT * MAX_SENSORS_PER_MODULE];	/* data received by can monitor for the skin */
	bool module_raw_data_dirty;

	/* microphone data */
	struct microphone_shm_data *micro_data;
	urt_mutex *micro_data_mutex;
	urt_mutex *micro_data_shmutex;
	uint8_t micro_raw_data[2 * MAX_AUDIO_FRAME_LENGTH];			/* data received by can monitor for the microphone */
	uint32_t micro_raw_data_sequence_number[2];
	uint32_t micro_raw_data_cur_index[2];
	bool micro_raw_data_dirty[2];

	/* gripper data */
	struct gripper_shm_data *gripper_data;
	urt_mutex *gripper_data_mutex;
	uint16_t gripper_raw_data[8];		/* data received by can monitor for the gripper */
	bool gripper_raw_data_dirty[2];

	/* book-keeping */
	urt_task *can_task;
	urt_task *writer_task;
	bool get_stat_warning_given;
	bool no_message_warning_given;
	bool read_error_warning_given;
	int no_message_count;
};

static int can_driver_start(struct ihb_data *ihbdata);
static void can_driver_body(struct ihb_data *ihbdata);
static void can_driver_exit(struct ihb_data *ihbdata);

URT_GLUE(can_driver_start, can_driver_body, can_driver_exit, struct ihb_data, interrupted, done)

static void cleanup(struct ihb_data *ihbdata)
{
	if (ihbdata->can_handle)
		CAN_Close(ihbdata->can_handle);
	urt_shmem_delete(ihbdata->taxel_data);
	urt_shmem_delete(ihbdata->motor_data);
	urt_shmem_delete(ihbdata->gripper_data);
	urt_shmem_delete(ihbdata->micro_data);
	urt_shrwlock_delete(ihbdata->data_rwl);
	urt_shmutex_delete(ihbdata->motor_data_mutex);
	urt_shmutex_delete(ihbdata->micro_data_shmutex);
	urt_mutex_delete(ihbdata->skin_data_mutex);
	urt_mutex_delete(ihbdata->gripper_data_mutex);
	urt_mutex_delete(ihbdata->micro_data_mutex);

	urt_task_delete(ihbdata->writer_task);
	urt_task_delete(ihbdata->can_task);

	*ihbdata = (struct ihb_data){0};
}

static int get_can_pending_reads(struct ihb_data *ihbdata, const char *phase)
{
	int ret;
	int pending_read, pending_write;

	if ((ret = LINUX_CAN_Extended_Status(ihbdata->can_handle, &pending_read, &pending_write)) < 0)
	{
		if (!ihbdata->get_stat_warning_given)
			urt_err("%s: stat error %d\n", phase, ret);
		ihbdata->get_stat_warning_given = true;
		return -1;
	}

	ihbdata->get_stat_warning_given = false;
	if (pending_read == 0)
	{
		++ihbdata->no_message_count;
		if (ihbdata->no_message_count >= frequency)
		{
			if (!ihbdata->no_message_warning_given)
				urt_err("%s: not receiving any messages\n", phase);
			ihbdata->no_message_warning_given = true;
		}
		return 0;
	}
	else if (ihbdata->no_message_warning_given)
		urt_err("%s: got back connection\n", phase);

	ihbdata->no_message_count = 0;
	ihbdata->no_message_warning_given = false;

	return pending_read;
}

static int get_can_message(struct ihb_data *ihbdata, TPCANMsg *msg, const char *phase)
{
	int ret;

	if ((ret = CAN_Read(ihbdata->can_handle, msg)))
	{
		if (!ihbdata->read_error_warning_given)
			urt_err("%s: read message error %d\n", phase, ret);
		ihbdata->read_error_warning_given = true;
		return -1;
	}
	if ((msg->ID & ~0xffff) != 0)
		return 1;

	ihbdata->read_error_warning_given = false;
	return 0;
}

static void monitor_can(urt_task *task, void *arg)
{
	struct ihb_data *ihbdata = arg;
	bool undetected_module_warning_given = false;

	while (!interrupted && !ihbdata->stop)
	{
		TPCANMsg msg;
		int i, j;
		int pending_read;

		/* send possible motor command */
		urt_mutex_lock(ihbdata->motor_data_mutex);

		for (i = 0; i < 2; ++i)
		{
			struct motor_shm_data *md = &ihbdata->motor_data[i];
			bool calibrate_cmd = false;
			if (md->new_command)
			{
				bool valid = true;

				msg.ID = COMMAND_ID_FOR(i);
				msg.DATA[0] = md->command;
				msg.LEN = 1;
				switch (md->command)
				{
				case MOTOR_CMD_MOV_ABS:
				case MOTOR_CMD_MOV_ABS_UPD:
				case MOTOR_CMD_MOV_REL_O:
				case MOTOR_CMD_MOV_REL_C:
				case MOTOR_CMD_MOV_ABS_PER:
				case MOTOR_CMD_MOV_ABS_UPD_PER:
				case MOTOR_CMD_MOV_REL_O_PER:
				case MOTOR_CMD_MOV_REL_C_PER:
				case MOTOR_CMD_MOV_FORCE_PER:
				case MOTOR_CMD_SET_FREQ:
					msg.DATA[1] = md->payload >> 8;
					msg.DATA[2] = md->payload & 0xFF;
					msg.LEN = 3;
					break;
				case MOTOR_CMD_SET_DIR:
				case MOTOR_CMD_SET_MOD:
					msg.DATA[1] = md->payload & 0xFF;
					msg.LEN = 2;
					break;
				case MOTOR_CMD_MOV_STOP:
				case MOTOR_CMD_MOV_OPEN:
				case MOTOR_CMD_MOV_CLOSE:
				case MOTOR_CMD_GET_FREQ:
				case MOTOR_CMD_GET_DIR:
				case MOTOR_CMD_GET_MOD:
				case MOTOR_CMD_GET_POSITION:
					break;
				case MOTOR_CMD_CALIB_POS:
				case MOTOR_CMD_CALIB_FORCE:
				case MOTOR_CMD_CALIB_AMB_PROX:
					/* do not send another calibration request while another one is still pending */
					if (ihbdata->calibrating[i])
					{
						valid = false;
						calibrate_cmd = true;
					}
					else
						ihbdata->calibrating[i] = true;
					break;
				case MOTOR_CMD_CALIB_FORCE_RESET:
					ihbdata->calibrating[i] = false;
					valid = false;
					break;
				default:
					valid = false;
					break;
				}

				if (valid)
				{
					if (LINUX_CAN_Write_Timeout(ihbdata->can_handle, &msg, -1))
						urt_err("can monitor: failed to send message (command %u)\n", msg.DATA[0]);
					else if (calibrate_cmd)
						ihbdata->calibrating[i] = true;
					else if (ihbdata->calibrating[i] && md->command == MOTOR_CMD_MOV_STOP)
						ihbdata->calibrating[i] = false;
				}

				md->new_command = false;
			}
		}

		urt_mutex_unlock(ihbdata->motor_data_mutex);

		/* send possible microphone command */
		urt_mutex_lock(ihbdata->micro_data_shmutex);

		for (i = 0; i < 2; ++i)
		{
			struct microphone_shm_data *md = &ihbdata->micro_data[i];
			if (md->new_command)
			{
				msg.ID = COMMAND_ID_FOR(i);
				msg.DATA[0] = md->command;
				msg.LEN = 1;

				if (LINUX_CAN_Write_Timeout(ihbdata->can_handle, &msg, -1))
					urt_err("can monitor: failed to send message (command %u)\n", msg.DATA[0]);
				else if (md->command == MICROPHONE_CMD_START)
				{
					/* if it's a start command, reset all sequence numbers to 0 */
					for (j = 0; j < MAX_AUDIO_FRAME_BUFFER; ++j)
						md->buffers[j].sequence_number = 0;
				}

				md->new_command = false;
			}
		}

		urt_mutex_unlock(ihbdata->micro_data_shmutex);

		/* handle responses */
		do
		{
			pending_read = get_can_pending_reads(ihbdata, "can monitor");

			for (i = 0; i < pending_read; ++i)
			{
				uint32_t msg_id;
				uint16_t mid;
				uint16_t start;
				bool valid = true;
				struct motor_shm_data *md;

				if (get_can_message(ihbdata, &msg, "can monitor"))
					continue;

				/* decode the message and put in appropriate memory */
				msg_id = msg.ID;

				switch (COMPONENT(msg_id))
				{
				case COMPONENT_MOTOR:
					urt_mutex_lock(ihbdata->motor_data_mutex);
					md = &ihbdata->motor_data[LEFT_OR_RIGHT(msg_id)];
					switch (msg.DATA[0])
					{
					case MOTOR_CMD_GET_FREQ:
						md->frequency = TO_U16_BE(msg.DATA[1], msg.DATA[2]);
						break;
					case MOTOR_CMD_GET_DIR:
						md->direction = msg.DATA[1];
						break;
					case MOTOR_CMD_GET_MOD:
						md->modality = msg.DATA[1];
						break;
					case MOTOR_CMD_GET_POSITION:
						md->position = TO_U16_BE(msg.DATA[1], msg.DATA[2]);
						md->position_steps = TO_U16_BE(msg.DATA[3], msg.DATA[4]);
						md->status = msg.DATA[5];

						/* check if finished calibrating */
						if (md->status & 0x10)
							ihbdata->calibrating[LEFT_OR_RIGHT(msg_id)] = false;
						break;
					default:
						valid = false;
						break;
					}
					if (valid)
						md->request_serviced = true;
					urt_mutex_unlock(ihbdata->motor_data_mutex);
					break;
				case COMPONENT_SKIN:
					if (SKIN_IS_TAXEL(msg_id))
					{
						/*
						 * the message id looks like this:
						 *
						 *     Component (2 bits) 00000 Module Id (2 bits) Packet # (1 bit) Left/Right (1 bit)
						 *
						 * where module id is taken as {ModuleId 0 Left/Right}.
						 */

						/* get the module id out of the message id */
						mid = MODULE_ID(msg_id);
						/* if it is invalid, then there has been some problem in initialization */
						if (ihbdata->module_map[mid] == (uint16_t)-1)
						{
							if (!undetected_module_warning_given)
								urt_err("can monitor: warning: data received from a skin module (%u)"
										" that was not detected during initialization\n", mid);
							undetected_module_warning_given = true;
							continue;
						}

						/* and find its mapping */
						mid = ihbdata->module_map[mid];
						start = (mid << 1 | MODULE_PACKET(msg_id)) * 4;

						/* store the contents of the message for the writer thread */
						if (urt_mutex_lock(ihbdata->skin_data_mutex, &ihbdata->stop))
							continue;

						for (j = 0; j < 4; ++j)
							ihbdata->module_raw_data[start + j] = TO_U16_LE(msg.DATA[2 * j], msg.DATA[2 * j + 1]);
						ihbdata->module_raw_data_dirty = true;

						urt_mutex_unlock(ihbdata->skin_data_mutex);
					}
					else if (SKIN_IS_MICROPHONE(msg_id))
					{
						/*
						 * the message id looks like this:
						 *
						 *     Component (2 bits) 00001 MessageType (2 bits) 0 Left/Right (1 bit)
						 *
						 * where MessageType is either 00 for sequence number indicating start of data stream,
						 * or 01 for the actual data.
						 */
						uint8_t lr = LEFT_OR_RIGHT(msg_id);
						start = lr * MAX_AUDIO_FRAME_LENGTH;

						/* if microphone data is dirty, it's still not picked up by acquire thread, so wait for it */
						if (ihbdata->micro_raw_data_dirty[lr])
						{
							while (ihbdata->micro_raw_data_dirty[lr] && !ihbdata->stop && !interrupted)
								urt_sleep(MAX_DELAY);
							if (ihbdata->stop || interrupted)
								continue;
						}

						if (urt_mutex_lock(ihbdata->micro_data_mutex, &ihbdata->stop))
							continue;

						if (MICROPHONE_IS_START(msg_id))
						{
							ihbdata->micro_raw_data_sequence_number[lr] = TO_U24_BE(msg.DATA[0], msg.DATA[1], msg.DATA[2]);
							ihbdata->micro_raw_data_cur_index[lr] = 0;
						}
						else if (MICROPHONE_IS_DATA(msg_id))
						{
							if (ihbdata->micro_raw_data_cur_index[lr] > MAX_AUDIO_FRAME_LENGTH - 8)
								urt_err("can monitor: warning: too much audio data, possibly due to"
										" missing start frame\n");
							else
							{
								for (j = 0; j < 8; ++j)
									ihbdata->micro_raw_data[start + ihbdata->micro_raw_data_cur_index[lr] + j] = msg.DATA[j];
								ihbdata->micro_raw_data_cur_index[lr] += 8;
								if (ihbdata->micro_raw_data_cur_index[lr] >= MAX_AUDIO_FRAME_LENGTH)
									ihbdata->micro_raw_data_dirty[lr] = true;
							}
						}
						else
							urt_err("can monitor: warning: packet received from a skin microphone"
									" with unknown type (%u)\n", MICROPHONE_MESSAGE_TYPE(msg_id));

						urt_mutex_unlock(ihbdata->micro_data_mutex);
					}
					else
						urt_err("can monitor: warning: patcket received from a skin component (%u)"
								" that is unrecognized\n", SKIN_COMPONENT(msg_id));
					break;
				case COMPONENT_GRIPPER:
					start = LEFT_OR_RIGHT(msg_id) * 4;
					urt_mutex_lock(ihbdata->gripper_data_mutex);
					for (j = 0; j < 4; ++j)
						ihbdata->gripper_raw_data[start + j] = TO_U16_LE(msg.DATA[2 * j], msg.DATA[2 * j + 1]);
					ihbdata->gripper_raw_data_dirty[LEFT_OR_RIGHT(msg_id)] = true;
					urt_mutex_unlock(ihbdata->gripper_data_mutex);
					break;
				default:
					break;
				}
			}
		} while (pending_read > 0);

		urt_task_wait_period(task);
	}

	ihbdata->can_running = false;
	done = 1;
}

static void acquire(urt_task *task, void *arg)
{
	uint32_t i, j;
	struct ihb_data *ihbdata = arg;
	bool lock_error_given = false, locked = false;
	urt_time cur_time;
	int ret;

	/* let the users start using the skin */
	ihbdata->taxel_data->initializing = false;
	ihbdata->taxel_data->users_must_stop = false;
	ihbdata->writer_running = true;

	while (!interrupted && !ihbdata->stop)
	{
		locked = false;

		/* lock the skin buffer */
		ret = urt_rwlock_timed_write_lock(ihbdata->data_rwl, MAX_DELAY);
		if (ret == ETIMEDOUT)
			continue;
		else if (ret)
		{
			if (!lock_error_given)
				urt_err("locking mechanism error (%d); acquiring data, but with no synchronization\n", ret);
			lock_error_given = true;
		}
		else
		{
			if (lock_error_given)
				urt_err("lock mechanism working now\n");
			lock_error_given = false;
			locked = true;
		}

		/* copy the data possibly acquired by the can monitor */
		cur_time = urt_get_time();

		/* tactile data */
		urt_mutex_lock(ihbdata->skin_data_mutex);

		if (ihbdata->module_raw_data_dirty)
		{
			ihbdata->taxel_data->last_update = cur_time;
			for (i = 0; i < MODULE_COUNT; ++i)
			{
				uint16_t mid;

				if (ihbdata->module_map[i] == (uint16_t)-1)
					continue;

				mid = ihbdata->module_map[i];
				for (j = 0; j < MAX_SENSORS_PER_MODULE; ++j)
				{
					ihbdata->taxel_data->received[mid * MAX_SENSORS_PER_MODULE + j] = cur_time;
					ihbdata->taxel_data->responses[mid * MAX_SENSORS_PER_MODULE + j] = ihbdata->module_raw_data[mid * MAX_SENSORS_PER_MODULE + j];
				}
			}
			ihbdata->module_raw_data_dirty = false;
		}

		urt_mutex_unlock(ihbdata->skin_data_mutex);

		/* gripper data */
		urt_mutex_lock(ihbdata->gripper_data_mutex);

		for (i = 0; i < 2; ++i)
			if (ihbdata->gripper_raw_data_dirty[i])
			{
				struct gripper_shm_data *gd = &ihbdata->gripper_data[i];
				uint16_t *d = &ihbdata->gripper_raw_data[i * 4];

				gd->last_update = cur_time;

				gd->light = d[0];
				gd->proximity = d[1];
				gd->holding_cloth = d[2]?0xFFFF:0;
				gd->magnetic_encoder = d[3];

				ihbdata->gripper_raw_data_dirty[i] = false;
			}

		urt_mutex_unlock(ihbdata->gripper_data_mutex);

		/* microphone data */
		urt_mutex_lock(ihbdata->micro_data_mutex);

		for (i = 0; i < 2; ++i)
			if (ihbdata->micro_raw_data_dirty[i])
			{
				struct microphone_shm_data *md = &ihbdata->micro_data[i];
				uint8_t *d = &ihbdata->micro_raw_data[i * MAX_AUDIO_FRAME_LENGTH];
				uint32_t min_seq = md->buffers[0].sequence_number, min_seq_index = 0;
				struct microphone_shm_data_buffer *b;

				for (j = 1; j < MAX_AUDIO_FRAME_BUFFER; ++j)
					if (md->buffers[j].sequence_number < min_seq)
					{
						min_seq = md->buffers[j].sequence_number;
						min_seq_index = j;
					}

				b = &md->buffers[min_seq_index];
				b->last_update = cur_time;

				b->sequence_number = ihbdata->micro_raw_data_sequence_number[i];
				for (j = 0; j < MAX_AUDIO_FRAME_LENGTH; ++j)
					b->data[j] = d[j];

				ihbdata->micro_raw_data_dirty[i] = false;
			}

		urt_mutex_unlock(ihbdata->micro_data_mutex);

		/* unlock the buffer for users */
		if (locked)
			urt_rwlock_write_unlock(ihbdata->data_rwl);

		urt_task_wait_period(task);
	}

	/* tell the users to stop too */
	ihbdata->taxel_data->users_must_stop = true;
	ihbdata->writer_running = false;

	done = 1;
}

static int prepare_can(struct ihb_data *ihbdata)
{
	int ret;
	char path[20] = "/dev/pcan";

	if (device >= 0)
	{
		sprintf(path + strlen(path), "%d", device);
		ihbdata->can_handle = LINUX_CAN_Open(path, O_RDWR);
	}
	else
		for (device = 0; device < 9; ++device)
		{
			sprintf(path + strlen(path), "%d", device);
			ihbdata->can_handle = LINUX_CAN_Open(path, O_RDWR);
			if (ihbdata->can_handle)
				break;
		}
	if (ihbdata->can_handle == NULL)
		goto exit_bad_handle;
	urt_dbg(stderr, "opened CAN port %d\n", device);

	ret = CAN_Init(ihbdata->can_handle, CAN_BAUD_1M, CAN_INIT_TYPE_EX);
	if (ret)
		goto exit_bad_init;
	urt_dbg(stderr, "initialized port\n");

	return 0;
exit_bad_init:
	urt_err("could not initialzie CAN device\n");
	goto exit_cleanup;
exit_bad_handle:
	urt_err("could not open CAN device\n");
	goto exit_cleanup;
exit_cleanup:
	cleanup(ihbdata);
	return -1;
}

static int discover_network(struct ihb_data *ihbdata)
{
	uint32_t i, j;
	uint16_t total_modules = 0;
	uint16_t extra_modules = 0;
	int pending_read;

	for (i = 0; i < MODULE_COUNT; ++i)
		ihbdata->module_map[i] = (uint16_t)-1;

	/* scan the bus for packets and deduce the number of modules */
	for (i = 0; i < net_scan_tries && !interrupted; ++i)
	{
		TPCANMsg msg;

		pending_read = get_can_pending_reads(ihbdata, "discovery");
		if (pending_read < 0)		/* get stat error */
			return -1;
		if (pending_read == 0)		/* no message received */
			--i;

		for (j = 0; j < pending_read; ++j)
		{
			if (get_can_message(ihbdata, &msg, "discovery") < 0)
				return -1;
			if (COMPONENT(msg.ID) == COMPONENT_SKIN && SKIN_IS_TAXEL(msg.ID))
			{
				uint32_t msg_id = MODULE_ID(msg.ID);
				ihbdata->module_map[msg_id] = 0;		/* mark it; later assigned */
			}
		}

		/* wait about one period */
		urt_sleep(1000000000 / frequency);
	}

	total_modules = 0;
	extra_modules = 0;

	/* create the actual module id map and count modules */
	for (i = 0; i < MODULE_COUNT; ++i)
		if (ihbdata->module_map[i] != (uint16_t)-1)
		{
			if (total_modules >= MAX_MODULES)
				++extra_modules;
			else
				ihbdata->module_map[i] = total_modules++;
		}

	if (extra_modules > 0)
		urt_err("discovery: detected %u more modules than was previsioned; ignoring them\n", extra_modules);

	urt_out("discovery: found %u modules\n", total_modules);

	ihbdata->taxel_data->taxels_count = total_modules * MAX_SENSORS_PER_MODULE;
	if (ihbdata->taxel_data->taxels_count == 0)
	{
		urt_err("error in initialization; stopping module\n");
		return -1;
	}

	return 0;
}

static int create_memory(struct ihb_data *ihbdata)
{
	int ret;

	/* get lock for skin data */
	ihbdata->skin_data_mutex = urt_mutex_new();
	if (ihbdata->skin_data_mutex == NULL)
		goto exit_skin_no_mutex;

	/* get rwlock on the data */
	ihbdata->data_rwl = urt_shrwlock_new(skin_shrwl_name);
	if (ihbdata->data_rwl == NULL)
		goto exit_no_rwl;

	/* lock the rwl before getting the memory */
	while (!interrupted && (ret = urt_rwlock_timed_write_lock(ihbdata->data_rwl, MAX_DELAY)) == ETIMEDOUT);
	if (interrupted)
		goto exit_interrupted;
	if (ret)
		goto exit_bad_skin_lock;

	/* get the memory and tell users it's still initializing */
	ihbdata->taxel_data = urt_shmem_new(skin_shmem_name, sizeof *ihbdata->taxel_data);
	if (ihbdata->taxel_data == NULL)
		goto exit_skin_no_mem;
	*ihbdata->taxel_data = (struct skin_shm_data){
		.initializing = true,
	};

	/* now it's safe to let users attach to the memory */
	urt_rwlock_write_unlock(ihbdata->data_rwl);

	/* get motor memory and mutex */
	ihbdata->motor_data_mutex = urt_shmutex_new(motor_shmutex_name);
	if (ihbdata->motor_data_mutex == NULL)
		goto exit_motor_no_mutex;

	if ((ret = urt_mutex_lock(ihbdata->motor_data_mutex)))
		goto exit_bad_motor_lock;

	ihbdata->motor_data = urt_shmem_new(motor_shmem_name, 2 * sizeof *ihbdata->motor_data);
	if (ihbdata->motor_data == NULL)
		goto exit_motor_no_mem;

	urt_mutex_unlock(ihbdata->motor_data_mutex);

	/* get gripper memory and mutex */
	ihbdata->gripper_data_mutex = urt_mutex_new();
	if (ihbdata->gripper_data_mutex == NULL)
		goto exit_gripper_no_mutex;

	if ((ret = urt_mutex_lock(ihbdata->gripper_data_mutex)))
		goto exit_bad_gripper_lock;

	ihbdata->gripper_data = urt_shmem_new(gripper_shmem_name, 2 * sizeof *ihbdata->gripper_data);
	if (ihbdata->gripper_data == NULL)
		goto exit_gripper_no_mem;

	urt_mutex_unlock(ihbdata->gripper_data_mutex);

	/* get micro memory and mutex */
	ihbdata->micro_data_mutex = urt_mutex_new();
	if (ihbdata->micro_data_mutex == NULL)
		goto exit_micro_no_mutex;
	ihbdata->micro_data_shmutex = urt_shmutex_new(micro_shmutex_name);
	if (ihbdata->micro_data_shmutex == NULL)
		goto exit_micro_no_shmutex;

	if ((ret = urt_mutex_lock(ihbdata->micro_data_mutex)))
		goto exit_bad_micro_lock;

	ihbdata->micro_data = urt_shmem_new(micro_shmem_name, 2 * sizeof *ihbdata->micro_data);
	if (ihbdata->micro_data == NULL)
		goto exit_micro_no_mem;

	urt_mutex_unlock(ihbdata->micro_data_mutex);

	return 0;
exit_skin_no_mem:
	urt_err("could not get shared memory with name \"%s\"\n", skin_shmem_name);
	goto exit_cleanup;
exit_motor_no_mem:
	urt_err("could not get shared memory with name \"%s\"\n", motor_shmem_name);
	goto exit_cleanup;
exit_gripper_no_mem:
	urt_err("could not get shared memory with name \"%s\"\n", gripper_shmem_name);
	goto exit_cleanup;
exit_micro_no_mem:
	urt_err("could not get shared memory with name \"%s\"\n", micro_shmem_name);
	goto exit_cleanup;
exit_skin_no_mutex:
	urt_err("could not get mutex for skin\n");
	goto exit_cleanup;
exit_no_rwl:
	urt_err("could not get shared rwlock with name \"%s\"\n", skin_shrwl_name);
	goto exit_cleanup;
exit_motor_no_mutex:
	urt_err("could not get shared mutex with name \"%s\"\n", motor_shmutex_name);
	goto exit_cleanup;
exit_gripper_no_mutex:
	urt_err("could not get mutex for gripper\n");
	goto exit_cleanup;
exit_micro_no_mutex:
	urt_err("could not get mutex for micro\n");
	goto exit_cleanup;
exit_micro_no_shmutex:
	urt_err("could not get shared mutex for micro\n");
	goto exit_cleanup;
exit_bad_skin_lock:
	urt_err("could not lock skin rwlock\n");
	goto exit_cleanup;
exit_bad_motor_lock:
	urt_err("could not lock motor mutex\n");
	goto exit_cleanup;
exit_bad_gripper_lock:
	urt_err("could not lock gripper mutex\n");
	goto exit_cleanup;
exit_bad_micro_lock:
	urt_err("could not lock micro mutex\n");
	goto exit_cleanup;
exit_interrupted:
	urt_err("interrupted during initialization\n");
	goto exit_cleanup;
exit_cleanup:
	cleanup(ihbdata);
	return -1;
}

static int create_task(struct ihb_data *ihbdata)
{
	ihbdata->writer_task = urt_task_new(acquire, ihbdata, &(urt_task_attr){ .period = 1000000000 / frequency });
	ihbdata->can_task = urt_task_new(monitor_can, ihbdata, &(urt_task_attr){ .period = 1000000000 / frequency });
	if (ihbdata->writer_task == NULL || ihbdata->can_task == NULL)
	{
		urt_err("could not create acquisition task or monitor task\n");
		return -1;
	}
	return 0;
}

static int can_driver_start(struct ihb_data *ihbdata)
{
	int ret = -1;

	*ihbdata = (struct ihb_data){0};

	/* make sure the parameters are ok */
	if (frequency < 1 || frequency > 1000)
		urt_err("invalid frequency %d\n", frequency);
	else if (device < -1)
		urt_err("invalid device number %d\n", device);
	else if (net_scan_tries < 1)
		urt_err("invalid net_scan_tries %d\n", net_scan_tries);
	else if (skin_shmem_name[0] == '\0')
		urt_err("cannot have an empty name for skin shared memory\n");
	else if (skin_shrwl_name[0] == '\0')
		urt_err("cannot have an empty name for skin rwlock\n");
	else if (motor_shmem_name[0] == '\0')
		urt_err("cannot have an empty name for motor shared memory\n");
	else if (motor_shmutex_name[0] == '\0')
		urt_err("cannot have an empty name for motor mutex\n");
	else if (gripper_shmem_name[0] == '\0')
		urt_err("cannot have an empty name for gripper shared memory\n");
	else
		ret = 0;

	return ret;
}

static void can_driver_body(struct ihb_data *ihbdata)
{
	int ret;

	/* start URT */
	ret = urt_init();
	if (ret)
		goto exit_no_urt;

	if (prepare_can(ihbdata) || create_memory(ihbdata) || discover_network(ihbdata) || create_task(ihbdata))
		goto exit_cleanup;

	/* start the actual acquisition */
	urt_task_start(ihbdata->can_task);
	urt_task_start(ihbdata->writer_task);

	urt_out("up and running\n");

	return;

exit_no_urt:
	urt_err("could not initialize URT (%d)\n", ret);
exit_cleanup:
	cleanup(ihbdata);
	done = 1;
}

static void can_driver_exit(struct ihb_data *ihbdata)
{
	/* wait for task to stop */
	ihbdata->stop = 1;
urt_err("exiting...\n");
	while (ihbdata->writer_task && ihbdata->writer_running)
		urt_sleep(MAX_DELAY);
urt_err("writer finished\n");
	while (ihbdata->can_task && ihbdata->can_running)
		urt_sleep(MAX_DELAY);
urt_err("can monitor finished\n");

	cleanup(ihbdata);
	urt_out("exited\n");
	urt_exit();
}
