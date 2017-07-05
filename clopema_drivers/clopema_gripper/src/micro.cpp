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

#define URT_LOG_PREFIX "micro library: "
#include "clopema_gripper_micro.h"

#define MAX_DELAY 20000000ll

void _reader_thread(urt_task *task, void *arg)
{
	gripper_micro *gm = (gripper_micro *)arg;
	struct microphone_shm_data *micro_data = &gm->micro_data[gm->limb];
	struct skin_shm_data *skin_data = gm->skin_data;
	urt_rwlock *rwl = gm->skin_rwl;
	urt_time last_update[MAX_AUDIO_FRAME_BUFFER] = {0};
	bool lock_error_given = false, locked = false;
	int ret;

	gm->task_running = true;

	while (!gm->task_must_stop && !skin_data->users_must_stop)
	{
		locked = false;

		ret = urt_rwlock_timed_read_lock(rwl, MAX_DELAY);
		if (ret == ETIMEDOUT)
			continue;
		if (ret)
		{
			if (!lock_error_given)
				urt_err("locking mechanism error (%d); acquiring data, but with no synchronization\n", ret);
			lock_error_given = true;
			/* acquire anyway */
			urt_sleep(MAX_DELAY);
		}
		else
		{
			if (lock_error_given)
				urt_err("locking mechanism working now\n");
			lock_error_given = false;
			locked = true;
		}

		while (true)
		{
			uint32_t min_seq, min_seq_index = MAX_AUDIO_FRAME_BUFFER;
			microphone_shm_data::microphone_shm_data_buffer *b;

			/* find the minimum sequence number and read that */
			for (unsigned int i = 0; i < MAX_AUDIO_FRAME_BUFFER; ++i)
			{
				b = &micro_data->buffers[i];
				/* if already read, ignore it */
				if (last_update[i] == b->last_update)
					continue;
				if (min_seq_index == MAX_AUDIO_FRAME_BUFFER || b->sequence_number < min_seq)
				{
					min_seq = b->sequence_number;
					min_seq_index = i;
				}
			}

			/* if all are up-to-date, stop */
			if (min_seq_index == MAX_AUDIO_FRAME_BUFFER)
				break;

			b = &micro_data->buffers[min_seq_index];

			/* perform the actual read */
			urt_mutex_lock(gm->frames_mutex);

			uint32_t frame = (gm->frames_start + gm->frames_count) % MICRO_BUFFER_LEN;
			for (unsigned int i = 0; i < MAX_AUDIO_FRAME_LENGTH; ++i)
				gm->frames[frame][i] = b->data[i];
			gm->frames_seq_num[frame] = b->sequence_number;

			if (gm->frames_count < MICRO_BUFFER_LEN)
				++gm->frames_count;
			else
			{
				++gm->frames_start;
				if (gm->frames_start >= MICRO_BUFFER_LEN)
					gm->frames_start -= MICRO_BUFFER_LEN;
			}

			urt_mutex_unlock(gm->frames_mutex);
			last_update[min_seq_index] = b->last_update;
		}

		if (locked)
			urt_rwlock_read_unlock(rwl);

		urt_task_wait_period(task);
	}

	gm->task_running = false;
}

gripper_micro::gripper_micro(micro_limb l, const char *shmem_name, const char *shmutex_name,
		const char *skin_shrwl_name, const char *skin_shmem_name): limb(l), task_running(false), task_must_stop(false)
{
	micro_data = (struct microphone_shm_data *)urt_shmem_attach(shmem_name?shmem_name:MICRO_SHMEM);
	micro_mutex = urt_shmutex_attach(shmutex_name?shmutex_name:MICRO_SHMUTEX);
	skin_rwl = urt_shrwlock_attach(skin_shrwl_name?skin_shrwl_name:SKIN_SHRWL);
	skin_data = (struct skin_shm_data *)urt_shmem_attach(skin_shmem_name?skin_shmem_name:SKIN_SHMEM);
	frames_mutex = urt_mutex_new();

	if (micro_data == NULL || micro_mutex == NULL || skin_rwl == NULL || skin_data == NULL || skin_data->initializing)
	{
		if (skin_data != NULL && skin_data->initializing)
			urt_err("the driver still being initialized\n");
		else
			urt_err("failed to attach to driver; is it running?\n");
		cleanup();
		exit(EXIT_FAILURE);
	}

	if (frames_mutex == NULL)
	{
		urt_err("could not acquire necessary locks\n");
		cleanup();
		exit(EXIT_FAILURE);
	}
}

gripper_micro::~gripper_micro()
{
	cleanup();
}

void gripper_micro::cleanup()
{
	urt_time start_time, wait_time = 0;

	task_must_stop = true;
	wait_time = 1000000000ull;
	start_time = urt_get_time();

	/* wait for task to stop */
	while (task_running && urt_get_time() - start_time < wait_time);
		urt_sleep(MAX_DELAY);

	/* if not stopped, kill it anyway */
	if (task && task_running)
		urt_err("waited for reader thread to exit for %fs, but it didn't stop;"
				" killing it\n", wait_time / 1000000000.0f);
	urt_task_delete(task);

	urt_shmem_detach(micro_data);
	urt_shmutex_detach(micro_mutex);
	urt_shrwlock_detach(skin_rwl);
	urt_shmem_detach(skin_data);
	urt_mutex_delete(frames_mutex);
	micro_data = NULL;
	micro_mutex = NULL;
	skin_rwl = NULL;
	skin_data = NULL;
	frames_mutex = NULL;
}

int gripper_micro::start()
{
	urt_task_attr attr = {0};
	attr.period = 10000000;
	task = urt_task_new(_reader_thread, this, &attr);
	if (task == NULL)
	{
		urt_err("could not create reader thread\n");
		return -1;
	}

	return urt_task_start(task);
}

int32_t gripper_micro::execute(microphone_cmd cmd)
{
	int timeout = 100;
	struct microphone_shm_data *md = &micro_data[limb];

	/* wait for previous message to be sent */
	while (md->new_command && --timeout)
		usleep(10000);

	if (timeout <= 0)
		return -1;

	urt_mutex_lock(micro_mutex);
	md->command = cmd;
	md->new_command = true;
	urt_mutex_unlock(micro_mutex);

	return 0;
}

int gripper_micro::next_data(uint32_t &seq_num, uint8_t data[MAX_AUDIO_FRAME_LENGTH])
{
	urt_mutex_lock(frames_mutex);

	if (frames_count == 0)
	{
		urt_mutex_unlock(frames_mutex);
		return -1;
	}

	seq_num = frames_seq_num[frames_start];
	for (unsigned int i = 0; i < MAX_AUDIO_FRAME_LENGTH; ++i)
		data[i] = frames[frames_start][i];

	++frames_start;
	--frames_count;
	if (frames_start >= MICRO_BUFFER_LEN)
		frames_start -= MICRO_BUFFER_LEN;

	urt_mutex_unlock(frames_mutex);
	return 0;
}
