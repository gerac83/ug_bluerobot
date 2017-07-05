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

#ifndef CLOPEMA_GRIPPER_MICRO_H
#define CLOPEMA_GRIPPER_MICRO_H

#include <urt.h>
#include <clopema_gripper_driver.h>

#define MICRO_BUFFER_LEN 50

enum micro_limb
{
	MICRO_LIMB_LEFT = 0,
	MICRO_LIMB_RIGHT = 1,
};

class gripper_micro
{
	public:
		gripper_micro(micro_limb l, const char *shmem_name = NULL, const char *shmutex_name = NULL,
				const char *skin_shrwl_name = NULL, const char *skin_shmem_name = NULL);
		~gripper_micro();

		int32_t execute(microphone_cmd cmd);
		int next_data(uint32_t &seq_num, uint8_t data[MAX_AUDIO_FRAME_LENGTH]);

		int start();
	private:
		void cleanup();

		micro_limb limb;

		struct microphone_shm_data *micro_data;
		urt_mutex *micro_mutex;
		struct skin_shm_data *skin_data;
		urt_rwlock *skin_rwl;

		urt_task *task;
		bool task_running;
		bool task_must_stop;

		uint8_t frames[MICRO_BUFFER_LEN][MAX_AUDIO_FRAME_LENGTH];
		uint32_t frames_seq_num[MICRO_BUFFER_LEN];
		uint8_t frames_start;
		uint8_t frames_count;
		urt_mutex *frames_mutex;

		friend void _reader_thread(urt_task *task, void *arg);
};

#endif
