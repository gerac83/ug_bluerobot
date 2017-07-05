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
#define URT_LOG_PREFIX "skin library: "
#include "clopema_gripper_skin.h"

#define MAX_DELAY 20000000ll

void skin_sensor_init(skin_sensor *sensor)
{
	*sensor = (skin_sensor){
		.id = (skin_sensor_id)SKIN_INVALID_ID
	};
}

void skin_sensor_free(skin_sensor *sensor)
{
	skin_sensor_init(sensor);
}

skin_sensor_type *skin_sensor_sensor_type(skin_sensor *sensor)
{
	return &sensor->p_object->p_sensor_types[sensor->type];
}

void skin_sensor_type_init(skin_sensor_type *sensor_type)
{
	*sensor_type = (skin_sensor_type){
		.id = (skin_sensor_type_id)SKIN_INVALID_ID
	};
}

void skin_sensor_type_free(skin_sensor_type *sensor_type)
{
	skin_sensor_type_init(sensor_type);
}

skin_sensor *skin_sensor_type_sensors(skin_sensor_type *sensor_type, skin_sensor_size *count)
{
	skin_object *obj;
	skin_sensor *sensors;
	skin_sensor_id begin;
	if (count)
		*count = sensor_type->sensors_end - sensor_type->sensors_begin;
	obj = sensor_type->p_object;
	sensors = obj->p_sensors;
	begin = sensor_type->sensors_begin;
	return sensors + begin;
}

#ifndef SKIN_DS_ONLY
typedef struct internal_list
{
	urt_sem *sem;
	struct internal_list *next;
} internal_list;

static int _list_add(void **list, urt_sem *m)
{
	internal_list *node;
	internal_list *tail;

	tail = *list;
	if (tail != NULL)
	{
		while (tail->next != NULL)
		{
			/* make sure it's not a duplicate */
			if (tail->sem == m)
				return 0;
			tail = tail->next;
		}
		/* same check for actual tail */
		if (tail->sem == m)
			return 0;
	}

	node = malloc(sizeof(*node));
	if (node == NULL)
		return -1;

	*node = (internal_list){
		.sem = m,
		.next = NULL
	};

	if (tail == NULL)
		*list = node;
	else
		tail->next = node;

	return 0;
}

static void _list_remove(void **list, urt_sem *m)
{
	internal_list *prev;
	internal_list *cur;
	internal_list *next;

	prev = NULL;
	cur = *list;
	while (cur != NULL)
	{
		next = cur->next;
		/* check if node should be removed */
		if (cur->sem == m)
		{
			/* unlink from list */
			if (prev == NULL)
				*list = next;
			else
				prev->next = next;
			/* remove */
			free(cur);
			/* there are no duplicates */
			return;
		}
		prev = cur;
		cur = next;
	}
}

static void _list_op(internal_list *list, int (*op)(urt_sem *))
{
	internal_list *cur;

	cur = list;
	while (cur != NULL)
	{
		op(cur->sem);
		cur = cur->next;
	}
}

static void _list_clear(void **list)
{
	internal_list *cur;
	internal_list *next;

	cur = *list;
	while (cur != NULL)
	{
		next = cur->next;
		free(cur);
		cur = next;
	}

	*list = NULL;
}

static int _sem_try_lock(urt_sem *sem)
{
	urt_sem_try_wait(sem);
	return 0;
}

static int _sem_unlock(urt_sem *sem)
{
	urt_sem_post(sem);
	return 0;
}

void _reader_thread(urt_task *task, void *arg)
{
	skin_reader *reader = arg;
	skin_object *so = reader->p_object;
	struct skin_shm_data *skin_data = reader->p_skin_data;
	struct gripper_shm_data *gripper_data = reader->p_gripper_data;
	urt_time last_skin_update = 0, last_gripper_update = 0;
	bool lock_error_given = false, locked = false;
	unsigned int i;
	int ret;
	skin_sensor *sensors;
	skin_sensor_size scount;

	reader->p_task_running = true;

	while (!reader->p_task_must_stop && !reader->p_skin_data->users_must_stop)
	{
		locked = false;

		/* wait for new data to arrive or if paused */
		if (!reader->p_task_enabled || (last_skin_update == skin_data->last_update && last_gripper_update == gripper_data->last_update))
		{
			urt_sleep(MAX_DELAY);
			continue;
		}

		ret = urt_rwlock_timed_read_lock(reader->p_rwl, MAX_DELAY);
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

		/*
		 * try-lock all semaphores registered for read, so that if they are free, they would be locked.
		 * You don't want a user to arrive while read is in progress.
		 */
		urt_mutex_lock(reader->p_registered_users_mutex);
		_list_op(reader->p_registered_users, _sem_try_lock);
		urt_mutex_unlock(reader->p_registered_users_mutex);

		/* perform the actual read */
		/* tactile data */
		sensors = skin_sensor_type_sensors(&so->p_sensor_types[0], &scount);
		for (i = 0; i < scount; ++i)
		{
			sensors[i].response = skin_data->responses[i];
			sensors[i].receive_time = skin_data->received[i];
		}
		last_skin_update = skin_data->last_update;

		sensors = skin_sensor_type_sensors(&so->p_sensor_types[1], &scount);
		for (i = 0; i < 2; ++i)
		{
			sensors[i].response = gripper_data[i].light;
			sensors[i].receive_time = gripper_data[i].last_update;
		}

		sensors = skin_sensor_type_sensors(&so->p_sensor_types[2], &scount);
		for (i = 0; i < 2; ++i)
		{
			sensors[i].response = gripper_data[i].proximity;
			sensors[i].receive_time = gripper_data[i].last_update;
		}

		sensors = skin_sensor_type_sensors(&so->p_sensor_types[3], &scount);
		for (i = 0; i < 2; ++i)
		{
			sensors[i].response = gripper_data[i].magnetic_encoder;
			sensors[i].receive_time = gripper_data[i].last_update;
		}

		sensors = skin_sensor_type_sensors(&so->p_sensor_types[4], &scount);
		for (i = 0; i < 2; ++i)
		{
			sensors[i].response = gripper_data[i].holding_cloth;
			sensors[i].receive_time = gripper_data[i].last_update;
		}
		last_gripper_update = gripper_data[0].last_update;
		if (last_gripper_update < gripper_data[1].last_update)
			last_gripper_update = gripper_data[1].last_update;

		/*
		 * unlock all semaphores registered for read, so that
		 * if there are users waiting on it, they would be unlocked
		 */
		urt_mutex_lock(reader->p_registered_users_mutex);
		_list_op(reader->p_registered_users, _sem_unlock);
		urt_mutex_unlock(reader->p_registered_users_mutex);

		if (locked)
			urt_rwlock_read_unlock(reader->p_rwl);

		urt_task_wait_period(task);
	}

	reader->p_task_running = false;
}

static int _reader_initialize(skin_reader *reader, const char *skin_shmem_name, const char *skin_shrwl_name,
		const char *gripper_shmem_name);
static int _reader_terminate(skin_reader *reader);

void skin_reader_init(skin_reader *reader)
{
	*reader = (skin_reader){0};
}

void skin_reader_free(skin_reader *reader)
{
	_reader_terminate(reader);
}

static int _reader_initialize(skin_reader *reader, const char *skin_shmem_name, const char *skin_shrwl_name,
		const char *gripper_shmem_name)
{
	if (reader->p_object == NULL || reader->p_object->p_sensors == NULL)
		return SKIN_FAIL;

	/* make thread real-time if not already */
	if (!urt_is_rt_context())
	{
		urt_err("thread initializing reader must have called urt_init()\n");
		goto exit_fail;
	}

	/* attach to driver's memory */
	reader->p_skin_data = urt_shmem_attach(skin_shmem_name);
	if (reader->p_skin_data == NULL)
	{
		urt_err("could not attach to skin shared memory\n");
		goto exit_fail;
	}
	if (reader->p_skin_data->initializing)
	{
		urt_err("the driver still being initialized\n");
		goto exit_fail;
	}
	reader->p_rwl = urt_shrwlock_attach(skin_shrwl_name);
	if (reader->p_rwl == NULL)
	{
		urt_err("could not attach to shared rwlock\n");
		goto exit_fail;
	}
	reader->p_gripper_data = urt_shmem_attach(gripper_shmem_name);
	if (reader->p_gripper_data == NULL)
	{
		urt_err("could not attach to gripper shared memory\n");
		goto exit_fail;
	}
	reader->p_registered_users_mutex = urt_mutex_new();
	if (!reader->p_registered_users_mutex)
	{
		urt_err("could not acquire necessary locks\n");
		goto exit_fail;
	}
	/* initialize tasks data */
	reader->p_task_running = false;
	reader->p_task_must_stop = true;
	reader->p_task_enabled = false;
	reader->p_registered_users = NULL;
	return SKIN_SUCCESS;
exit_fail:
	_reader_terminate(reader);
	return SKIN_FAIL;
}

static void _remove_task(skin_reader *reader, urt_time wait_time)
{
	if (reader->p_task)
	{
		if (reader->p_task_running)
			urt_err("waited for reader thread to exit for %fs, but it didn't stop;"
					" killing it\n", wait_time / 1000000000.0f);
		urt_task_delete(reader->p_task);
	}
	reader->p_task = NULL;
}

static int _reader_terminate(skin_reader *reader)
{
	/* stop the task */
	skin_reader_stop(reader);

	/* detach from driver */
	urt_shmem_detach(reader->p_skin_data);
	urt_shmem_detach(reader->p_gripper_data);
	urt_shrwlock_detach(reader->p_rwl);

	/* clean up of user synchronization data */
	_list_clear(&reader->p_registered_users);
	if (reader->p_registered_users_mutex)
		urt_mutex_delete(reader->p_registered_users_mutex);
	reader->p_registered_users_mutex = NULL;

	/* other clean up */
	skin_reader_init(reader);

	return SKIN_SUCCESS;
}

static int _start(skin_reader *reader)
{
	reader->p_task = urt_task_new(_reader_thread, reader, &(urt_task_attr){ .period = 40000000 });
	if (reader->p_task == NULL)
	{
		urt_err("could not create reader thread\n");
		skin_reader_stop(reader);
		return SKIN_FAIL;
	}

	reader->p_task_must_stop = false;
	reader->p_task_enabled = true;
	urt_task_start(reader->p_task);

	return SKIN_SUCCESS;
}

int skin_reader_start(skin_reader *reader)
{
	int ret;

	skin_reader_stop(reader);

	if (reader->p_object == NULL || reader->p_object->p_sensors == NULL)
		return SKIN_FAIL;
	if (reader->p_skin_data == NULL || reader->p_gripper_data == NULL || reader->p_rwl == NULL)
	{
		urt_err("the reader must be initialized before start could be called.\n"
				"\t- this initialization is done by the main object.\n");
		return SKIN_FAIL;
	}

	if (!urt_is_rt_context())
	{
		urt_err("thread starting reader must have called urt_init()\n");
		return SKIN_FAIL;
	}

	ret = _start(reader);
	return ret;
}

int skin_reader_stop(skin_reader *reader)
{
	urt_time start_time, wait_time = 0;

	if (reader->p_object == NULL || reader->p_skin_data == NULL || reader->p_gripper_data == NULL)
		return SKIN_FAIL;
	reader->p_task_must_stop = true;

	wait_time = 1000000000ull;
	start_time = urt_get_time();

	/* wait for task to stop */
	while (reader->p_task_running && urt_get_time() - start_time < wait_time);
		urt_sleep(MAX_DELAY);

	/* if not stopped, kill it anyway */
	_remove_task(reader, wait_time);

	return SKIN_SUCCESS;
}

int skin_reader_pause(skin_reader *reader)
{
	if (reader->p_object == NULL || reader->p_skin_data == NULL || reader->p_gripper_data == NULL)
		return SKIN_FAIL;
	reader->p_task_enabled = false;
	return SKIN_SUCCESS;
}

int skin_reader_resume(skin_reader *reader)
{
	if (reader->p_object == NULL || reader->p_skin_data == NULL || reader->p_gripper_data == NULL)
		return SKIN_FAIL;
	reader->p_task_enabled = true;
	return SKIN_SUCCESS;
}

int skin_reader_is_paused(skin_reader *reader)
{
	if (reader->p_object == NULL || reader->p_skin_data == NULL || reader->p_gripper_data == NULL)
		return SKIN_FAIL;
	return !reader->p_task_enabled;
}

int skin_reader_register_user(skin_reader *reader, urt_sem *sem)
{
	if (!urt_is_rt_context())
	{
		urt_err("thread registering reader must have called urt_init()\n");
		return SKIN_FAIL;
	}

	if (!reader->p_registered_users_mutex || urt_mutex_lock(reader->p_registered_users_mutex) != SKIN_SUCCESS)
	{
		urt_err("could not register user due to failure in locking\n");
		return SKIN_SUCCESS;
	}
	_list_add(&reader->p_registered_users, sem);
	urt_mutex_unlock(reader->p_registered_users_mutex);

	return SKIN_SUCCESS;
}

int skin_reader_wait_read(skin_reader *reader, urt_sem *sem, urt_time wait_time, volatile sig_atomic_t *must_stop)
{
	int ret = SKIN_SUCCESS;

	if (reader->p_object == NULL)
		return SKIN_FAIL;

	if (!urt_is_rt_context())
	{
		urt_err("thread waiting reader must have called urt_init()\n");
		return SKIN_FAIL;
	}
	if (wait_time == (urt_time)-1)
		ret = urt_sem_wait(sem, must_stop);
	else if (wait_time == 0)
		ret = urt_sem_try_wait(sem);
	else
		ret = urt_sem_timed_wait(sem, wait_time);
	return ret;
}

int skin_reader_unregister_user(skin_reader *reader, urt_sem *sem)
{
	if (!urt_is_rt_context())
	{
		urt_err("thread unregistering reader must have called urt_init()\n");
		return SKIN_FAIL;
	}

	if (!reader->p_registered_users_mutex || urt_mutex_lock(reader->p_registered_users_mutex) != SKIN_SUCCESS)
	{
		urt_err("could not unregister user due to failure in locking\n");
		return SKIN_FAIL;
	}
	_list_remove(&reader->p_registered_users, sem);
	urt_mutex_unlock(reader->p_registered_users_mutex);

	return SKIN_SUCCESS;
}
#endif

#define SKIN_CHECK_FILE_INTEGRITY(so, f, x)						\
	do {										\
		if (feof(f))								\
		{									\
			urt_err("file passed as %s file is incomplete\n", x);	\
			ret = SKIN_FILE_INCOMPLETE;					\
			goto exit_fail;							\
		}									\
		else if (ferror(f))							\
		{									\
			urt_err("file passed as %s file is erroneous\n", x);	\
			ret = SKIN_FILE_PARSE_ERROR;					\
			goto exit_fail;							\
		}									\
	} while (0)

void skin_object_init(skin_object *so)
{
	*so = (skin_object){0};
#ifndef SKIN_DS_ONLY
	skin_reader_init(&so->p_reader);
#endif
}

void skin_object_free(skin_object *so)
{
#ifndef SKIN_DS_ONLY
	skin_reader_free(&so->p_reader);
#endif
	free(so->p_sensors);
	free(so->p_sensor_types);
	free(so->p_sensor_neighbors);
	skin_object_init(so);
}

#ifndef SKIN_DS_ONLY
int skin_object_load(skin_object *so, const char *calibration_file, const char *skin_shmem_name, const char *skin_shrwl_name,
		const char *gripper_shmem_name)
{
	struct skin_shm_data *driver_skin_data = NULL;
	int ret = SKIN_FAIL;
	FILE *fin = NULL;
	unsigned int sensors_count, sensor_neighbors_count;
	unsigned int i, j, cur;

	if (calibration_file == NULL)
		return SKIN_FAIL;
	if (skin_shmem_name == NULL)
		skin_shmem_name = SKIN_SHMEM;
	if (skin_shrwl_name == NULL)
		skin_shrwl_name = SKIN_SHRWL;
	if (gripper_shmem_name == NULL)
		gripper_shmem_name = GRIPPER_SHMEM;

	/* get general information from the driver */
	driver_skin_data = urt_shmem_attach(skin_shmem_name);
	if (driver_skin_data == NULL)
		return SKIN_NO_DRIVER;
	if (driver_skin_data->initializing)
	{
		urt_shmem_detach(driver_skin_data);
		return SKIN_NO_DRIVER;
	}

	fin = fopen(calibration_file, "r");
	if (fin == NULL)
		goto exit_no_file;

	fscanf(fin, "%u %u", &sensors_count, &sensor_neighbors_count);
	SKIN_CHECK_FILE_INTEGRITY(so, fin, "calibration");

	/* furthermore, there are 4 other types of sensors, two sensors of each type */
	if (sensors_count != driver_skin_data->taxels_count + 8)
	{
		urt_err("bad calibration file: mismatch between number of detected sensors (%u + 8) and\n"
				"  number of sensors in the calibration file (%u)\n", driver_skin_data->taxels_count, sensors_count);
		goto exit_fail;
	}
	if (sensors_count == 0 || sensor_neighbors_count == 0)
	{
		urt_err("Encountered 0 sensors or 0 neighbors in the calibration file.\n");
		goto exit_bad_file;
	}
	urt_shmem_detach(driver_skin_data);
	driver_skin_data = NULL;

	/* allocate user memories */
	so->p_sensor_types_count = 5;
	so->p_sensors_count = sensors_count;
	so->p_sensor_neighbors_count = sensor_neighbors_count;
	so->p_sensor_types = malloc(so->p_sensor_types_count * sizeof(*so->p_sensor_types));
	so->p_sensors = malloc(so->p_sensors_count * sizeof(*so->p_sensors));
	so->p_sensor_neighbors = malloc(so->p_sensor_neighbors_count * sizeof(*so->p_sensor_neighbors));
	if (!so->p_sensor_types || !so->p_sensors || !so->p_sensor_neighbors)
	{
		urt_err("could not acquire memory for main data structures\n");
		goto exit_no_mem;
	}

	/* create sensor/type relation */
	for (i = 0; i < so->p_sensors_count - 8; ++i)
		so->p_sensors[i].type = 0;
	for (i = 0; i < 8; ++i)
		so->p_sensors[so->p_sensors_count - 8 + i].type = i / 2 + 1;
	so->p_sensor_types[0].sensors_begin = 0;
	so->p_sensor_types[0].sensors_end = so->p_sensors_count - 8;
	so->p_sensor_types[0].p_object = so;
	for (i = 1; i < 5; ++i)
	{
		so->p_sensor_types[i].sensors_begin = so->p_sensor_types[i - 1].sensors_end;
		so->p_sensor_types[i].sensors_end = so->p_sensor_types[i].sensors_begin + 2;
		so->p_sensor_types[i].p_object = so;
	}

	/* load calibration data */
	cur = 0;
	for (i = 0; i < so->p_sensors_count; ++i)
	{
		unsigned long robot_link, neighbors_count;

		fscanf(fin, "%f %f %f %f %f %f %f %f %f %lu %lu",
				&so->p_sensors[i].relative_position[0],
				&so->p_sensors[i].relative_position[1],
				&so->p_sensors[i].relative_position[2],
				&so->p_sensors[i].relative_orientation[0],
				&so->p_sensors[i].relative_orientation[1],
				&so->p_sensors[i].relative_orientation[2],
				&so->p_sensors[i].flattened_position[0],
				&so->p_sensors[i].flattened_position[1],
				&so->p_sensors[i].radius,
				&robot_link, &neighbors_count);
		so->p_sensors[i].robot_link = robot_link;
		so->p_sensors[i].neighbors_count = neighbors_count;
		SKIN_CHECK_FILE_INTEGRITY(so, fin, "calibration");

		so->p_sensors[i].neighbors = so->p_sensor_neighbors + cur;
		for (j = 0; j < so->p_sensors[i].neighbors_count; ++j)
		{
			unsigned long id;

			fscanf(fin, "%lu", &id);
			so->p_sensors[i].neighbors[j] = id;
		}
		cur += so->p_sensors[i].neighbors_count;

		so->p_sensors[i].id = i;
		so->p_sensors[i].p_object = so;
	}
	SKIN_CHECK_FILE_INTEGRITY(so, fin, "calibration");

	so->p_reader.p_object = so;
	ret = _reader_initialize(&so->p_reader, skin_shmem_name, skin_shrwl_name, gripper_shmem_name);
	if (ret != SKIN_SUCCESS)
		goto exit_fail;
exit_cleanup:
	urt_shmem_detach(driver_skin_data);
	if (fin)
		fclose(fin);
	return ret;
exit_no_mem:
	ret = SKIN_NO_MEM;
	goto exit_fail;
exit_no_file:
	ret = SKIN_NO_FILE;
	goto exit_fail;
exit_bad_file:
	ret = SKIN_FILE_INVALID;
	goto exit_fail;
exit_fail:
	free(so->p_sensor_types);
	free(so->p_sensors);
	free(so->p_sensor_neighbors);
	so->p_sensors = NULL;
	so->p_sensor_neighbors = NULL;
	goto exit_cleanup;
}
#endif

void skin_sensor_type_for_each_sensor(skin_sensor_type *sensor_type, skin_callback_sensor c, void *data)
{
	skin_sensor *sensors;
	skin_sensor_size scount;
	skin_sensor_id i;

	sensors = skin_sensor_type_sensors(sensor_type, &scount);

	for (i = 0; i < scount; ++i)
		if (c(&sensors[i], data) == SKIN_CALLBACK_STOP)
			return;
}

void skin_object_for_each_sensor_type(skin_object *so, skin_callback_sensor_type c, void *data)
{
	skin_sensor_type_id i;
	skin_sensor_type_id begin, end;
	skin_sensor_type *sensor_types;

	sensor_types = so->p_sensor_types;
	begin = 0;
	end = so->p_sensor_types_count;
	for (i = begin; i < end; ++i)
		if (c(&sensor_types[i], data) == SKIN_CALLBACK_STOP)
			return;
}
void skin_object_for_each_sensor(skin_object *so, skin_callback_sensor c, void *data)
{
	skin_sensor_id i;
	skin_sensor_id begin, end;
	skin_sensor *sensors;

	sensors = so->p_sensors;
	begin = 0;
	end = so->p_sensors_count;
	for (i = begin; i < end; ++i)
		if (c(&sensors[i], data) == SKIN_CALLBACK_STOP)
			return;
}
