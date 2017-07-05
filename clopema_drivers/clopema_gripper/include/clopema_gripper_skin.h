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

#ifndef CLOPEMA_GRIPPER_SKIN_H
#define CLOPEMA_GRIPPER_SKIN_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <clopema_gripper_driver.h>
#ifndef SKIN_DS_ONLY
#include <urt.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t skin_sensor_id;
typedef uint16_t skin_sensor_type_id;
typedef uint16_t skin_sensor_response;
typedef skin_sensor_id skin_sensor_size;
typedef skin_sensor_type_id skin_sensor_type_size;
#ifdef SKIN_DS_ONLY
typedef long long urt_time;
#endif

typedef struct skin_sensor skin_sensor;
typedef struct skin_sensor_type skin_sensor_type;

#define SKIN_CALLBACK_STOP 0
#define SKIN_CALLBACK_CONTINUE 1

typedef int (*skin_callback_sensor_type)(struct skin_sensor_type *s, void *data);
typedef int (*skin_callback_sensor)(struct skin_sensor *s, void *data);

#define SKIN_SUCCESS			0		/* returned upon success */
#define SKIN_FAIL			-1		/* if operation failed */
#define SKIN_NO_FILE			-2		/* if a file could not be opened */
#define SKIN_NO_MEM			-3		/* if a memory could not be allocated */
#define SKIN_FILE_INCOMPLETE		-4		/* if unexpected eof encountered */
#define SKIN_FILE_PARSE_ERROR		-5		/* if file was erroneous */
#define SKIN_FILE_INVALID		-6		/* if file data does not match kernel data */
#define SKIN_NO_DRIVER			-7		/* if could not attach to the driver */

#define SKIN_INVALID_ID			0xffffffffu
#define SKIN_INVALID_SIZE		SKIN_INVALID_ID

/*
 * skin_sensor_type is a different sensor type in the skin
 *
 * init				initialize to an invalid sensor
 * free				release any resources and make invalid
 * sensors			get a pointer to sensors of this type and their count
 */
void skin_sensor_type_init(skin_sensor_type *sensor_type);
void skin_sensor_type_free(skin_sensor_type *sensor_type);
skin_sensor *skin_sensor_type_sensors(skin_sensor_type *sensor_type, skin_sensor_size *count);
static inline skin_sensor_size skin_sensor_type_sensors_count(skin_sensor_type *sensor_type);
void skin_sensor_type_for_each_sensor(skin_sensor_type *sensor_type, skin_callback_sensor c, void *data);

struct skin_sensor_type
{
	skin_sensor_type_id	id;				/* id of sensor type.  Is equal to its index in the sensor type list */
	skin_sensor_id		sensors_begin, sensors_end;	/* sensors of this type are at indices [begin, end) */

	/* internal */
	struct skin_object	*p_object;

#ifdef __cplusplus
	skin_sensor_type() { skin_sensor_type_init(this); }
	~skin_sensor_type() { skin_sensor_type_free(this); }
	skin_sensor *sensors(skin_sensor_size *count) { return skin_sensor_type_sensors(this, count); }
	skin_sensor_size sensors_count() { return skin_sensor_type_sensors_count(this); }

	class skin_sensor_iterator
	{
	private:
		skin_sensor_id current;
		skin_sensor_type *sensor_type;
		skin_sensor_iterator(skin_sensor_type *sensor_type, skin_sensor_id c);
	public:
		skin_sensor_iterator();
		skin_sensor_iterator &operator ++();
		skin_sensor_iterator operator ++(int);
		bool operator ==(const skin_sensor_iterator &rhs) const;
		bool operator !=(const skin_sensor_iterator &rhs) const;
		skin_sensor &operator *() const;
		skin_sensor *operator ->() const;
		operator skin_sensor *() const;
		friend struct skin_sensor_type;
	};
	skin_sensor_iterator sensors_iter_begin();
	const skin_sensor_iterator sensors_iter_end();
#endif
};

static inline skin_sensor_size skin_sensor_type_sensors_count(skin_sensor_type *sensor_type)
{
	return sensor_type->sensors_end - sensor_type->sensors_begin;
}

/*
 * skin_sensor is a sensor in the skin.
 *
 * init				initialize to an invalid sensor
 * free				release any resources and make invalid
 * get_response			get the current response of the sensor.  If receive_time is provided, it will be
 *				set to the receive time of the sensor response
 * sensor_type			get the sensor type of this sensor
 */
void skin_sensor_init(skin_sensor *sensor);
void skin_sensor_free(skin_sensor *sensor);
static inline skin_sensor_response skin_sensor_get_response(skin_sensor *sensor, urt_time *receive_time);
skin_sensor_type *skin_sensor_sensor_type(skin_sensor *sensor);

struct skin_sensor
{
	skin_sensor_id		id;				/* id of the sensor itself.  Is equal to its index in the sensor list */
	skin_sensor_type_id	type;				/* type of sensor.  Is equal to its index in the sensor type list */
	skin_sensor_response	response;			/* response of the sensor.  Has a value between 0 and 65535 */
	urt_time		receive_time;			/* acquisition time of sensor response */
	float			relative_position[3];		/* 3d position of the sensor (local to robot_link reference frame) */
	float			relative_orientation[3];	/* normal axis at this sensor.  Since skin module is deformed, could be different for */
								/* every sensor (local to robot_link reference frame).  If non-zero, */
								/* then it is already made normal (norm == 1) */
	float			global_position[3];
	float			global_orientation[3];
	float			flattened_position[2];		/* the position in the flattened image of the skin */
	float			radius;				/* the radius of the sensor */
	skin_sensor_id		*neighbors;			/* list of sensors connected to this neighbor in the flattened image */
	skin_sensor_size	neighbors_count;		/* size of neighbors */
	uint32_t		robot_link;			/* robot link this sensor is located on */

	/* internal */
	struct skin_object	*p_object;

#ifdef __cplusplus
	skin_sensor() { skin_sensor_init(this); }
	~skin_sensor() { skin_sensor_free(this); }
	skin_sensor_response get_response(urt_time *receive_time = NULL) { return skin_sensor_get_response(this, receive_time); }
	skin_sensor_type *sensor_type() { return skin_sensor_sensor_type(this); }
#endif
};

static inline skin_sensor_response skin_sensor_get_response(skin_sensor *sensor, urt_time *receive_time)
{
	if (receive_time)
		*receive_time = sensor->receive_time;
	return sensor->response;
}

#ifndef SKIN_DS_ONLY
/*
 * skin_reader manages the readers.  In all these functions, unless stated otherwise, SKIN_SUCCESS would be returned if everything is ok
 * or SKIN_FAIL if failed.
 *
 * init				initialize the reader, but it wouldn't still work until initialized by skin_object
 * free				stop the reader and release resources
 * start			start acquisition
 *				Detail on why it failed would be logged.
 *				Note: There are some stuff inside that need to be done in real-time mode.  Thus, if the thread
 *				calling it is not already real-time, it would become real-time temporarily and turned back
 *				to normal before this function returns
 * stop				stop data acquisition
 *				Detail on why it failed would be logged
 * pause			pause acquisition
 * resume			resume acquisition
 * is_paused			Check whether sensor values are being acquired.  SKIN_FAIL would be returned in case of error
 * register_user		register a semaphore so that whenever a read is completed, the user waiting for the read could be awakened.
 *				The user would call wait_read and wait until awakened.  This way, it can be guaranteed that the user of
 *				data always works _after_ the data are read.  Note that this doesn't mean _another_ read could not have
 *				been started.
 * wait_read			helper function which can wait on a registered semaphore (with the possibility to cancel the wait), try-wait it,
 *				or timed-wait it.  If time parameter is (urt_time)-1, then it will wait until semaphore is unlocked.
 *				If time is 0, try-locks the semaphore and if time > 0, timed-locks the semaphore.
 *				If must_stop is NULL, it will be ignored
 * unregister_user		removes the semaphore from the list of registered semaphores
 */
typedef struct skin_reader skin_reader;
void skin_reader_init(skin_reader *reader);
void skin_reader_free(skin_reader *reader);
int skin_reader_start(skin_reader *reader);
int skin_reader_stop(skin_reader *reader);
int skin_reader_pause(skin_reader *reader);
int skin_reader_resume(skin_reader *reader);
int skin_reader_is_paused(skin_reader *reader);
int skin_reader_register_user(skin_reader *reader, urt_sem *sem);
int skin_reader_wait_read(skin_reader *reader, urt_sem *sem, urt_time wait_time, volatile sig_atomic_t *must_stop);
int skin_reader_unregister_user(skin_reader *reader, urt_sem *sem);

struct skin_reader
{
	/* internal */
	struct skin_shm_data	*p_skin_data;
	struct gripper_shm_data	*p_gripper_data;
	urt_rwlock		*p_rwl;
	void			*p_registered_users;
	urt_mutex		*p_registered_users_mutex;	/* mutex for handling work with p_registered_users */
	struct skin_object	*p_object;

	/* acquisition task data */
	urt_task		*p_task;
	bool			p_task_running;
	bool			p_task_must_stop;
	bool			p_task_enabled;

#ifdef __cplusplus
	skin_reader() { skin_reader_init(this); }
	~skin_reader() { skin_reader_free(this); }
	int start() { return skin_reader_start(this); }
	int stop() { return skin_reader_stop(this); }
	int pause() { return skin_reader_pause(this); }
	int resume() { return skin_reader_resume(this); }
	int is_paused() { return skin_reader_is_paused(this); }
	int register_user(urt_sem *sem)
			{ return skin_reader_register_user(this, sem); }
	int wait_read(urt_sem *sem, urt_time wait_time = (urt_time)-1, volatile sig_atomic_t *must_stop = NULL)
			{ return skin_reader_wait_read(this, sem, wait_time, must_stop); }
	int unregister_user(urt_sem *sem)
			{ return skin_reader_unregister_user(this, sem); }
#endif
};
#endif

/*
 * skin_object is the main data structure of the skin.  It holds all structures of the skin, as well as
 * the reader.  The functions here are used for accessing these structures, as well as initializing and performing
 * the calibration.  The functions, unless stated otherwise, may return SKIN_SUCCESS if operation was
 * successful, SKIN_NO_MEM if out of memory or SKIN_FAIL if any other error.  Additionally, functions that
 * load calibration or other data from file may return SKIN_NO_FILE if the file couldn't be opened,
 * SKIN_FILE_INCOMPLETE if the cache file was not complete, SKIN_FILE_PARSE_ERROR if there was an error
 * reading the cache file or SKIN_FILE_INVALID if the file didn't match the current skin.
 *
 * Initialization and termination:
 * init				initialize to an invalid object
 * free				release any resources and make invalid
 * load				load the skin from the driver.  init must have been called before this function.  Before using
 *				the skin, this function should be called.  Calibration file must be provided.  If names of shared memory
 *				or shared mutex with the driver are not provided (or are NULL), they would be set to default.
 *
 * Data structures access:
 * reader			get a reference to the skin_reader object
 * sensor_types			get the array of all sensor types in the skin and its count.  If count is NULL, it will be ignored
 * sensor_types_count		get the number of all sensor types in the skin
 * sensors			get the array of all sensors in the skin and its count.  If count is NULL, it will be ignored
 * sensors_count		get the number of all sensors in the skin
 * for_each_sensor		call a callback for each sensor in the skin.  Note that the callback is called for sensors with an
 *				ordering over sensor types.  That is, sensors of each sensor type are given contiguously
 */
typedef struct skin_object skin_object;
void skin_object_init(skin_object *so);
void skin_object_free(skin_object *so);
#ifndef SKIN_DS_ONLY
int skin_object_load(skin_object *so, const char *calibration_file, const char *skin_shmem_name, const char *skin_shrwl_name,
		const char *gripper_shmem_name);
static inline skin_reader *skin_object_reader(skin_object *so);
#endif
static inline struct skin_sensor_type *skin_object_sensor_types(skin_object *so, skin_sensor_type_size *count);
static inline skin_sensor_type_size skin_object_sensor_types_count(skin_object *so);
static inline struct skin_sensor *skin_object_sensors(skin_object *so, skin_sensor_size *count);
static inline skin_sensor_size skin_object_sensors_count(skin_object *so);
void skin_object_for_each_sensor(skin_object *so, skin_callback_sensor c, void *data);
void skin_object_for_each_sensor_type(skin_object *so, skin_callback_sensor_type c, void *data);

/* normally, there would be only one object from the following struct */
struct skin_object
{
	/* internal */
	struct skin_sensor_type	*p_sensor_types;
	struct skin_sensor	*p_sensors;
	skin_sensor_id		*p_sensor_neighbors;


	skin_sensor_type_size	p_sensor_types_count;
	skin_sensor_size	p_sensors_count;
	uint32_t		p_sensor_neighbors_count;

#ifndef SKIN_DS_ONLY
	skin_reader		p_reader;
#endif

#ifdef __cplusplus
	skin_object() { skin_object_init(this); }
	~skin_object() { skin_object_free(this); }
	void unload() { return skin_object_free(this); }
#ifndef SKIN_DS_ONLY
	int load(const char *calibration_file, const char *skin_shmem_name = NULL, const char *skin_shrwl_name = NULL,
			const char *gripper_shmem_name = NULL)
			{ return skin_object_load(this, calibration_file, skin_shmem_name, skin_shrwl_name, gripper_shmem_name); }
	skin_reader *reader() { return skin_object_reader(this); }
#endif
	struct skin_sensor_type *sensor_types(skin_sensor_type_size *count = NULL) { return skin_object_sensor_types(this, count); }
	skin_sensor_type_size sensor_types_count() { return skin_object_sensor_types_count(this); }
	struct skin_sensor *sensors(skin_sensor_size *count = NULL) { return skin_object_sensors(this, count); }
	skin_sensor_size sensors_count() { return skin_object_sensors_count(this); }

	class skin_sensor_type_iterator
	{
	private:
		skin_sensor_type_id current;
		skin_object *object;
		skin_sensor_type_iterator(skin_object *object, skin_sensor_type_id c);
	public:
		skin_sensor_type_iterator();
		skin_sensor_type_iterator &operator ++();
		skin_sensor_type_iterator operator ++(int);
		bool operator ==(const skin_sensor_type_iterator &rhs) const;
		bool operator !=(const skin_sensor_type_iterator &rhs) const;
		skin_sensor_type &operator *() const;
		skin_sensor_type *operator ->() const;
		operator skin_sensor_type *() const;
		friend struct skin_object;
	};
	skin_sensor_type_iterator sensor_types_iter_begin();
	const skin_sensor_type_iterator sensor_types_iter_end();

	class skin_sensor_iterator
	{
	private:
		skin_sensor_id current;
		skin_object *object;
		skin_sensor_iterator(skin_object *object, skin_sensor_id c);
	public:
		skin_sensor_iterator();
		skin_sensor_iterator &operator ++();
		skin_sensor_iterator operator ++(int);
		bool operator ==(const skin_sensor_iterator &rhs) const;
		bool operator !=(const skin_sensor_iterator &rhs) const;
		skin_sensor &operator *() const;
		skin_sensor *operator ->() const;
		operator skin_sensor *() const;
		friend struct skin_object;
	};
	skin_sensor_iterator sensors_iter_begin();
	const skin_sensor_iterator sensors_iter_end();
#endif
};

#ifndef SKIN_DS_ONLY
static inline skin_reader *skin_object_reader(skin_object *so)					{ return &so->p_reader; }
#endif
static inline struct skin_sensor *skin_object_sensors(skin_object *so, skin_sensor_size *count)
{
	if (count)
		*count = so->p_sensors_count;
	return so->p_sensors;
}
static inline skin_sensor_size skin_object_sensors_count(skin_object *so)			{ return so->p_sensors_count; }
static inline struct skin_sensor_type *skin_object_sensor_types(skin_object *so, skin_sensor_type_size *count)
{
	if (count)
		*count = so->p_sensor_types_count;
	return so->p_sensor_types;
}
static inline skin_sensor_type_size skin_object_sensor_types_count(skin_object *so)		{ return so->p_sensor_types_count; }

#ifdef __cplusplus
}
#endif

#endif
