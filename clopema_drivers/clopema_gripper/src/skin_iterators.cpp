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

#include "clopema_gripper_skin.h"

/* skin_sensor_type's sensor iterator */
skin_sensor_type::skin_sensor_iterator skin_sensor_type::sensors_iter_begin()
{
	return skin_sensor_iterator(this, sensors_begin);
}

const skin_sensor_type::skin_sensor_iterator skin_sensor_type::sensors_iter_end()
{
	return skin_sensor_iterator(this, sensors_end);
}

skin_sensor_type::skin_sensor_iterator::skin_sensor_iterator()
{
	sensor_type = NULL;
	current = 0;
}

skin_sensor_type::skin_sensor_iterator::skin_sensor_iterator(skin_sensor_type *st, skin_sensor_id c)
{
	sensor_type = st;
	current = c;
}

skin_sensor_type::skin_sensor_iterator &skin_sensor_type::skin_sensor_iterator::operator ++()
{
	++current;
	return *this;
}

skin_sensor_type::skin_sensor_iterator skin_sensor_type::skin_sensor_iterator::operator ++(int)
{
	skin_sensor_iterator i = *this;
	++*this;
	return i;
}

bool skin_sensor_type::skin_sensor_iterator::operator ==(const skin_sensor_iterator &rhs) const
{
	return current == rhs.current;
}

bool skin_sensor_type::skin_sensor_iterator::operator !=(const skin_sensor_iterator &rhs) const
{
	return current != rhs.current;
}

skin_sensor &skin_sensor_type::skin_sensor_iterator::operator *() const
{
	return sensor_type->p_object->p_sensors[current];
}

skin_sensor *skin_sensor_type::skin_sensor_iterator::operator ->() const
{
	return &sensor_type->p_object->p_sensors[current];
}

skin_sensor_type::skin_sensor_iterator::operator skin_sensor *() const
{
	if (!sensor_type || !sensor_type->p_object || !sensor_type->p_object->p_sensors)
		return NULL;
	return sensor_type->p_object->p_sensors + current;
}

/* skin_object's sensor iterator */
skin_object::skin_sensor_iterator skin_object::sensors_iter_begin()
{
	return skin_sensor_iterator(this, 0);
}

const skin_object::skin_sensor_iterator skin_object::sensors_iter_end()
{
	return skin_sensor_iterator(this, p_sensors_count);
}

skin_object::skin_sensor_iterator::skin_sensor_iterator()
{
	object = NULL;
	current = 0;
}

skin_object::skin_sensor_iterator::skin_sensor_iterator(skin_object *obj, skin_sensor_id c)
{
	object = obj;
	current = c;
}

skin_object::skin_sensor_iterator &skin_object::skin_sensor_iterator::operator ++()
{
	++current;
	return *this;
}

skin_object::skin_sensor_iterator skin_object::skin_sensor_iterator::operator ++(int)
{
	skin_sensor_iterator i = *this;
	++*this;
	return i;
}

bool skin_object::skin_sensor_iterator::operator ==(const skin_sensor_iterator &rhs) const
{
	return current == rhs.current;
}

bool skin_object::skin_sensor_iterator::operator !=(const skin_sensor_iterator &rhs) const
{
	return current != rhs.current;
}

skin_sensor &skin_object::skin_sensor_iterator::operator *() const
{
	return object->p_sensors[current];
}

skin_sensor *skin_object::skin_sensor_iterator::operator ->() const
{
	return &object->p_sensors[current];
}

skin_object::skin_sensor_iterator::operator skin_sensor *() const
{
	if (!object || !object->p_sensors)
		return NULL;
	return object->p_sensors + current;
}

/* skin_object's sensor type iterator */
skin_object::skin_sensor_type_iterator skin_object::sensor_types_iter_begin()
{
	return skin_sensor_type_iterator(this, 0);
}

const skin_object::skin_sensor_type_iterator skin_object::sensor_types_iter_end()
{
	return skin_sensor_type_iterator(this, p_sensor_types_count);
}

skin_object::skin_sensor_type_iterator::skin_sensor_type_iterator()
{
	object = NULL;
	current = 0;
}

skin_object::skin_sensor_type_iterator::skin_sensor_type_iterator(skin_object *obj, skin_sensor_type_id c)
{
	object = obj;
	current = c;
}

skin_object::skin_sensor_type_iterator &skin_object::skin_sensor_type_iterator::operator ++()
{
	++current;
	return *this;
}

skin_object::skin_sensor_type_iterator skin_object::skin_sensor_type_iterator::operator ++(int)
{
	skin_sensor_type_iterator i = *this;
	++*this;
	return i;
}

bool skin_object::skin_sensor_type_iterator::operator ==(const skin_sensor_type_iterator &rhs) const
{
	return current == rhs.current;
}

bool skin_object::skin_sensor_type_iterator::operator !=(const skin_sensor_type_iterator &rhs) const
{
	return current != rhs.current;
}

skin_sensor_type &skin_object::skin_sensor_type_iterator::operator *() const
{
	return object->p_sensor_types[current];
}

skin_sensor_type *skin_object::skin_sensor_type_iterator::operator ->() const
{
	return &object->p_sensor_types[current];
}

skin_object::skin_sensor_type_iterator::operator skin_sensor_type *() const
{
	if (!object || !object->p_sensor_types)
		return NULL;
	return object->p_sensor_types + current;
}
