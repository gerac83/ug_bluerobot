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

#include "filter.h"

using namespace std;

filter::filter()
{
	type = FILTER_AVERAGE;
	size = 1;
}

filter::filter(filter_type t)
{
	type = t;
	size = 1;
}

filter::filter(unsigned int s)
{
	type = FILTER_AVERAGE;
	size = s;
}

filter::filter(filter_type t, unsigned int s)
{
	type = t;
	size = s;
}

filter::~filter()
{
}

void filter::change_type(filter_type t)
{
	type = t;
}

void filter::change_size(unsigned int s)
{
	responses.clear();
	size = s;
}

void filter::new_responses(const uint8_t *r, unsigned int count)
{
	++current;
	if (current >= size)
		current = 0;
	vector<uint8_t> v(r, r + count);
	if (responses.size() < size)
		while (responses.size() < size)
			responses.push_back(v);
	else
		responses[current] = v;
}

void filter::new_responses(const vector<uint8_t> &v)
{
	++current;
	if (current >= size)
		current = 0;
	if (responses.size() < size)
		while (responses.size() < size)
			responses.push_back(v);
	else
	{
		unsigned int sz = v.size();
		if (responses[current].size() < sz)
			sz = responses[current].size();
		for (unsigned int i = 0; i < sz; ++i)
			responses[current][i] = v[i];
	}
}

uint8_t filter::get_response(uint16_t id) const
{
	unsigned int result = 0;
	unsigned int count = 0;
	for (unsigned int i = 0; i < size; ++i)
	{
		unsigned int c = current + i + 1;
		if (c >= size)
			c -= size;
		if (c >= responses.size())
			continue;
		switch (type)
		{
			default:
			case FILTER_AVERAGE:
				result += responses[c][id];
				++count;
				break;
		}
	}
	switch (type)
	{
		default:
		case FILTER_AVERAGE:
			if (count == 0)
				result = 0;
			else
				result /= count;
			break;
	}
	return result;
}

void filter::get_responses(vector<uint8_t> &result) const
{
	if (size == 0)
		return;
	unsigned int sz = result.size();
	if (responses[0].size() < sz)
		sz = responses[0].size();
	for (unsigned int i = 0, s = responses[0].size(); i < s; ++i)
		result[i] = get_response(i);
}

vector<uint8_t> filter::get_responses() const
{
	if (size == 0)
		return vector<uint8_t>();
	vector<uint8_t> res(responses[0].size());
	get_responses(res);
	return res;
}
