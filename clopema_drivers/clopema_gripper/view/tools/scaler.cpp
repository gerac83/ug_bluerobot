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

#include <algorithm>
#include "scaler.h"

using namespace std;

scaler::scaler()
{
	min_range = 500;
}

scaler::scaler(unsigned int c)
{
	initialize(c);
	min_range = 500;
}

scaler::scaler(unsigned int c, unsigned int r)
{
	initialize(c);
	set_range(r);
}

scaler::~scaler()
{
}

void scaler::initialize(unsigned int count)
{
	mins.resize(count);
	fill(mins.begin(), mins.end(), 65535);
	maxes.resize(count);
	fill(maxes.begin(), maxes.end(), 0);
}

void scaler::set_range(unsigned int r)
{
	if (r == 0)
		r = 1;
	min_range = r;
}

#include <iostream>

using namespace std;

void scaler::scale(const uint16_t *r, unsigned int count, std::vector<uint8_t> &result)
{
	if (result.size() < count)
		count = result.size();
	for (unsigned int i = 0; i < count; ++i)
		result[i] = r[i];
}

vector<uint8_t> scaler::scale(const uint16_t *r, unsigned int count)
{
	vector<uint16_t> v(r, r + count);
	return scale(v);
}

#define FIX_RESPONSE \
	do {\
		if (response < /*800*/0)		/* the first time could be 0 while the driver actually starts reading */\
		{\
			result[i] = 0;\
			continue;\
		}\
		if (response < mins[i])\
			mins[i] = response;\
		if (response > maxes[i])\
			maxes[i] = response;\
		unsigned int rmin = mins[i];\
		unsigned int rmax = maxes[i];\
		response -= rmin;\
		unsigned int range = rmax - rmin;\
		if (range < min_range)\
			range = min_range;\
		response = response * 255 / range;\
		result[i] = response;\
	} while (0)

void scaler::scale(const std::vector<uint16_t> &v, std::vector<uint8_t> &result)
{
	unsigned int size = v.size();
	if (result.size() < size)
		size = result.size();
	if (size > mins.size())
		return;		// bad initialization.  Use scale(object) once before getting to real-time context
	for (unsigned int i = 0; i < size; ++i)
	{
		unsigned int response = v[i];
		FIX_RESPONSE;
	}
}

vector<uint8_t> scaler::scale(const vector<uint16_t> &v)
{
	vector<uint8_t> res(v.size(), 0);
	if (v.size() > mins.size())
		initialize(v.size());
	scale(v, res);
	return res;
}

void scaler::dampen(unsigned int amount)
{
#if 0
	if (amount > 65535)
		amount = 65535;
#else
	if (amount > 100)
		amount = 100;
#endif
	for (unsigned int i = 0, size = mins.size(); i < size; ++i)
	{
#if 0
		// linearly change the mins and maxes
		if (65535 - amount > mins[i])
			mins[i] += amount;
		else
			mins[i] = 65535;
		if (maxes[i] > amount)
			maxes[i] -= amount;
		else
			maxes[i] = 0;
#else
		// change the mins and maxes by a percentage of their range
		if (mins[i] < maxes[i])
		{
			unsigned int range = maxes[i] - mins[i];
			unsigned int to_cut = range * amount / 100;
			if (to_cut < amount)
			{
				to_cut = amount;
				if (to_cut > range)
					to_cut = range;
			}
			mins[i] += to_cut;
			maxes[i] -= to_cut;
		}
	}
#endif
}

void scaler::reset()
{
	initialize(mins.size());
}
