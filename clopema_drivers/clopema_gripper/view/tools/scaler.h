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

#ifndef SCALER_H_BY_CYCLOPS
#define SCALER_H_BY_CYCLOPS

#include <vector>
#include <stdint.h>

class scaler
{
private:
	unsigned int min_range;
	std::vector<uint16_t> mins, maxes;
	void initialize(unsigned int count);
public:
	scaler();
	scaler(unsigned int c);		// If scaler is to be used in real-time threads, us this constructor or call scale(*) once to preallocate memory
	scaler(unsigned int c, unsigned int r);
	~scaler();
	void set_range(unsigned int r);
	void scale(const uint16_t *, unsigned int count, std::vector<uint8_t> &result);
	void scale(const std::vector<uint16_t> &, std::vector<uint8_t> &result);
	void dampen(unsigned int amount);
	void reset();
	// Convenience functions - Not to be used in real-time context
	std::vector<uint8_t> scale(const uint16_t *, unsigned int count);
	std::vector<uint8_t> scale(const std::vector<uint16_t> &);
};

#endif
