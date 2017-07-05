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

#ifndef AMPLIFIER_H_BY_CYCLOPS
#define AMPLIFIER_H_BY_CYCLOPS

#include <vector>
#include <stdint.h>

enum amplifier_type
{
	AMPLIFIER_LINEAR,		// apply a linear function (ax+b) and cap the result
	AMPLIFIER_QUADRATIC,		// apply a quadratic function ((1/a)(x+b)^2+c)
	AMPLIFIER_CUSTOM		// apply a custom callback
};

typedef uint8_t (*amplifier_callback)(uint8_t);

class amplifier
{
private:
	amplifier_type type;
	double a, b, c;
	amplifier_callback cb;
public:
	amplifier();
	amplifier(double, double);
	amplifier(double, double, double);
	amplifier(amplifier_callback);
	~amplifier();
	void make_linear(double, double);
	void make_quadatic(double, double, double);
	void make_custom(amplifier_callback);
	uint8_t amplify(uint8_t);
	void amplify(const std::vector<uint8_t> &, std::vector<uint8_t> &result);
	// Convenience function - Not to be used in real-time context
	std::vector<uint8_t> amplify(const std::vector<uint8_t> &);
};

#endif
