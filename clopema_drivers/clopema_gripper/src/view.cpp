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

#include <SDL/SDL.h>
#include <SDL/SDL_opengl.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstring>
#include <string>
#include <dirent.h>
#include <unistd.h>
#include <signal.h>
#include <Ngin.h>
#include <Ngin3d.h>
#include <shfont.h>
#include <triangulate.h>
#include <scaler.h>
#include <filter.h>
#include <amplifier.h>
#include <vecmath.h>
#include <clopema_gripper_skin.h>
#ifdef THROUGH_ROS
#include <clopema_gripper_rosskin.h>
#include <clopema_gripper/MotorPosition.h>
#include <clopema_gripper/MotorStatus.h>
#include <clopema_gripper/MoveAbsoluteUpdatePercentage.h>
#include <clopema_gripper/ForcePercentage.h>
#include <clopema_gripper/CalibratePosition.h>
#include <clopema_gripper/CalibrateForce.h>
#include <clopema_gripper/CalibrateAmbientProximity.h>
#include <clopema_gripper/Stop.h>
#include <clopema_gripper/SetGripperState.h>
#include <clopema_gripper/SetFrequency.h>
#include <clopema_rubbing/Calibrate.h>
#include <clopema_rubbing/Stiffness.h>
#include <clopema_rubbing/Rubbing.h>
#else
#include <clopema_gripper_motor.h>
#endif
#include <pthread.h>
#include "shconio.h"

using namespace std;

ofstream				fout("log.out");

bool					fullscreen		= false;
int					res_width		= 800;
int					res_height		= 600;
#define					SH_WIDTH		res_width
#define					SH_HEIGHT		res_height
#define					FONT_SCALE		(res_width / 1000.0f)

/* skin */
#ifdef THROUGH_ROS
static clopema_gripper::Connector	connector;
static skin_object			&skin			= connector.skin;
#else
static skin_object			skin;
static skin_reader			*reader;
static urt_sem				*read_sem		= NULL;
#endif
static volatile sig_atomic_t		stop_read		= false;
static const char			*shmem_name		= NULL;
static const char			*mutex_name		= NULL;
static bool				calibration_file_given	= false;
static string				calibration_file;

/* motor */
struct motor_data
{
	float				position;
	bool				status_opening;
	bool				status_closing;
	bool				status_pos_cal;
	bool				status_force_cal;
	bool				status_prox_cal;
#ifdef THROUGH_ROS
	ros::Subscriber			pos_sub;
	ros::Subscriber			stat_sub;
	ros::ServiceClient		move_client;
	ros::ServiceClient		stop_client;
	ros::ServiceClient		calib_pos_client;
	ros::ServiceClient		calib_force_client;
	ros::ServiceClient		calib_prox_client;
	ros::ServiceClient		set_state_client;
	ros::ServiceClient		force_client;
	ros::ServiceClient		set_freq_client;

	void pos_callback(const clopema_gripper::MotorPosition::ConstPtr &msg);
	void stat_callback(const clopema_gripper::MotorStatus::ConstPtr &msg);
#else
	gripper_motor			*motor;
#endif
	bool				stop;
	bool				open;
	bool				close;
	bool				fully_close;
	bool				fully_open;
	bool				calibrate_pos;
	bool				calibrate_force;
	bool				calibrate_prox;
	bool				calibrate_reset;
	bool				no_response;
	int				force;
	bool				more_force;
	bool				less_force;
	bool				force_control_reset;
	uint16_t			frequency;
};

#ifdef THROUGH_ROS
struct rubbing_data
{
	bool				calibrate;
	int				stiffness;
	int				count;
	bool				rub;
	bool				stop;
	bool				no_response;

	ros::ServiceClient		calib_client;
	ros::ServiceClient		stiffness_client;
	ros::ServiceClient		rub_client;
};
#endif

static motor_data			motors[2]		= {{0}};
#ifdef THROUGH_ROS
static rubbing_data			rubbings[2]		= {{0}};
#endif
static bool				dual_limb		= true;		// whether both limbs are attached
static uint8_t				which_limb		= 0;		// either 1 (right) or 0 (left)

/* rendering data */
static vector<vector<triangle> >	modules_triangulated;
#define					TEMP_MESSAGE_TIME	3000
static string				temp_message;
static unsigned int			t_temp_message;
static bool				gripped[2]		= {false, false};

/* processing */
static bool				raw_results		= false;	// if raw results, all do_* will be ignored
static bool				do_scale		= true;
static bool				do_dampen		= false;
static bool				do_filter		= false;
static bool				do_amplify		= true;
static bool				raw_remove_baseline	= false;	// only if raw results
static int				filter_size		= 3;
static unsigned int			damp_size		= 6;
static unsigned int			t_last_damp		= 0;
#define					DAMP_PERIOD		100
static amplifier			amp(-255, -255, 255);
struct layer_processed
{
	vector<uint8_t>	responses;
	scaler		sclr;
	filter		fltr;
};
/* TODO: turn processed layers into 2 layers, so the taxels from each finger can be manipulated individually */
static layer_processed			processed_layers;
static vector<skin_sensor_response>	baseline_response;

/* gui */
static bool				no_gui			= false;
static bool				show_triangles		= false;
static bool				show_values		= false;
static bool				show_ids		= false;
static bool				show_keys		= false;
static float				meter_scale		= 1.0f;
static float				sensor_radius_mult	= 1.5;
static float				value_font_size		= 0.01;
static float				sensor_color_good[]	= {0, 0.5f, 1};
static float				sensor_color_gripped[]	= {0, 1, 0.5f};

/* output */
static bool				log_data		= false;
struct logger
{
	ofstream	lout;
	urt_time	last_write;
	logger() { last_write = 0; }
	bool open(unsigned int i) { char name[40]; sprintf(name, "taxel%02u", i); lout.open(name); return lout.is_open(); }
	void close() { if (lout.is_open()) lout.close(); }
	void write(urt_time t, uint16_t r) { if (last_write != t) lout << t << " " << r << "\n"; last_write = t; }
};
static logger				loggers[MAX_MODULES * MAX_SENSORS_PER_MODULE];

/* other */
static volatile sig_atomic_t		_must_exit		= 0;
static int				_error_code		= EXIT_SUCCESS;

static unsigned int get_time_ms()
{
	if (no_gui)
	{
		static struct timespec start_time = {0};
		struct timespec t;

		if (start_time.tv_sec == 0 && start_time.tv_nsec == 0)
			clock_gettime(CLOCK_MONOTONIC, &start_time);
		clock_gettime(CLOCK_MONOTONIC, &t);
		return (t.tv_sec - start_time.tv_sec) * 1000 + (t.tv_nsec - start_time.tv_nsec) / 1000000;
	}
	else
		return SDL_GetTicks();
}

void toggle_log()
{
	if (log_data)
		for (unsigned int i = 0; i < skin.sensors_count(); ++i)
			loggers[i].close();
	log_data = !log_data;
	if (log_data)
		for (unsigned int i = 0; i < skin.sensors_count(); ++i)
			loggers[i].open(i);
}

void quit(int error_code)
{
	_must_exit = 1;
	stop_read = 1;
	motors[0].stop = true;
	motors[1].stop = true;
#ifdef THROUGH_ROS
	rubbings[0].stop = true;
	rubbings[1].stop = true;
#endif
	_error_code = error_code;
}

void signal_handler(int signal)
{
	quit(signal);
}

void cleanup()
{
	if (log_data)
		toggle_log();
	fout << "Quitting with error code " << _error_code << endl;
#ifdef THROUGH_ROS
	connector.unload();
#else
	skin.unload();
	if (read_sem)
		urt_sem_delete(read_sem);
#endif
	if (!no_gui)
	{
		shNgin3dQuit();
		shNginQuit();
		SDL_Quit();
	}
}

bool initializing = true;

void initialize_SDL()
{
	if (SDL_Init(SDL_INIT_VIDEO) == -1)
	{
		fout << "Could not initialize SDL video..." << SDL_GetError() << endl;
		fout << "Exiting program" << endl;
		exit(0);
	}
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
	fflush(stdout);
	FILE *resin = popen("xdpyinfo | grep 'dimensions:'", "r");
	if (resin)
	{
		int w, h;
		if (fscanf(resin, "%*s %dx%d", &w, &h) != 2)
			fout << "Failed to detect screen resolution.  Using defaults" << endl;
		else
		{
			res_width = w;
			res_height = h;
			fout << "Screen resolution detected as " << res_width << "x" << res_height << endl;
		}
		pclose(resin);
	}
	if (!fullscreen)
	{
//		res_height -= 100;
//		res_width -= 75;
		res_height /= 2;
		res_width /= 2;
	}
	if (SDL_SetVideoMode(res_width, res_height, 0, SDL_OPENGL | (fullscreen?SDL_FULLSCREEN:0)) == NULL)
	{
		fout << "Could not open an OpenGL window..." << SDL_GetError() << endl;
		quit(EXIT_FAILURE);
		return;
	}
	initializing = true;
	SDL_ShowCursor(SDL_DISABLE);
}

static void recalc_baseline()
{
	skin_sensor *sensors = skin.sensors();
	for (unsigned int i = 0; i < skin.sensors_count(); ++i)
		baseline_response[i] = sensors[i].get_response();
}

void keyboard(SDL_KeyboardEvent *kbe)
{
	shNgin3dReceivedInput(NGIN3D_KEY, (shNgin3dKey)kbe->keysym.sym, (kbe->type == SDL_KEYDOWN)?NGIN3D_PRESSED:NGIN3D_RELEASED);
	switch (kbe->keysym.sym)
	{
		/* processing */
		case SDLK_r:
			if (kbe->type == SDL_KEYUP)
			{
				raw_results = !raw_results;
				if (raw_results)
				{
					value_font_size *= 0.7;
					recalc_baseline();
				}
				else
					value_font_size /= 0.7;
			}
			break;
		case SDLK_h:
			if (kbe->type == SDL_KEYUP)
				do_dampen = !do_dampen;
			break;
		case SDLK_f:
			if (kbe->type == SDL_KEYUP)
				do_filter = !do_filter;
			break;
		case SDLK_l:
			if (kbe->type == SDL_KEYUP)
				do_scale = !do_scale;
			break;
		case SDLK_k:
			if (kbe->type == SDL_KEYUP)
				do_amplify = !do_amplify;
			break;
		case SDLK_b:
			if (kbe->type == SDL_KEYUP)
			{
				raw_remove_baseline = !raw_remove_baseline;
				if (raw_remove_baseline)
					recalc_baseline();
			}
			break;
		case SDLK_RETURN:
			if (kbe->type == SDL_KEYUP)
				processed_layers.sclr.reset();
			break;
		case SDLK_MINUS:
		case SDLK_UNDERSCORE:
			if (kbe->type == SDL_KEYUP)
				if (filter_size > 2)
				{
					--filter_size;
					processed_layers.fltr.change_size(filter_size);
				}
			break;
		case SDLK_PLUS:
		case SDLK_EQUALS:
			if (kbe->type == SDL_KEYUP)
			{
				++filter_size;
				processed_layers.fltr.change_size(filter_size);
			}
			break;
		/* motor */
		case SDLK_GREATER:
		case SDLK_PERIOD:
			motors[which_limb].force = 0;
			motors[which_limb].less_force = 0;
			motors[which_limb].more_force = 0;
			motors[which_limb].force_control_reset = true;
			motors[which_limb].close = false;
			if (kbe->keysym.mod & (KMOD_LSHIFT | KMOD_RSHIFT))
			{
				if (kbe->type == SDL_KEYUP)
					motors[which_limb].fully_close = true;
			}
			else
			{
				if (kbe->type == SDL_KEYDOWN)
					motors[which_limb].close = true;
			}
			break;
		case SDLK_LESS:
		case SDLK_COMMA:
			motors[which_limb].force = 0;
			motors[which_limb].less_force = 0;
			motors[which_limb].more_force = 0;
			motors[which_limb].force_control_reset = true;
			motors[which_limb].open = false;
			if (kbe->keysym.mod & (KMOD_LSHIFT | KMOD_RSHIFT))
			{
				if (kbe->type == SDL_KEYUP)
					motors[which_limb].fully_open = true;
			}
			else
			{
				if (kbe->type == SDL_KEYDOWN)
					motors[which_limb].open = true;
			}
			break;
		case SDLK_LEFTBRACKET:
		case 250:
			motors[which_limb].less_force = false;
			if (kbe->keysym.mod & (KMOD_LSHIFT | KMOD_RSHIFT))
			{
				if (kbe->type == SDL_KEYUP)
				{
					motors[which_limb].force = 0;
					motors[which_limb].force_control_reset = true;
				}
			}
			else
			{
				if (kbe->type == SDL_KEYDOWN)
					motors[which_limb].less_force = true;
			}
			break;
		case SDLK_RIGHTBRACKET:
		case 228:
			motors[which_limb].more_force = false;
			if (kbe->keysym.mod & (KMOD_LSHIFT | KMOD_RSHIFT))
			{
				if (kbe->type == SDL_KEYUP)
					motors[which_limb].force = 100;
			}
			else
			{
				if (kbe->type == SDL_KEYDOWN)
					motors[which_limb].more_force = true;
			}
			break;
		case SDLK_F1:
			motors[which_limb].force = 0;
			motors[which_limb].less_force = 0;
			motors[which_limb].more_force = 0;
			motors[which_limb].force_control_reset = true;
			if (kbe->type == SDL_KEYUP)
				motors[which_limb].calibrate_pos = true;
			break;
		case SDLK_F2:
			motors[which_limb].force = 0;
			motors[which_limb].less_force = 0;
			motors[which_limb].more_force = 0;
			motors[which_limb].force_control_reset = true;
			if (kbe->type == SDL_KEYUP)
				motors[which_limb].calibrate_force = true;
			break;
		case SDLK_F3:
			motors[which_limb].force = 0;
			motors[which_limb].less_force = 0;
			motors[which_limb].more_force = 0;
			motors[which_limb].force_control_reset = true;
			if (kbe->type == SDL_KEYUP)
				motors[which_limb].calibrate_prox = true;
			break;
		case SDLK_F4:
			if (kbe->type == SDL_KEYUP)
			{
				if (motors[which_limb].frequency == 25000)
					motors[which_limb].frequency = 2500;
				else
					motors[which_limb].frequency = 25000;
			}
			break;
		case SDLK_F9:
			if (kbe->type == SDL_KEYUP)
				/* TODO: CHECK TO SEE IF SOME OTHER VARIABLE NEEDS TO BE RESET HERE */
				which_limb = 1 - which_limb;
			break;
		case SDLK_F12:
			if (kbe->type == SDL_KEYUP && (kbe->keysym.mod & KMOD_RCTRL))
			{
				cout << "Force reset of calibration" << endl;
				motors[which_limb].calibrate_reset = true;
			}
			break;
#ifdef THROUGH_ROS
		/* rubbing */
		case SDLK_F5:
			if (kbe->type == SDL_KEYUP)
				rubbings[which_limb].calibrate = true;
			break;
		case SDLK_F6:
			if (kbe->type == SDL_KEYUP)
			{
				if (rubbings[which_limb].stiffness <= 10)
					rubbings[which_limb].stiffness += 5;
				else
					rubbings[which_limb].stiffness = 15;
			}
			break;
		case SDLK_F7:
			if (kbe->type == SDL_KEYUP)
			{
				if (rubbings[which_limb].stiffness >= 5)
					rubbings[which_limb].stiffness -= 5;
				else
					rubbings[which_limb].stiffness = 0;
			}
			break;
		case SDLK_F8:
			if (kbe->type == SDL_KEYUP)
				rubbings[which_limb].rub = true;
			break;
		case SDLK_KP_PLUS:
			if (kbe->type == SDL_KEYUP && rubbings[which_limb].count < 5)
				++rubbings[which_limb].count;
			break;
		case SDLK_KP_MINUS:
			if (kbe->type == SDL_KEYUP && rubbings[which_limb].count > 1)
				--rubbings[which_limb].count;
			break;
#endif
		/* gui */
		case SDLK_t:
			if (kbe->type == SDL_KEYUP)
				show_triangles = !show_triangles;
			break;
		case SDLK_v:
			if (kbe->type == SDL_KEYUP)
			{
				show_values = !show_values;
				if (show_values)
					show_ids = false;
			}
			break;
		case SDLK_i:
			if (kbe->type == SDL_KEYUP)
			{
				show_ids = !show_ids;
				if (show_ids)
					show_values = false;
			}
			break;
		case SDLK_PAGEUP:
			if (kbe->type == SDL_KEYUP)
				sensor_radius_mult *= 1.1;
			break;
		case SDLK_PAGEDOWN:
			if (kbe->type == SDL_KEYUP)
				if (sensor_radius_mult > 0.01)
					sensor_radius_mult /= 1.1;
			break;
		/* output */
		case SDLK_p:
			if (kbe->type == SDL_KEYUP)
				toggle_log();
			break;
		case SDLK_z:
			if (kbe->type == SDL_KEYUP)
				show_keys = !show_keys;
			break;
		/* other */
		case SDLK_ESCAPE:
			quit(EXIT_SUCCESS);
			break;
		default:
			break;
	}
}

int warpmotionx = 0, warpmotiony = 0;

void mouse_motion(SDL_MouseMotionEvent *mme)
{
	if (initializing)
	{
		if (mme->x == res_width / 2 && mme->y == res_height / 2)
			initializing = false;
		else
			SDL_WarpMouse(res_width / 2, res_height / 2);
	}
	else if (mme->xrel != -warpmotionx || -mme->yrel != -warpmotiony)
	{
		shNgin3dReceivedInput(NGIN3D_MOUSE_MOVE, mme->xrel, -mme->yrel);
		warpmotionx += mme->xrel;
		warpmotiony += -mme->yrel;
		SDL_WarpMouse(res_width / 2, res_height / 2);
	}
	else
		warpmotionx = warpmotiony = 0;
}

void *motor_handler(void *l)
{
	bool needs_stop = false;
	int recently_moved = 0;
	bool previously_force_controlling = false;
	int limb = *(int *)l;
	uint16_t previous_frequency = motors[limb].frequency;
	while (!motors[limb].stop)
	{
		if (recently_moved)
			--recently_moved;
		if (previously_force_controlling && motors[limb].force_control_reset)
		{
			motors[limb].force = 0;
#ifdef THROUGH_ROS
			clopema_gripper::ForcePercentage srv;
			srv.request.force = 0;
			motors[limb].no_response = motors[limb].force_client.call(srv) == false;
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_MOV_FORCE_PER, 0);
#endif
			previously_force_controlling = false;
		}
		motors[limb].force_control_reset = false;

		if (motors[limb].calibrate_reset)
		{
#ifdef THROUGH_ROS
			/* no force reset exposed through ROS */
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_CALIB_FORCE_RESET);
#endif
			motors[limb].calibrate_reset = false;
		}
		else if (motors[limb].calibrate_pos)
		{
#ifdef THROUGH_ROS
			clopema_gripper::CalibratePosition srv;
			motors[limb].no_response = motors[limb].calib_pos_client.call(srv) == false;
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_CALIB_POS);
#endif
			motors[limb].calibrate_pos = false;
		}
		else if (motors[limb].calibrate_force)
		{
#ifdef THROUGH_ROS
			clopema_gripper::CalibrateForce srv;
			motors[limb].no_response = motors[limb].calib_force_client.call(srv) == false;
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_CALIB_FORCE);
#endif
			motors[limb].calibrate_force = false;
		}
		else if (motors[limb].calibrate_prox)
		{
#ifdef THROUGH_ROS
			clopema_gripper::CalibrateAmbientProximity srv;
			motors[limb].no_response = motors[limb].calib_prox_client.call(srv) == false;
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_CALIB_AMB_PROX);
#endif
			motors[limb].calibrate_prox = false;
		}
#define MOTOR_MOVE_SPEED 5
		else if (motors[limb].fully_close)
		{
#ifdef THROUGH_ROS
			clopema_gripper::SetGripperState srv;
			srv.request.open = 0;
			motors[limb].no_response = motors[limb].set_state_client.call(srv) == false;
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_MOV_CLOSE);
#endif
			motors[limb].fully_close = false;
		}
		else if (motors[limb].fully_open)
		{
#ifdef THROUGH_ROS
			clopema_gripper::SetGripperState srv;
			srv.request.open = 1;
			motors[limb].no_response = motors[limb].set_state_client.call(srv) == false;
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_MOV_OPEN);
#endif
			motors[limb].fully_open = false;
		}
		else if (motors[limb].frequency != previous_frequency)
		{
#ifdef THROUGH_ROS
			clopema_gripper::SetFrequency srv;
			srv.request.frequency = motors[limb].frequency;
			motors[limb].no_response = motors[limb].set_freq_client.call(srv) == false;
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_SET_FREQ, motors[limb].frequency);
#endif
			previous_frequency = motors[limb].frequency;
		}
		else if (motors[limb].force > 0 || motors[limb].less_force || motors[limb].more_force)
		{
#define MOTOR_FORCE_SPEED 5
			motors[limb].force += -(motors[limb].less_force?MOTOR_FORCE_SPEED:0) + (motors[limb].more_force?MOTOR_FORCE_SPEED:0);
			if (motors[limb].force < 0)
				motors[limb].force = 0;
			if (motors[limb].force > 100)
				motors[limb].force = 100;

#ifdef THROUGH_ROS
			clopema_gripper::ForcePercentage srv;
			srv.request.force = motors[limb].force;
			motors[limb].no_response = motors[limb].force_client.call(srv) == false;
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_MOV_FORCE_PER, motors[limb].force);
#endif
			previously_force_controlling = true;
		}
		else if (motors[limb].open || motors[limb].close)
		{
			if (!recently_moved)
			{
				float new_position = motors[limb].position - (motors[limb].open?MOTOR_MOVE_SPEED:0) + (motors[limb].close?MOTOR_MOVE_SPEED:0);
				if (new_position < 0)
					new_position = 0;
				if (new_position > 100)
					new_position = 100;
#ifdef THROUGH_ROS
				clopema_gripper::MoveAbsoluteUpdatePercentage srv;

				srv.request.movement = new_position;

				motors[limb].no_response = motors[limb].move_client.call(srv) == false;
				recently_moved = 1;
#else
				motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_MOV_ABS_UPD_PER, new_position);
				recently_moved = 1;
#endif
				if (!motors[limb].no_response)
					needs_stop = true;
			}
		}
		else if (needs_stop)
		{
#ifdef THROUGH_ROS
			clopema_gripper::Stop srv;
			motors[limb].no_response = motors[limb].stop_client.call(srv) == false;
#else
			motors[limb].no_response = motors[limb].motor->execute(MOTOR_CMD_MOV_STOP);
#endif
			needs_stop = false;
		}
		usleep(50000);
	}
	return NULL;
}

#ifdef THROUGH_ROS
void *rubbing_handler(void *l)
{
	int previous_stiffness = 0;
	int limb = *(int *)l;
	while (!rubbings[limb].stop)
	{
		int new_stiffness = rubbings[limb].stiffness;
		if (rubbings[limb].calibrate)
		{
			clopema_rubbing::Calibrate srv;
			rubbings[limb].no_response = rubbings[limb].calib_client.call(srv) == false;
			rubbings[limb].calibrate = false;
			if (!rubbings[limb].no_response)
				previous_stiffness = 0;
		}
		else if (new_stiffness != previous_stiffness)
		{
			clopema_rubbing::Stiffness srv;
			srv.request.stiffness = new_stiffness;
			rubbings[limb].no_response = rubbings[limb].stiffness_client.call(srv) == false;
			if (!rubbings[limb].no_response)
				previous_stiffness = new_stiffness;
		}
		else if (rubbings[limb].rub)
		{
			clopema_rubbing::Rubbing srv;
			srv.request.times = rubbings[limb].count;
			rubbings[limb].no_response = rubbings[limb].rub_client.call(srv) == false;
			rubbings[limb].rub = false;
		}
		usleep(50000);
	}
	return NULL;
}
#endif

void process_events()
{
	if (no_gui)
	{
		while (kbhit() && !_must_exit)
		{
			int c = getch();
			switch (c)
			{
			case 27:
				quit(EXIT_SUCCESS);
				break;
			default:
				break;
			}
		}
	}
	else
	{
		SDL_Event e;
		while (SDL_PollEvent(&e) && !_must_exit)
		{
			switch (e.type)
			{
				case SDL_KEYUP:
				case SDL_KEYDOWN:
					keyboard(&e.key);
					break;
				case SDL_MOUSEMOTION:
					mouse_motion(&e.motion);
					break;
				case SDL_QUIT:
					quit(EXIT_SUCCESS);
				default:
					break;
			}
		}
	}
}

shNginTexture skybox[6];		// right, front, left, back, up, down

void draw_sky_box()
{
#define SKYBOX_DIMENSION 10000
#define SKYBOX_NOTCH_REMOVE 10
	shNginEnableTexture();
	glColor3ub(255, 255, 255);
	skybox[0].shNginTBind();
	glBegin(GL_QUADS);
		skybox[0].shNginTTextureCoord(0, 0); glVertex3f(SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE, -SKYBOX_DIMENSION, -SKYBOX_DIMENSION);
		skybox[0].shNginTTextureCoord(1, 0); glVertex3f(SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE, -SKYBOX_DIMENSION, SKYBOX_DIMENSION);
		skybox[0].shNginTTextureCoord(1, 1); glVertex3f(SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE, SKYBOX_DIMENSION, SKYBOX_DIMENSION);
		skybox[0].shNginTTextureCoord(0, 1); glVertex3f(SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE, SKYBOX_DIMENSION, -SKYBOX_DIMENSION);
	glEnd();
	skybox[1].shNginTBind();
	glBegin(GL_QUADS);
		skybox[1].shNginTTextureCoord(0, 0); glVertex3f(-SKYBOX_DIMENSION, -SKYBOX_DIMENSION, -SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE);
		skybox[1].shNginTTextureCoord(1, 0); glVertex3f(SKYBOX_DIMENSION, -SKYBOX_DIMENSION, -SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE);
		skybox[1].shNginTTextureCoord(1, 1); glVertex3f(SKYBOX_DIMENSION, SKYBOX_DIMENSION, -SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE);
		skybox[1].shNginTTextureCoord(0, 1); glVertex3f(-SKYBOX_DIMENSION, SKYBOX_DIMENSION, -SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE);
	glEnd();
	skybox[2].shNginTBind();
	glBegin(GL_QUADS);
		skybox[2].shNginTTextureCoord(0, 0); glVertex3f(-SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE, -SKYBOX_DIMENSION, SKYBOX_DIMENSION);
		skybox[2].shNginTTextureCoord(1, 0); glVertex3f(-SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE, -SKYBOX_DIMENSION, -SKYBOX_DIMENSION);
		skybox[2].shNginTTextureCoord(1, 1); glVertex3f(-SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE, SKYBOX_DIMENSION, -SKYBOX_DIMENSION);
		skybox[2].shNginTTextureCoord(0, 1); glVertex3f(-SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE, SKYBOX_DIMENSION, SKYBOX_DIMENSION);
	glEnd();
	skybox[3].shNginTBind();
	glBegin(GL_QUADS);
		skybox[3].shNginTTextureCoord(0, 0); glVertex3f(SKYBOX_DIMENSION, -SKYBOX_DIMENSION, SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE);
		skybox[3].shNginTTextureCoord(1, 0); glVertex3f(-SKYBOX_DIMENSION, -SKYBOX_DIMENSION, SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE);
		skybox[3].shNginTTextureCoord(1, 1); glVertex3f(-SKYBOX_DIMENSION, SKYBOX_DIMENSION, SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE);
		skybox[3].shNginTTextureCoord(0, 1); glVertex3f(SKYBOX_DIMENSION, SKYBOX_DIMENSION, SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE);
	glEnd();
	skybox[4].shNginTBind();
	glBegin(GL_QUADS);
		skybox[4].shNginTTextureCoord(0, 0); glVertex3f(SKYBOX_DIMENSION, SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE, SKYBOX_DIMENSION);
		skybox[4].shNginTTextureCoord(1, 0); glVertex3f(-SKYBOX_DIMENSION, SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE, SKYBOX_DIMENSION);
		skybox[4].shNginTTextureCoord(1, 1); glVertex3f(-SKYBOX_DIMENSION, SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE, -SKYBOX_DIMENSION);
		skybox[4].shNginTTextureCoord(0, 1); glVertex3f(SKYBOX_DIMENSION, SKYBOX_DIMENSION - SKYBOX_NOTCH_REMOVE, -SKYBOX_DIMENSION);
	glEnd();
	skybox[5].shNginTBind();
	glBegin(GL_QUADS);
		skybox[5].shNginTTextureCoord(0, 0); glVertex3f(-SKYBOX_DIMENSION, -SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE, SKYBOX_DIMENSION);
		skybox[5].shNginTTextureCoord(1, 0); glVertex3f(SKYBOX_DIMENSION, -SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE, SKYBOX_DIMENSION);
		skybox[5].shNginTTextureCoord(1, 1); glVertex3f(SKYBOX_DIMENSION, -SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE, -SKYBOX_DIMENSION);
		skybox[5].shNginTTextureCoord(0, 1); glVertex3f(-SKYBOX_DIMENSION, -SKYBOX_DIMENSION + SKYBOX_NOTCH_REMOVE, -SKYBOX_DIMENSION);
	glEnd();
}

int FPS = 0;
unsigned int dt;

#define PI 3.14159265358979323

#ifdef THROUGH_ROS
void get_responses(clopema_gripper::Connector &con, void *d)
#else
void get_responses()
#endif
{
	skin_sensor *sensors = skin.sensors();
	int did_read = SKIN_SUCCESS;
#ifndef THROUGH_ROS
	if (read_sem)
		did_read = reader->wait_read(read_sem, 15000000, &stop_read) == SKIN_SUCCESS;
#endif
	if (!raw_results && do_dampen)
		if (get_time_ms() > t_last_damp + DAMP_PERIOD)
		{
			processed_layers.sclr.dampen(damp_size);
			t_last_damp += DAMP_PERIOD;
		}
	if (did_read != SKIN_SUCCESS)
		return;
	if (log_data)
		for (unsigned int j = 0; j < skin.sensors_count(); ++j)
		{
			skin_sensor_response response;
			urt_time received;
			response = sensors[j].get_response(&received);
			loggers[j].write(received, response);
		}
	if (raw_results || !do_scale)
		for (unsigned int j = 0; j < skin.sensors_count(); ++j)
			processed_layers.responses[j] = sensors[j].get_response() >> 8;
	else
	{
#if 1
		vector<uint16_t> all_responses(skin.sensors_count());
		for (unsigned int j = 0; j < skin.sensors_count(); ++j)
			all_responses[j] = sensors[j].get_response();
		processed_layers.responses = processed_layers.sclr.scale(all_responses);
#else
		vector<uint8_t> all_responses(skin.sensors_count());
		for (unsigned int j = 0; j < skin.sensors_count(); ++j)
		{
			uint16_t response = sensors[j].get_response();
			if (response < 5000)
				response = 0;
			else if (response > 5255)
				response = 255;
			else
				response -= 5000;
			all_responses[j] = response;
		}
		processed_layers.responses = all_responses;
#endif
	}
	if (raw_results)
		return;
	if (do_filter)
	{
		processed_layers.fltr.new_responses(processed_layers.responses);
		processed_layers.responses = processed_layers.fltr.get_responses();
	}
	if (do_amplify)
		processed_layers.responses = amp.amplify(processed_layers.responses);
}

#ifndef THROUGH_ROS
void get_motor_data()
{
	for (int i = 0; i < 2; ++i)
	{
		motors[i].position = motors[i].motor->last_position();
		uint8_t status = motors[i].motor->last_status();
		motors[i].status_opening = status & MOTOR_STATUS_OPENING;
		motors[i].status_closing = status & MOTOR_STATUS_CLOSING;
		motors[i].status_pos_cal = status & MOTOR_STATUS_POS_CALIB;
		motors[i].status_force_cal = status & MOTOR_STATUS_FORCE_CALIB;
		motors[i].status_prox_cal = status & MOTOR_STATUS_AMBPROX_CALIB;
	}
}
#endif

void draw_triangles()
{
	for (unsigned int m = 0, size = modules_triangulated.size(); m < size; ++m)
	{
		glColor3ub(128, 255, 128);
		for (unsigned int t = 0, size2 = modules_triangulated[m].size(); t < size2; ++t)
		{
			glBegin(GL_TRIANGLES);
#define TRIANGLE_DISP 0.1
			glNormal3fv(modules_triangulated[m][t].normal);
			glVertex3f(modules_triangulated[m][t].p1[0] * meter_scale,
					modules_triangulated[m][t].p1[1] * meter_scale,
					modules_triangulated[m][t].p1[2] * meter_scale - TRIANGLE_DISP);
			glVertex3f(modules_triangulated[m][t].p2[0] * meter_scale,
					modules_triangulated[m][t].p2[1] * meter_scale,
					modules_triangulated[m][t].p2[2] * meter_scale - TRIANGLE_DISP);
			glVertex3f(modules_triangulated[m][t].p3[0] * meter_scale,
					modules_triangulated[m][t].p3[1] * meter_scale,
					modules_triangulated[m][t].p3[2] * meter_scale - TRIANGLE_DISP);
			glEnd();
		}
	}
}

void draw_sensor(unsigned int taxel, bool is_gripped)
{
	skin_sensor *sensors = skin.sensors();

	/* if no acquisition from the sensor, don't show it */
	if (sensors[taxel].receive_time == 0)
		return;

	/* if not tactile, use raw value */
	uint8_t response = sensors[taxel].type > 0?sensors[taxel].get_response() >> 8:processed_layers.responses[taxel];
	uint8_t color = ((255u - response) * 2 + 0) / 3;
	float *color_coef = sensor_color_good;

	if (is_gripped)
		color_coef = sensor_color_gripped;
	else
		color_coef = sensor_color_good;
	glColor3ub(color * color_coef[0], color * color_coef[1], color * color_coef[2]);
	glPushMatrix();
	glTranslatef(sensors[taxel].relative_position[0] * meter_scale,
			sensors[taxel].relative_position[1] * meter_scale,
			sensors[taxel].relative_position[2] * meter_scale);
	float z[3] = {sensors[taxel].relative_orientation[0], sensors[taxel].relative_orientation[1], sensors[taxel].relative_orientation[2]};
	float x[3] = {0.0f, 0.0f, 1.0f};	// if normal was (0, 1, 0)
	if (!ZERO(z[0]) || !ZERO(z[1]-1.0f) || !ZERO(z[2]))
	{
		// multiply (0, 1, 0) by normal which should give a vector perpendecular to normal
		x[0] = z[2];
		x[1] = 0.0f;
		x[2] = -z[0];
	}
	float y[3];
	CROSS(y, z, x);
	float height = response / 5.0f;//(255 - response)/60.0f;
	glBegin(GL_QUADS);
	float radius = sensors[taxel].radius*sensor_radius_mult*meter_scale;
	for (float angle = 0; angle < PI; angle += 0.5)
	{
		float sa = sin(angle)*radius, ca = cos(angle)*radius;
		float sb = sin(angle+0.5)*radius, cb = cos(angle+0.5)*radius;
		glVertex3f(x[0]*ca+y[0]*sa, x[1]*ca+y[1]*sa, x[2]*ca+y[2]*sa);
		glVertex3f(x[0]*cb+y[0]*sb, x[1]*cb+y[1]*sb, x[2]*cb+y[2]*sb);
		glVertex3f(x[0]*cb+y[0]*sb+z[0]*height, x[1]*cb+y[1]*sb+z[1]*height, x[2]*cb+y[2]*sb+z[2]*height);
		glVertex3f(x[0]*ca+y[0]*sa+z[0]*height, x[1]*ca+y[1]*sa+z[1]*height, x[2]*ca+y[2]*sa+z[2]*height);

		glVertex3f(-(x[0]*ca+y[0]*sa), -(x[1]*ca+y[1]*sa), -(x[2]*ca+y[2]*sa));
		glVertex3f(-(x[0]*cb+y[0]*sb), -(x[1]*cb+y[1]*sb), -(x[2]*cb+y[2]*sb));
		glVertex3f(-(x[0]*cb+y[0]*sb)+z[0]*height, -(x[1]*cb+y[1]*sb)+z[1]*height, -(x[2]*cb+y[2]*sb)+z[2]*height);
		glVertex3f(-(x[0]*ca+y[0]*sa)+z[0]*height, -(x[1]*ca+y[1]*sa)+z[1]*height, -(x[2]*ca+y[2]*sa)+z[2]*height);
	}
	glEnd();
	glBegin(GL_TRIANGLE_FAN);
		glVertex3f(z[0]*height, z[1]*height, z[2]*height);
	for (float angle = 0; angle < PI; angle += 0.5)
	{
		float sa = sin(angle)*radius, ca = cos(angle)*radius;
		glVertex3f(x[0]*ca+y[0]*sa+z[0]*height, x[1]*ca+y[1]*sa+z[1]*height, x[2]*ca+y[2]*sa+z[2]*height);
	}
		glVertex3f(-(x[0]*radius)+z[0]*height, -(x[1]*radius)+z[1]*height, -(x[2]*radius)+z[2]*height);
	glEnd();
	glBegin(GL_TRIANGLE_FAN);
		glVertex3f(z[0]*height, z[1]*height, z[2]*height);
	for (float angle = 0; angle < PI; angle += 0.5)
	{
		float sa = sin(angle)*radius, ca = cos(angle)*radius;
		glVertex3f(-(x[0]*ca+y[0]*sa)+z[0]*height, -(x[1]*ca+y[1]*sa)+z[1]*height, -(x[2]*ca+y[2]*sa)+z[2]*height);
	}
		glVertex3f(x[0]*radius+z[0]*height, x[1]*radius+z[1]*height, x[2]*radius+z[2]*height);
	glEnd();

	glPopMatrix();
}

void draw_value(unsigned int taxel)
{
	skin_sensor *sensors = skin.sensors();

	/* if no acquisition from the sensor, don't show it */
	if (sensors[taxel].receive_time == 0)
		return;

	glPushMatrix();
	glTranslatef(sensors[taxel].relative_position[0] * meter_scale,
			sensors[taxel].relative_position[1] * meter_scale,
			sensors[taxel].relative_position[2] * meter_scale);
	float z[3] = {sensors[taxel].relative_orientation[0], sensors[taxel].relative_orientation[1], sensors[taxel].relative_orientation[2]};
	uint8_t response = sensors[taxel].type > 0?sensors[taxel].get_response() >> 8:processed_layers.responses[taxel];
	float height = response / 5.0f;//(255 - response)/60.0f;

	shFontColor(0, 255, 0);
	shFontSize(value_font_size * (raw_results || sensors[taxel].type > 0?1.0f:1.5f));
	char value[10] = "";
	float loc = height + 1;
	if (show_values)
		if (raw_results || sensors[taxel].type > 0)
		{
			skin_sensor_response r = skin.sensors()[taxel].get_response();	// in this case, get the most recent value circumventing the rt thread
			/* if baseline is being removed, remove it only for tactile sensors */
			if (sensors[taxel].type == 0 && raw_remove_baseline)
			{
#if 0
				if (baseline_response[taxel] > r)
					baseline_response[taxel] = r;
				r -= baseline_response[taxel];
				snprintf(value, 10, "%u", r);
#else
				int r2 = (int)r - baseline_response[taxel];
				snprintf(value, 10, "%d", r2);
#endif
			}
			else
				snprintf(value, 10, "%u", r);
		}
		else
			snprintf(value, 10, "%u", response);
	else // if (show_ids)
		snprintf(value, 10, "%u", taxel);
	glTranslatef(z[0] * loc - shFontTextWidth(value) / 2, z[1] * loc + shFontTextHeight(value) / 2, z[2] * loc);
	shFontWrite(NULL, "%s", value);

	glPopMatrix();
}

shNginTexture unige_logo;

void draw_hud_motor_position_status(int line, int limb)
{
	glLoadIdentity();
	glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line, 0);
	shFontColor(255, 255, 255);
	if (!dual_limb)
	{
		shFontWrite(NULL, "Motor @%5.2f%% (%s)", (int)(motors[limb].position * 100) / 100.0f, motors[limb].frequency == 25000?"Fast":"Slow");
		glTranslatef(shFontTextWidth("Motor @000.00% (XXXX) "), 0, 0);
	}
	else
	{
		if (limb == 1)
		{
			/* move to the right side of the screen */
			glTranslatef(SH_WIDTH - 40 - shFontTextWidth("RIGHT Motor @000.00% (XXXX) "), 0, 0);
			glTranslatef(-shFontTextWidth(motors[limb].no_response?"-- No communication":motors[limb].status_opening?
						"-- Opening":motors[limb].status_closing?"-- Closing":""), 0, 0);
		}
		shFontWrite(NULL, "%s Motor @%5.2f%% (%s)", limb == 0?" Left":"Right",
				(int)(motors[limb].position * 100) / 100.0f, motors[limb].frequency == 25000?"Fast":"Slow");
		glTranslatef(shFontTextWidth("RIGHT Motor @000.00% (XXXX) "), 0, 0);
	}

	if (motors[limb].no_response)
		shFontColor(255, 0, 0);
	else if (motors[limb].status_opening || motors[limb].status_closing)
		shFontColor(0, 255, 255);
	else
		shFontColor(255, 255, 255);
	shFontWrite(NULL, "%s", motors[limb].no_response?"-- No communication":motors[limb].status_opening?"-- Opening":motors[limb].status_closing?"-- Closing":"");
}

#ifdef THROUGH_ROS
void draw_hud_rubbing_stiffness(int line, int limb)
{
	glLoadIdentity();
	glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line, 0);
	shFontColor(255, 255, 255);
	if (!dual_limb)
		shFontWrite(NULL, "Stiffness: %d", rubbings[limb].stiffness);
	else
	{
		if (limb == 1)
		{
			/* move to the right side of the screen */
			glTranslatef(SH_WIDTH - 40 - shFontTextWidth("Stiffness: 00  "), 0, 0);
			if (rubbings[limb].no_response)
				glTranslatef(-shFontTextWidth("-- No communication"), 0, 0);
		}
		shFontWrite(NULL, "Stiffness: %d", rubbings[limb].stiffness);
	}

	if (rubbings[limb].no_response)
	{
		glTranslatef(shFontTextWidth("Stiffness: 00  "), 0, 0);
		shFontColor(255, 0, 0);
		shFontWrite(NULL, "-- No communication");
	}
}
#endif

void draw_hud()
{
	glDisable(GL_DEPTH_TEST);
	glPushMatrix();				// save modelview
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();				// save projection
	glLoadIdentity();
	glOrtho(0, res_width, 0, res_height, -10000.0f, 10000.0f);
	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();
	shFontColor(200, 150, 0);
	shFontColor(255, 200, 50);
	shFontSize(0.5f * FONT_SCALE);
	glTranslatef(20, SH_HEIGHT - 20, 0);
	if (raw_results)
		shFontWrite(NULL, "Raw values%s", raw_remove_baseline?" (baseline removed)":"");
	else
		shFontWrite(NULL, "Scale: %s\t\tFilter: %s (Size: %d)\tAmplification: %s\tHysteresis Compensation: %s",
				do_scale?"Yes":"No", do_filter?"Yes":"No", filter_size, do_amplify?"Yes":"No", do_dampen?"Yes":"No");

	int line = 1;

	glLoadIdentity();
	shFontColor(0, 200, 60);
	shFontColor(50, 255, 100);
	shFontSize(0.5f * FONT_SCALE);
	glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line++, 0);
	shFontWrite(NULL, "Acquisition Method: ASAP\t\t\t%s", log_data?"Logging data":"Not logging data");

	glLoadIdentity();
	shFontColor(255, 255, 255);
	shFontSize(0.5f * FONT_SCALE);
	glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line++, 0);
	shFontWrite(NULL, "Legend:");
	glTranslatef(shFontTextWidth("Legend: "), 0, 0);
	shFontColor(255 * sensor_color_good[0], 255 * sensor_color_good[1], 255 * sensor_color_good[2]);
	shFontWrite(NULL, "Working");
	glTranslatef(shFontTextWidth("Working"), 0, 0);
	shFontColor(255, 255, 255);
	shFontWrite(NULL, ",");
	glTranslatef(shFontTextWidth(", "), 0, 0);
	shFontColor(255 * sensor_color_gripped[0], 255 * sensor_color_gripped[1], 255 * sensor_color_gripped[2]);
	shFontWrite(NULL, "Gripped");
	glTranslatef(shFontTextWidth("Gripped"), 0, 0);
	shFontColor(255, 255, 255);
	shFontWrite(NULL, ",");

	glLoadIdentity();
	glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line++, 0);
	shFontColor(255, 255, 255);
	shFontWrite(NULL, "Controlling %s Limb", which_limb == 0?"Left":"Right");

	if (!dual_limb)
		draw_hud_motor_position_status(line++, which_limb);
	else
	{
		draw_hud_motor_position_status(line++, 0);
		draw_hud_motor_position_status(line++, 1);
	}
	/* TODO: continue from here: show status from both limbs, on left and right accordingly */

	glLoadIdentity();
	glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line++, 0);
	shFontColor(255, 255, 255);
	if (!dual_limb)
		shFontWrite(NULL, "%s", motors[which_limb].status_pos_cal?"Position Calibrated":"");
	else
	{
		shFontWrite(NULL, "%s", motors[0].status_pos_cal?"Position Calibrated":"");
		glTranslatef(SH_WIDTH - 40 - shFontTextWidth(motors[1].status_pos_cal?"Position Calibrated":""), 0, 0);
		shFontWrite(NULL, "%s", motors[1].status_pos_cal?"Position Calibrated":"");
	}

	glLoadIdentity();
	glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line++, 0);
	shFontColor(255, 255, 255);
	if (!dual_limb)
		shFontWrite(NULL, "%s", motors[which_limb].status_force_cal?"Force Calibrated":"");
	else
	{
		shFontWrite(NULL, "%s", motors[0].status_force_cal?"Force Calibrated":"");
		glTranslatef(SH_WIDTH - 40 - shFontTextWidth(motors[1].status_force_cal?"Force Calibrated":""), 0, 0);
		shFontWrite(NULL, "%s", motors[1].status_force_cal?"Force Calibrated":"");
	}

	glLoadIdentity();
	glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line++, 0);
	shFontColor(255, 255, 255);
	if (!dual_limb)
		shFontWrite(NULL, "%s", motors[which_limb].status_prox_cal?"Ambient/Proximity Calibrated":"");
	else
	{
		shFontWrite(NULL, "%s", motors[0].status_prox_cal?"Ambient/Proximity Calibrated":"");
		glTranslatef(SH_WIDTH - 40 - shFontTextWidth(motors[1].status_prox_cal?"Ambient/Proximity Calibrated":""), 0, 0);
		shFontWrite(NULL, "%s", motors[1].status_prox_cal?"Ambient/Proximity Calibrated":"");
	}

	if (!dual_limb)
	{
		if (motors[which_limb].force > 0)
		{
			glLoadIdentity();
			glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line++, 0);
			shFontColor(255, 255, 255);
			shFontWrite(NULL, "Force control @%3d%%", motors[which_limb].force);
		}
	}
	else
	{
		if (motors[0].force > 0 || motors[1].force > 0)
		{
			glLoadIdentity();
			glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line++, 0);
			shFontColor(255, 255, 255);
		}
		if (motors[0].force > 0)
			shFontWrite(NULL, "Force control @%3d%%", motors[0].force);
		if (motors[1].force > 0)
		{
			glTranslatef(SH_WIDTH - 40 - shFontTextWidth("Force control @000%"), 0, 0);
			shFontWrite(NULL, "Force control @%3d%%", motors[1].force);
		}
	}

#ifdef THROUGH_ROS
	if (!dual_limb)
		draw_hud_rubbing_stiffness(line++, which_limb);
	else
	{
		draw_hud_rubbing_stiffness(line++, 0);
		draw_hud_rubbing_stiffness(line++, 1);
	}

	glLoadIdentity();
	glTranslatef(20, SH_HEIGHT - 20 - shFontTextHeight("\n") * line++, 0);
	shFontColor(255, 255, 255);
	if (!dual_limb)
		shFontWrite(NULL, "Rub times: %d", rubbings[which_limb].count);
	else
	{
		shFontWrite(NULL, "Rub times: %d", rubbings[0].count);
		glTranslatef(SH_WIDTH - 40 - shFontTextWidth("Rub times: 0"), 0, 0);
		shFontWrite(NULL, "Rub times: %d", rubbings[1].count);
	}
#endif

	glLoadIdentity();
	shFontColor(255, 255, 255);
	shFontSize(0.5f * FONT_SCALE);
	if (show_keys)
	{
		glTranslatef(20, 20 + shFontTextHeight("\n") *
				/* number of lines */
#ifdef THROUGH_ROS
				12,
#else
				10,
#endif
				0);
		shFontWrite(NULL, "Keys:\n"
				  "w/a/s/d/Ctrl/Space: Move   Mouse: Look Around   Page Up/Down: Sensor Size\n"
				  "r: Raw Values        p: Start/Stop Logging      i: Show Ids\n"
				  "v: Show Values       l: Scale      f: Filter    k: Amplify\n"
				  "h: Hysteresis Compensation   +/-: Filter Size   Return: Reset Scaler\n"
				  "F1: Calibrate Position   F2: Calibrate Force\n"
				  "F4: Switch Motor Speed   F3: Calibrate Ambient & Proximity\n"
				  "%s"
				  "F9: Switch Motor%s Limb Control\n"
				  "</>: Open/Close Motor    SHIFT+</>: Fully Open/Close Motor\n"
				  "[/]: Less/More Force     SHIFT+[/]: No/Maximum Force\n",
#ifdef THROUGH_ROS
				  "F5: Calibrate Rubbing    F6/F7: Rubbing Stifness\n"
				  "KP+/KP-: Rubbing Count   F8: Rub\n",
				  " and Rubbing"
#else
				  "",
				  ""
#endif
				  );
	}
	else
	{
		glTranslatef(20, 20 + shFontTextHeight("\n") * 1 /* number of lines */, 0);
		shFontWrite(NULL, "z: Show/Hide keys\n");
	}

	if (get_time_ms() < t_temp_message + TEMP_MESSAGE_TIME)
	{
		glLoadIdentity();
		shFontColor(255, 50, 50);
		shFontSize(0.6f * FONT_SCALE);
		glTranslatef((SH_WIDTH - shFontTextWidth(temp_message.c_str())) / 2, SH_HEIGHT / 2 - 32 * 0.6f, 0);
		shFontWrite(NULL, "%s", temp_message.c_str());
	}

#define LOGO_WIDTH 80
#define LOGO_HEIGHT 100
	glLoadIdentity();
	glColor3ub(255, 255, 255);
	glTranslatef(res_width - LOGO_WIDTH - 20, res_height - LOGO_HEIGHT - 35, 0);
	shNginEnableTexture();
	unige_logo.shNginTBind();
	glBegin(GL_QUADS);
		unige_logo.shNginTTextureCoord(0, 0); glVertex2f(0, 0);
		unige_logo.shNginTTextureCoord(1, 0); glVertex2f(LOGO_WIDTH, 0);
		unige_logo.shNginTTextureCoord(1, 1); glVertex2f(LOGO_WIDTH, LOGO_HEIGHT);
		unige_logo.shNginTTextureCoord(0, 1); glVertex2f(0, LOGO_HEIGHT);
	glEnd();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();				// restore projection
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();				// restore modelview
	glEnable(GL_DEPTH_TEST);
}

void render_screen()
{
#ifndef THROUGH_ROS
	get_responses();
	get_motor_data();
#endif

	/* if reached calibration point, reset scaler */
//	if (motor_position < 5)
//		processed_layers.sclr.reset();

	if (!no_gui)
	{
		shNgin3dEffectuate(dt);
		shNginClear();
		draw_sky_box();
		shNginDisableTexture();
		if (show_triangles)
			draw_triangles();
		skin_sensor_size scount = skin.sensors_count();
		skin_sensor_size taxelcount = skin.sensor_types()[0].sensors_count();
		gripped[0] = skin.sensors()[scount - 2].get_response() != 0;
		gripped[1] = skin.sensors()[scount - 1].get_response() != 0;
		for (int j = 0; j < scount - 2; ++j)
		{
			/* if not dual limb, use whichever gripped.  Otherwise use gripped for correspoding finger */
			bool g = !dual_limb?gripped[0] || gripped[1]:j < 24?gripped[0]:gripped[1];
			draw_sensor(j, j < taxelcount && g);
		}
		if (show_values || show_ids)
			for (int j = 0; j < scount - 2; ++j)
				draw_value(j);
		draw_hud();
		SDL_GL_SwapBuffers();
	}
}

#ifdef THROUGH_ROS
void motor_data::pos_callback(const clopema_gripper::MotorPosition::ConstPtr &msg)
{
	position = msg->motorPosition;
}

void motor_data::stat_callback(const clopema_gripper::MotorStatus::ConstPtr &msg)
{
	status_opening = msg->opening;
	status_closing = msg->closing;
	status_pos_cal = msg->position_calibrated;
	status_force_cal = msg->force_calibrated;
	status_prox_cal = msg->ambient_and_proximity_calibrated;
}
#endif

void load_data(string home)
{
	skybox[0].shNginTLoad((home + "room_east.bmp").c_str());
	skybox[1].shNginTLoad((home + "room_north.bmp").c_str());
	skybox[2].shNginTLoad((home + "room_west.bmp").c_str());
	skybox[3].shNginTLoad((home + "room_south.bmp").c_str());
	skybox[4].shNginTLoad((home + "room_up.bmp").c_str());
	skybox[5].shNginTLoad((home + "room_down.bmp").c_str());
	for (int i = 0; i < 6; ++i)
		if (!skybox[i].shNginTIsLoaded())
			fout << "Sky box " << i << " didn't load" << endl;
	if (shFontLoad((home + "font.shf").c_str()) == SH_FONT_FILE_NOT_FOUND)
		fout << "Could not load font" << endl;
	else
	{
		shFontSize(0.5f);
		shFontColor(255, 0, 0);
		shFontShadowColor(0, 0, 0);
		shFontShadow(SH_FONT_FULL_SHADOW);
		fout << "shFont done (version: "SH_FONT_VERSION")" << endl;
	        fout << "Using shImage (version " << SH_IMAGE_VERSION << ")" << endl;
	}
	unige_logo.shNginTLoad((home + "unigelogo.bmp").c_str());
	if (!unige_logo.shNginTIsLoaded())
		fout << "Unige logo could not be loaded" << endl;
	else
		unige_logo.shNginTSetTransparent(255, 255, 255);
}

void initialize_Ngin()
{
	shNginInitialize(res_width, res_height, 90);
	shNginViewableArea(SH_WIDTH, SH_HEIGHT, 90);
	shNginSetNearZ(1);
	shNginSetFarZ(5000);
	shNginDisableTexture();
	glDisable(GL_CULL_FACE);
	glDisable(GL_LIGHTING);
}

void initialize_Ngin3d(const char *settings, float middleX, float middleY)
{
	shNgin3dInitialize();
	shNgin3dCameraPositionFunction(shNginSetCameraPositionv);
	shNgin3dCameraTargetFunction(shNginSetCameraTargetv);
	shNgin3dCameraUpFunction(shNginSetCameraUpv);
	shNgin3dUpright();
	shNgin3dUpsideDownNotPossible();
	shNgin3dReadSettings(settings);
	shNgin3dRegisterAction(NGIN3D_KEY, NGIN3D_MOVE_FORWARD, NGIN3D_w);
	shNgin3dRegisterAction(NGIN3D_KEY, NGIN3D_MOVE_BACKWARD, NGIN3D_s);
	shNgin3dRegisterAction(NGIN3D_KEY, NGIN3D_MOVE_LEFT, NGIN3D_a);
	shNgin3dRegisterAction(NGIN3D_KEY, NGIN3D_MOVE_RIGHT, NGIN3D_d);
	shNgin3dRegisterAction(NGIN3D_KEY, NGIN3D_FLY_UP, NGIN3D_SPACE);
	shNgin3dRegisterAction(NGIN3D_KEY, NGIN3D_FLY_DOWN, NGIN3D_LCTRL);
	shNgin3dMove(-100, middleX, middleY, 1000);
}

int parse_args(int argc, char **argv)
{
	for (int i = 1; i < argc; ++i)
	{
		if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
		{
			cout << "Usage: " << argv[0] << " [--help] [--fullscreen] [--no-gui] [--calibrate file] [--shmem name] [--mutex name]" << endl << endl;
			return -1;
		}
		else if (strcmp(argv[i], "--fullscreen") == 0 || strcmp(argv[i], "-f") == 0)
			fullscreen = true;
		else if (strcmp(argv[i], "--no-gui") == 0 || strcmp(argv[i], "-n") == 0)
			no_gui = true;
		else if ((strcmp(argv[i], "--calibrate") == 0 || strcmp(argv[i], "-c") == 0) && i + 1 != argc)
		{
			calibration_file_given = true;
			calibration_file = argv[++i];
		}
		else if ((strcmp(argv[i], "--shmem") == 0 || strcmp(argv[i], "-s") == 0) && i + 1 != argc)
			shmem_name = argv[++i];
		else if ((strcmp(argv[i], "--mutex") == 0 || strcmp(argv[i], "-m") == 0) && i + 1 != argc)
			mutex_name = argv[++i];
	}
	return 0;
}

int main(int argc, char **argv)
{
	int frame;
	fout << "Initializing..." << endl;

	struct sigaction sa = {{0}};
	sa.sa_handler = signal_handler;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGSEGV, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGQUIT, &sa, NULL);
	sigaction(SIGUSR1, &sa, NULL);
	sigaction(SIGUSR2, &sa, NULL);

	const char *home_env = getenv("HOME");
	string home;
	if (home_env == NULL)
		home = ".";
	else
		home = string(home_env) + "/.clopema_view";

	calibration_file = home + "/data/clbr";

	if (parse_args(argc, argv))
		return EXIT_SUCCESS;

#ifdef THROUGH_ROS
	ros::init(argc, argv, "visualize", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	ros::NodeHandle ros_node;

	/* skin */
	connector.load(ros_node, true, get_responses, NULL);

	/* motor and rubbing */
	for (int i = 0; i < 2; ++i)
	{
		/* decide if dual_limb based on whether these succeed or fail.  If not dual_limb, set which_limb accordingly */
		string g = i == 0?"r2_gripper":"r1_gripper";
		motors[i].pos_sub = ros_node.subscribe(g + "/MotorPosition", 10, &motor_data::pos_callback, &motors[i]);
		motors[i].stat_sub = ros_node.subscribe(g + "/MotorStatus", 10, &motor_data::stat_callback, &motors[i]);
		motors[i].move_client = ros_node.serviceClient<clopema_gripper::MoveAbsoluteUpdatePercentage>(g + "/MoveAbsoluteUpdatePercentage");
		motors[i].stop_client = ros_node.serviceClient<clopema_gripper::Stop>(g + "/Stop");
		motors[i].calib_pos_client = ros_node.serviceClient<clopema_gripper::CalibratePosition>(g + "/CalibratePosition");
		motors[i].calib_force_client = ros_node.serviceClient<clopema_gripper::CalibrateForce>(g + "/CalibrateForce");
		motors[i].calib_prox_client = ros_node.serviceClient<clopema_gripper::CalibrateAmbientProximity>(g + "/CalibrateAmbientProximity");
		motors[i].set_state_client = ros_node.serviceClient<clopema_gripper::SetGripperState>(g + "/SetGripperState");
		motors[i].force_client = ros_node.serviceClient<clopema_gripper::ForcePercentage>(g + "/ForcePercentage");
		motors[i].set_freq_client = ros_node.serviceClient<clopema_gripper::SetFrequency>(g + "/SetFrequency");
		rubbings[i].calib_client = ros_node.serviceClient<clopema_rubbing::Calibrate>(g + "/CalibrateRubbing");
		rubbings[i].stiffness_client = ros_node.serviceClient<clopema_rubbing::Stiffness>(g + "/SetRubbingStiffness");
		rubbings[i].rub_client = ros_node.serviceClient<clopema_rubbing::Rubbing>(g + "/Rub");
	}
#else
	int ret;
	if ((ret = urt_init()))
	{
		fout << "Failed to initialize URT (error code " << ret << ")" << endl;
		quit(EXIT_FAILURE);
		cleanup();
		return _error_code;
	}

	/* skin */
	if ((ret = skin.load(calibration_file.c_str(), shmem_name, mutex_name)) != SKIN_SUCCESS)
	{
		/* if calibration file is not given, let's try /data/clbr-2 instead */
		if (!calibration_file_given)
		{
			calibration_file = home + "/data/clbr-2";
			ret = skin.load(calibration_file.c_str(), shmem_name, mutex_name);
		}
		if (ret != SKIN_SUCCESS)
		{
			fout << "Failed to initialize skin (error code " << ret << ")" << endl;
			quit(EXIT_FAILURE);
			cleanup();
			return _error_code;
		}
	}
	reader = skin.reader();
	read_sem = urt_sem_new(1);
	if (read_sem == NULL)
		cout << "Failed to acquire synchronization semaphore with skin" << endl;
	else
		reader->register_user(read_sem);

	/* motor */
	motors[0].motor = new gripper_motor(MOTOR_LIMB_LEFT);
	motors[1].motor = new gripper_motor(MOTOR_LIMB_RIGHT);
#endif

	motors[0].frequency = 25000;
	motors[1].frequency = 25000;
#ifdef THROUGH_ROS
	rubbings[0].count = 3;
	rubbings[1].count = 3;
#endif

	cout << "Skin done" << endl;
	skin_sensor *sensors = skin.sensors();
	dual_limb = skin.sensors_count() == 40;		/* if 24, it's single limb */

	pthread_t motor_thread[2];
	int motor_thread_params[2] = {0, 1};
	pthread_create(&motor_thread[0], NULL, motor_handler, &motor_thread_params[0]);
	pthread_create(&motor_thread[1], NULL, motor_handler, &motor_thread_params[1]);
	cout << "motors done" << endl;

#ifdef THROUGH_ROS
	pthread_t rubbing_thread[2];
	int rubbing_thread_params[2] = {0, 1};
	pthread_create(&rubbing_thread[0], NULL, rubbing_handler, &rubbing_thread_params[0]);
	pthread_create(&rubbing_thread[1], NULL, rubbing_handler, &rubbing_thread_params[1]);
	cout << "rubbing done" << endl;
#endif

	baseline_response.resize(skin.sensors_count());
	processed_layers.responses.resize(skin.sensors_count());
	processed_layers.fltr.change_size(filter_size);
	processed_layers.sclr.set_range(4096);
	if (!no_gui)
	{
		read_triangulation_settings((home + "/settings/triangulation_settings").c_str());
		char percent[] = "    ";
		cout << "Constructing module shapes...  0%" << flush;
		for (unsigned int i = 0; i < skin.sensors_count(); i += MAX_SENSORS_PER_MODULE)
		{
			float positions[MAX_SENSORS_PER_MODULE][3];
			float orientations[MAX_SENSORS_PER_MODULE][3];
			/* triangles are all of type 0 */
			if (sensors[i].type > 0)
				break;
			for (unsigned int j = 0; j < MAX_SENSORS_PER_MODULE; ++j)
			{
				positions[j][0] = sensors[i + j].relative_position[0];
				positions[j][1] = sensors[i + j].relative_position[1];
				positions[j][2] = sensors[i + j].relative_position[2];
				orientations[j][0] = sensors[i + j].relative_orientation[0];
				orientations[j][1] = sensors[i + j].relative_orientation[1];
				orientations[j][2] = sensors[i + j].relative_orientation[2];
			}
			modules_triangulated.push_back(triangulate(positions, orientations, MAX_SENSORS_PER_MODULE));
			sprintf(percent, "%3d", i * 100 / skin.sensors_count());
			cout << "\b\b\b\b" << percent << '%' << flush;
		}
		cout << "\b\b\b\b\b done" << endl;
		fout << "Constructing module shapes done" << endl;
		float minX = 10000000.0f, maxX = -10000000.0f;
		float minY = 10000000.0f, maxY = -10000000.0f;
		float minZ = 10000000.0f, maxZ = -10000000.0f;
		for (unsigned int i = 0; i < skin.sensors_count(); ++i)
		{
			if (sensors[i].relative_position[0] < minX)
				minX = sensors[i].relative_position[0];
			if (sensors[i].relative_position[0] > maxX)
				maxX = sensors[i].relative_position[0];
			if (sensors[i].relative_position[1] < minY)
				minY = sensors[i].relative_position[1];
			if (sensors[i].relative_position[1] > maxY)
				maxY = sensors[i].relative_position[1];
			if (sensors[i].relative_position[2] < minZ)
				minZ = sensors[i].relative_position[2];
			if (sensors[i].relative_position[2] > maxZ)
				maxZ = sensors[i].relative_position[2];
		}
		if (maxX == minX)
		{
			fout << "maxX is equal to minX" << endl;
			meter_scale = 1;
		}
		else if (maxX - minX > maxY - minY)
			meter_scale = 100.0f / (maxX - minX);
		else
			meter_scale = 100.0f / (maxY - minY);
		if (skin.sensors_count() >= MAX_SENSORS_PER_MODULE)
		{
			double min_distance = 1e12;
			float min_dist_diameter = 1;
			for (unsigned int i = 0; i < MAX_SENSORS_PER_MODULE - 1; ++i)
				for (unsigned int j = i + 1; j < MAX_SENSORS_PER_MODULE; ++j)
				{
					double dist = DISTANCE(sensors[i].relative_position, sensors[j].relative_position);
					if (dist < min_distance)
					{
						min_distance = dist;
						min_dist_diameter = sensors[i].radius+sensors[j].radius;
					}
				}
			sensor_radius_mult = min_distance / min_dist_diameter * 1.25;	// visually, * 1.25 looks good
			value_font_size = min_distance * meter_scale * 0.008;
		}
		initialize_SDL();
		if (_must_exit)
			return _error_code;
		fout << "SDL done" << endl;
		initialize_Ngin();
		fout << "Ngin done (version: "NGIN_VERSION")" << endl;
		initialize_Ngin3d((home + "/settings/Ngin3d_settings").c_str(), (maxX + minX) / 2 * meter_scale, (maxY + minY) / 2 * meter_scale);
		fout << "Ngin3d done (version: "NGIN3D_VERSION")" << endl;
		load_data((home + "/gui/").c_str());
		fout << "gui data loaded" << endl;

		/* move back to see all sensors in the screen */
		shNgin3dReceivedInput(NGIN3D_KEY, NGIN3D_s, NGIN3D_PRESSED);
		shNgin3dEffectuate(maxZ * 50 + 1000);
		shNgin3dReceivedInput(NGIN3D_KEY, NGIN3D_s, NGIN3D_RELEASED);
	}
	else
	{
		cout << "Logging data. Press ESC to terminate" << endl;
		if (!log_data)
			toggle_log();
	}
#ifndef THROUGH_ROS
	reader->start();
#endif
	frame = 0;
	t_last_damp = get_time_ms();
	unsigned int last = get_time_ms();
	unsigned int last_time = last;
	while (true)
	{
		unsigned int now = get_time_ms();
		if (now >= last+1000)
		{
			fout << (FPS = frame * 1000 / (now - last)) << " fps" << endl;
			last += 1000;
			frame = 0;
		}
		dt = now - last_time;
		last_time = now;
		process_events();
		if (_must_exit)
			break;
		render_screen();
		++frame;
		if (no_gui)
			usleep(15000);
#ifdef THROUGH_ROS
		ros::spinOnce();
		if (!ros::ok())
			break;
#endif
	}
	cleanup();
#ifdef THROUGH_ROS
	ros::shutdown();
#endif

	pthread_join(motor_thread[0], NULL);
	pthread_join(motor_thread[1], NULL);
#ifdef THROUGH_ROS
	pthread_join(rubbing_thread[0], NULL);
	pthread_join(rubbing_thread[1], NULL);
#endif

#ifndef THROUGH_ROS
	delete motors[0].motor;
	delete motors[1].motor;

	if (urt_is_rt_context())
		urt_exit();
#endif

	return _error_code;
}
