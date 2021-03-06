/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file angular_acc_estimator_main.cpp
 *
 * Attitude estimator (quaternion based)
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/angular_acc.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>

#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>


extern "C" __EXPORT int angular_acc_estimator_main(int argc, char *argv[]);

using math::Vector;
using math::Matrix;
using math::Quaternion;

class AngularAccEstimator;

namespace angular_acc_estimator
{
AngularAccEstimator *instance;
}


class AngularAccEstimator
{
public:
	/**
	 * Constructor
	 */
	AngularAccEstimator();

	/**
	 * Destructor, also kills task.
	 */
	~AngularAccEstimator();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

	void		print();

private:
	static constexpr float _dt_max = 0.02;
	bool		_task_should_exit = false;		/**< if true, task should exit */
	int		_control_task = -1;			/**< task handle for task */
	
	/*Subscriptions*/
	int		_attitude_sub = -1;
	int		_params_sub = -1;
	int		_rates_setpoint_sub = -1;

	/* publishers */
	orb_advert_t	_angular_acc_pub = nullptr;

	struct {
		param_t	theta_r_roll[5];
		param_t	theta_w_roll[5];
		param_t	theta_r_pitch[5];
		param_t	theta_w_pitch[5];
		param_t	theta_r_yaw[5];
		param_t	theta_w_yaw[5];

	}		_params_handles;		/**< handles for interesting parameters */

	/* parameter vectors */
	Vector<5>	_theta_r_roll;
	Vector<5>	_theta_w_roll;
	Vector<5>	_theta_r_pitch;
	Vector<5>	_theta_w_pitch;
	Vector<5>	_theta_r_yaw;
	Vector<5>	_theta_w_yaw;

	/* subscriptions */
	Vector<3>	_cur_rates;
	Vector<3>	_cur_references;

	/* last 5 references and measurements */
	Vector<5> _ref_roll;
	Vector<5> _w_roll;
	Vector<5> _ref_pitch;
	Vector<5> _w_pitch;
	Vector<5> _ref_yaw;
	Vector<5> _w_yaw;

	float _cur_acc_roll;
	float _cur_acc_pitch;
	float _cur_acc_yaw;


	hrt_abstime _vel_prev_t = 0;

	bool		_inited = false;
	bool		_data_good = false;
	bool		_failsafe = false;
	bool		_vibration_warning = false;

	int		_mavlink_fd = -1;

	/* private methods */
	void update_parameters(bool force);

	int update_subscriptions();

	bool init();

	bool update();

};


AngularAccEstimator::AngularAccEstimator()
{
	
	_params_handles.theta_r_roll[0] = param_find("THETA_R_ROLL1");
	_params_handles.theta_r_roll[1]	= param_find("THETA_R_ROLL2");
	_params_handles.theta_r_roll[2]	= param_find("THETA_R_ROLL3");
	_params_handles.theta_r_roll[3]	= param_find("THETA_R_ROLL4");
	_params_handles.theta_r_roll[4]	= param_find("THETA_R_ROLL5");
	_params_handles.theta_w_roll[0]	= param_find("THETA_W_ROLL1");
	_params_handles.theta_w_roll[1]	= param_find("THETA_W_ROLL2");
	_params_handles.theta_w_roll[2]	= param_find("THETA_W_ROLL3");
	_params_handles.theta_w_roll[3]	= param_find("THETA_W_ROLL4");
	_params_handles.theta_w_roll[4]	= param_find("THETA_W_ROLL5");
	_params_handles.theta_r_pitch[0] = param_find("THETA_R_PITCH1");
	_params_handles.theta_r_pitch[1] = param_find("THETA_R_PITCH2");
	_params_handles.theta_r_pitch[2] = param_find("THETA_R_PITCH3");
	_params_handles.theta_r_pitch[3] = param_find("THETA_R_PITCH4");
	_params_handles.theta_r_pitch[4] = param_find("THETA_R_PITCH5");
	_params_handles.theta_w_pitch[0] = param_find("THETA_W_PITCH1");
	_params_handles.theta_w_pitch[1] = param_find("THETA_W_PITCH2");
	_params_handles.theta_w_pitch[2] = param_find("THETA_W_PITCH3");
	_params_handles.theta_w_pitch[3] = param_find("THETA_W_PITCH4");
	_params_handles.theta_w_pitch[4] = param_find("THETA_W_PITCH5");
	_params_handles.theta_r_yaw[0] = param_find("THETA_R_YAW1");
	_params_handles.theta_r_yaw[1] = param_find("THETA_R_YAW2");
	_params_handles.theta_r_yaw[2] = param_find("THETA_R_YAW3");
	_params_handles.theta_r_yaw[3] = param_find("THETA_R_YAW4");
	_params_handles.theta_r_yaw[4] = param_find("THETA_R_YAW5");
	_params_handles.theta_w_yaw[0] = param_find("THETA_W_YAW1");
	_params_handles.theta_w_yaw[1] = param_find("THETA_W_YAW2");
	_params_handles.theta_w_yaw[2] = param_find("THETA_W_YAW3");
	_params_handles.theta_w_yaw[3] = param_find("THETA_W_YAW4");
	_params_handles.theta_w_yaw[4] = param_find("THETA_W_YAW5");


}

/**
 * Destructor, also kills task.
 */
AngularAccEstimator::~AngularAccEstimator()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	angular_acc_estimator::instance = nullptr;
}

int AngularAccEstimator::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("angular_acc_estimator",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2400,
					   (px4_main_t)&AngularAccEstimator::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void AngularAccEstimator
::print()
{
	warnx("test status:");
/*	_voter_gyro.print();*/
}

void AngularAccEstimator::task_main_trampoline(int argc, char *argv[])
{
	angular_acc_estimator::instance->task_main();
}

void AngularAccEstimator::task_main()
{

	_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_rates_setpoint_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	update_parameters(true);

	//hrt_abstime last_time = 0;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _attitude_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		int ret = px4_poll(fds, 1, 100);

		if (_mavlink_fd < 0) {
			_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		}

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			PX4_WARN("Q POLL ERROR");
			continue;

		} else if (ret == 0) {
			// Poll timeout, do nothing
			PX4_WARN("Q POLL TIMEOUT");
			continue;
		}
		
		update_parameters(false);

		// Update attitude
		vehicle_attitude_s attitude;

		if (!orb_copy(ORB_ID(vehicle_attitude), _attitude_sub, &attitude))
		{
			// Feed validator with recent sensor data
			// Feed validator with recent reference data
			continue;
		}

		vehicle_rates_setpoint_s rates_reference;

		orb_copy(ORB_ID(vehicle_rates_setpoint), _rates_setpoint_sub, &rates_reference);

		// update measurement and reference vectors
		int i = 0;
		for(i = 1; i < 5; i++) {
				_w_roll(i) =  _w_roll(i-1);
				_ref_roll(i) = _ref_roll(i-1);
				_w_pitch(i) = _w_pitch(i-1);
				_ref_pitch(i) = _ref_pitch(i-1);
				_w_yaw(i) = _w_yaw(i-1);
				_ref_yaw(i) = _ref_yaw(i-1);

		}
		_w_roll(0) = attitude.rollspeed;
		_ref_roll(0) = rates_reference.roll;
		_w_pitch(0) = attitude.pitchspeed;
		_ref_pitch(0) = rates_reference.pitch;
		_w_yaw(0) = attitude.yawspeed;
		_ref_yaw(0) = rates_reference.yaw;

		if(!update()) {
				continue;
		}
		
		int angular_acc_inst;
		angular_acc_s angular_acc;
		angular_acc.timestamp = attitude.timestamp;
		angular_acc.rollacc = _cur_acc_roll;
		angular_acc.pitchacc = _cur_acc_pitch;
		angular_acc.yawacc = _cur_acc_yaw;
		/* publish to control state topic */
		orb_publish_auto(ORB_ID(angular_acc), &_angular_acc_pub, &angular_acc, &angular_acc_inst, ORB_PRIO_HIGH);
	}
}

void AngularAccEstimator::update_parameters(bool force)
{
	bool updated = force;
	
	if (!updated) {
		orb_check(_params_sub, &updated);
	}

	if (updated) {
		int i = 0;
		parameter_update_s param_update;

		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		for(i=0;i < 5;i++){ 
			param_get(_params_handles.theta_r_roll[i], &_theta_r_roll(i));
			param_get(_params_handles.theta_w_roll[i], &_theta_w_roll(i));
			param_get(_params_handles.theta_r_pitch[i], &_theta_r_pitch(i));
			param_get(_params_handles.theta_w_pitch[i], &_theta_w_pitch(i));
			param_get(_params_handles.theta_r_yaw[i], &_theta_r_yaw(i));
			param_get(_params_handles.theta_w_yaw[i], &_theta_w_yaw(i));
		}
	}
}

bool AngularAccEstimator::init()
{
	_ref_roll.zero();
	_w_roll.zero();
	_ref_pitch.zero();
	_w_pitch.zero();
	_ref_yaw.zero();
	_w_yaw.zero();
	
	return true;
}

bool AngularAccEstimator::update()
{
	if (!_inited) {

		if (!_data_good) {
			return false;
		}

		return init();
	}
	
	// update angular accelerations
	_cur_acc_roll = _theta_w_roll*_w_roll + _theta_r_roll*_ref_roll;
	_cur_acc_pitch = _theta_w_pitch*_w_pitch + _theta_r_pitch*_ref_pitch;
	_cur_acc_yaw = _theta_w_yaw*_w_yaw + _theta_r_yaw*_ref_yaw;

	return true;
}


int angular_acc_estimator_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: angular_acc_estimator {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (angular_acc_estimator::instance != nullptr) {
			warnx("already running");
			return 1;
		}

		angular_acc_estimator::instance = new AngularAccEstimator;

		if (angular_acc_estimator::instance == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != angular_acc_estimator::instance->start()) {
			delete angular_acc_estimator::instance;
			angular_acc_estimator::instance = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (angular_acc_estimator::instance == nullptr) {
			warnx("not running");
			return 1;
		}

		delete angular_acc_estimator::instance;
		angular_acc_estimator::instance = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (angular_acc_estimator::instance) {
			angular_acc_estimator::instance->print();
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
