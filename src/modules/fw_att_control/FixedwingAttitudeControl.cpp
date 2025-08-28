/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "FixedwingAttitudeControl.hpp"

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::radians;

FixedwingAttitudeControl::FixedwingAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
FixedwingAttitudeControl::parameters_update()
{
	_roll_ctrl.set_time_constant(_param_fw_r_tc.get());
	_roll_ctrl.set_max_rate(radians(_param_fw_r_rmax.get()));

	_pitch_ctrl.set_time_constant(_param_fw_p_tc.get());
	_pitch_ctrl.set_max_rate_pos(radians(_param_fw_p_rmax_pos.get()));
	_pitch_ctrl.set_max_rate_neg(radians(_param_fw_p_rmax_neg.get()));

	_yaw_ctrl.set_max_rate(radians(_param_fw_y_rmax.get()));

	_wheel_ctrl.set_k_p(_param_fw_wr_p.get());
	_wheel_ctrl.set_k_i(_param_fw_wr_i.get());
	_wheel_ctrl.set_k_ff(_param_fw_wr_ff.get());
	_wheel_ctrl.set_integrator_max(_param_fw_wr_imax.get());
	_wheel_ctrl.set_max_rate(radians(_param_fw_w_rmax.get()));
}

void
FixedwingAttitudeControl::vehicle_manual_poll(const float yaw_body)
{
	if (_vcontrol_mode.flag_control_manual_enabled && _in_fw_or_transition_wo_tailsitter_transition) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (!_vcontrol_mode.flag_control_climb_rate_enabled && _vcontrol_mode.flag_control_attitude_enabled) {

				// STABILIZED mode generate the attitude setpoint from manual user inputs

				const float roll_body = _manual_control_setpoint.roll * radians(_param_fw_man_r_max.get());

				float pitch_body = -_manual_control_setpoint.pitch * radians(_param_fw_man_p_max.get())
						   + radians(_param_fw_psp_off.get());
				pitch_body = constrain(pitch_body,
						       -radians(_param_fw_man_p_max.get()), radians(_param_fw_man_p_max.get()));

				_att_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

				const Quatf q(Eulerf(roll_body, pitch_body, yaw_body));
				q.copyTo(_att_sp.q_d);

				_att_sp.reset_integral = false;

				_att_sp.timestamp = hrt_absolute_time();

			        _attitude_sp_pub.publish(_att_sp);

			}
		}
	}
}


void FixedwingAttitudeControl::computeDirectionVector(float dt, float target_x, float target_y, float roll_act, float pitch_act)
{
    // ==== YENİ: Hedefin X Eksenindeki Kayma Hızını Hesaplama ====
    // Bir önceki frame'deki hedef konumunu saklamak için static değişken.
    // İlk çağrıda mevcut target_x ile başlatılır, böylece ilk hız 0 olur.
    static float previous_target_x = target_x;
    float target_speed_x_pixels_per_second = 0.0f;

    // dt'nin çok küçük veya sıfır olmasını kontrol et (sıfıra bölme hatasını önle)
    if (dt > 1e-6f) {
        // Hız = Konum Değişimi / Zaman Değişimi
        target_speed_x_pixels_per_second = (target_x - previous_target_x) / dt;
    }

    // Bir sonraki döngü için mevcut x konumunu sakla
    previous_target_x = target_x;

    // Hesaplanan hızı loglamak veya başka bir yerde kullanmak için:
    // PX4_INFO("Hedef Hızı X: %.2f piksel/saniye", (double)target_speed_x_pixels_per_second);
    // ===============================================================


    // ==== 1) Piksel → fiziksel koordinat dönüşümü ====
    /* static float target_x_f = 0.0f;
    static float target_y_f = 0.0f;
    const float alpha_xy = 0.1f; // 0.8-0.95 arası denenebilir
    target_x_f = alpha_xy * target_x_f + (1.0f - alpha_xy) * ;
    target_y_f = alpha_xy * target_y_f + (1.0f - alpha_xy) * target_y; */
    const float sx = SENSOR_WIDTH / static_cast<float>(RESOLUTION_WIDTH);
    const float sy = SENSOR_HEIGHT / static_cast<float>(RESOLUTION_HEIGHT);

    const float cx = (RESOLUTION_WIDTH+1) / 2.0f;
    const float cy = (RESOLUTION_HEIGHT+1) / 2.0f;

    const float u = target_x - cx;
    const float v = target_y - cy;

    const float cos_roll = cosf(roll_act);
    const float sin_roll = sinf(roll_act);
    const float u_corrected = u * cos_roll + v * sin_roll;

    x_cam = u_corrected * sx;
    y_cam = -v * sy; // görüntü y ekseni ters
    z_cam = FOCAL_LENGTH;

    // Normalize et
    const float norm = sqrtf(x_cam * x_cam + y_cam * y_cam + z_cam * z_cam);
    if (norm > 1e-6f) {
        x_cam /= norm;
        y_cam /= norm;
        z_cam /= norm;
    }

    matrix::Vector3f vec_cam(x_cam, y_cam, z_cam);

    // ==== 2) Kamera → gövde dönüşüm matrisi ====
    // cam_z -> body_x, cam_x -> body_y, cam_y -> body_z
    matrix::Dcmf R_offset;
    R_offset(0, 0) = 0.f;  R_offset(0, 1) = 0.f;  R_offset(0, 2) = 1.f; // body_x = cam_z
    R_offset(1, 0) = 1.f;  R_offset(1, 1) = 0.f;  R_offset(1, 2) = 0.f; // body_y = cam_x
    R_offset(2, 0) = 0.f;  R_offset(2, 1) = 1.f;  R_offset(2, 2) = 0.f; // body_z = cam_y

    matrix::Vector3f vec_body = R_offset * vec_cam;

    // ==== 3) Açı hesapları (singularite önleme) ====
    float denom_x = fabsf(vec_body(0)) < 1e-3f ? copysignf(1e-3f, vec_body(0)) : vec_body(0);
    float pitch_ref_cam = atan2f(-vec_body(2), denom_x);
    float roll_ref_cam  = atan2f(vec_body(1), denom_x);

    // ==== 4) Ölçüm yumuşatma (low-pass filter) ====
    static float roll_f = 0.f, pitch_f = 0.f;
    float alpha = _param_x_error_kp.get(); // 0.7..0.95 arası denenebilir
    roll_f  = alpha * roll_f  + (1.f - alpha) * roll_ref_cam;
    pitch_f = alpha * pitch_f + (1.f - alpha) * pitch_ref_cam;

    // Check if roll_f is NaN and set to zero if so
    if (!PX4_ISFINITE(roll_f)) {
        roll_f = 0.f;
    }
    // Check if pitch_f is NaN and set to zero if so
    if (!PX4_ISFINITE(pitch_f)) {
        pitch_f = 0.f;
    }


    // ==== 5) Deadzone ====
    float dead_roll = _param_x_error_ki.get(); // ~0.06°
    const float dead_pitch = 0.02f; // ~1.2°
    PX4_INFO("roll_f: %.4f", (double)roll_f);
    if (fabsf(roll_f) < dead_roll){roll_f = 0.f;}
    if (fabsf(pitch_f) < dead_pitch){pitch_f = 0.f;}

    // ==== 7) Mevcut attitude ile harmanlama ====
    // K_vis: vision düzeltmesinin ne kadar etkili olacağı
    const float K_vis = 1.0f;
    float roll_ref  = roll_act  + K_vis * roll_f;
    float pitch_ref = pitch_act + K_vis * pitch_f;

    // ==== 8) Sonuçlar ====
    roll_pitch_ref[0] = roll_f;
    roll_pitch_ref[1] = -pitch_f;

    const float roll_limit = radians(_param_roll_lim_x.get());
    const float pitch_limit = radians(_param_roll_lim_y.get());
    roll_pitch_ref[0] = constrain(roll_pitch_ref[0], -roll_limit, roll_limit);
    roll_pitch_ref[1] = constrain(roll_pitch_ref[1], -pitch_limit, pitch_limit);

    PX4_INFO("roll_act: %.2f roll_ref: %.2f | pitch_act: %.2f pitch_ref: %.2f",
             (double)roll_act, (double)roll_f,
             (double)pitch_act, (double)pitch_f);



    _visual_location.timestamp = hrt_absolute_time();
    _visual_location.x_err = target_speed_x_pixels_per_second;
    _visual_location.y_err = roll_f;
    _visual_location.roll_sp = roll_pitch_ref[0];
    _visual_location.pitch_sp = roll_pitch_ref[1];
    _visual_location_pub.publish(_visual_location);
}






void
FixedwingAttitudeControl::vision_based_nav(const float yaw_body, const float dt,float roll_act, float pitch_act)
{
	if (_intercept_roll_pitch_sub.update(&_intercept_roll_pitch)) {
		target_roll =  M_DEG_TO_RAD_F*constrain(_intercept_roll_pitch.target_roll, -_param_fw_man_r_max.get(), _param_fw_man_r_max.get());
		target_pitch = M_DEG_TO_RAD_F*constrain(_intercept_roll_pitch.target_pitch, -_param_fw_man_p_max.get(), _param_fw_man_p_max.get());
	}
	if (_target_location_sub.update(&_target_location)) {
		target_x = _target_location.target_x;
		target_y = _target_location.target_y;
	}

	target_roll = _roll_vis_slew_rate.update(target_roll, dt);
	target_pitch = _pitch_vis_slew_rate.update(target_pitch, dt);



/*
	const Quatf q_sp(Eulerf(roll_pitch_ref[0], roll_pitch_ref[1],yaw_body));

	q_sp.copyTo(_att_sp.q_d);
	_att_sp.reset_integral = false;
	_att_sp.timestamp = hrt_absolute_time();
	_attitude_sp_pub.publish(_att_sp); */


}


void
FixedwingAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingAttitudeControl::get_airspeed_constrained()
{
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if (_param_fw_use_airspd.get() && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}
	}

	return math::constrain(airspeed, _param_fw_airspd_stall.get(), _param_fw_airspd_max.get());
}

void FixedwingAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if attitude changed
	if (_att_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) {



		// only update parameters if they changed
		const bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			_roll_vis_slew_rate.setSlewRate(radians(_param_fw_pn_r_slew_max.get()));
			_pitch_vis_slew_rate.setSlewRate(radians(_param_fw_pn_r_slew_max.get()));
			parameters_update();
		}


		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

		vehicle_attitude_s att{};

		if (_att_sub.copy(&att)) {
			// get dt from attitude message
			dt = math::constrain((att.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = att.timestamp_sample;

			// get current rotation matrix and euler angles from control state quaternions
			_R = matrix::Quatf(att.q);
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		if (_vehicle_status.is_vtol_tailsitter) {
			/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
			 *
			 * Since the VTOL airframe is initialized as a multicopter we need to
			 * modify the estimated attitude for the fixed wing operation.
			 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
			 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
			 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
			 * Additionally, in order to get the correct sign of the pitch, we need to multiply
			 * the new x axis of the rotation matrix with -1
			 *
			 * original:			modified:
			 *
			 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
			 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
			 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
			 * */
			matrix::Dcmf R_adapted = _R;		//modified rotation matrix

			/* move z to x */
			R_adapted(0, 0) = _R(0, 2);
			R_adapted(1, 0) = _R(1, 2);
			R_adapted(2, 0) = _R(2, 2);

			/* move x to z */
			R_adapted(0, 2) = _R(0, 0);
			R_adapted(1, 2) = _R(1, 0);
			R_adapted(2, 2) = _R(2, 0);

			/* change direction of pitch (convert to right handed system) */
			R_adapted(0, 0) = -R_adapted(0, 0);
			R_adapted(1, 0) = -R_adapted(1, 0);
			R_adapted(2, 0) = -R_adapted(2, 0);

			/* fill in new attitude data */
			_R = R_adapted;
		}

		const matrix::Eulerf euler_angles(_R);

		vehicle_manual_poll(euler_angles.psi());

		vehicle_attitude_setpoint_poll();

		// vehicle status update must be before the vehicle_control_mode poll, otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);
		const bool is_in_transition_except_tailsitter = _vehicle_status.in_transition_mode
				&& !_vehicle_status.is_vtol_tailsitter;
		const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		_in_fw_or_transition_wo_tailsitter_transition =  is_fixed_wing || is_in_transition_except_tailsitter;

		_vehicle_control_mode_sub.update(&_vcontrol_mode);

		vehicle_land_detected_poll();

		bool wheel_control = false;

		if (_param_fw_w_en.get() && _att_sp.fw_control_yaw_wheel && _vcontrol_mode.flag_control_auto_enabled) {
			wheel_control = true;
		}

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		if (_vcontrol_mode.flag_control_rates_enabled) {

			/* Reset integrators if commanded by attitude setpoint, or the aircraft is on ground
			 * or a multicopter (but not transitioning VTOL or tailsitter)
			 */
			if (_att_sp.reset_integral
			    || _landed
			    || !_in_fw_or_transition_wo_tailsitter_transition) {

				_rates_sp.reset_integral = true;
				_wheel_ctrl.reset_integrator();

			} else {
				_rates_sp.reset_integral = false;
			}

			float groundspeed_scale = 1.f;

			if (wheel_control) {
				if (_local_pos_sub.updated()) {
					vehicle_local_position_s vehicle_local_position;

					if (_local_pos_sub.copy(&vehicle_local_position)) {
						_groundspeed = sqrtf(vehicle_local_position.vx * vehicle_local_position.vx + vehicle_local_position.vy *
								     vehicle_local_position.vy);
					}
				}

				// Use stall airspeed to calculate ground speed scaling region. Don't scale below gspd_scaling_trim
				float gspd_scaling_trim = (_param_fw_airspd_stall.get());

				if (_groundspeed > gspd_scaling_trim) {
					groundspeed_scale = gspd_scaling_trim / _groundspeed;

				}
			}

			/* Run attitude controllers */

			if (_vcontrol_mode.flag_control_attitude_enabled && _in_fw_or_transition_wo_tailsitter_transition) {
				const Quatf q_sp(_att_sp.q_d);

				if (q_sp.isAllFinite()) {
					const Eulerf euler_sp(q_sp);
					const float roll_sp = euler_sp.phi();
					const float pitch_sp = euler_sp.theta();

					_roll_ctrl.control_roll(roll_sp, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
								euler_angles.theta());
					_pitch_ctrl.control_pitch(pitch_sp, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
								  euler_angles.theta());
					_yaw_ctrl.control_yaw(roll_sp, _pitch_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
							      euler_angles.theta(), get_airspeed_constrained());

					if (wheel_control) {
						_wheel_ctrl.control_attitude(euler_sp.psi(), euler_angles.psi());

					} else {
						_wheel_ctrl.reset_integrator();
					}

					/* Update input data for rate controllers */
					Vector3f body_rates_setpoint = Vector3f(_roll_ctrl.get_body_rate_setpoint(), _pitch_ctrl.get_body_rate_setpoint(),
										_yaw_ctrl.get_body_rate_setpoint());

					autotune_attitude_control_status_s pid_autotune;
					matrix::Vector3f bodyrate_autotune_ff;

					if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
						if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
						    && ((hrt_absolute_time() - pid_autotune.timestamp) < 1_s)) {

							bodyrate_autotune_ff = matrix::Vector3f(pid_autotune.rate_sp);
							body_rates_setpoint += bodyrate_autotune_ff;
						}
					}

					/* add yaw rate setpoint from sticks in all attitude-controlled modes */
					if (_vcontrol_mode.flag_control_manual_enabled) {
						body_rates_setpoint(2) += math::constrain(_manual_control_setpoint.yaw * radians(_param_man_yr_max.get()),
									  -radians(_param_fw_y_rmax.get()), radians(_param_fw_y_rmax.get()));
					}

					// Tailsitter: transform from FW to hover frame (all interfaces are in hover (body) frame)
					if (_vehicle_status.is_vtol_tailsitter) {
						body_rates_setpoint = Vector3f(body_rates_setpoint(2), body_rates_setpoint(1), -body_rates_setpoint(0));
					}

					/* Publish the rate setpoint for analysis once available */
					_rates_sp.roll = body_rates_setpoint(0);
					_rates_sp.pitch = body_rates_setpoint(1);
					_rates_sp.yaw = body_rates_setpoint(2);

					_rates_sp.timestamp = hrt_absolute_time();

					_rate_sp_pub.publish(_rates_sp);
				}
			}

			// wheel control
			float wheel_u = 0.f;

			if (_vcontrol_mode.flag_control_manual_enabled) {
				// always direct control of steering wheel with yaw stick in manual modes
				wheel_u = _manual_control_setpoint.yaw;

			} else {
				vehicle_angular_velocity_s angular_velocity{};
				_vehicle_rates_sub.copy(&angular_velocity);

				// XXX: yaw_sp_move_rate here is an abuse -- used to ferry manual yaw inputs from
				// position controller during auto modes _manual_control_setpoint.r gets passed
				// whenever nudging is enabled, otherwise zero
				const float wheel_controller_output = _wheel_ctrl.control_bodyrate(dt, angular_velocity.xyz[2], _groundspeed,
								      groundspeed_scale);
				wheel_u = wheel_control ? wheel_controller_output +  _att_sp.yaw_sp_move_rate : 0.f;
			}

			_landing_gear_wheel.normalized_wheel_setpoint = PX4_ISFINITE(wheel_u) ? wheel_u : 0.f;
			_landing_gear_wheel.timestamp = hrt_absolute_time();
			_landing_gear_wheel_pub.publish(_landing_gear_wheel);

		} else {
			// full manual
			_wheel_ctrl.reset_integrator();

			_landing_gear_wheel.normalized_wheel_setpoint = PX4_ISFINITE(_manual_control_setpoint.yaw) ?
					_manual_control_setpoint.yaw : 0.f;
			_landing_gear_wheel.timestamp = hrt_absolute_time();
			_landing_gear_wheel_pub.publish(_landing_gear_wheel);

		}
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int FixedwingAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingAttitudeControl *instance = new FixedwingAttitudeControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}
