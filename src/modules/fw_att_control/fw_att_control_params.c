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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

/**
 * @file fw_att_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/**
 * Attitude Roll Time Constant
 *
 * This defines the latency between a roll step input and the achieved setpoint
 * (inverse to a P gain). Smaller systems may require smaller values.
 *
 * @unit s
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_R_TC, 0.4f);

/**
 * Attitude pitch time constant
 *
 * This defines the latency between a pitch step input and the achieved setpoint
 * (inverse to a P gain). Smaller systems may require smaller values.
 *
 * @unit s
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_TC, 0.4f);

/**
 * Maximum positive / up pitch rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_RMAX_POS, 60.0f);

/**
 * Maximum negative / down pitch rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_RMAX_NEG, 60.0f);

/**
 * Maximum roll rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_R_RMAX, 70.0f);

/**
 * Maximum yaw rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_Y_RMAX, 50.0f);

/**
 * Enable wheel steering controller
 *
 * Only enabled during automatic runway takeoff and landing.
 * In all manual modes the wheel is directly controlled with yaw stick.
 *
 * @boolean
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_W_EN, 0);

/**
 * Wheel steering rate proportional gain
 *
 * This defines how much the wheel steering input will be commanded depending on the
 * current body angular rate error.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_P, 0.5f);

/**
 * Wheel steering rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_I, 0.1f);

/**
 * Wheel steering rate integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_IMAX, 0.4f);

/**
 * Maximum wheel steering rate
 *
 * This limits the maximum wheel steering rate the controller will output (in degrees per
 * second).
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_W_RMAX, 30.0f);

/**
 * Wheel steering rate feed forward
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_FF, 0.2f);

/**
 * Pitch setpoint offset (pitch at level flight)
 *
 * An airframe specific offset of the pitch setpoint in degrees, the value is
 * added to the pitch setpoint and should correspond to the pitch at
 * typical cruise speed of the airframe.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PSP_OFF, 0.0f);

/**
 * Maximum manually added yaw rate
 *
 * This is the maximally added yaw rate setpoint from the yaw stick in any attitude controlled flight mode.
 * It is added to the yaw rate setpoint generated by the controller for turn coordination.
 *
 * @unit deg/s
 * @min 0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_YR_MAX, 30.f);

/**
 * Maximum manual roll angle
 *
 * Applies to both directions in all manual modes with attitude stabilization
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_R_MAX, 45.0f);

/**
 * Maximum manual pitch angle
 *
 * Applies to both directions in all manual modes with attitude stabilization but without altitude control
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_P_MAX, 30.0f);



/**
 * Roll scale factor for vision based navigation
 *
 * This parameter defines the roll scale factor for vision-based navigation.
 * It should be a value between 0 and 1.
 *
 * @unit deg
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group FW Attitude Control
 *
 */
PARAM_DEFINE_FLOAT(FW_ROLL_SCALE,0.0417f);


/**
 * Pitch scale factor for vision based navigation
 *
 * This parameter defines the pitch scale factor for vision-based navigation.
 * It should be a value between 0 and 1.
 *
 * @unit deg
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group FW Attitude Control
 *
 */
PARAM_DEFINE_FLOAT(FW_PITCH_SCALE,0.0417f);



/**
 * Enable vision based navigation
 *
 * This parameter enables or disables the vision-based navigation.
 * 0: disabled, 1: enabled
 *
 * @boolean
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_VIS_NAV_EN, 0);

/**
 * X-axis error proportional gain
 *
 * This parameter defines the proportional gain for the X-axis error controller.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(X_ERROR_KP, 1.0f);

/**
 * X-axis error integrator gain
 *
 * This parameter defines the integrator gain for the X-axis error controller.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(X_ERROR_KI, 0.0f);

/**
 * X-axis error integrator limit
 *
 * This parameter defines the maximum absolute value for the X-axis error integrator.
 *
 * @unit deg
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(X_INT_LIM, 1.0f);

/**
 * X-axis error integrator reset
 *
 * Set this parameter to 1 to reset the X-axis error integrator to zero.
 * The parameter will automatically reset to 0 after the integrator is cleared.
 *
 * @boolean
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(INT_RESET_X, 0);

/**
 * X-axis roll angle limit for vision based navigation
 *
 * This parameter defines the maximum allowed roll angle (in degrees) for X-axis vision-based navigation corrections.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(ROLL_LIM_X, 45.0f);

/**
 * Y-axis error proportional gain
 *
 * This parameter defines the proportional gain for the Y-axis error controller.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(Y_ERROR_KP, 1.0f);

/**
 * Y-axis error integrator gain
 *
 * This parameter defines the integrator gain for the Y-axis error controller.
 *
 * @unit deg
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(Y_ERROR_KI, 0.0f);

/**
 * Y-axis error integrator limit
 *
 * This parameter defines the maximum absolute value for the Y-axis error integrator.
 *
 * @unit deg
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(Y_INT_LIM, 1.0f);

/**
 * Y-axis error integrator reset
 *
 * Set this parameter to 1 to reset the Y-axis error integrator to zero.
 * The parameter will automatically reset to 0 after the integrator is cleared.
 *
 * @boolean
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(INT_RESET_Y, 0);


/**
 * Y-axis roll angle limit for vision based navigation
 *
 * This parameter defines the maximum allowed roll angle (in degrees) for Y-axis vision-based navigation corrections.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(ROLL_LIM_Y, 45.0f);
