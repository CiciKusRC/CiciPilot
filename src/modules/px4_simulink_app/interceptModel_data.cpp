//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: interceptModel_data.cpp
//
// Code generated for Simulink model 'interceptModel'.
//
// Model version                  : 1.41
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun Jul 20 21:25:48 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "interceptModel.h"

// Block parameters (default storage)
P_interceptModel_T interceptModel_P = {
  // Mask Parameter: Discrete_Integrator_Reset_reset
  //  Referenced by: '<S9>/Constant2'

  1.0,

  // Mask Parameter: Discrete_Integrator_Reset_res_f
  //  Referenced by: '<S13>/Constant2'

  1.0,

  // Mask Parameter: DirectionCosineMatrixtoRotation
  //  Referenced by:
  //    '<S48>/Constant'
  //    '<S50>/Constant'

  4.4408920985006262E-16,

  // Mask Parameter: DirectionCosineMatrixtoRotati_c
  //  Referenced by:
  //    '<S27>/Constant'
  //    '<S37>/Constant'
  //    '<S39>/Constant'

  1U,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S65>/Out1'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample
    (0ULL),                            // ref_timestamp
    0.0,                               // ref_lat
    0.0,                               // ref_lon
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // z

    {
      0.0F, 0.0F }
    ,                                  // delta_xy
    0.0F,                              // delta_z
    0.0F,                              // vx
    0.0F,                              // vy
    0.0F,                              // vz
    0.0F,                              // z_deriv

    {
      0.0F, 0.0F }
    ,                                  // delta_vxy
    0.0F,                              // delta_vz
    0.0F,                              // ax
    0.0F,                              // ay
    0.0F,                              // az
    0.0F,                              // heading
    0.0F,                              // heading_var
    0.0F,                              // unaided_heading
    0.0F,                              // delta_heading
    0.0F,                              // tilt_var
    0.0F,                              // ref_alt
    0.0F,                              // dist_bottom
    0.0F,                              // dist_bottom_var
    0.0F,                              // delta_dist_bottom
    0.0F,                              // eph
    0.0F,                              // epv
    0.0F,                              // evh
    0.0F,                              // evv
    0.0F,                              // vxy_max
    0.0F,                              // vz_max
    0.0F,                              // hagl_min
    0.0F,                              // hagl_max_z
    0.0F,                              // hagl_max_xy
    false,                             // xy_valid
    false,                             // z_valid
    false,                             // v_xy_valid
    false,                             // v_z_valid
    0U,                                // xy_reset_counter
    0U,                                // z_reset_counter
    0U,                                // vxy_reset_counter
    0U,                                // vz_reset_counter
    0U,                                // heading_reset_counter
    false,                             // heading_good_for_control
    false,                             // xy_global
    false,                             // z_global
    false,                             // dist_bottom_valid
    0U,                                // dist_bottom_reset_counter
    0U,                                // dist_bottom_sensor_bitfield
    false                              // dead_reckoning
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S19>/Constant'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample
    (0ULL),                            // ref_timestamp
    0.0,                               // ref_lat
    0.0,                               // ref_lon
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // z

    {
      0.0F, 0.0F }
    ,                                  // delta_xy
    0.0F,                              // delta_z
    0.0F,                              // vx
    0.0F,                              // vy
    0.0F,                              // vz
    0.0F,                              // z_deriv

    {
      0.0F, 0.0F }
    ,                                  // delta_vxy
    0.0F,                              // delta_vz
    0.0F,                              // ax
    0.0F,                              // ay
    0.0F,                              // az
    0.0F,                              // heading
    0.0F,                              // heading_var
    0.0F,                              // unaided_heading
    0.0F,                              // delta_heading
    0.0F,                              // tilt_var
    0.0F,                              // ref_alt
    0.0F,                              // dist_bottom
    0.0F,                              // dist_bottom_var
    0.0F,                              // delta_dist_bottom
    0.0F,                              // eph
    0.0F,                              // epv
    0.0F,                              // evh
    0.0F,                              // evv
    0.0F,                              // vxy_max
    0.0F,                              // vz_max
    0.0F,                              // hagl_min
    0.0F,                              // hagl_max_z
    0.0F,                              // hagl_max_xy
    false,                             // xy_valid
    false,                             // z_valid
    false,                             // v_xy_valid
    false,                             // v_z_valid
    0U,                                // xy_reset_counter
    0U,                                // z_reset_counter
    0U,                                // vxy_reset_counter
    0U,                                // vz_reset_counter
    0U,                                // heading_reset_counter
    false,                             // heading_good_for_control
    false,                             // xy_global
    false,                             // z_global
    false,                             // dist_bottom_valid
    0U,                                // dist_bottom_reset_counter
    0U,                                // dist_bottom_sensor_bitfield
    false                              // dead_reckoning
  },

  // Computed Parameter: Out1_Y0_p
  //  Referenced by: '<S64>/Out1'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // q

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // delta_q_reset
    0U,                                // quat_reset_counter

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U }
    // _padding0
  },

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S18>/Constant'

  {
    (0ULL),                            // timestamp
    (0ULL),                            // timestamp_sample

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // q

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // delta_q_reset
    0U,                                // quat_reset_counter

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U }
    // _padding0
  },

  // Computed Parameter: Out1_Y0_l
  //  Referenced by: '<S21>/Out1'

  {
    (0ULL),                            // timestamp
    0.0F,                              // indicated_airspeed_m_s
    0.0F,                              // calibrated_airspeed_m_s
    0.0F,                              // true_airspeed_m_s
    0.0F,                              // calibrated_ground_minus_wind_m_s
    0.0F,                              // true_ground_minus_wind_m_s
    0.0F,                              // airspeed_derivative_filtered
    0.0F,                              // throttle_filtered
    0.0F,                              // pitch_filtered
    false,                             // airspeed_sensor_measurement_valid
    0,                                 // selected_airspeed_index

    {
      0U, 0U, 0U, 0U, 0U, 0U }
    // _padding0
  },

  // Computed Parameter: Constant_Value_hp
  //  Referenced by: '<S16>/Constant'

  {
    (0ULL),                            // timestamp
    0.0F,                              // indicated_airspeed_m_s
    0.0F,                              // calibrated_airspeed_m_s
    0.0F,                              // true_airspeed_m_s
    0.0F,                              // calibrated_ground_minus_wind_m_s
    0.0F,                              // true_ground_minus_wind_m_s
    0.0F,                              // airspeed_derivative_filtered
    0.0F,                              // throttle_filtered
    0.0F,                              // pitch_filtered
    false,                             // airspeed_sensor_measurement_valid
    0,                                 // selected_airspeed_index

    {
      0U, 0U, 0U, 0U, 0U, 0U }
    // _padding0
  },

  // Computed Parameter: Out1_Y0_n
  //  Referenced by: '<S20>/Out1'

  {
    (0ULL),                            // timestamp
    0.0F,                              // target_x
    0.0F,                              // target_y
    false,                             // tracker_status

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U }
    // _padding0
  },

  // Computed Parameter: Constant_Value_f
  //  Referenced by: '<S15>/Constant'

  {
    (0ULL),                            // timestamp
    0.0F,                              // target_x
    0.0F,                              // target_y
    false,                             // tracker_status

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U }
    // _padding0
  },

  // Computed Parameter: Constant_Value_d
  //  Referenced by: '<S1>/Constant'

  {
    (0ULL),                            // timestamp
    0.0F,                              // target_roll
    0.0F                               // target_pitch
  },

  // Expression: 1
  //  Referenced by: '<S10>/Constant2'

  1.0,

  // Expression: 1
  //  Referenced by: '<S14>/Constant2'

  1.0,

  // Expression: 1
  //  Referenced by: '<S33>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S34>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S29>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S30>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S39>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S37>/Constant1'

  0.0,

  // Expression: -eye(3)
  //  Referenced by: '<S40>/Bias1'

  { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0 },

  // Expression: -1
  //  Referenced by: '<S41>/Bias'

  -1.0,

  // Expression: 2
  //  Referenced by: '<S54>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S57>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S52>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S58>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S53>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S56>/Gain'

  2.0,

  // Expression: [1 1 -1]
  //  Referenced by: '<S26>/Gain1'

  { 1.0, 1.0, -1.0 },

  // Expression: [1 1]
  //  Referenced by: '<S26>/Gain2'

  { 1.0, 1.0 },

  // Expression: [-1 1]
  //  Referenced by: '<S26>/Gain3'

  { -1.0, 1.0 },

  // Expression: 0
  //  Referenced by: '<S4>/Constant'

  0.0,

  // Expression: 100
  //  Referenced by: '<S4>/Saturation'

  100.0,

  // Expression: -100
  //  Referenced by: '<S4>/Saturation'

  -100.0,

  // Expression: 0
  //  Referenced by: '<S4>/Constant1'

  0.0,

  // Expression: 4
  //  Referenced by: '<S9>/Constant3'

  4.0,

  // Expression: -1
  //  Referenced by: '<S4>/Gain1'

  -1.0,

  // Expression: 0
  //  Referenced by: '<S5>/Constant'

  0.0,

  // Expression: 100
  //  Referenced by: '<S5>/Saturation'

  100.0,

  // Expression: -100
  //  Referenced by: '<S5>/Saturation'

  -100.0,

  // Expression: 0
  //  Referenced by: '<S5>/Constant1'

  0.0,

  // Expression: 4
  //  Referenced by: '<S13>/Constant3'

  4.0,

  // Expression: -1
  //  Referenced by: '<S5>/Gain1'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S14>/Constant1'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S5>/Gain'

  -1.0,

  // Expression: 0
  //  Referenced by: '<S14>/Constant3'

  0.0,

  // Expression: 0.01
  //  Referenced by: '<S13>/Constant'

  0.01,

  // Expression: 0
  //  Referenced by: '<S10>/Constant3'

  0.0,

  // Expression: -1
  //  Referenced by: '<S10>/Constant1'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S4>/Gain'

  -1.0,

  // Expression: 0.01
  //  Referenced by: '<S9>/Constant'

  0.01,

  // Computed Parameter: RateTransition_1_InitialConditi
  //  Referenced by: '<Root>/Rate Transition'

  0.0F,

  // Computed Parameter: RateTransition_4_InitialConditi
  //  Referenced by: '<Root>/Rate Transition'

  0.0F,

  // Computed Parameter: RateTransition_5_InitialConditi
  //  Referenced by: '<Root>/Rate Transition'

  0.0F,

  // Computed Parameter: RateTransition_3_InitialConditi
  //  Referenced by: '<Root>/Rate Transition'

  0.0F,

  // Computed Parameter: RateTransition_2_InitialConditi
  //  Referenced by: '<Root>/Rate Transition'

  0.0F,

  // Computed Parameter: Assertion_Enabled
  //  Referenced by: '<S47>/Assertion'

  true,

  // Computed Parameter: Assertion_Enabled_a
  //  Referenced by: '<S45>/Assertion'

  true,

  // Computed Parameter: Assertion_Enabled_g
  //  Referenced by: '<S44>/Assertion'

  true,

  // Computed Parameter: Assertion_Enabled_f
  //  Referenced by: '<S42>/Assertion'

  true,

  // Expression: false
  //  Referenced by: '<S9>/Constant4'

  false,

  // Expression: false
  //  Referenced by: '<S13>/Constant4'

  false
};

//
// File trailer for generated code.
//
// [EOF]
//
