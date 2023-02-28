#pragma once

// from include/motion_velocity_smoother/resample.hpp
struct ResampleParam
{
  const double max_trajectory_length;
  const double min_trajectory_length;
  const double resample_time;
  const double dense_resample_dt;
  const double dense_min_interval_distance;
  const double sparse_resample_dt;
  const double sparse_min_interval_distance;
};

// from include/motion_velocity_smoother/motion_velocity_smoother_node.hpp
struct Param
{
  // from
  // autoware_launch/config/planning/scenario_planning/common/motion_velocity_smoother/motion_velocity_smoother.param.yaml
  // See MotionVelocitySmootherNode::initCommonParam()
  const double max_velocity = 20.0;
  const double margin_to_insert_external_velocity_limit = 0.3;
  const double replan_vel_deviation = 5.53;

  const double engage_velocity = 0.25;
  const double engage_acceleration = 0.1;
  const double engage_exit_ratio = 0.5;

  const double stopping_velocity = 2.778;
  const double stopping_distance = 0.0;
  const double extract_ahead_dist = 200.0;
  const double extract_behind_dist = 5.0;
  const double stop_dist_to_prohibit_engage = 0.5;
  const double over_stop_velocity_warn_thr = 1.389;

  // from autoware_launch/config/planning/scenario_planning/common/nearest_search.param.yaml
  // See MotionVelocitySmootherNode::initCommonParam()
  const double ego_nearest_dist_threshold = 3.0;
  const double ego_nearest_yaw_threshold = 1.046;

  // from include/motion_velocity_smoother/smoother/smoother_base.hpp
  const struct BaseParam
  {
    // from
    // autoware_launch/config/planning/scenario_planning/common/motion_velocity_smoother/motion_velocity_smoother.param.yaml
    // and autoware_launch/config/planning/scenario_planning/common/common.param.yaml
    // See SmootherBase::SmootherBase()
    const double max_accel = 1.0;
    const double min_decel = -1.0;
    const double stop_decel = 0.0;
    const double max_jerk = 1.0;
    const double min_jerk = -1.0;
    const double max_lateral_accel = 1.0;
    const double min_decel_for_lateral_acc_lim_filter = -2.5;
    const double min_curve_velocity = 2.74;
    const double decel_distance_before_curve = 3.5;
    const double decel_distance_after_curve = 2.0;
    const double max_steering_angle_rate = 40.0;
    const double wheel_base = 2.79;  // wheel base [m]
    const double sample_ds = 0.1;
    const double curvature_threshold = 0.02;
    const double curvature_calculation_distance = 1.0;

    // from
    // autoware_launch/config/planning/scenario_planning/common/motion_velocity_smoother/motion_velocity_smoother.param.yaml
    // See SmootherBase::SmootherBase()
    ResampleParam resample_param{
      200.0, /* max_trajectory_length */
      180.0, /* min_trajectory_length */
      2.0,   /* resample_time */
      0.2,   /* dense_resample_dt */
      0.1,   /* dense_min_interval_distance */
      0.5,   /* sparse_resamle_dt */
      4.0    /* sparse_min_interval_distance */
    };
  } base_param;

  // from
  // autoware_launch/config/planning/scenario_planning/common/motion_velocity_smoother/motion_velocity_smoother.param.yaml
  // See MotionVelocitySmootherNode::initCommonParam()
  ResampleParam post_resample_param{300.0, 30.0, 10.0, 0.1, 0.1, 0.1, 1.0};

  // from
  // autoware_launch/config/planning/scenario_planning/common/motion_velocity_smoother/JerkFiltered.param.yaml
  struct JerkFiltered
  {
    const double jerk_weight = 0.1;
    const double over_v_weight = 10000.0;
    const double over_a_weight = 500.0;
    const double over_j_weight = 200.0;
    const double jerk_filter_ds = 0.1;
  } smoother_param;

  struct QPSolver
  {
    const double max_iter = 20000;
    const double rho_interval = 0;  // 0 means automatic
    const double eps_rel = 1.0e-6;  // def: 1.0e-4
    const double eps_abs = 1.0e-8;  // def: 1.0e-4
    const bool verbose = false;
  } solver_setting;
};
