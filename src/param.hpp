// copied from include/motion_velocity_smoother/motion_velocity_smoother_node.hpp
struct Param
{
  // from
  // autoware_launch/config/planning/scenario_planning/common/motion_velocity_smoother/motion_velocity_smoother.param.yaml
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

  // from autoware_launch/config/planning/scenario_planning/common/nearest_search.param.yaml
  const double ego_nearest_dist_threshold = 3.0;
  const double ego_nearest_yaw_threshold = 1.046;

  // from include/motion_velocity_smoother/smoother/smoother_base.hpp
  const struct BaseParam
  {
    const double max_accel = 1.0;
    const double min_decel = -0.5;
    const double stop_decel = 0.0;
    const double max_jerk = 1.0;
    const double min_jerk = -0.5;
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

    const struct ResampleParam
    {
      const double max_trajectory_length = 300.0;
      const double min_trajectory_length = 30.0;
      const double resample_time = 10.0;
      const double dense_resample_dt = 0.1;
      const double dense_min_interval_distance = 0.1;
      const double sparse_resample_dt = 0.1;
      const double sparse_min_interval_distance = 1.0;
    } resample_param;

  } base_param;

  // from include/motion_velocity_smoother/resample.hpp
  const struct PostResampleParam
  {
    const double max_trajectory_length = 200.0;
    const double min_trajectory_length = 150.0;
    const double resample_time = 2.0;
    const double dense_resample_dt = 0.2;
    const double dense_min_interval_distance = 0.1;
    const double sparse_resample_dt = 0.5;
    const double sparse_min_interval_distance = 4.0;
  } post_resample_param;
};
