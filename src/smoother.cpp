#include "./smoother.hpp"

#include "./resample.hpp"
#include "./trajectory_utils.hpp"

#include <motion_utils/motion_utils.hpp>

void SmootherFrontEnd::onCurrentTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & sub_trajectory,
  const nav_msgs::msg::Odometry & current_odom)
{
  if (prev_output_.empty()) {
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      prev_output_, current_odom.pose.pose, param_.ego_nearest_dist_threshold,
      param_.ego_nearest_yaw_threshold);
    current_closest_point_from_prev_output_ = trajectory_utils::calcInterpolatedTrajectoryPoint(
      prev_output_, current_odom.pose.pose, current_seg_idx);
  }

  const auto input = motion_utils::convertToTrajectoryPointArray(sub_trajectory);
  const size_t input_closest = findNearestIndexFromEgo(input, current_odom);

  // Size changes(trimming)
  auto input_trimmed = trajectory_utils::extractPathAroundIndex(
    input, input_closest, param_.extract_ahead_dist, param_.extract_behind_dist);
  if (input_trimmed.empty()) {
    return;
  }
  const size_t input_trimmed_closest = findNearestIndexFromEgo(input_trimmed, current_odom);

  // Size does not change
  applyStopApproachingVelocity(&input_trimmed);

  const auto [initial_motion, type] =
    calcInitialMotion(input_trimmed, current_odom, input_trimmed_closest);

  // Size changes(resample)
  const auto input_acc_filtered =
    applyLateralAccelerationFilter(input_trimmed, initial_motion.vel, initial_motion.acc, true);
  if (not input_acc_filtered.has_value()) {
    return;
  }

  // Size changes(resample)
  const auto input_steer_filtered = applySteeringRateLimit(input_acc_filtered.value());
  if (not input_steer_filtered.has_value()) {
    return;
  }

  // Size changes(resample): this functions is virtual in smoother
  auto input_filtered_resample = resampling::resampleTrajectory(
    input_steer_filtered.value(), current_odom.pose.pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold, param_.base_param.resample_param,
    param_.smoother_param.jerk_filter_ds);
  const size_t input_filtered_resample_closest =
    findNearestIndexFromEgo(input_filtered_resample, current_odom);
  // Set 0[m/s] in the terminal point
  if (not input_filtered_resample.empty()) {
    input_filtered_resample.back().longitudinal_velocity_mps = 0.0;
  }

  // Size changes ?(do trimming to run optimization only on the interval [closest, end))
  TrajectoryPoints optimization_part;
  optimization_part.insert(
    optimization_part.end(), input_filtered_resample.begin() + input_filtered_resample_closest,
    input_filtered_resample.end());

  std::optional<TrajectoryPoints> optimized_opt =
    apply(initial_motion.vel, initial_motion.acc, optimization_part);
  if (not optimized_opt.has_value()) {
    return;
  }
  TrajectoryPoints optimized = optimized_opt.value();

  overwriteStopPoint(optimization_part, &optimized);

  // Becomes the same size input_filltered_resample (push [strat, closest) part)
  optimized.insert(
    optimized.begin(), input_filtered_resample.begin(),
    input_filtered_resample.begin() + input_filtered_resample_closest);

  // For the endpoint of the trajectory
  if (not optimized.empty()) {
    optimized.back().longitudinal_velocity_mps = 0.0;
  }

  // Max velocity filter for safety
  trajectory_utils::applyMaximumVelocityLimit(
    input_filtered_resample_closest, optimized.size(), param_.max_velocity, &optimized);

  // Insert behind velocity for output's consistency
  insertBehindVelocity(input_filtered_resample_closest, type, &optimized);

  // COMMENT: save soltion to prev_output_
  updatePrevValues(optimized, current_odom);

  // NOTE: `solution_resample` is published to controller
  const auto solution_resampled = resampling::resampleTrajectory(
    optimized, current_odom.pose.pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold, param_.post_resample_param, false);
}

// optimizer function
std::optional<TrajectoryPoints> SmootherFrontEnd::apply(
  [[maybe_unused]] const double v, [[maybe_unused]] const double a,
  [[maybe_unused]] const TrajectoryPoints & input) const
{
  return std::nullopt;
}

// misc functions
size_t SmootherFrontEnd::findNearestIndexFromEgo(
  const TrajectoryPoints & points, const nav_msgs::msg::Odometry & current_odom) const
{
  return motion_utils::findFirstNearestIndexWithSoftConstraints(
    points, current_odom.pose.pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold);
}

void SmootherFrontEnd::updatePrevValues(
  const TrajectoryPoints & solution, const nav_msgs::msg::Odometry & current_odom)
{
  prev_output_ = solution;
  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    solution, current_odom.pose.pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold);
  prev_closest_point_ = trajectory_utils::calcInterpolatedTrajectoryPoint(
    solution, current_odom.pose.pose, current_seg_idx);
}

void SmootherFrontEnd::applyStopApproachingVelocity(TrajectoryPoints * input) const
{
  const auto stop_idx = motion_utils::searchZeroVelocityIndex(*input);
  if (!stop_idx) {
    // do nothing
    return;
  }

  double distance_sum = 0.0;
  // Bug? (size_t i = *stop_idx - 1; i > 0; --i)
  for (size_t i = *stop_idx - 1; i < input->size(); --i) {
    distance_sum += tier4_autoware_utils::calcDistance2d(input->at(i), input->at(i + 1));
    if (distance_sum > param_.stopping_distance) {
      break;
    }
    if (input->at(i).longitudinal_velocity_mps > param_.stopping_velocity) {
      input->at(i).longitudinal_velocity_mps = param_.stopping_velocity;
    }
  }
}

std::pair<SmootherFrontEnd::Motion, SmootherFrontEnd::InitializeType>
SmootherFrontEnd::calcInitialMotion(
  const TrajectoryPoints & input, const nav_msgs::msg::Odometry & current_odom,
  const size_t input_closest) const
{
  const double vehicle_speed = std::fabs(current_odom.twist.twist.linear.x);

  if (!current_closest_point_from_prev_output_.has_value()) {
    return {{vehicle_speed, 0.0}, InitializeType::INIT};
  }

  const double desired_vel =
    current_closest_point_from_prev_output_.value().longitudinal_velocity_mps;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);
  if (std::fabs(vel_error) > param_.replan_vel_deviation) {
    return {
      {vehicle_speed, current_closest_point_from_prev_output_.value().acceleration_mps2},
      InitializeType::LARGE_DEVIATION_REPLAN};
  }

  const double target_vel = std::fabs(input.at(input_closest).longitudinal_velocity_mps);
  const double engage_vel_thr = param_.engage_velocity * param_.engage_exit_ratio;

  // if current vehicle velocity is low && base_desired speed is high && and next_stop is not near,
  // use engage_velocity for engage vehicle
  if (vehicle_speed < engage_vel_thr and target_vel >= param_.engage_velocity) {
    const auto idx = motion_utils::searchZeroVelocityIndex(input);
    const double dist_to_next_stop =
      idx ? tier4_autoware_utils::calcDistance2d(input.at(*idx), input.at(input_closest)) : 0.0;
    if (!idx or dist_to_next_stop > param_.stop_dist_to_prohibit_engage) {
      return {{param_.engage_velocity, param_.engage_acceleration}, InitializeType::ENGAGING};
    }
  }

  // normal update: use closest in current_closest_point_from_prev_output
  return {
    {current_closest_point_from_prev_output_.value().longitudinal_velocity_mps,
     current_closest_point_from_prev_output_.value().acceleration_mps2},
    InitializeType::NORMAL};
}

std::optional<TrajectoryPoints> SmootherFrontEnd::applyLateralAccelerationFilter(
  const TrajectoryPoints & input, const double v0, const double a0,
  const bool enable_smooth_limit) const
{
  if (input.empty()) {
    return std::nullopt;
  }

  if (input.size() < 3) {
    return std::make_optional<TrajectoryPoints>(
      input);  // cannot calculate lateral acc. do nothing.
  }

  // Interpolate with constant interval distance for lateral acceleration calculation.
  constexpr double points_interval = 0.1;  // [m]
  std::vector<double> out_arclength;
  const auto traj_length = motion_utils::calcArcLength(input);
  for (double s = 0; s < traj_length; s += points_interval) {
    out_arclength.push_back(s);
  }
  const auto output_traj =
    motion_utils::resampleTrajectory(motion_utils::convertToTrajectory(input), out_arclength);
  auto output = motion_utils::convertToTrajectoryPointArray(output_traj);
  output.back() = input.back();  // keep the final speed.

  constexpr double curvature_calc_dist = 5.0;  // [m] calc curvature with 5m away points
  const size_t idx_dist =
    static_cast<size_t>(std::max(static_cast<int>((curvature_calc_dist) / points_interval), 1));

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(output, idx_dist);

  //  Decrease speed according to lateral G
  const size_t before_decel_index = static_cast<size_t>(
    std::round(param_.base_param.decel_distance_before_curve / points_interval));
  const size_t after_decel_index =
    static_cast<size_t>(std::round(param_.base_param.decel_distance_after_curve / points_interval));
  const double max_lateral_accel_abs = std::fabs(param_.base_param.max_lateral_accel);

  const auto latacc_min_vel_arr =
    enable_smooth_limit ? trajectory_utils::calcVelocityProfileWithConstantJerkAndAccelerationLimit(
                            output, v0, a0, param_.base_param.min_jerk, param_.base_param.max_accel,
                            param_.base_param.min_decel_for_lateral_acc_lim_filter)
                        : std::vector<double>{};

  for (size_t i = 0; i < output.size(); ++i) {
    double curvature = 0.0;
    const size_t start = i > after_decel_index ? i - after_decel_index : 0;
    const size_t end = std::min(output.size(), i + before_decel_index + 1);
    for (size_t j = start; j < end; ++j) {
      if (j >= curvature_v.size()) return std::make_optional<TrajectoryPoints>(output);
      curvature = std::max(curvature, std::fabs(curvature_v.at(j)));
    }
    double v_curvature_max = std::sqrt(max_lateral_accel_abs / std::max(curvature, 1.0E-5));
    v_curvature_max = std::max(v_curvature_max, param_.base_param.min_curve_velocity);

    if (enable_smooth_limit) {
      if (i >= latacc_min_vel_arr.size()) return std::make_optional<TrajectoryPoints>(output);
      v_curvature_max = std::max(v_curvature_max, latacc_min_vel_arr.at(i));
    }
    if (output.at(i).longitudinal_velocity_mps > v_curvature_max) {
      output.at(i).longitudinal_velocity_mps = v_curvature_max;
    }
  }
  return std::make_optional<TrajectoryPoints>(output);
}

std::optional<TrajectoryPoints> SmootherFrontEnd::applySteeringRateLimit(
  const TrajectoryPoints & input) const
{
  if (input.empty()) {
    return std::nullopt;
  }

  if (input.size() < 3) {
    return std::make_optional<TrajectoryPoints>(
      input);  // cannot calculate the desired velocity. do nothing.
  }
  // Interpolate with constant interval distance for lateral acceleration calculation.
  std::vector<double> out_arclength;
  const auto traj_length = motion_utils::calcArcLength(input);
  for (double s = 0; s < traj_length; s += param_.base_param.sample_ds) {
    out_arclength.push_back(s);
  }
  const auto output_traj =
    motion_utils::resampleTrajectory(motion_utils::convertToTrajectory(input), out_arclength);
  auto output = motion_utils::convertToTrajectoryPointArray(output_traj);
  output.back() = input.back();  // keep the final speed.

  const size_t idx_dist = static_cast<size_t>(std::max(
    static_cast<int>(
      (param_.base_param.curvature_calculation_distance) / param_.base_param.sample_ds),
    1));

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(output, idx_dist);

  for (size_t i = 0; i + 1 < output.size(); i++) {
    if (fabs(curvature_v.at(i)) > param_.base_param.curvature_threshold) {
      // calculate the just 2 steering angle
      output.at(i).front_wheel_angle_rad =
        std::atan(param_.base_param.wheel_base * curvature_v.at(i));
      output.at(i + 1).front_wheel_angle_rad =
        std::atan(param_.base_param.wheel_base * curvature_v.at(i + 1));

      const double mean_vel =
        (output.at(i).longitudinal_velocity_mps + output.at(i + 1).longitudinal_velocity_mps) / 2.0;
      const double dt =
        std::max(param_.base_param.sample_ds / mean_vel, std::numeric_limits<double>::epsilon());
      const double steering_diff =
        fabs(output.at(i).front_wheel_angle_rad - output.at(i + 1).front_wheel_angle_rad);
      const double dt_steering =
        steering_diff / tier4_autoware_utils::deg2rad(param_.base_param.max_steering_angle_rate);

      if (dt_steering > dt) {
        const double target_mean_vel = (param_.base_param.sample_ds / dt_steering);
        for (size_t k = 0; k < 2; k++) {
          const double temp_vel =
            output.at(i + k).longitudinal_velocity_mps * (target_mean_vel / mean_vel);
          if (temp_vel < output.at(i + k).longitudinal_velocity_mps) {
            output.at(i + k).longitudinal_velocity_mps = temp_vel;
          } else {
            if (target_mean_vel < output.at(i + k).longitudinal_velocity_mps) {
              output.at(i + k).longitudinal_velocity_mps = target_mean_vel;
            }
          }
          if (output.at(i + k).longitudinal_velocity_mps < param_.base_param.min_curve_velocity) {
            output.at(i + k).longitudinal_velocity_mps = param_.base_param.min_curve_velocity;
          }
        }
      }
    }
  }

  return std::make_optional<TrajectoryPoints>(output);
}

void SmootherFrontEnd::overwriteStopPoint(
  const TrajectoryPoints & input, TrajectoryPoints * output) const
{
  const auto stop_idx = motion_utils::searchZeroVelocityIndex(input);
  if (!stop_idx) {
    return;
  }

  // Get Closest Point from Output
  // TODO(planning/control team) deal with overlapped lanes with the same directions
  const auto nearest_output_point_idx = motion_utils::findNearestIndex(
    *output, input.at(*stop_idx).pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold);

  // check over velocity
  if (nearest_output_point_idx) {
    trajectory_utils::applyMaximumVelocityLimit(
      *nearest_output_point_idx, output->size(), 0.0, output);
  }
}

void SmootherFrontEnd::insertBehindVelocity(
  const size_t output_closest, const InitializeType type, TrajectoryPoints * output) const
{
  const bool keep_closest_vel_for_behind =
    (type == InitializeType::INIT || type == InitializeType::LARGE_DEVIATION_REPLAN ||
     type == InitializeType::ENGAGING);

  for (size_t i = output_closest - 1; i < output->size(); --i) {
    if (keep_closest_vel_for_behind) {
      output->at(i).longitudinal_velocity_mps =
        output->at(output_closest).longitudinal_velocity_mps;
      output->at(i).acceleration_mps2 = output->at(output_closest).acceleration_mps2;
    } else {
      // TODO(planning/control team) deal with overlapped lanes with the same direction
      const size_t seg_idx = [&]() {
        // with distance and yaw thresholds
        const auto opt_nearest_seg_idx = motion_utils::findNearestSegmentIndex(
          prev_output_, output->at(i).pose, param_.ego_nearest_dist_threshold,
          param_.ego_nearest_yaw_threshold);
        if (opt_nearest_seg_idx) {
          return opt_nearest_seg_idx.get();
        }

        // with distance threshold
        const auto opt_second_nearest_seg_idx = motion_utils::findNearestSegmentIndex(
          prev_output_, output->at(i).pose, param_.ego_nearest_dist_threshold);
        if (opt_second_nearest_seg_idx) {
          return opt_second_nearest_seg_idx.get();
        }

        return motion_utils::findNearestSegmentIndex(prev_output_, output->at(i).pose.position);
      }();
      const auto prev_output_point = trajectory_utils::calcInterpolatedTrajectoryPoint(
        prev_output_, output->at(i).pose, seg_idx);

      // output should be always positive: TODO(Horibe) think better way
      output->at(i).longitudinal_velocity_mps =
        std::abs(prev_output_point.longitudinal_velocity_mps);
      output->at(i).acceleration_mps2 = prev_output_point.acceleration_mps2;
    }
  }
}
