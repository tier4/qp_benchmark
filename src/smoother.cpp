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

  // COMMENT: is trimming necessary given the usage of warm-start ?
  auto input_trimmed = trajectory_utils::extractPathAroundIndex(
    input, input_closest, param_.extract_ahead_dist, param_.extract_behind_dist);
  if (input_trimmed.empty()) {
    return;
  }
  const size_t input_trimmed_closest = findNearestIndexFromEgo(input_trimmed, current_odom);

  // COMMENT: this process looks necessary to avoid sudden braking
  applyStopApproachingVelocity(&input_trimmed);

  [[maybe_unused]] const auto intial_motion =
    calcInitialMotion(input_trimmed, current_odom, input_trimmed_closest);

  // COMMENT: main QP-solver function
  const std::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>> solution =
    std::nullopt;
  if (not solution.has_value()) {
    return;
  }

  // COMMENT: save soltion to prev_output_
  updatePrevValues(solution.value(), current_odom);

  // NOTE: `solution_resample` is published to controller
  const auto solution_resampled = resampling::resampleTrajectory(
    solution.value(), current_odom.twist.twist.linear.x, current_odom.pose.pose,
    param_.ego_nearest_dist_threshold, param_.ego_nearest_yaw_threshold, param_.post_resample_param,
    false);
}

// misc functions
size_t SmootherFrontEnd::findNearestIndexFromEgo(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const nav_msgs::msg::Odometry & current_odom) const
{
  return motion_utils::findFirstNearestIndexWithSoftConstraints(
    points, current_odom.pose.pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold);
}

void SmootherFrontEnd::updatePrevValues(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & solution,
  const nav_msgs::msg::Odometry & current_odom)
{
  prev_output_ = solution;
  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    solution, current_odom.pose.pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold);
  prev_closest_point_ = trajectory_utils::calcInterpolatedTrajectoryPoint(
    solution, current_odom.pose.pose, current_seg_idx);
}

void SmootherFrontEnd::applyStopApproachingVelocity(
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> * input) const
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
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & input,
  const nav_msgs::msg::Odometry & current_odom, const size_t input_closest) const
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
