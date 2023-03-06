#include "./smoother.hpp"

#include "./resample.hpp"
#include "./trajectory_utils.hpp"

#include <motion_utils/motion_utils.hpp>

TrajectoryPoints SmootherFrontEnd::onCurrentTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & sub_trajectory,
  const nav_msgs::msg::Odometry & current_odom)
{
  if (not prev_output_.empty()) {
    // NOTE: calcProjectedTrajectoryPointFromEgo(prev_output_)と同一
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      prev_output_, current_odom.pose.pose, param_.ego_nearest_dist_threshold,
      param_.ego_nearest_yaw_threshold);
    current_closest_point_from_prev_output_ = trajectory_utils::calcInterpolatedTrajectoryPoint(
      prev_output_, current_odom.pose.pose, current_seg_idx);
  }

  const auto input = motion_utils::convertToTrajectoryPointArray(sub_trajectory);
  const size_t input_closest = findNearestIndexFromEgo(input, current_odom);

  // Size changes(trimming)
  auto traj_extracted = trajectory_utils::extractPathAroundIndex(
    input, input_closest, param_.extract_ahead_dist, param_.extract_behind_dist);
  std::cout << "extract_ahead_dist = " << param_.extract_ahead_dist << std::endl;
  std::cout << "extract_behind_dist = " << param_.extract_behind_dist << std::endl;

  if (traj_extracted.empty()) {
    return prev_output_;
  }
  const size_t traj_extracted_closest = findNearestIndexFromEgo(traj_extracted, current_odom);

  // Size does not change
  applyStopApproachingVelocity(&traj_extracted);

  const auto [initial_motion, type] =
    calcInitialMotion(traj_extracted, current_odom, traj_extracted_closest);

  // Size changes(resample)
  const auto traj_lateral_acc_filtered =
    applyLateralAccelerationFilter(traj_extracted, initial_motion.vel, initial_motion.acc, true);
  if (not traj_lateral_acc_filtered.has_value()) {
    return prev_output_;
  }

  // Size changes(resample)
  const auto traj_steering_rate_limited = applySteeringRateLimit(traj_lateral_acc_filtered.value());
  if (not traj_steering_rate_limited.has_value()) {
    return prev_output_;
  }

  // Size changes(resample)
  // JerkFilteredSmootherのresampleTrajectory()
  auto traj_resampled = resampling::resampleTrajectory(
    traj_steering_rate_limited.value(), current_odom.pose.pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold, param_.base_param.resample_param,
    param_.smoother_param.jerk_filter_ds, true);
  const size_t traj_resampled_closest = findNearestIndexFromEgo(traj_resampled, current_odom);
  // Set 0[m/s] in the terminal point
  if (not traj_resampled.empty()) {
    traj_resampled.back().longitudinal_velocity_mps = 0.0;
  }

  // Size changes ?(do trimming to run optimization only on the interval [closest, end))
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(), traj_resampled.begin() + traj_resampled_closest, traj_resampled.end());

  std::optional<TrajectoryPoints> traj_smoothed_opt =
    apply(initial_motion.vel, initial_motion.acc, clipped);
  if (not traj_smoothed_opt.has_value()) {
    return prev_output_;
  }
  TrajectoryPoints traj_smoothed = traj_smoothed_opt.value();

  overwriteStopPoint(clipped, &traj_smoothed);

  // Becomes the same size input_filltered_resample (push [strat, closest) part)
  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled.begin(), traj_resampled.begin() + traj_resampled_closest);

  // For the endpoint of the trajectory
  if (not traj_smoothed.empty()) {
    traj_smoothed.back().longitudinal_velocity_mps = 0.0;
  }

  // Max velocity filter for safety
  trajectory_utils::applyMaximumVelocityLimit(
    traj_resampled_closest, traj_smoothed.size(), param_.max_velocity, &traj_smoothed);
  std::cout << "max_velocity = " << param_.max_velocity << std::endl;

  // Insert behind velocity for output's consistency
  insertBehindVelocity(traj_resampled_closest, type, &traj_smoothed);

  // COMMENT: save solution to prev_output_
  updatePrevValues(traj_smoothed, current_odom);

  return traj_smoothed;
}

TrajectoryPoints SmootherFrontEnd::onCurrentTrajectory2(
  const autoware_auto_planning_msgs::msg::Trajectory & sub_trajectory,
  const nav_msgs::msg::Odometry & current_odom,
  const std::shared_ptr<motion_velocity_smoother::JerkFilteredSmoother> smoother)
{
  if (not prev_output_.empty()) {
    // NOTE: calcProjectedTrajectoryPointFromEgo(prev_output_)と同一
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      prev_output_, current_odom.pose.pose, param_.ego_nearest_dist_threshold,
      param_.ego_nearest_yaw_threshold);
    current_closest_point_from_prev_output_ = trajectory_utils::calcInterpolatedTrajectoryPoint(
      prev_output_, current_odom.pose.pose, current_seg_idx);
  }

  const auto input = motion_utils::convertToTrajectoryPointArray(sub_trajectory);
  const size_t input_closest = findNearestIndexFromEgo(input, current_odom);

  // Size changes(trimming)
  auto traj_extracted = trajectory_utils::extractPathAroundIndex(
    input, input_closest, param_.extract_ahead_dist, param_.extract_behind_dist);
  std::cout << "extract_ahead_dist = " << param_.extract_ahead_dist << std::endl;
  std::cout << "extract_behind_dist = " << param_.extract_behind_dist << std::endl;

  if (traj_extracted.empty()) {
    return prev_output_;
  }
  const size_t traj_extracted_closest = findNearestIndexFromEgo(traj_extracted, current_odom);

  // Size does not change
  applyStopApproachingVelocity(&traj_extracted);

  const auto [initial_motion, type] =
    calcInitialMotion(traj_extracted, current_odom, traj_extracted_closest);

  // Size changes(resample)
  const auto traj_lateral_acc_filtered =
    applyLateralAccelerationFilter(traj_extracted, initial_motion.vel, initial_motion.acc, true);
  if (not traj_lateral_acc_filtered.has_value()) {
    return prev_output_;
  }

  // Size changes(resample)
  const auto traj_steering_rate_limited = applySteeringRateLimit(traj_lateral_acc_filtered.value());
  if (not traj_steering_rate_limited.has_value()) {
    return prev_output_;
  }

  // Size changes(resample)
  // JerkFilteredSmootherのresampleTrajectory()
  auto traj_resampled = resampling::resampleTrajectory(
    traj_steering_rate_limited.value(), current_odom.pose.pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold, param_.base_param.resample_param,
    param_.smoother_param.jerk_filter_ds, true);
  const size_t traj_resampled_closest = findNearestIndexFromEgo(traj_resampled, current_odom);
  // Set 0[m/s] in the terminal point
  if (not traj_resampled.empty()) {
    traj_resampled.back().longitudinal_velocity_mps = 0.0;
  }

  // Size changes ?(do trimming to run optimization only on the interval [closest, end))
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(), traj_resampled.begin() + traj_resampled_closest, traj_resampled.end());

  TrajectoryPoints traj_smoothed;
  std::vector<TrajectoryPoints> debug_trajectories;
  if (!smoother->apply(
        initial_motion.vel, initial_motion.acc, clipped, traj_smoothed, debug_trajectories)) {
    return prev_output_;
  }

  overwriteStopPoint(clipped, &traj_smoothed);

  // Becomes the same size input_filltered_resample (push [strat, closest) part)
  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled.begin(), traj_resampled.begin() + traj_resampled_closest);

  // For the endpoint of the trajectory
  if (not traj_smoothed.empty()) {
    traj_smoothed.back().longitudinal_velocity_mps = 0.0;
  }

  // Max velocity filter for safety
  trajectory_utils::applyMaximumVelocityLimit(
    traj_resampled_closest, traj_smoothed.size(), param_.max_velocity, &traj_smoothed);
  std::cout << "max_velocity = " << param_.max_velocity << std::endl;

  // Insert behind velocity for output's consistency
  insertBehindVelocity(traj_resampled_closest, type, &traj_smoothed);

  // COMMENT: save solution to prev_output_
  updatePrevValues(traj_smoothed, current_odom);

  return traj_smoothed;
}
// optimizer function
std::optional<TrajectoryPoints> SmootherFrontEnd::apply(
  const double v0, const double a0, const TrajectoryPoints & input)
{
  if (input.size() == 0) {
    return std::nullopt;
  }

  auto output = input;
  if (input.size() == 1) {
    output.front().longitudinal_velocity_mps = v0;
    output.front().acceleration_mps2 = a0;
    return std::make_optional<TrajectoryPoints>(output);
  }

  const double a_max = param_.base_param.max_accel;
  const double a_min = param_.base_param.min_decel;
  const double a_stop_accel = 0.0;
  const double a_stop_decel = param_.base_param.stop_decel;
  const double j_max = param_.base_param.max_jerk;
  const double j_min = param_.base_param.min_jerk;
  const double over_j_weight = param_.smoother_param.over_j_weight;
  const double over_v_weight = param_.smoother_param.over_v_weight;
  const double over_a_weight = param_.smoother_param.over_a_weight;

  // jerk filter
  const auto forward_filtered =
    forwardJerkFilter(v0, std::max(a0, a_min), a_max, a_stop_accel, j_max, input);
  const auto backward_filtered = backwardJerkFilter(
    input.back().longitudinal_velocity_mps, a_stop_decel, a_min, a_stop_decel, j_min, input);
  const auto filtered =
    mergeFilteredTrajectory(v0, a0, a_min, j_min, forward_filtered, backward_filtered);

  // Resample TrajectoryPoints for Optimization
  // TODO(planning/control team) deal with overlapped lanes with the same direction
  const auto initial_traj_pose = filtered.front().pose;

  auto opt_resampled_trajectory = resampling::resampleTrajectory(
    filtered, v0, initial_traj_pose, std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max(), param_.base_param.resample_param);

  // Ensure terminal velocity is zero
  opt_resampled_trajectory.back().longitudinal_velocity_mps = 0.0;

  // If Resampled Size is too small, we don't do optimization
  if (opt_resampled_trajectory.size() == 1) {
    // No need to do optimization
    output.front().longitudinal_velocity_mps = v0;
    output.front().acceleration_mps2 = a0;
    return std::make_optional<TrajectoryPoints>(output);
  }

  // to avoid getting 0 as a stop point, search zero velocity index from 1.
  // the size of the resampled trajectory must not be less than 2.
  const auto zero_vel_id = motion_utils::searchZeroVelocityIndex(
    opt_resampled_trajectory, 1, opt_resampled_trajectory.size());

  if (!zero_vel_id) {
    return std::nullopt;
  }

  // Clip trajectory from 0 to zero_vel_id (the size becomes zero_vel_id_ + 1)
  const size_t N = *zero_vel_id + 1;

  output = opt_resampled_trajectory;

  const std::vector<double> interval_dist_arr =
    trajectory_utils::calcTrajectoryIntervalDistance(opt_resampled_trajectory);

  std::vector<double> v_max_arr(N, 0.0);
  for (size_t i = 0; i < N; ++i) {
    v_max_arr.at(i) = opt_resampled_trajectory.at(i).longitudinal_velocity_mps;
  }

  /*
   * x = [
   *      b[0], b[1], ..., b[N],               : 0~N
   *      a[0], a[1], .... a[N],               : N~2N
   *      delta[0], ..., delta[N],             : 2N~3N
   *      sigma[0], sigma[1], ...., sigma[N],  : 3N~4N
   *      gamma[0], gamma[1], ..., gamma[N]    : 4N~5N
   *     ]
   *
   * b[i]  : velocity^2
   * delta : 0 < b[i]-delta[i] < max_vel[i]*max_vel[i]
   * sigma : a_min < a[i] - sigma[i] < a_max
   * gamma : jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
   */
  const uint32_t IDX_B0 = 0;
  const uint32_t IDX_A0 = N;
  const uint32_t IDX_DELTA0 = 2 * N;
  const uint32_t IDX_SIGMA0 = 3 * N;
  const uint32_t IDX_GAMMA0 = 4 * N;

  const uint32_t l_variables = 5 * N;
  const uint32_t l_constraints = 4 * N + 1;

  // the matrix size depends on constraint numbers.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(l_constraints, l_variables);

  std::vector<double> lower_bound(l_constraints, 0.0);
  std::vector<double> upper_bound(l_constraints, 0.0);

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(l_variables, l_variables);
  std::vector<double> q(l_variables, 0.0);

  /**************************************************************/
  /**************************************************************/
  /**************** design objective function *******************/
  /**************************************************************/
  /**************************************************************/

  // jerk: d(ai)/ds * v_ref -> minimize weight * ((a1 - a0) / ds * v_ref)^2 * ds
  constexpr double ZERO_VEL_THR_FOR_DT_CALC = 0.3;
  const double smooth_weight = param_.smoother_param.jerk_weight;
  for (size_t i = 0; i < N - 1; ++i) {
    const double ref_vel = v_max_arr.at(i);
    const double interval_dist = std::max(interval_dist_arr.at(i), 0.0001);
    const double w_x_ds_inv = (1.0 / interval_dist) * ref_vel;
    P(IDX_A0 + i, IDX_A0 + i) += smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i, IDX_A0 + i + 1) -= smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i + 1, IDX_A0 + i) -= smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i + 1, IDX_A0 + i + 1) += smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
  }

  for (size_t i = 0; i < N; ++i) {
    const double v_max = std::max(v_max_arr.at(i), 0.1);
    q.at(IDX_B0 + i) =
      -1.0 / (v_max * v_max);  // |v_max_i^2 - b_i|/v_max^2 -> minimize (-bi) * ds / v_max^2
    if (i < N - 1) {
      q.at(IDX_B0 + i) *= std::max(interval_dist_arr.at(i), 0.0001);
    }
    P(IDX_DELTA0 + i, IDX_DELTA0 + i) += over_v_weight;  // over velocity cost
    P(IDX_SIGMA0 + i, IDX_SIGMA0 + i) += over_a_weight;  // over acceleration cost
    P(IDX_GAMMA0 + i, IDX_GAMMA0 + i) += over_j_weight;  // over jerk cost
  }

  /**************************************************************/
  /**************************************************************/
  /**************** design constraint matrix ********************/
  /**************************************************************/
  /**************************************************************/

  /*
  NOTE: The delta allows b to be negative. This is actually invalid because the definition is b=v^2.
  But mathematically, the strict b>0 constraint may make the problem infeasible, such as the case of
  v=0 & a<0. To avoid the infeasibility, we allow b<0. The negative b is dealt as b=0 when it is
  converted to v with sqrt. If the weight of delta^2 is large (the value of delta is very small),
  b is almost 0, and is not a big problem.
  */

  size_t constr_idx = 0;

  // Soft Constraint Velocity Limit: 0 < b - delta < v_max^2
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_B0 + i) = 1.0;       // b_i
    A(constr_idx, IDX_DELTA0 + i) = -1.0;  // -delta_i
    upper_bound[constr_idx] = v_max_arr.at(i) * v_max_arr.at(i);
    lower_bound[constr_idx] = 0.0;
  }

  // Soft Constraint Acceleration Limit: a_min < a - sigma < a_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_A0 + i) = 1.0;       // a_i
    A(constr_idx, IDX_SIGMA0 + i) = -1.0;  // -sigma_i

    constexpr double stop_vel = 1e-3;
    if (v_max_arr.at(i) < stop_vel) {
      // Stop Point
      upper_bound[constr_idx] = a_stop_decel;
      lower_bound[constr_idx] = a_stop_decel;
    } else {
      upper_bound[constr_idx] = a_max;
      lower_bound[constr_idx] = a_min;
    }
  }

  // Soft Constraint Jerk Limit: jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
  // -> jerk_min * ds < (a[i+1] - a[i]) * ref_vel[i] - gamma[i] * ds < jerk_max * ds
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    const double ref_vel = std::max(v_max_arr.at(i), ZERO_VEL_THR_FOR_DT_CALC);
    const double ds = interval_dist_arr.at(i);
    A(constr_idx, IDX_A0 + i) = -ref_vel;     // -a[i] * ref_vel
    A(constr_idx, IDX_A0 + i + 1) = ref_vel;  //  a[i+1] * ref_vel
    A(constr_idx, IDX_GAMMA0 + i) = -ds;      // -gamma[i] * ds
    upper_bound[constr_idx] = j_max * ds;     //  jerk_max * ds
    lower_bound[constr_idx] = j_min * ds;     //  jerk_min * ds
  }

  // b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    A(constr_idx, IDX_B0 + i) = -1.0;                            // b(i)
    A(constr_idx, IDX_B0 + i + 1) = 1.0;                         // b(i+1)
    A(constr_idx, IDX_A0 + i) = -2.0 * interval_dist_arr.at(i);  // a(i) * ds
    upper_bound[constr_idx] = 0.0;
    lower_bound[constr_idx] = 0.0;
  }

  // initial condition
  {
    A(constr_idx, IDX_B0) = 1.0;  // b0
    upper_bound[constr_idx] = v0 * v0;
    lower_bound[constr_idx] = v0 * v0;
    ++constr_idx;

    A(constr_idx, IDX_A0) = 1.0;  // a0
    upper_bound[constr_idx] = a0;
    lower_bound[constr_idx] = a0;
    ++constr_idx;
  }

  // execute optimization
  const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);
  const std::vector<double> optval = std::get<0>(result);

  /*
  const auto tf1 = std::chrono::system_clock::now();
  const double dt_ms1 =
    std::chrono::duration_cast<std::chrono::nanoseconds>(tf1 - ts).count() * 1.0e-6;
  RCLCPP_DEBUG(logger_, "optimization time = %f [ms]", dt_ms1);
  */

  // get velocity & acceleration
  for (size_t i = 0; i < N; ++i) {
    double b = optval.at(IDX_B0 + i);
    output.at(i).longitudinal_velocity_mps = std::sqrt(std::max(b, 0.0));
    output.at(i).acceleration_mps2 = optval.at(IDX_A0 + i);
  }
  for (size_t i = N; i < output.size(); ++i) {
    output.at(i).longitudinal_velocity_mps = 0.0;
    output.at(i).acceleration_mps2 = a_stop_decel;
  }

  qp_solver_.logUnsolvedStatus("[motion_velocity_smoother]");

  /*
  const int status_polish = std::get<2>(result);
  if (status_polish != 1) {
    const auto msg = status_polish == 0    ? "unperformed"
                     : status_polish == -1 ? "unsuccessful"
                                           : "unknown";
    RCLCPP_WARN(logger_, "osqp polish process failed : %s. The result may be inaccurate", msg);
  }

  if (VERBOSE_TRAJECTORY_VELOCITY) {
    const auto s_output = trajectory_utils::calcArclengthArray(output);

    std::cerr << "\n\n" << std::endl;
    for (size_t i = 0; i < N; ++i) {
      const auto v_opt = output.at(i).longitudinal_velocity_mps;
      const auto a_opt = output.at(i).acceleration_mps2;
      const auto ds = i < interval_dist_arr.size() ? interval_dist_arr.at(i) : 0.0;
      const auto v_rs = i < opt_resampled_trajectory.size()
                          ? opt_resampled_trajectory.at(i).longitudinal_velocity_mps
                          : 0.0;
      RCLCPP_INFO(
        logger_, "i =  %4lu | s: %5f | ds: %5f | rs: %9f | op_v: %10f | op_a: %10f |", i,
        s_output.at(i), ds, v_rs, v_opt, a_opt);
    }
  }
  */

  return std::make_optional<TrajectoryPoints>(output);
}

// pre-optimization function
TrajectoryPoints SmootherFrontEnd::forwardJerkFilter(
  const double v0, const double a0, const double a_max, const double a_start, const double j_max,
  const TrajectoryPoints & input) const
{
  auto applyLimits = [&input, &a_start](double & v, double & a, size_t i) {
    double v_lim = input.at(i).longitudinal_velocity_mps;
    static constexpr double ep = 1.0e-5;
    if (v > v_lim + ep) {
      v = v_lim;
      a = 0.0;

      if (v_lim < 1e-3 && i < input.size() - 1) {
        double next_v_lim = input.at(i + 1).longitudinal_velocity_mps;
        if (next_v_lim >= 1e-3) {
          a = a_start;  // start from stop velocity
        }
      }
    }

    if (v < 0.0) {
      v = a = 0.0;
    }
  };

  auto output = input;

  double current_vel = v0;
  double current_acc = a0;
  applyLimits(current_vel, current_acc, 0);

  output.front().longitudinal_velocity_mps = current_vel;
  output.front().acceleration_mps2 = current_acc;
  for (size_t i = 1; i < input.size(); ++i) {
    const double ds = tier4_autoware_utils::calcDistance2d(input.at(i), input.at(i - 1));
    const double max_dt = std::pow(6.0 * ds / j_max, 1.0 / 3.0);  // assuming v0 = a0 = 0.
    const double dt = std::min(ds / std::max(current_vel, 1.0e-6), max_dt);

    if (current_acc + j_max * dt >= a_max) {
      const double tmp_jerk = std::min((a_max - current_acc) / dt, j_max);
      current_vel = current_vel + current_acc * dt + 0.5 * tmp_jerk * dt * dt;
      current_acc = a_max;
    } else {
      current_vel = current_vel + current_acc * dt + 0.5 * j_max * dt * dt;
      current_acc = current_acc + j_max * dt;
    }
    applyLimits(current_vel, current_acc, i);
    output.at(i).longitudinal_velocity_mps = current_vel;
    output.at(i).acceleration_mps2 = current_acc;
  }
  return output;
}

TrajectoryPoints SmootherFrontEnd::backwardJerkFilter(
  const double v0, const double a0, const double a_min, const double a_stop, const double j_min,
  const TrajectoryPoints & input) const
{
  auto input_rev = input;
  std::reverse(input_rev.begin(), input_rev.end());
  auto filtered = forwardJerkFilter(
    v0, std::fabs(a0), std::fabs(a_min), std::fabs(a_stop), std::fabs(j_min), input_rev);
  std::reverse(filtered.begin(), filtered.end());
  for (size_t i = 0; i < filtered.size(); ++i) {
    filtered.at(i).acceleration_mps2 *= -1.0;  // Deceleration
  }
  return filtered;
}

TrajectoryPoints SmootherFrontEnd::mergeFilteredTrajectory(
  const double v0, const double a0, const double a_min, const double j_min,
  const TrajectoryPoints & forward_filtered, const TrajectoryPoints & backward_filtered) const
{
  TrajectoryPoints merged;
  merged = forward_filtered;

  auto getVx = [](const TrajectoryPoints & trajectory, int i) {
    return trajectory.at(i).longitudinal_velocity_mps;
  };

  size_t i = 0;

  if (getVx(backward_filtered, 0) < v0) {
    double current_vel = v0;
    double current_acc = a0;
    while (getVx(backward_filtered, i) < current_vel && i < merged.size() - 1) {
      merged.at(i).longitudinal_velocity_mps = current_vel;
      merged.at(i).acceleration_mps2 = current_acc;

      const double ds =
        tier4_autoware_utils::calcDistance2d(forward_filtered.at(i + 1), forward_filtered.at(i));
      const double max_dt =
        std::pow(6.0 * ds / std::fabs(j_min), 1.0 / 3.0);  // assuming v0 = a0 = 0.
      const double dt = std::min(ds / std::max(current_vel, 1.0e-6), max_dt);

      if (current_acc + j_min * dt < a_min) {
        const double tmp_jerk = std::max((a_min - current_acc) / dt, j_min);
        current_vel = current_vel + current_acc * dt + 0.5 * tmp_jerk * dt * dt;
        current_acc = std::max(current_acc + tmp_jerk * dt, a_min);
      } else {
        current_vel = current_vel + current_acc * dt + 0.5 * j_min * dt * dt;
        current_acc = current_acc + j_min * dt;
      }

      if (current_vel > getVx(forward_filtered, i)) {
        current_vel = getVx(forward_filtered, i);
      }
      ++i;
    }
  }

  // take smaller velocity point
  for (; i < merged.size(); ++i) {
    merged.at(i) = (getVx(forward_filtered, i) < getVx(backward_filtered, i))
                     ? forward_filtered.at(i)
                     : backward_filtered.at(i);
  }
  return merged;
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
  std::cout << "stopping_distance = " << param_.stopping_distance << std::endl;
  std::cout << "stopping_velocity = " << param_.stopping_velocity << std::endl;

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
  std::cout << "replan_vel_deviation = " << param_.replan_vel_deviation << std::endl;
  std::cout << "engage_velocity = " << param_.engage_velocity << std::endl;
  std::cout << "engage_exit_ratio = " << param_.engage_exit_ratio << std::endl;
  std::cout << "engage_acceleration = " << param_.engage_acceleration << std::endl;

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
  std::cout << "base_param.decel_distance_before_curve = "
            << param_.base_param.decel_distance_before_curve << std::endl;
  std::cout << "base_param.decel_distance_after_curve = "
            << param_.base_param.decel_distance_after_curve << std::endl;
  std::cout << "base_param.min_decel_for_lateral_acc_lim_filter = "
            << param_.base_param.min_decel_for_lateral_acc_lim_filter << std::endl;
  std::cout << "base_param.min_curve_velocity = " << param_.base_param.min_curve_velocity
            << std::endl;
  std::cout << "base_param.curvature_calculation_distance = "
            << param_.base_param.curvature_calculation_distance << std::endl;
  std::cout << "base_param.sample_ds = " << param_.base_param.sample_ds << std::endl;
  std::cout << "base_param.curvature_threshold = " << param_.base_param.curvature_threshold
            << std::endl;
  std::cout << "base_param.sample_ds = " << param_.base_param.sample_ds << std::endl;
  std::cout << "base_param.wheel_base = " << param_.base_param.wheel_base << std::endl;
  std::cout << "base_param.max_steering_angle_rate = " << param_.base_param.max_steering_angle_rate
            << std::endl;

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
