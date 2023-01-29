#pragma once

#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <map>
#include <optional>
#include <tuple>
#include <vector>

namespace trajectory_utils
{
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using geometry_msgs::msg::Pose;

TrajectoryPoint calcInterpolatedTrajectoryPoint(
  const TrajectoryPoints & trajectory, const Pose & target_pose, const size_t seg_idx);

TrajectoryPoints extractPathAroundIndex(
  const TrajectoryPoints & trajectory, const size_t index, const double & ahead_length,
  const double & behind_length);

std::vector<double> calcArclengthArray(const TrajectoryPoints & trajectory);

std::vector<double> calcTrajectoryIntervalDistance(const TrajectoryPoints & trajectory);

std::vector<double> calcTrajectoryCurvatureFrom3Points(
  const TrajectoryPoints & trajectory, size_t idx_dist);

void setZeroVelocity(TrajectoryPoints & trajectory);

double getMaxVelocity(const TrajectoryPoints & trajectory);

double getMaxAbsVelocity(const TrajectoryPoints & trajectory);

void applyMaximumVelocityLimit(
  const size_t from, const size_t to, const double max_vel, TrajectoryPoints * trajectory);

std::optional<size_t> searchZeroVelocityIdx(const TrajectoryPoints & trajectory);

bool calcStopDistWithJerkConstraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, std::map<double, double> & jerk_profile,
  double & stop_dist);

bool isValidStopDist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);

std::optional<TrajectoryPoints> applyDecelFilterWithJerkConstraint(
  const TrajectoryPoints & input, const size_t start_index, const double v0, const double a0,
  const double min_acc, const double decel_target_vel,
  const std::map<double, double> & jerk_profile);

std::optional<std::tuple<double, double, double, double>> updateStateWithJerkConstraint(
  const double v0, const double a0, const std::map<double, double> & jerk_profile, const double t);

std::vector<double> calcVelocityProfileWithConstantJerkAndAccelerationLimit(
  const TrajectoryPoints & trajectory, const double v0, const double a0, const double jerk,
  const double acc_max, const double acc_min);

}  // namespace trajectory_utils
