#pragma once

#include "./param.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>

using TrajectoryPoints = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;

class SmootherFrontEnd
{
public:
  const Param param_;
  struct Motion
  {
    double vel = 0.0;
    double acc = 0.0;
  };
  enum class InitializeType {
    INIT = 0,
    LARGE_DEVIATION_REPLAN = 1,
    ENGAGING = 2,
    NORMAL = 3,
  };

public:
  void onCurrentTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory &, const nav_msgs::msg::Odometry &);
  TrajectoryPoints calcTrajectoryVelocity(
    const TrajectoryPoints &, const nav_msgs::msg::Odometry &) const;
  void updatePrevValues(const TrajectoryPoints &, const nav_msgs::msg::Odometry &);
  size_t findNearestIndexFromEgo(const TrajectoryPoints &, const nav_msgs::msg::Odometry &) const;
  void applyStopApproachingVelocity(TrajectoryPoints *) const;
  std::pair<Motion, InitializeType> calcInitialMotion(
    const TrajectoryPoints &, const nav_msgs::msg::Odometry &, const size_t input_closest) const;
  std::optional<TrajectoryPoints> applyLateralAccelerationFilter(
    const TrajectoryPoints & input, const double v0, const double a0,
    const bool enable_smooth_limit) const;
  std::optional<TrajectoryPoints> applySteeringRateLimit(const TrajectoryPoints & input) const;
  std::optional<TrajectoryPoints> apply(
    const double v, const double a, const TrajectoryPoints & input) const;
  void overwriteStopPoint(const TrajectoryPoints & input, TrajectoryPoints * output) const;
  void insertBehindVelocity(
    const size_t output_closest, const InitializeType type, TrajectoryPoints * output) const;

private:
  autoware_auto_planning_msgs::msg::TrajectoryPoint prev_closest_point_;
  TrajectoryPoints prev_output_;
  std::optional<autoware_auto_planning_msgs::msg::TrajectoryPoint>
    current_closest_point_from_prev_output_ = std::nullopt;
};
