#pragma once

#include "./param.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>

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
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> calcTrajectoryVelocity(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
    const nav_msgs::msg::Odometry &) const;
  void updatePrevValues(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
    const nav_msgs::msg::Odometry &);
  size_t findNearestIndexFromEgo(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
    const nav_msgs::msg::Odometry &) const;
  void applyStopApproachingVelocity(
    std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> *) const;
  std::pair<Motion, InitializeType> calcInitialMotion(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
    const nav_msgs::msg::Odometry &, const size_t input_closest) const;

private:
  autoware_auto_planning_msgs::msg::TrajectoryPoint prev_closest_point_;
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> prev_output_;
  std::optional<autoware_auto_planning_msgs::msg::TrajectoryPoint>
    current_closest_point_from_prev_output_ = std::nullopt;
};
