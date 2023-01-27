// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "./param.hpp"
#include "./trajectory_utils.hpp"
#include "motion_utils/trajectory/trajectory.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace resampling
{
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

TrajectoryPoints resampleTrajectory(
  const TrajectoryPoints & input, const double v_current,
  const geometry_msgs::msg::Pose & current_pose, const double nearest_dist_threshold,
  const double nearest_yaw_threshold, const ResampleParam & param, const bool use_zoh_for_v = true);

TrajectoryPoints resampleTrajectory(
  const TrajectoryPoints & input, const geometry_msgs::msg::Pose & current_pose,
  const double nearest_dist_threshold, const double nearest_yaw_threshold,
  const ResampleParam & param, const double nominal_ds, const bool use_zoh_for_v = true);
}  // namespace resampling
