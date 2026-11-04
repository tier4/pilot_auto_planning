// Copyright 2026 TIER IV, Inc.
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

#ifndef VELOCITY_SMOOTHER_HPP_
#define VELOCITY_SMOOTHER_HPP_

#include "type_alias.hpp"

#include <autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

struct VelocitySmootherParams
{
  double nearest_dist_threshold_m;
  double nearest_yaw_threshold_deg;
  double target_pull_out_speed_mps;
  double target_pull_out_acc_mps2;
  double max_speed_mps;
  double max_lateral_accel_mps2;
  double stop_dist_to_prohibit_engage;
  bool set_engage_speed;
  bool limit_speed;
  bool limit_lateral_acceleration;
  bool smooth_velocities;
};

class VelocitySmoother
{
public:
  VelocitySmoother(
    const VelocitySmootherParams & params, const rclcpp::Logger & logger,
    rclcpp::Clock::SharedPtr clock, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    std::shared_ptr<autoware::velocity_smoother::JerkFilteredSmoother> jerk_filtered_smoother);

  //! @param update_prev_output whether to save the result as the reference for the next cycle's
  //! initial motion. Only one trajectory stream (the always-published go trajectory) may update
  //! it; secondary streams (the stop trajectory) must pass false, otherwise their decelerating
  //! profile leaks into the other stream's initial speed on the next cycle.
  //! @param external_velocity_limit_mps external speed limit; the effective max speed is the
  //! smaller of this and params.max_speed_mps.
  void optimize(
    TrajectoryPoints & traj_points, const nav_msgs::msg::Odometry & current_odometry,
    double current_acceleration, bool update_prev_output = true,
    std::optional<double> external_velocity_limit_mps = std::nullopt);

private:
  static void clamp_velocities(
    TrajectoryPoints & traj_points, float min_velocity, float min_acceleration);

  //! @param start_idx points before this index are left untouched
  static void set_max_velocity(
    TrajectoryPoints & traj_points, float max_velocity, size_t start_idx = 0);

  //! Distance ahead of ego at which a lowered max speed starts to apply, so that the ego
  //! decelerates within the smoother's acceleration and jerk limits instead of at once.
  //! nullopt when there is no smoother to take those limits from.
  std::optional<double> calc_braking_distance(
    double max_speed_mps, double initial_speed, double initial_acc) const;

  static void limit_lateral_acceleration(
    TrajectoryPoints & traj_points, double max_lateral_accel_mps2,
    const nav_msgs::msg::Odometry & current_odometry);

  void filter_velocity(
    TrajectoryPoints & traj_points, double initial_speed, double initial_acc,
    const nav_msgs::msg::Odometry & current_odometry);

  std::optional<TrajectoryPoint> calc_projected_trajectory_point_from_ego(
    const TrajectoryPoints & trajectory, const geometry_msgs::msg::Pose & ego_pose) const;

  VelocitySmootherParams params_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<autoware::velocity_smoother::JerkFilteredSmoother> jerk_filtered_smoother_;
  TrajectoryPoints prev_output_;
  //! the max speed the ego has already been seen to meet, held until the max speed changes
  std::optional<double> satisfied_max_speed_mps_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // VELOCITY_SMOOTHER_HPP_
