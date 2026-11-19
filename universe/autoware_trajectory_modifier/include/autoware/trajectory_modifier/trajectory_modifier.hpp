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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_context.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_data.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier
{
namespace namespace_polling = autoware::agnocast_wrapper::polling;

/// @brief Runs the complete ordered modifier and optimizer plugin pipeline.
class TrajectoryModifier : public autoware::agnocast_wrapper::Node
{
public:
  /// @brief Construct the modifier node and load its configured plugin pipeline.
  explicit TrajectoryModifier(const rclcpp::NodeOptions & options);

private:
  using CandidateTrajectories = autoware_internal_planning_msgs::msg::CandidateTrajectories;
  using Trajectory = autoware_planning_msgs::msg::Trajectory;
  using Odometry = nav_msgs::msg::Odometry;
  using Acceleration = geometry_msgs::msg::AccelWithCovarianceStamped;
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using TrackedObjects = autoware_perception_msgs::msg::TrackedObjects;
  using SteeringReport = autoware_vehicle_msgs::msg::SteeringReport;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Plugin = plugin::TrajectoryModifierPluginBase;
  using DiagnosticsInterface =
    autoware_utils_diagnostics::BasicDiagnosticsInterface<autoware::agnocast_wrapper::Node>;

  /// @brief Process every candidate through the configured plugin sequence.
  void on_trajectories(AUTOWARE_MESSAGE_CONST_SHARED_PTR(CandidateTrajectories) msg);
  /// @brief Store the latest lanelet map used by map-dependent plugins.
  void on_map(AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_map_msgs::msg::LaneletMapBin) msg);
  /// @brief Take one snapshot of all polling inputs for the current callback.
  tl::expected<TrajectoryModifierData, std::string> make_input_data();
  /// @brief Load every configured plugin in pipeline order.
  void load_plugins();
  /// @brief Create and initialize one plugin instance.
  void load_plugin(const std::string & class_name, std::size_t pipeline_index);
  /// @brief Apply changed parameters and reload the pipeline if its order changed.
  void update_params();
  /// @brief Publish the total callback processing duration.
  void publish_processing_time(double processing_time_ms);
  /// @brief Heartbeat for missing ~/input/trajectories when the planner is silent.
  void on_input_trajectories_watchdog();
  /// @brief Publish input_trajectories diagnostics from the latest receive time.
  void publish_input_trajectories_diagnostic();

  std::unique_ptr<trajectory_modifier_params::ParamListener> param_listener_;
  TrajectoryModifierParams params_;

  pluginlib::ClassLoader<Plugin> plugin_loader_;
  std::vector<std::shared_ptr<Plugin>> plugins_;
  std::shared_ptr<TrajectoryModifierContext> context_;

  AUTOWARE_SUBSCRIPTION_PTR(CandidateTrajectories) trajectories_sub_;
  AUTOWARE_PUBLISHER_PTR(CandidateTrajectories) trajectories_pub_;
  AUTOWARE_PUBLISHER_PTR(Trajectory) trajectory_pub_;

  namespace_polling::PollingSubscriber<Odometry>::SharedPtr sub_current_odometry_;
  namespace_polling::PollingSubscriber<Acceleration>::SharedPtr sub_current_acceleration_;
  namespace_polling::PollingSubscriber<SteeringReport>::SharedPtr sub_current_steering_;
  namespace_polling::PollingSubscriber<PredictedObjects>::SharedPtr sub_objects_;
  namespace_polling::PollingSubscriber<TrackedObjects>::SharedPtr sub_tracked_objects_;
  namespace_polling::PollingSubscriber<PointCloud2>::SharedPtr sub_pointcloud_;
  namespace_polling::PollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr sub_traffic_lights_;
  namespace_polling::PollingSubscriber<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr
    sub_route_;
  AUTOWARE_SUBSCRIPTION_PTR(autoware_map_msgs::msg::LaneletMapBin) sub_map_;

  AUTOWARE_PUBLISHER_PTR(autoware_utils_debug::ProcessingTimeDetail)
  debug_processing_time_detail_pub_;
  std::shared_ptr<autoware_utils_debug::BasicDebugPublisher<autoware::agnocast_wrapper::Node>>
    debug_publisher_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_map_msgs::msg::LaneletMapBin) lanelet_map_bin_ptr_;

  std::unique_ptr<DiagnosticsInterface> diagnostics_input_trajectories_;
  AUTOWARE_TIMER_PTR input_trajectories_watchdog_timer_;
  rclcpp::Time watchdog_origin_time_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> last_input_time_;
  std::size_t last_candidate_count_{0};
  bool ever_received_input_{false};
};

}  // namespace autoware::trajectory_modifier

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_HPP_
