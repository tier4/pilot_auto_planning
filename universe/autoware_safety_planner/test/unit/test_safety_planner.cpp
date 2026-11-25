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

#include "safety_planner.hpp"
#include "safety_planner_node.hpp"
#include "utils/route_tracking.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{
namespace
{

constexpr lanelet::Id kRouteLane1Id = 100;
constexpr lanelet::Id kRouteLane2Id = 101;
constexpr lanelet::Id kExtensionLaneId = 102;
constexpr lanelet::Id kShoulderLaneId = 103;
constexpr lanelet::Id kRoadBorderId = 200;

Pose make_pose(const double x, const double y, const double yaw = 0.0, const double z = 0.0)
{
  Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  return pose;
}

//! Synthetic straight road along +x:
//! - lane 100: x in [0, 50], y in [-2, 2]
//! - lane 101: x in [50, 100], y in [-2, 2]
//! - lane 102 (optional off-route extension): x in [100, 150], y in [-2, 2]
//! - shoulder 103: x in [0, 100], y in [-4.5, -2]
//! - road_border 200: x in [0, 150], y = 2.5
LaneletMapBin make_synthetic_map(const bool with_extension_lane = true)
{
  lanelet::Id next_id = 1000;
  auto make_point = [&](const double x, const double y) {
    return lanelet::Point3d(next_id++, x, y, 0.0);
  };
  auto make_linestring = [&](const std::vector<lanelet::Point3d> & pts) {
    return lanelet::LineString3d(next_id++, pts);
  };

  // Shared boundary points along x = 0, 10, ..., 150
  std::vector<lanelet::Point3d> left_pts;
  std::vector<lanelet::Point3d> right_pts;
  std::vector<lanelet::Point3d> shoulder_right_pts;
  for (int i = 0; i <= 15; ++i) {
    const double x = static_cast<double>(i) * 10.0;
    left_pts.push_back(make_point(x, 2.0));
    right_pts.push_back(make_point(x, -2.0));
    if (i <= 10) {
      shoulder_right_pts.push_back(make_point(x, -4.5));
    }
  }

  auto make_lane = [&](
                     const lanelet::Id id, const std::size_t begin_idx, const std::size_t end_idx,
                     const std::string & subtype, const std::vector<lanelet::Point3d> & left_src,
                     const std::vector<lanelet::Point3d> & right_src) {
    std::vector<lanelet::Point3d> l_pts(
      left_src.begin() + static_cast<std::ptrdiff_t>(begin_idx),
      left_src.begin() + static_cast<std::ptrdiff_t>(end_idx + 1));
    std::vector<lanelet::Point3d> r_pts(
      right_src.begin() + static_cast<std::ptrdiff_t>(begin_idx),
      right_src.begin() + static_cast<std::ptrdiff_t>(end_idx + 1));
    lanelet::Lanelet llt(id, make_linestring(l_pts), make_linestring(r_pts));
    llt.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Lanelet;
    llt.attributes()[lanelet::AttributeName::Subtype] = subtype;
    llt.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
    llt.attributes()[lanelet::AttributeName::SpeedLimit] = "30";
    return llt;
  };

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(
    make_lane(kRouteLane1Id, 0, 5, lanelet::AttributeValueString::Road, left_pts, right_pts));
  map->add(
    make_lane(kRouteLane2Id, 5, 10, lanelet::AttributeValueString::Road, left_pts, right_pts));
  if (with_extension_lane) {
    map->add(make_lane(
      kExtensionLaneId, 10, 15, lanelet::AttributeValueString::Road, left_pts, right_pts));
  }
  map->add(make_lane(kShoulderLaneId, 0, 10, "road_shoulder", right_pts, shoulder_right_pts));

  lanelet::LineString3d road_border(
    kRoadBorderId, {make_point(0.0, 2.5), make_point(75.0, 2.5), make_point(150.0, 2.5)});
  road_border.attributes()[lanelet::AttributeName::Type] = "road_border";
  map->add(road_border);

  return experimental::lanelet2_utils::to_autoware_map_msgs(map);
}

LaneletRoute make_route(const Pose & start_pose, const Pose & goal_pose)
{
  LaneletRoute route;
  route.header.frame_id = "map";
  route.uuid = autoware_utils_uuid::generate_uuid();
  route.start_pose = start_pose;
  route.goal_pose = goal_pose;

  for (const auto id : {kRouteLane1Id, kRouteLane2Id}) {
    auto & segment = route.segments.emplace_back();
    segment.preferred_primitive.id = id;
    segment.preferred_primitive.primitive_type = "lane";
    auto & primitive = segment.primitives.emplace_back();
    primitive.id = id;
    primitive.primitive_type = "lane";
  }
  return route;
}

VehicleInfo make_vehicle_info()
{
  VehicleInfo info;
  info.wheel_radius_m = 0.383;
  info.wheel_width_m = 0.235;
  info.wheel_base_m = 2.79;
  info.wheel_tread_m = 1.64;
  info.front_overhang_m = 1.0;
  info.rear_overhang_m = 1.1;
  info.left_overhang_m = 0.128;
  info.right_overhang_m = 0.128;
  info.vehicle_height_m = 2.5;
  info.max_steer_angle_rad = 0.70;
  info.vehicle_length_m = info.front_overhang_m + info.wheel_base_m + info.rear_overhang_m;
  info.vehicle_width_m = info.wheel_tread_m + info.left_overhang_m + info.right_overhang_m;
  info.min_longitudinal_offset_m = -info.rear_overhang_m;
  info.max_longitudinal_offset_m = info.wheel_base_m + info.front_overhang_m;
  info.max_lateral_offset_m = 0.5 * info.vehicle_width_m;
  info.min_lateral_offset_m = -info.max_lateral_offset_m;
  return info;
}

Params make_test_params()
{
  Params params;
  params.ego_nearest_lanelet.dist_threshold_m = 6.0;
  params.ego_nearest_lanelet.yaw_threshold_rad = 1.57;
  params.reference_path.policy = "goal_connection_and_smooth";
  params.reference_path.goal_connection.search_radius_m = 15.0;
  params.reference_path.goal_connection.shift_length_m = 30.0;
  params.reference_path.goal_connection.curvature_margin = 0.9;
  params.reference_path.smoother.clearance_m = 1.0;
  params.reference_path.ego_blend.enable = true;
  params.reference_path.ego_blend.lateral_threshold_m = 1.0;
  params.reference_path.ego_blend.forward_length_m = 15.0;
  params.reference_path.ego_blend.backward_length_m = 15.0;
  params.reference_path.forward_length_m = 200.0;
  params.reference_path.backward_length_m = 15.0;
  params.constraint_generator_plugins = {
    "autoware::safety_planner::VehicleKinematicsConstraintGenerator",
    "autoware::safety_planner::LaneFollowingDrivableAreaConstraintGenerator"};
  params.trajectory_planner_plugin =
    "autoware::safety_planner::experimental::FrenetSamplingBasedPlanner";
  return params;
}

struct PlannerHarness
{
  Params params;
  LaneletMapBin map_bin;
  LaneletRoute route;
  SafetyPlannerInput input;
  std::unique_ptr<SafetyPlanner> planner;

  explicit PlannerHarness(
    const Pose & start_pose = make_pose(10.0, 0.0), const Pose & goal_pose = make_pose(80.0, 0.0),
    const bool with_extension_lane = true)
  : params(make_test_params()),
    map_bin(make_synthetic_map(with_extension_lane)),
    route(make_route(start_pose, goal_pose)),
    planner(std::make_unique<SafetyPlanner>(params, std::make_shared<TimeKeeper>()))
  {
    input.vehicle_info = make_vehicle_info();
    input.odometry.header.frame_id = "map";
    input.odometry.pose.pose = start_pose;
    input.goal_pose = goal_pose;
    input.route_manager = RouteManager::create(map_bin, route, start_pose);
  }

  void move_ego(const Pose & pose)
  {
    input.odometry.pose.pose = pose;
    ASSERT_TRUE(input.route_manager.has_value());
    input.route_manager = track_current_lanelet(
      std::move(*input.route_manager), pose, params.ego_nearest_lanelet.dist_threshold_m,
      params.ego_nearest_lanelet.yaw_threshold_rad);
    if (!input.route_manager) {
      input.route_manager = RouteManager::create(map_bin, route, pose);
    }
    ASSERT_TRUE(input.route_manager.has_value());
  }
};

}  // namespace

TEST(SafetyPlannerValidation, AcceptsNormalPoseOnRouteAndShoulder)
{
  PlannerHarness harness;
  EXPECT_TRUE(harness.planner->plan(harness.input).has_value());

  // Road shoulder start (outside the road lanelet polygon, within threshold, no road_border)
  harness.move_ego(make_pose(10.0, -3.5));
  EXPECT_TRUE(harness.planner->plan(harness.input).has_value());

  // At the goal and within the 1.0 m overrun tolerance
  harness.move_ego(make_pose(80.0, 0.0));
  EXPECT_TRUE(harness.planner->plan(harness.input).has_value());
  harness.move_ego(make_pose(80.5, 0.0));
  EXPECT_TRUE(harness.planner->plan(harness.input).has_value());
}

TEST(SafetyPlannerValidation, RejectsEgoMovedFarOutsideRoute)
{
  PlannerHarness harness;
  ASSERT_TRUE(harness.planner->plan(harness.input).has_value());

  harness.move_ego(make_pose(30.0, 20.0));
  EXPECT_FALSE(harness.planner->plan(harness.input).has_value());

  harness.move_ego(make_pose(30.0, -15.0));
  EXPECT_FALSE(harness.planner->plan(harness.input).has_value());
}

TEST(SafetyPlannerValidation, RejectsEgoFacingOppositeDirection)
{
  PlannerHarness harness;
  ASSERT_TRUE(harness.planner->plan(harness.input).has_value());

  harness.move_ego(make_pose(30.0, 0.0, M_PI));
  EXPECT_FALSE(harness.planner->plan(harness.input).has_value());
}

TEST(SafetyPlannerValidation, RejectsEgoSeparatedByRoadBorder)
{
  PlannerHarness harness;
  ASSERT_TRUE(harness.planner->plan(harness.input).has_value());

  // 1.5 m from left lane bound (within dist_threshold_m = 6.0 m) and aligned with the lane,
  // but across the road_border at y = 2.5 m
  harness.move_ego(make_pose(30.0, 3.5));
  EXPECT_FALSE(harness.planner->plan(harness.input).has_value());
}

TEST(SafetyPlannerValidation, RejectsEgoMovedBeyondGoalWithAndWithoutLateralOffset)
{
  PlannerHarness harness;
  // Reach the goal first
  harness.move_ego(make_pose(80.0, 0.0));
  ASSERT_TRUE(harness.planner->plan(harness.input).has_value());

  // Straight past the goal
  harness.move_ego(make_pose(85.0, 0.0));
  EXPECT_FALSE(harness.planner->plan(harness.input).has_value());

  // Past the goal with lateral offset >= ego_blend.lateral_threshold_m (1.0 m)
  harness.move_ego(make_pose(85.0, 1.5));
  EXPECT_FALSE(harness.planner->plan(harness.input).has_value());
}

TEST(SafetyPlannerValidation, RejectsEgoMovedBeyondRouteEnd)
{
  // With an off-route extension lane beyond x = 100
  {
    PlannerHarness harness(
      make_pose(10.0, 0.0), make_pose(99.0, 0.0), /*with_extension_lane=*/true);
    ASSERT_TRUE(harness.planner->plan(harness.input).has_value());

    harness.move_ego(make_pose(103.0, 0.0));
    EXPECT_FALSE(harness.planner->plan(harness.input).has_value());
  }

  // Without any lane beyond x = 100 (ego projection clamps to path end)
  {
    PlannerHarness harness(
      make_pose(10.0, 0.0), make_pose(99.0, 0.0), /*with_extension_lane=*/false);
    ASSERT_TRUE(harness.planner->plan(harness.input).has_value());

    harness.move_ego(make_pose(103.0, 0.0));
    EXPECT_FALSE(harness.planner->plan(harness.input).has_value());
  }
}

TEST(SafetyPlannerValidation, RecoversWhenEgoReturnsToRoute)
{
  PlannerHarness harness;
  ASSERT_TRUE(harness.planner->plan(harness.input).has_value());

  harness.move_ego(make_pose(30.0, 20.0));
  EXPECT_FALSE(harness.planner->plan(harness.input).has_value());

  harness.move_ego(make_pose(30.0, 0.0));
  EXPECT_TRUE(harness.planner->plan(harness.input).has_value());
}

class SafetyPlannerNodeOutputTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(SafetyPlannerNodeOutputTest, PublishesEgoOnlyTrajectoryWhenEgoMovesOutsideRoute)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"wheel_radius", 0.383},
    {"wheel_width", 0.235},
    {"wheel_base", 2.79},
    {"wheel_tread", 1.64},
    {"front_overhang", 1.0},
    {"rear_overhang", 1.1},
    {"left_overhang", 0.128},
    {"right_overhang", 0.128},
    {"vehicle_height", 2.5},
    {"max_steer_angle", 0.70},
    {"ego_nearest_lanelet.dist_threshold_m", 6.0},
    {"ego_nearest_lanelet.yaw_threshold_rad", 1.57},
    {"reference_path.policy", std::string("goal_connection_and_smooth")},
    {"reference_path.ego_blend.enable", true},
    {"reference_path.ego_blend.lateral_threshold_m", 1.0},
    {"constraint_generator_plugins",
     std::vector<std::string>{
       "autoware::safety_planner::VehicleKinematicsConstraintGenerator",
       "autoware::safety_planner::LaneFollowingDrivableAreaConstraintGenerator"}},
    {"trajectory_planner_plugin",
     std::string("autoware::safety_planner::experimental::FrenetSamplingBasedPlanner")},
  });

  auto planner_node = std::make_shared<SafetyPlannerNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("safety_planner_test_io");

  const auto transient_qos = rclcpp::QoS{1}.transient_local();
  auto pub_map = io_node->create_publisher<LaneletMapBin>(
    "/safety_planner_node/input/vector_map", transient_qos);
  auto pub_route =
    io_node->create_publisher<LaneletRoute>("/safety_planner_node/input/route", transient_qos);
  auto pub_odom = io_node->create_publisher<Odometry>("/safety_planner_node/input/odometry", 10);
  auto pub_accel = io_node->create_publisher<AccelWithCovarianceStamped>(
    "/safety_planner_node/input/acceleration", 10);
  auto pub_steer =
    io_node->create_publisher<SteeringReport>("/safety_planner_node/input/steering", 10);

  std::optional<CandidateTrajectories> latest_output;
  auto sub_candidates = io_node->create_subscription<CandidateTrajectories>(
    "/safety_planner_node/output/candidate_trajectories", 10,
    [&](const CandidateTrajectories::ConstSharedPtr msg) { latest_output = *msg; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner_node);
  executor.add_node(io_node);

  const auto start_pose = make_pose(10.0, 0.0);
  const auto goal_pose = make_pose(80.0, 0.0);
  pub_map->publish(make_synthetic_map());
  pub_route->publish(make_route(start_pose, goal_pose));

  const auto publish_ego_state = [&](const Pose & pose) {
    const auto now = io_node->now();
    Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = now;
    odom.pose.pose = pose;
    pub_odom->publish(odom);

    AccelWithCovarianceStamped accel;
    accel.header.frame_id = "map";
    accel.header.stamp = now;
    pub_accel->publish(accel);

    SteeringReport steer;
    steer.stamp = now;
    pub_steer->publish(steer);
  };

  const auto wait_for_output = [&](const auto & predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (latest_output && predicate(*latest_output)) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  };

  // 1. On route: planner outputs multi-point trajectories
  publish_ego_state(start_pose);
  ASSERT_TRUE(wait_for_output([](const CandidateTrajectories & msg) {
    return msg.candidate_trajectories.size() == 2 &&
           msg.candidate_trajectories[0].points.size() > 1 &&
           msg.candidate_trajectories[1].points.size() > 1;
  }));

  const auto is_stopped_at = [](const CandidateTrajectories & msg, const Pose & pose) {
    if (msg.candidate_trajectories.size() != 2) {
      return false;
    }
    for (const auto & traj : msg.candidate_trajectories) {
      if (traj.points.size() != 2) {
        return false;
      }
      for (const auto & point : traj.points) {
        if (
          std::abs(point.pose.position.x - pose.position.x) > 1e-6 ||
          std::abs(point.pose.position.y - pose.position.y) > 1e-6) {
          return false;
        }
      }
    }
    return true;
  };

  // 2. Move ego off route: both candidates are the ego pose twice, so the length is zero
  const auto off_route_pose = make_pose(30.0, 20.0);
  latest_output.reset();
  publish_ego_state(off_route_pose);
  ASSERT_TRUE(wait_for_output(
    [&](const CandidateTrajectories & msg) { return is_stopped_at(msg, off_route_pose); }));

  // 3. Move ego past the goal (with lateral offset that would trigger ego_blend)
  const auto past_goal_pose = make_pose(85.0, 1.5);
  latest_output.reset();
  publish_ego_state(past_goal_pose);
  ASSERT_TRUE(wait_for_output(
    [&](const CandidateTrajectories & msg) { return is_stopped_at(msg, past_goal_pose); }));
}

}  // namespace autoware::safety_planner
