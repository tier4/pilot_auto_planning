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

#include <gtest/gtest.h>

// This translation unit tests the shared margin math with an ordinary C++17 compiler, without
// requiring CUDA headers, libraries or a device.
#define __host__
#define __device__
#include <mppi/cost_functions/obstacle_margin_profile.cuh>
#undef __host__
#undef __device__

namespace
{

using mppi::cost::detail::makeObstacleMarginProfile;
using mppi::cost::detail::resolveLongitudinalMargin;

TEST(ObstacleMarginProfile, NegativeLongitudinalMarginInheritsLateral)
{
  EXPECT_FLOAT_EQ(resolveLongitudinalMargin(0.3F, -1.0F), 0.3F);
  EXPECT_FLOAT_EQ(resolveLongitudinalMargin(0.3F, 0.0F), 0.0F);
  EXPECT_FLOAT_EQ(resolveLongitudinalMargin(0.3F, 0.8F), 0.8F);
}

TEST(ObstacleMarginProfile, IsotropicMarginsHaveNoGeometricExtras)
{
  for (const bool inherit : {true, false}) {
    const auto profile = makeObstacleMarginProfile(
      0.3F, inherit ? -1.0F : 0.3F, 0.5F, inherit ? -1.0F : 0.5F);
    // Exactly the previous formulation: the barrier starts at collision + safe, the collision test
    // inflates only the ego box, and the obstacle boxes are untouched.
    EXPECT_EQ(profile.barrier_radius, 0.3F + 0.5F);
    EXPECT_EQ(profile.barrier_extra_length, 0.0F);
    EXPECT_EQ(profile.barrier_extra_width, 0.0F);
    EXPECT_EQ(profile.soft_zone_width, 0.5F);
    EXPECT_EQ(profile.collision_ego_margin, 0.3F);
    EXPECT_EQ(profile.collision_extra_length, 0.0F);
    EXPECT_EQ(profile.collision_extra_width, 0.0F);
    EXPECT_EQ(profile.collision_broad_phase_offset, 0.0F);
  }
}

TEST(ObstacleMarginProfile, WiderLongitudinalMarginGrowsTheObstacleLength)
{
  // Lateral total 0.5, longitudinal total 1.25.
  const auto profile = makeObstacleMarginProfile(0.2F, 0.5F, 0.3F, 0.75F);
  EXPECT_FLOAT_EQ(profile.barrier_radius, 0.5F);
  EXPECT_FLOAT_EQ(profile.barrier_extra_length, 0.75F);
  EXPECT_FLOAT_EQ(profile.barrier_extra_width, 0.0F);
  EXPECT_FLOAT_EQ(profile.soft_zone_width, 0.3F);
  EXPECT_FLOAT_EQ(profile.collision_ego_margin, 0.2F);
  EXPECT_FLOAT_EQ(profile.collision_extra_length, 0.3F);
  EXPECT_FLOAT_EQ(profile.collision_extra_width, 0.0F);
}

TEST(ObstacleMarginProfile, WiderLateralMarginGrowsTheObstacleWidth)
{
  const auto profile = makeObstacleMarginProfile(0.5F, 0.2F, 0.75F, 0.3F);
  EXPECT_FLOAT_EQ(profile.barrier_radius, 0.5F);
  EXPECT_FLOAT_EQ(profile.barrier_extra_length, 0.0F);
  EXPECT_FLOAT_EQ(profile.barrier_extra_width, 0.75F);
  EXPECT_FLOAT_EQ(profile.collision_ego_margin, 0.2F);
  EXPECT_FLOAT_EQ(profile.collision_extra_length, 0.0F);
  EXPECT_FLOAT_EQ(profile.collision_extra_width, 0.3F);
}

TEST(ObstacleMarginProfile, ZonesNeverShrinkTheObstacle)
{
  for (const float lateral_collision : {0.0F, 0.2F, 0.6F}) {
    for (const float longitudinal_collision : {0.0F, 0.2F, 0.6F}) {
      for (const float lateral_safe : {0.6F, 0.9F}) {
        for (const float longitudinal_safe : {0.6F, 1.5F}) {
          const auto profile = makeObstacleMarginProfile(
            lateral_collision, longitudinal_collision, lateral_safe, longitudinal_safe);
          EXPECT_GE(profile.barrier_radius, 0.0F);
          EXPECT_GE(profile.barrier_extra_length, 0.0F);
          EXPECT_GE(profile.barrier_extra_width, 0.0F);
          EXPECT_GE(profile.collision_ego_margin, 0.0F);
          EXPECT_GE(profile.collision_extra_length, 0.0F);
          EXPECT_GE(profile.collision_extra_width, 0.0F);
          EXPECT_GE(profile.collision_broad_phase_offset, 0.0F);
          // Per axis the barrier zone is the total margin and the collision zone the collision
          // margin, so each axis reproduces its own margin.
          EXPECT_FLOAT_EQ(
            profile.barrier_radius + profile.barrier_extra_length,
            longitudinal_collision + longitudinal_safe);
          EXPECT_FLOAT_EQ(
            profile.barrier_radius + profile.barrier_extra_width,
            lateral_collision + lateral_safe);
          EXPECT_FLOAT_EQ(
            profile.collision_ego_margin + profile.collision_extra_length, longitudinal_collision);
          EXPECT_FLOAT_EQ(
            profile.collision_ego_margin + profile.collision_extra_width, lateral_collision);
        }
      }
    }
  }
}

TEST(ObstacleMarginProfile, BroadPhaseOffsetCoversCollisionZoneLargerThanTheField)
{
  // Collision zone grows by (0.6, 0) while the barrier field grows by (0, 0): the field's box is
  // smaller than the collision box by 0.6 m, which the broad phase has to allow for.
  const auto profile = makeObstacleMarginProfile(0.0F, 0.6F, 1.2F, 0.6F);
  EXPECT_GT(profile.collision_broad_phase_offset, 0.0F);
}

}  // namespace
