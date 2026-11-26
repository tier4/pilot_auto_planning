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

using mppi::cost::detail::anisotropicSoftViolation;
using mppi::cost::detail::makeObstacleMarginProfile;
using mppi::cost::detail::resolveLongitudinalMargin;

TEST(ObstacleMarginProfile, NegativeLongitudinalMarginInheritsLateral)
{
  EXPECT_FLOAT_EQ(resolveLongitudinalMargin(0.3F, -1.0F), 0.3F);
  EXPECT_FLOAT_EQ(resolveLongitudinalMargin(0.3F, 0.0F), 0.0F);
  EXPECT_FLOAT_EQ(resolveLongitudinalMargin(0.3F, 0.8F), 0.8F);
}

TEST(ObstacleMarginProfile, CollisionExtrasMatchResolvedMargins)
{
  const auto isotropic = makeObstacleMarginProfile(0.3F, -1.0F, 0.5F, -1.0F);
  EXPECT_FLOAT_EQ(isotropic.collision_lon, 0.3F);
  EXPECT_FLOAT_EQ(isotropic.collision_lat, 0.3F);
  EXPECT_FLOAT_EQ(isotropic.safe_lon, 0.5F);
  EXPECT_FLOAT_EQ(isotropic.safe_lat, 0.5F);
  EXPECT_FLOAT_EQ(isotropic.collision_ego_margin, 0.3F);
  EXPECT_FLOAT_EQ(isotropic.collision_extra_length, 0.0F);
  EXPECT_FLOAT_EQ(isotropic.collision_extra_width, 0.0F);

  const auto wider_lon = makeObstacleMarginProfile(0.2F, 0.5F, 0.3F, 0.75F);
  EXPECT_FLOAT_EQ(wider_lon.collision_ego_margin, 0.2F);
  EXPECT_FLOAT_EQ(wider_lon.collision_extra_length, 0.3F);
  EXPECT_FLOAT_EQ(wider_lon.collision_extra_width, 0.0F);
  EXPECT_FLOAT_EQ(wider_lon.safe_lon, 0.75F);
  EXPECT_FLOAT_EQ(wider_lon.safe_lat, 0.3F);
}

TEST(AnisotropicSoftViolation, FullInsideCollisionBox)
{
  // Obstacle half 1x0.5 at origin, yaw 0. Collision 0.2 / 0.2.
  EXPECT_FLOAT_EQ(
    anisotropicSoftViolation(0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.5F, 0.2F, 0.2F, 0.5F, 0.5F),
    1.0F);
}

TEST(AnisotropicSoftViolation, ZeroOutsideSoftOuter)
{
  EXPECT_FLOAT_EQ(
    anisotropicSoftViolation(3.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.5F, 0.2F, 0.2F, 0.5F, 0.5F),
    0.0F);
}

TEST(AnisotropicSoftViolation, LinearRampAlongEachAxis)
{
  // Collision face at x = 1.2, soft face at x = 1.7 (safe_lon = 0.5). Midpoint -> 0.5.
  EXPECT_NEAR(
    anisotropicSoftViolation(1.45F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.5F, 0.2F, 0.2F, 0.5F, 0.3F),
    0.5F, 1.0E-5F);
  // Lateral: collision face at y = 0.7, soft at y = 1.0 (safe_lat = 0.3). Midpoint -> 0.5.
  EXPECT_NEAR(
    anisotropicSoftViolation(0.0F, 0.85F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.5F, 0.2F, 0.2F, 0.5F, 0.3F),
    0.5F, 1.0E-5F);
}

TEST(AnisotropicSoftViolation, LongerLongitudinalSoftRamp)
{
  // Longitudinal safe 1.0, lateral safe 0.2. Same geometric overshoot 0.1 past collision.
  // Along length: 0.1/1.0 -> violation 0.9. Along width: 0.1/0.2 -> violation 0.5.
  const float along_length = anisotropicSoftViolation(
    1.3F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.5F, 0.2F, 0.2F, 1.0F, 0.2F);
  const float along_width = anisotropicSoftViolation(
    0.0F, 0.8F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.5F, 0.2F, 0.2F, 1.0F, 0.2F);
  EXPECT_NEAR(along_length, 0.9F, 1.0E-5F);
  EXPECT_NEAR(along_width, 0.5F, 1.0E-5F);
  EXPECT_GT(along_length, along_width);
}

}  // namespace
