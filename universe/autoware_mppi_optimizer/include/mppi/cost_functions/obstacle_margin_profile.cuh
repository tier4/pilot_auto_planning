/**
 * Direction-dependent obstacle margins, expressed in the obstacle's own box axes.
 *
 * "Longitudinal" runs along the obstacle heading (its front and back faces) and "lateral" along
 * its width (its sides). The longitudinal margins are optional: a negative value means "same as
 * the lateral margin".
 *
 * Soft cost is an anisotropic shell around the collision-inflated box:
 *
 *   collision box   half extents (hl + collision_lon, hw + collision_lat)
 *   soft outer box  half extents (hl + collision_lon + safe_lon, hw + collision_lat + safe_lat)
 *
 * Soft violation is 1 at/inside the collision box, 0 at/outside the soft outer box, and falls off
 * separately along each axis between those faces (normalized by that axis's safe margin). Hard
 * collision is a separate exact OBB test: the ego box inflated by min(collision_lon, collision_lat)
 * against the obstacle grown by the remaining collision extras.
 */
#pragma once

#ifndef MPPI_COST_FUNCTIONS_OBSTACLE_MARGIN_PROFILE_CUH_
#define MPPI_COST_FUNCTIONS_OBSTACLE_MARGIN_PROFILE_CUH_

#include <cmath>

namespace mppi
{
namespace cost
{
namespace detail
{

/** Longitudinal margins are optional: a negative value inherits the lateral margin. */
__host__ __device__ inline float resolveLongitudinalMargin(
  const float lateral, const float longitudinal)
{
  return longitudinal < 0.0F ? lateral : longitudinal;
}

struct ObstacleMarginProfile
{
  float collision_lon{0.0F};
  float collision_lat{0.0F};
  float safe_lon{0.0F};
  float safe_lat{0.0F};
  /** Isotropic ego-box inflation of the collision zone: min(collision_lon, collision_lat) [m]. */
  float collision_ego_margin{0.0F};
  /** Obstacle half-length growth of the collision zone, relative to the true obstacle [m]. */
  float collision_extra_length{0.0F};
  /** Obstacle half-width growth of the collision zone, relative to the true obstacle [m]. */
  float collision_extra_width{0.0F};
};

__host__ __device__ inline ObstacleMarginProfile makeObstacleMarginProfile(
  const float collision_lateral, const float collision_longitudinal, const float safe_lateral,
  const float safe_longitudinal)
{
  ObstacleMarginProfile profile;
  profile.collision_lat = collision_lateral;
  profile.collision_lon = resolveLongitudinalMargin(collision_lateral, collision_longitudinal);
  profile.safe_lat = safe_lateral;
  profile.safe_lon = resolveLongitudinalMargin(safe_lateral, safe_longitudinal);
  profile.collision_ego_margin =
    profile.collision_lon < profile.collision_lat ? profile.collision_lon : profile.collision_lat;
  profile.collision_extra_length = profile.collision_lon - profile.collision_ego_margin;
  profile.collision_extra_width = profile.collision_lat - profile.collision_ego_margin;
  return profile;
}

/**
 * Soft violation in [0, 1] for a point against one oriented obstacle box.
 *
 * 1 at/inside the collision-inflated box, 0 at/outside the soft-outer box. Between them the value
 * falls with the max axis-normalized overshoot past the collision faces, so each face uses its
 * own safe margin as the ramp length.
 *
 * To model an ego footprint circle of radius r, pass collision_lon/lat already increased by r
 * (Minkowski sum with a disk, rectangularly expanded); leave the safe widths unchanged.
 */
__host__ __device__ inline float anisotropicSoftViolation(
  const float px, const float py, const float ox, const float oy, const float cos_yaw,
  const float sin_yaw, const float half_length, const float half_width, const float collision_lon,
  const float collision_lat, const float safe_lon, const float safe_lat)
{
  const float dx = px - ox;
  const float dy = py - oy;
  const float local_x = fabsf(cos_yaw * dx + sin_yaw * dy);
  const float local_y = fabsf(-sin_yaw * dx + cos_yaw * dy);

  const float collision_x = half_length + fmaxf(collision_lon, 0.0F);
  const float collision_y = half_width + fmaxf(collision_lat, 0.0F);
  const float soft_lon = fmaxf(safe_lon, 0.0F);
  const float soft_lat = fmaxf(safe_lat, 0.0F);
  const float soft_x = collision_x + soft_lon;
  const float soft_y = collision_y + soft_lat;

  if (local_x >= soft_x || local_y >= soft_y) {
    return 0.0F;
  }
  if (local_x <= collision_x && local_y <= collision_y) {
    return 1.0F;
  }

  const float overshoot_x =
    soft_lon > 1.0E-6F ? fmaxf(0.0F, local_x - collision_x) / soft_lon : 1.0F;
  const float overshoot_y =
    soft_lat > 1.0E-6F ? fmaxf(0.0F, local_y - collision_y) / soft_lat : 1.0F;
  return 1.0F - fminf(1.0F, fmaxf(overshoot_x, overshoot_y));
}

}  // namespace detail
}  // namespace cost
}  // namespace mppi

#endif  // MPPI_COST_FUNCTIONS_OBSTACLE_MARGIN_PROFILE_CUH_
