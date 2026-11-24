/**
 * Direction-dependent obstacle margins, expressed in the obstacle's own box axes.
 *
 * "Longitudinal" runs along the obstacle heading (its front and back faces) and "lateral" along
 * its width (its sides). The longitudinal margins are optional: a negative value means "same as
 * the lateral margin", so isotropic configurations keep their exact previous behavior.
 *
 * Both obstacle zones are rounded boxes, i.e. the obstacle box grown per axis and then offset by
 * a circle radius, because that is what a single scalar signed-distance field can represent:
 *
 *   barrier zone    total margin M_a = collision_a + safe_a      (soft cost starts here)
 *   collision zone  collision margin c_a                         (hard collision / validation)
 *
 * With r = min(M_lon, M_lat) the barrier zone is `box(half_length + M_lon - r,
 * half_width + M_lat - r)` offset by r, so the distance field only needs the obstacle box grown
 * by the (non-negative) extras and the barrier margin is the scalar r. The collision zone is the
 * ego box inflated by m = min(c_lon, c_lat) against the obstacle box grown by the extras
 * `c_a - m`. For equal margins every extra is zero and both reduce to the previous isotropic
 * formulation exactly.
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
  /** Scalar barrier margin r = min(total_lon, total_lat) of the rounded barrier zone [m]. */
  float barrier_radius{0.0F};
  /** Obstacle half-length growth that defines the barrier zone's distance field [m]. */
  float barrier_extra_length{0.0F};
  /** Obstacle half-width growth that defines the barrier zone's distance field [m]. */
  float barrier_extra_width{0.0F};
  /**
   * Width of the soft zone used to normalize the barrier weight: min(safe_lon, safe_lat). The
   * barrier reaches the crash penalty at the narrower soft zone's collision boundary and is
   * steeper (a higher penalty) at the wider one.
   */
  float soft_zone_width{0.0F};
  /** Isotropic ego-box inflation of the collision zone: min(collision_lon, collision_lat) [m]. */
  float collision_ego_margin{0.0F};
  /** Obstacle half-length growth of the collision zone, relative to the true obstacle [m]. */
  float collision_extra_length{0.0F};
  /** Obstacle half-width growth of the collision zone, relative to the true obstacle [m]. */
  float collision_extra_width{0.0F};
  /**
   * Conservative amount [m] by which the collision zone's obstacle box can be larger than the
   * barrier field's box. Subtract it from the field's clearance before the broad-phase test.
   */
  float collision_broad_phase_offset{0.0F};
};

__host__ __device__ inline ObstacleMarginProfile makeObstacleMarginProfile(
  const float collision_lateral, const float collision_longitudinal, const float safe_lateral,
  const float safe_longitudinal)
{
  const float collision_lon = resolveLongitudinalMargin(collision_lateral, collision_longitudinal);
  const float safe_lon = resolveLongitudinalMargin(safe_lateral, safe_longitudinal);
  const float total_lat = collision_lateral + safe_lateral;
  const float total_lon = collision_lon + safe_lon;

  ObstacleMarginProfile profile;
  profile.barrier_radius = total_lon < total_lat ? total_lon : total_lat;
  profile.barrier_extra_length = total_lon - profile.barrier_radius;
  profile.barrier_extra_width = total_lat - profile.barrier_radius;
  profile.soft_zone_width = safe_lon < safe_lateral ? safe_lon : safe_lateral;
  profile.collision_ego_margin = collision_lon < collision_lateral ? collision_lon : collision_lateral;
  profile.collision_extra_length = collision_lon - profile.collision_ego_margin;
  profile.collision_extra_width = collision_lateral - profile.collision_ego_margin;

  const float grow_length = profile.collision_extra_length - profile.barrier_extra_length;
  const float grow_width = profile.collision_extra_width - profile.barrier_extra_width;
  const float positive_length = grow_length > 0.0F ? grow_length : 0.0F;
  const float positive_width = grow_width > 0.0F ? grow_width : 0.0F;
  profile.collision_broad_phase_offset =
    sqrtf(positive_length * positive_length + positive_width * positive_width);
  return profile;
}

}  // namespace detail
}  // namespace cost
}  // namespace mppi

#endif  // MPPI_COST_FUNCTIONS_OBSTACLE_MARGIN_PROFILE_CUH_
