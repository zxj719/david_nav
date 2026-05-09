#include "g1_nav/scan_priority_geometry.hpp"

#include <cmath>

namespace g1_nav
{

std::optional<GroundIntersection> intersect_ray_with_ground(
  const CameraRay & ray,
  double ground_plane_z_in_base)
{
  const double dz = ray.dir_z;
  if (std::abs(dz) < 1e-9) {
    return std::nullopt;
  }

  const double t = (ground_plane_z_in_base - ray.origin_z) / dz;
  if (t <= 0.0) {
    return std::nullopt;
  }

  GroundIntersection hit;
  hit.hit_x = ray.origin_x + t * ray.dir_x;
  hit.hit_y = ray.origin_y + t * ray.dir_y;

  const double hit_z = ray.origin_z + t * ray.dir_z;
  if (std::abs(hit_z - ground_plane_z_in_base) > 1e-6) {
    return std::nullopt;
  }

  hit.range_m = std::sqrt(
    std::pow(t * ray.dir_x, 2.0) +
    std::pow(t * ray.dir_y, 2.0) +
    std::pow(t * ray.dir_z, 2.0));
  return hit;
}

BeamDecision classify_beam_against_ground(
  const BeamSample & sample,
  double ground_range_m,
  double obstacle_margin_m)
{
  if (!sample.valid || !std::isfinite(sample.range_m) || !std::isfinite(ground_range_m)) {
    return BeamDecision::kUnknown;
  }

  if (sample.range_m < ground_range_m - obstacle_margin_m) {
    return BeamDecision::kObstacle;
  }

  return BeamDecision::kFloorCompatible;
}

std::vector<ObstacleRun> find_obstacle_runs(
  const std::vector<BeamDecision> & decisions,
  std::size_t min_contiguous_beams)
{
  std::vector<ObstacleRun> runs;
  std::size_t start = 0;
  bool in_run = false;

  for (std::size_t i = 0; i < decisions.size(); ++i) {
    if (decisions[i] == BeamDecision::kObstacle) {
      if (!in_run) {
        start = i;
        in_run = true;
      }
      continue;
    }

    if (in_run && (i - start) >= min_contiguous_beams) {
      runs.push_back({start, i - 1});
    }
    in_run = false;
  }

  if (in_run && (decisions.size() - start) >= min_contiguous_beams) {
    runs.push_back({start, decisions.size() - 1});
  }

  return runs;
}

}  // namespace g1_nav
