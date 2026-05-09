#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace g1_nav
{

struct CameraRay
{
  double origin_x;
  double origin_y;
  double origin_z;
  double dir_x;
  double dir_y;
  double dir_z;
};

struct GroundIntersection
{
  double range_m;
  double hit_x;
  double hit_y;
};

struct BeamSample
{
  double range_m;
  bool valid;
};

enum class BeamDecision
{
  kUnknown,
  kFloorCompatible,
  kObstacle,
};

struct ObstacleRun
{
  std::size_t start_index;
  std::size_t end_index;
};

std::optional<GroundIntersection> intersect_ray_with_ground(
  const CameraRay & ray,
  double ground_plane_z_in_base);

BeamDecision classify_beam_against_ground(
  const BeamSample & sample,
  double ground_range_m,
  double obstacle_margin_m);

std::vector<ObstacleRun> find_obstacle_runs(
  const std::vector<BeamDecision> & decisions,
  std::size_t min_contiguous_beams);

}  // namespace g1_nav
