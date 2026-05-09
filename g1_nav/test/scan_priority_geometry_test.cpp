#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "g1_nav/scan_priority_geometry.hpp"

namespace
{

using g1_nav::BeamDecision;
using g1_nav::BeamSample;
using g1_nav::CameraRay;

TEST(ScanPriorityGeometryTest, IntersectsForwardDownwardRayWithGroundPlane)
{
  CameraRay ray;
  ray.origin_x = 0.0;
  ray.origin_y = 0.0;
  ray.origin_z = 0.5;
  ray.dir_x = 1.0;
  ray.dir_y = 0.0;
  ray.dir_z = -1.0;

  const auto hit = g1_nav::intersect_ray_with_ground(ray, 0.0);

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->range_m, std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(hit->hit_x, 0.5, 1e-6);
  EXPECT_NEAR(hit->hit_y, 0.0, 1e-6);
}

TEST(ScanPriorityGeometryTest, ReturnsNulloptWhenRayNeverHitsGround)
{
  CameraRay ray;
  ray.origin_x = 0.0;
  ray.origin_y = 0.0;
  ray.origin_z = 0.5;
  ray.dir_x = 1.0;
  ray.dir_y = 0.0;
  ray.dir_z = 0.1;

  EXPECT_FALSE(g1_nav::intersect_ray_with_ground(ray, 0.0).has_value());
}

TEST(ScanPriorityGeometryTest, ClassifiesEarlierBeamAsObstacle)
{
  BeamSample sample;
  sample.range_m = 1.2;
  sample.valid = true;

  const auto decision = g1_nav::classify_beam_against_ground(
    sample,
    /*ground_range_m=*/1.6,
    /*obstacle_margin_m=*/0.15);

  EXPECT_EQ(decision, BeamDecision::kObstacle);
}

TEST(ScanPriorityGeometryTest, ClassifiesNearGroundMatchAsFloorCompatible)
{
  BeamSample sample;
  sample.range_m = 1.52;
  sample.valid = true;

  const auto decision = g1_nav::classify_beam_against_ground(
    sample,
    /*ground_range_m=*/1.6,
    /*obstacle_margin_m=*/0.15);

  EXPECT_EQ(decision, BeamDecision::kFloorCompatible);
}

TEST(ScanPriorityGeometryTest, TreatsInvalidBeamAsUnknown)
{
  BeamSample sample;
  sample.range_m = std::numeric_limits<double>::quiet_NaN();
  sample.valid = false;

  const auto decision = g1_nav::classify_beam_against_ground(
    sample,
    /*ground_range_m=*/1.6,
    /*obstacle_margin_m=*/0.15);

  EXPECT_EQ(decision, BeamDecision::kUnknown);
}

TEST(ScanPriorityGeometryTest, SuppressesShortObstacleRunsBelowThreshold)
{
  const std::vector<BeamDecision> decisions = {
    BeamDecision::kFloorCompatible,
    BeamDecision::kObstacle,
    BeamDecision::kObstacle,
    BeamDecision::kFloorCompatible,
    BeamDecision::kObstacle,
    BeamDecision::kFloorCompatible,
  };

  const auto runs = g1_nav::find_obstacle_runs(decisions, /*min_contiguous_beams=*/2);

  ASSERT_EQ(runs.size(), 1u);
  EXPECT_EQ(runs[0].start_index, 1u);
  EXPECT_EQ(runs[0].end_index, 2u);
}

}  // namespace
