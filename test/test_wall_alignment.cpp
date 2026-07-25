#include "weldline_reflectivity_detector/wall_alignment.hpp"

#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

namespace weldline_reflectivity_detector
{
namespace
{
constexpr double kDegreesToRadians = 0.01745329251994329577;
constexpr double kRadiansToDegrees = 57.2957795130823208768;

std::vector<Point2D> make_wall(
  const double angle_degrees, const Point2D center, const int count,
  const double length, const unsigned int seed)
{
  const double angle = angle_degrees * kDegreesToRadians;
  const Point2D direction{std::cos(angle), std::sin(angle)};
  const Point2D normal{-direction.y, direction.x};
  std::mt19937 engine(seed);
  std::normal_distribution<double> noise(0.0, 0.004);
  std::vector<Point2D> points;
  points.reserve(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    const double fraction =
      static_cast<double>(index) / static_cast<double>(count - 1) - 0.5;
    const double along = fraction * length;
    const double across = noise(engine);
    points.push_back(
      {center.x + along * direction.x + across * normal.x,
        center.y + along * direction.y + across * normal.y});
  }
  return points;
}
}  // namespace

TEST(WallAlignment, RecoversPreciseSignedAngleWithOutliers)
{
  auto points = make_wall(2.75, {0.0, 1.2}, 180, 3.0, 7U);
  std::mt19937 engine(11U);
  std::uniform_real_distribution<double> random_coordinate(-2.5, 2.5);
  for (int index = 0; index < 120; ++index) {
    points.push_back({random_coordinate(engine), random_coordinate(engine)});
  }

  WallEstimatorConfig config;
  config.ransac_distance_threshold = 0.025;
  config.min_inliers = 100U;
  config.min_wall_length = 2.0;
  config.max_fit_rmse = 0.015;
  const auto result = estimate_wall(points, config);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->angle_rad * kRadiansToDegrees, 2.75, 0.08);
  EXPECT_NEAR(result->distance, 1.2, 0.03);
  EXPECT_LT(result->rmse, 0.01);
}

TEST(WallAlignment, SelectsNearestWallInRequestedSector)
{
  auto left = make_wall(0.0, {0.0, 0.8}, 120, 2.5, 17U);
  auto front = make_wall(90.0, {1.8, 0.0}, 180, 2.5, 19U);
  left.insert(left.end(), front.begin(), front.end());

  WallEstimatorConfig config;
  config.ransac_distance_threshold = 0.025;
  config.min_inliers = 70U;
  config.min_wall_length = 1.5;
  config.max_fit_rmse = 0.015;
  config.sector = "left";
  config.sector_half_angle_rad = 35.0 * kDegreesToRadians;
  const auto result = estimate_wall(left, config);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->angle_rad * kRadiansToDegrees, 0.0, 0.1);
  EXPECT_NEAR(result->distance, 0.8, 0.03);
}

TEST(WallAlignment, StrongestSelectionRewardsLongObservedSupport)
{
  auto long_wall = make_wall(10.0, {0.0, 1.4}, 80, 3.2, 23U);
  auto short_wall = make_wall(-30.0, {0.8, 0.0}, 110, 1.0, 29U);
  long_wall.insert(long_wall.end(), short_wall.begin(), short_wall.end());

  WallEstimatorConfig config;
  config.ransac_distance_threshold = 0.025;
  config.min_inliers = 50U;
  config.min_wall_length = 0.8;
  config.max_fit_rmse = 0.015;
  config.selection = "strongest";
  const auto result = estimate_wall(long_wall, config);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->angle_rad * kRadiansToDegrees, 10.0, 0.1);
  EXPECT_GT(result->length, 3.0);
}

TEST(WallAlignment, TrackedSelectionMaintainsAngularContinuity)
{
  auto tracked_wall = make_wall(12.0, {0.0, 1.4}, 70, 2.4, 31U);
  auto dominant_wall = make_wall(-42.0, {1.2, 0.0}, 140, 3.0, 37U);
  tracked_wall.insert(tracked_wall.end(), dominant_wall.begin(), dominant_wall.end());

  WallEstimatorConfig config;
  config.ransac_distance_threshold = 0.025;
  config.min_inliers = 50U;
  config.min_wall_length = 0.8;
  config.max_fit_rmse = 0.015;
  config.selection = "tracked";
  config.tracking_max_angle_rad = 15.0 * kDegreesToRadians;
  const auto result = estimate_wall(
    tracked_wall, config, 10.0 * kDegreesToRadians);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->angle_rad * kRadiansToDegrees, 12.0, 0.15);
}

TEST(WallAlignment, NormalizesParallelDirectionModuloOneHundredEightyDegrees)
{
  EXPECT_NEAR(normalize_wall_angle(179.0 * kDegreesToRadians) * kRadiansToDegrees, -1.0, 1e-9);
  EXPECT_NEAR(normalize_wall_angle(-179.0 * kDegreesToRadians) * kRadiansToDegrees, 1.0, 1e-9);
}
}  // namespace weldline_reflectivity_detector
