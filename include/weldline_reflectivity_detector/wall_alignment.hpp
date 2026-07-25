#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace weldline_reflectivity_detector
{
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct WallEstimate
{
  double angle_rad{0.0};
  double distance{0.0};
  double rmse{0.0};
  double length{0.0};
  std::size_t inlier_count{0};
  Point2D start;
  Point2D end;
  Point2D closest;
};

struct WallEstimatorConfig
{
  double ransac_distance_threshold{0.04};
  int ransac_iterations{300};
  int max_wall_candidates{6};
  std::size_t min_inliers{30};
  double min_wall_length{0.8};
  double max_fit_rmse{0.04};
  std::string selection{"tracked"};
  std::string sector{"any"};
  double sector_half_angle_rad{1.2217304763960306};  // 70 degrees
  double tracking_max_angle_rad{0.3490658503988659};  // 20 degrees
};

/// Fits multiple 2D wall lines and returns the selected wall in the reference-link frame.
///
/// angle_rad is the signed, pi-periodic error from the link +X axis to the wall tangent.
/// It is normalized to [-pi/2, pi/2), so a parallel link reports zero regardless of the
/// arbitrary direction assigned to the fitted line.
std::optional<WallEstimate> estimate_wall(
  const std::vector<Point2D> & points, const WallEstimatorConfig & config,
  std::optional<double> preferred_angle_rad = std::nullopt);

double normalize_wall_angle(double angle_rad);
}  // namespace weldline_reflectivity_detector
