#include "weldline_reflectivity_detector/wall_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <utility>

namespace weldline_reflectivity_detector
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

struct LineFit
{
  Point2D center;
  Point2D direction;
  Point2D start;
  Point2D end;
  Point2D closest;
  double distance{0.0};
  double rmse{0.0};
  double length{0.0};
  std::vector<std::size_t> inliers;
};

double squared_distance(const Point2D & first, const Point2D & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  return dx * dx + dy * dy;
}

double point_line_distance(
  const Point2D & point, const Point2D & origin, const Point2D & unit_direction)
{
  return std::abs(
    unit_direction.x * (point.y - origin.y) -
    unit_direction.y * (point.x - origin.x));
}

std::optional<LineFit> fit_tls(
  const std::vector<Point2D> & points, const std::vector<std::size_t> & indices)
{
  if (indices.size() < 2U) {
    return std::nullopt;
  }

  Point2D center;
  for (const auto index : indices) {
    center.x += points[index].x;
    center.y += points[index].y;
  }
  const double inverse_count = 1.0 / static_cast<double>(indices.size());
  center.x *= inverse_count;
  center.y *= inverse_count;

  double covariance_xx = 0.0;
  double covariance_xy = 0.0;
  double covariance_yy = 0.0;
  for (const auto index : indices) {
    const double dx = points[index].x - center.x;
    const double dy = points[index].y - center.y;
    covariance_xx += dx * dx;
    covariance_xy += dx * dy;
    covariance_yy += dy * dy;
  }
  if (covariance_xx + covariance_yy <= std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }

  const double angle =
    0.5 * std::atan2(2.0 * covariance_xy, covariance_xx - covariance_yy);
  const Point2D direction{std::cos(angle), std::sin(angle)};
  std::vector<double> projections;
  projections.reserve(indices.size());
  double squared_error_sum = 0.0;
  for (const auto index : indices) {
    const double dx = points[index].x - center.x;
    const double dy = points[index].y - center.y;
    const double projection = dx * direction.x + dy * direction.y;
    projections.push_back(projection);
    const double residual = direction.x * dy - direction.y * dx;
    squared_error_sum += residual * residual;
  }
  // A wall crossing contributes a few valid perpendicular inliers far beyond the
  // observed segment. Robust projection quantiles stop those corner points from
  // inflating line length and therefore the strongest-wall score.
  std::sort(projections.begin(), projections.end());
  const auto trim_count = static_cast<std::size_t>(
    std::floor(0.02 * static_cast<double>(projections.size())));
  const double minimum_projection = projections[trim_count];
  const double maximum_projection = projections[projections.size() - 1U - trim_count];

  LineFit fit;
  fit.center = center;
  fit.direction = direction;
  fit.start = {
    center.x + minimum_projection * direction.x,
    center.y + minimum_projection * direction.y};
  fit.end = {
    center.x + maximum_projection * direction.x,
    center.y + maximum_projection * direction.y};
  fit.length = maximum_projection - minimum_projection;
  fit.rmse = std::sqrt(squared_error_sum * inverse_count);
  fit.inliers = indices;

  const double origin_projection =
    -center.x * direction.x - center.y * direction.y;
  const double clamped_projection =
    std::clamp(origin_projection, minimum_projection, maximum_projection);
  fit.closest = {
    center.x + clamped_projection * direction.x,
    center.y + clamped_projection * direction.y};
  fit.distance = std::hypot(fit.closest.x, fit.closest.y);
  return fit;
}

bool angle_in_sector(double bearing, const std::string & sector, double half_angle)
{
  if (sector == "any") {
    return true;
  }

  double center = 0.0;
  if (sector == "front") {
    center = 0.0;
  } else if (sector == "left") {
    center = kPi / 2.0;
  } else if (sector == "rear") {
    center = kPi;
  } else if (sector == "right") {
    center = -kPi / 2.0;
  } else {
    return false;
  }
  return std::abs(std::remainder(bearing - center, 2.0 * kPi)) <= half_angle;
}

std::optional<LineFit> find_line(
  const std::vector<Point2D> & points, const std::vector<std::size_t> & remaining,
  const WallEstimatorConfig & config, std::mt19937 & random_engine)
{
  if (remaining.size() < config.min_inliers) {
    return std::nullopt;
  }

  std::uniform_int_distribution<std::size_t> distribution(0U, remaining.size() - 1U);
  std::vector<std::size_t> best_inliers;
  best_inliers.reserve(remaining.size());
  const double minimum_sample_separation_squared =
    4.0 * config.ransac_distance_threshold * config.ransac_distance_threshold;

  for (int iteration = 0; iteration < config.ransac_iterations; ++iteration) {
    const auto first_index = remaining[distribution(random_engine)];
    const auto second_index = remaining[distribution(random_engine)];
    if (first_index == second_index ||
      squared_distance(points[first_index], points[second_index]) <
      minimum_sample_separation_squared)
    {
      continue;
    }

    const Point2D origin = points[first_index];
    const double dx = points[second_index].x - origin.x;
    const double dy = points[second_index].y - origin.y;
    const double inverse_length = 1.0 / std::hypot(dx, dy);
    const Point2D direction{dx * inverse_length, dy * inverse_length};
    std::vector<std::size_t> inliers;
    for (const auto point_index : remaining) {
      if (point_line_distance(points[point_index], origin, direction) <=
        config.ransac_distance_threshold)
      {
        inliers.push_back(point_index);
      }
    }
    if (inliers.size() > best_inliers.size()) {
      best_inliers = std::move(inliers);
    }
  }

  if (best_inliers.size() < config.min_inliers) {
    return std::nullopt;
  }

  // Reclassify twice around the TLS line. This removes the random two-point sample's
  // angular bias while retaining RANSAC's robustness against corners and clutter.
  for (int refinement = 0; refinement < 2; ++refinement) {
    const auto fit = fit_tls(points, best_inliers);
    if (!fit) {
      return std::nullopt;
    }
    std::vector<std::size_t> refined_inliers;
    refined_inliers.reserve(best_inliers.size());
    for (const auto point_index : remaining) {
      if (point_line_distance(points[point_index], fit->center, fit->direction) <=
        config.ransac_distance_threshold)
      {
        refined_inliers.push_back(point_index);
      }
    }
    best_inliers = std::move(refined_inliers);
    if (best_inliers.size() < config.min_inliers) {
      return std::nullopt;
    }
  }
  return fit_tls(points, best_inliers);
}
}  // namespace

double normalize_wall_angle(double angle_rad)
{
  double normalized = std::remainder(angle_rad, kPi);
  if (normalized >= kPi / 2.0) {
    normalized -= kPi;
  }
  if (normalized < -kPi / 2.0) {
    normalized += kPi;
  }
  return normalized;
}

std::optional<WallEstimate> estimate_wall(
  const std::vector<Point2D> & points, const WallEstimatorConfig & config,
  const std::optional<double> preferred_angle_rad)
{
  if (config.ransac_distance_threshold <= 0.0 || config.ransac_iterations <= 0 ||
    config.max_wall_candidates <= 0 || config.min_inliers < 2U ||
    config.min_wall_length <= 0.0 || config.max_fit_rmse <= 0.0 ||
    config.sector_half_angle_rad <= 0.0 || config.sector_half_angle_rad > kPi ||
    config.tracking_max_angle_rad <= 0.0 || config.tracking_max_angle_rad > kPi / 2.0)
  {
    throw std::invalid_argument("Invalid wall estimator configuration");
  }
  if (
    config.sector != "any" && config.sector != "front" && config.sector != "rear" &&
    config.sector != "left" && config.sector != "right")
  {
    throw std::invalid_argument("sector must be any, front, rear, left, or right");
  }
  if (
    config.selection != "nearest" && config.selection != "strongest" &&
    config.selection != "tracked")
  {
    throw std::invalid_argument("selection must be tracked, strongest, or nearest");
  }
  if (points.size() < config.min_inliers) {
    return std::nullopt;
  }

  std::vector<std::size_t> remaining(points.size());
  std::iota(remaining.begin(), remaining.end(), 0U);
  std::vector<LineFit> candidates;
  std::mt19937 random_engine(0xC0CE10U);

  for (int candidate_index = 0;
    candidate_index < config.max_wall_candidates &&
    remaining.size() >= config.min_inliers;
    ++candidate_index)
  {
    const auto fit = find_line(points, remaining, config, random_engine);
    if (!fit) {
      break;
    }

    if (
      fit->inliers.size() >= config.min_inliers &&
      fit->length >= config.min_wall_length &&
      fit->rmse <= config.max_fit_rmse)
    {
      const double bearing = std::atan2(fit->closest.y, fit->closest.x);
      if (angle_in_sector(bearing, config.sector, config.sector_half_angle_rad)) {
        candidates.push_back(*fit);
      }
    }

    std::vector<bool> is_inlier(points.size(), false);
    for (const auto inlier : fit->inliers) {
      is_inlier[inlier] = true;
    }
    remaining.erase(
      std::remove_if(
        remaining.begin(), remaining.end(),
        [&is_inlier](const std::size_t index) {return is_inlier[index];}),
      remaining.end());
  }

  if (candidates.empty()) {
    return std::nullopt;
  }
  const auto strongest = std::max_element(
    candidates.begin(), candidates.end(),
    [](const LineFit & left, const LineFit & right) {
      const double left_support =
        static_cast<double>(left.inliers.size()) * left.length;
      const double right_support =
        static_cast<double>(right.inliers.size()) * right.length;
      return left_support < right_support;
    });

  auto selected = strongest;
  if (config.selection == "tracked" && preferred_angle_rad) {
    const auto tracked = std::min_element(
      candidates.begin(), candidates.end(),
      [&preferred_angle_rad](const LineFit & left, const LineFit & right) {
        const double left_angle =
          normalize_wall_angle(std::atan2(left.direction.y, left.direction.x));
        const double right_angle =
          normalize_wall_angle(std::atan2(right.direction.y, right.direction.x));
        return std::abs(normalize_wall_angle(left_angle - *preferred_angle_rad)) <
               std::abs(normalize_wall_angle(right_angle - *preferred_angle_rad));
      });
    const double tracked_angle =
      normalize_wall_angle(std::atan2(tracked->direction.y, tracked->direction.x));
    if (
      std::abs(normalize_wall_angle(tracked_angle - *preferred_angle_rad)) <=
      config.tracking_max_angle_rad)
    {
      selected = tracked;
    }
  } else if (config.selection == "nearest") {
    selected = std::min_element(
      candidates.begin(), candidates.end(),
      [](const LineFit & left, const LineFit & right) {
        if (std::abs(left.distance - right.distance) > 1e-9) {
          return left.distance < right.distance;
        }
        return left.inliers.size() > right.inliers.size();
      });
  }

  WallEstimate result;
  result.angle_rad =
    normalize_wall_angle(std::atan2(selected->direction.y, selected->direction.x));
  result.distance = selected->distance;
  result.rmse = selected->rmse;
  result.length = selected->length;
  result.inlier_count = selected->inliers.size();
  result.start = selected->start;
  result.end = selected->end;
  result.closest = selected->closest;
  return result;
}
}  // namespace weldline_reflectivity_detector
