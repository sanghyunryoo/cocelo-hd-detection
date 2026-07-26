#pragma once

#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace weldline_reflectivity_detector
{
struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

std::vector<Point3D> decode_point_cloud_xyz(
  const sensor_msgs::msg::PointCloud2 & message, int point_stride);
}  // namespace weldline_reflectivity_detector
