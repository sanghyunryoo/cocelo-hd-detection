#include "weldline_reflectivity_detector/point_cloud_xyz.hpp"

#include "weldline_reflectivity_detector/scenario_commander_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <type_traits>

#include <sensor_msgs/msg/point_field.hpp>

namespace weldline_reflectivity_detector
{
namespace
{
bool host_is_big_endian()
{
  const std::uint16_t value = 0x0102U;
  std::array<std::uint8_t, sizeof(value)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(value));
  return bytes[0] == 0x01U;
}

template<typename T>
T read_scalar(const std::uint8_t * data, const bool swap_bytes)
{
  static_assert(std::is_trivially_copyable<T>::value, "Point fields must be trivially copyable");
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::copy_n(data, sizeof(T), bytes.begin());
  if (swap_bytes) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value{};
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

std::size_t field_size(const std::uint8_t datatype)
{
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::FLOAT32:
      return sizeof(float);
    case PointField::FLOAT64:
      return sizeof(double);
    default:
      return 0U;
  }
}

double read_coordinate(
  const std::uint8_t * point_data, const sensor_msgs::msg::PointField & field,
  const bool swap_bytes)
{
  using sensor_msgs::msg::PointField;
  const auto * field_data = point_data + field.offset;
  if (field.datatype == PointField::FLOAT32) {
    return static_cast<double>(read_scalar<float>(field_data, swap_bytes));
  }
  if (field.datatype == PointField::FLOAT64) {
    return read_scalar<double>(field_data, swap_bytes);
  }
  throw std::runtime_error("PointCloud2 x/y/z fields must be FLOAT32 or FLOAT64");
}
}  // namespace

std::vector<Point3D> decode_point_cloud_xyz(
  const sensor_msgs::msg::PointCloud2 & message, const int point_stride)
{
  if (point_stride <= 0) {
    throw std::invalid_argument("PointCloud2 point_stride must be positive");
  }
  const sensor_msgs::msg::PointField * x_field = nullptr;
  const sensor_msgs::msg::PointField * y_field = nullptr;
  const sensor_msgs::msg::PointField * z_field = nullptr;
  for (const auto & field : message.fields) {
    if (field.name == "x") {
      x_field = &field;
    } else if (field.name == "y") {
      y_field = &field;
    } else if (field.name == "z") {
      z_field = &field;
    }
  }
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
    throw std::runtime_error("PointCloud2 is missing x, y, or z field");
  }
  for (const auto * field : {x_field, y_field, z_field}) {
    const auto size = field_size(field->datatype);
    if (size == 0U || field->count < 1U ||
      static_cast<std::size_t>(field->offset) + size > message.point_step)
    {
      throw std::runtime_error("PointCloud2 has an unsupported or invalid XYZ layout");
    }
  }
  if (message.point_step == 0U ||
    (message.height > 0U &&
    static_cast<std::size_t>(message.row_step) * message.height > message.data.size()))
  {
    throw std::runtime_error("PointCloud2 data buffer is smaller than its declared layout");
  }

  const bool swap_bytes = message.is_bigendian != host_is_big_endian();
  const std::size_t declared_count = static_cast<std::size_t>(message.width) * message.height;
  std::vector<Point3D> points;
  points.reserve(declared_count / static_cast<std::size_t>(point_stride) + 1U);
  std::size_t flat_index = 0U;
  for (std::uint32_t row = 0U; row < message.height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * message.row_step;
    for (std::uint32_t column = 0U; column < message.width; ++column, ++flat_index) {
      if (flat_index % static_cast<std::size_t>(point_stride) != 0U) {
        continue;
      }
      const std::size_t offset =
        row_offset + static_cast<std::size_t>(column) * message.point_step;
      if (offset + message.point_step > message.data.size()) {
        throw std::runtime_error("PointCloud2 point exceeds its data buffer");
      }
      const auto * point_data = message.data.data() + offset;
      const Point3D point{
        read_coordinate(point_data, *x_field, swap_bytes),
        read_coordinate(point_data, *y_field, swap_bytes),
        read_coordinate(point_data, *z_field, swap_bytes)};
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        points.push_back(point);
      }
    }
  }
  return points;
}

void ScenarioCommanderNode::on_global_map(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  if (message->header.frame_id.empty()) {
    RCLCPP_ERROR(logger_, "Rejected global map with an empty frame_id");
    return;
  }
  try {
    auto decoded = std::make_shared<CachedMap>();
    decoded->header = message->header;
    decoded->points = decode_point_cloud_xyz(*message, point_stride_);
    {
      std::scoped_lock lock(map_mutex_);
      cached_map_ = decoded;
    }
    RCLCPP_INFO(
      logger_, "Cached global map: %zu finite points in frame '%s'",
      decoded->points.size(), decoded->header.frame_id.c_str());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Rejected global map: %s", error.what());
  }
}
}  // namespace weldline_reflectivity_detector
