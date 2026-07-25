#include "weldline_reflectivity_detector/scenario_commander_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/msg/marker.hpp>

namespace weldline_reflectivity_detector
{
namespace
{
constexpr double kRadiansToDegrees = 57.2957795130823208768;

struct VoxelAccumulator
{
  double x_sum{0.0};
  double y_sum{0.0};
  double minimum_z{std::numeric_limits<double>::infinity()};
  double maximum_z{-std::numeric_limits<double>::infinity()};
  std::size_t count{0U};
};

std::uint64_t voxel_key(const std::int32_t x, const std::int32_t y)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
         static_cast<std::uint32_t>(y);
}

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

geometry_msgs::msg::Point marker_point(const Point2D & point, const double z)
{
  geometry_msgs::msg::Point output;
  output.x = point.x;
  output.y = point.y;
  output.z = z;
  return output;
}
}  // namespace

ScenarioCommanderNode::ScenarioCommanderNode(const rclcpp::NodeOptions & options)
: Node("scenario_commander_node", options)
{
  const auto declare_int_parameter =
    [this](const std::string & name, const std::int64_t default_value) {
      const auto value = declare_parameter<std::int64_t>(name, default_value);
      if (value < INT_MIN || value > INT_MAX) {
        throw std::invalid_argument(name + " is outside the supported integer range");
      }
      return static_cast<int>(value);
    };

  global_map_topic_ = declare_parameter<std::string>(
    "global_map_topic", "/point_lio/global_map_refined");
  reference_link_ = declare_parameter<std::string>("reference_link", "base_link");
  scenario_file_ = declare_parameter<std::string>("scenario_file", "");
  processing_rate_hz_ = declare_parameter<double>("wall_processing_rate_hz", 5.0);
  tf_timeout_sec_ = declare_parameter<double>("wall_tf_timeout_sec", 0.05);
  use_cloud_stamp_for_tf_ = declare_parameter<bool>("use_cloud_stamp_for_tf", false);
  minimum_range_ = declare_parameter<double>("wall_minimum_range", 0.25);
  search_radius_ = declare_parameter<double>("wall_search_radius", 3.0);
  minimum_height_ = declare_parameter<double>("wall_minimum_height", -2.0);
  maximum_height_ = declare_parameter<double>("wall_maximum_height", 2.0);
  xy_voxel_size_ = declare_parameter<double>("wall_xy_voxel_size", 0.04);
  minimum_vertical_span_ = declare_parameter<double>("wall_minimum_vertical_span", 0.35);
  minimum_voxel_points_ = declare_int_parameter("wall_minimum_voxel_points", 3);
  point_stride_ = declare_int_parameter("wall_point_stride", 1);
  smoothing_alpha_ = declare_parameter<double>("wall_angle_smoothing_alpha", 1.0);

  estimator_config_.ransac_distance_threshold =
    declare_parameter<double>("wall_ransac_distance_threshold", 0.04);
  estimator_config_.ransac_iterations =
    declare_int_parameter("wall_ransac_iterations", 300);
  estimator_config_.max_wall_candidates =
    declare_int_parameter("wall_max_candidates", 6);
  const auto minimum_inliers = declare_int_parameter("wall_minimum_inliers", 30);
  estimator_config_.min_inliers =
    minimum_inliers > 0 ? static_cast<std::size_t>(minimum_inliers) : 0U;
  estimator_config_.min_wall_length =
    declare_parameter<double>("wall_minimum_length", 0.8);
  estimator_config_.max_fit_rmse =
    declare_parameter<double>("wall_max_fit_rmse", 0.04);
  estimator_config_.selection =
    declare_parameter<std::string>("wall_selection", "tracked");
  estimator_config_.sector =
    declare_parameter<std::string>("wall_sector", "any");
  estimator_config_.sector_half_angle_rad =
    declare_parameter<double>("wall_sector_half_angle_deg", 70.0) /
    kRadiansToDegrees;
  estimator_config_.tracking_max_angle_rad =
    declare_parameter<double>("wall_tracking_max_angle_deg", 20.0) /
    kRadiansToDegrees;

  const auto angle_topic = declare_parameter<std::string>(
    "wall_angle_topic", "/commander/wall_alignment/angle_deg");
  const auto valid_topic = declare_parameter<std::string>(
    "wall_valid_topic", "/commander/wall_alignment/valid");
  const auto metrics_topic = declare_parameter<std::string>(
    "wall_metrics_topic", "/commander/wall_alignment/metrics");
  const auto marker_topic = declare_parameter<std::string>(
    "wall_marker_topic", "/commander/wall_alignment/markers");

  if (global_map_topic_.empty() || reference_link_.empty()) {
    throw std::invalid_argument("global_map_topic and reference_link must not be empty");
  }
  if (processing_rate_hz_ <= 0.0 || processing_rate_hz_ > 50.0) {
    throw std::invalid_argument("wall_processing_rate_hz must be in (0, 50]");
  }
  if (tf_timeout_sec_ < 0.0 || minimum_range_ < 0.0 ||
    search_radius_ <= minimum_range_)
  {
    throw std::invalid_argument(
            "wall ranges and wall_tf_timeout_sec are inconsistent");
  }
  if (minimum_height_ >= maximum_height_ || xy_voxel_size_ <= 0.0 ||
    minimum_vertical_span_ < 0.0 || minimum_voxel_points_ <= 0 ||
    point_stride_ <= 0 || smoothing_alpha_ <= 0.0 || smoothing_alpha_ > 1.0)
  {
    throw std::invalid_argument("Invalid wall preprocessing configuration");
  }
  // Validates all estimator enum/range parameters before any data arrives.
  (void)estimate_wall({}, estimator_config_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  initialize_scenario(scenario_file_);

  angle_pub_ = create_publisher<std_msgs::msg::Float64>(
    angle_topic, rclcpp::QoS(10).reliable());
  valid_pub_ = create_publisher<std_msgs::msg::Bool>(
    valid_topic, rclcpp::QoS(10).reliable());
  metrics_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    metrics_topic, rclcpp::QoS(10).reliable());
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic, rclcpp::QoS(5).reliable());

  // POINT_LIO publishes the refined global map as reliable transient-local data.
  // Matching durability is essential: it lets a commander started later receive
  // the already-built map without waiting for another full-map publication.
  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    global_map_topic_, map_qos,
    std::bind(&ScenarioCommanderNode::on_global_map, this, std::placeholders::_1));

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / processing_rate_hz_));
  processing_timer_ = create_wall_timer(
    timer_period, std::bind(&ScenarioCommanderNode::process_wall_alignment, this));

  RCLCPP_INFO(
    logger_,
    "Commander wall alignment ready: map=%s, link=%s, radius=%.2f m, sector=%s, output=%s",
    global_map_topic_.c_str(), reference_link_.c_str(), search_radius_,
    estimator_config_.sector.c_str(), angle_topic.c_str());
}

std::vector<ScenarioCommanderNode::Point3D> ScenarioCommanderNode::decode_xyz(
  const sensor_msgs::msg::PointCloud2 & message) const
{
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
  const std::size_t declared_count =
    static_cast<std::size_t>(message.width) * message.height;
  std::vector<Point3D> points;
  points.reserve(declared_count / static_cast<std::size_t>(point_stride_) + 1U);
  std::size_t flat_index = 0U;
  for (std::uint32_t row = 0U; row < message.height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * message.row_step;
    for (std::uint32_t column = 0U; column < message.width; ++column, ++flat_index) {
      if (flat_index % static_cast<std::size_t>(point_stride_) != 0U) {
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
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message)
{
  if (message->header.frame_id.empty()) {
    RCLCPP_ERROR(logger_, "Rejected global map with an empty frame_id");
    return;
  }
  try {
    auto decoded = std::make_shared<CachedMap>();
    decoded->header = message->header;
    decoded->points = decode_xyz(*message);
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

void ScenarioCommanderNode::process_wall_alignment()
{
  std::shared_ptr<const CachedMap> map;
  {
    std::scoped_lock lock(map_mutex_);
    map = cached_map_;
  }
  if (!map || map->points.empty()) {
    publish_invalid("waiting for global map");
    return;
  }

  geometry_msgs::msg::TransformStamped transform_message;
  try {
    if (reference_link_ == map->header.frame_id) {
      transform_message.header.frame_id = reference_link_;
      transform_message.child_frame_id = map->header.frame_id;
      transform_message.transform.rotation.w = 1.0;
    } else if (use_cloud_stamp_for_tf_) {
      try {
        transform_message = tf_buffer_->lookupTransform(
          reference_link_, map->header.frame_id, rclcpp::Time(map->header.stamp),
          rclcpp::Duration::from_seconds(tf_timeout_sec_));
      } catch (const tf2::TransformException &) {
        // A transient-local map may predate this node's TF cache. The map geometry
        // remains valid, so use the latest robot pose rather than dropping it forever.
        transform_message = tf_buffer_->lookupTransform(
          reference_link_, map->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_sec_));
      }
    } else {
      transform_message = tf_buffer_->lookupTransform(
        reference_link_, map->header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    }
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      logger_, *get_clock(), 2000, "Cannot transform global map %s -> %s: %s",
      map->header.frame_id.c_str(), reference_link_.c_str(), error.what());
    publish_invalid("missing map-to-link transform");
    return;
  }

  const auto & translation = transform_message.transform.translation;
  const auto & rotation_message = transform_message.transform.rotation;
  const tf2::Quaternion quaternion(
    rotation_message.x, rotation_message.y, rotation_message.z, rotation_message.w);
  if (quaternion.length2() <= std::numeric_limits<double>::epsilon()) {
    publish_invalid("invalid map-to-link rotation");
    return;
  }
  const tf2::Matrix3x3 rotation(quaternion.normalized());
  const tf2::Vector3 translation_vector(
    translation.x, translation.y, translation.z);
  const double minimum_range_squared = minimum_range_ * minimum_range_;
  const double search_radius_squared = search_radius_ * search_radius_;

  std::unordered_map<std::uint64_t, VoxelAccumulator> voxels;
  voxels.reserve(std::min<std::size_t>(map->points.size(), 50000U));
  for (const auto & map_point : map->points) {
    const tf2::Vector3 point =
      rotation * tf2::Vector3(map_point.x, map_point.y, map_point.z) +
      translation_vector;
    const double x = point.x();
    const double y = point.y();
    const double z = point.z();
    const double range_squared = x * x + y * y;
    if (
      range_squared < minimum_range_squared || range_squared > search_radius_squared ||
      z < minimum_height_ || z > maximum_height_)
    {
      continue;
    }
    const auto voxel_x = static_cast<std::int32_t>(std::floor(x / xy_voxel_size_));
    const auto voxel_y = static_cast<std::int32_t>(std::floor(y / xy_voxel_size_));
    auto & voxel = voxels[voxel_key(voxel_x, voxel_y)];
    voxel.x_sum += x;
    voxel.y_sum += y;
    voxel.minimum_z = std::min(voxel.minimum_z, z);
    voxel.maximum_z = std::max(voxel.maximum_z, z);
    ++voxel.count;
  }

  std::vector<Point2D> wall_points;
  wall_points.reserve(voxels.size());
  for (const auto & item : voxels) {
    const auto & voxel = item.second;
    if (
      voxel.count < static_cast<std::size_t>(minimum_voxel_points_) ||
      voxel.maximum_z - voxel.minimum_z < minimum_vertical_span_)
    {
      continue;
    }
    const double inverse_count = 1.0 / static_cast<double>(voxel.count);
    wall_points.push_back(
      {voxel.x_sum * inverse_count, voxel.y_sum * inverse_count});
  }

  const auto estimate = estimate_wall(
    wall_points, estimator_config_, tracked_angle_rad_);
  if (!estimate) {
    RCLCPP_DEBUG(
      logger_, "No valid wall: %zu vertical voxels from %zu local voxels",
      wall_points.size(), voxels.size());
    publish_invalid("no valid wall candidate");
    return;
  }
  publish_estimate(*estimate);
}

void ScenarioCommanderNode::publish_invalid(const std::string & reason)
{
  tracked_angle_rad_.reset();
  filtered_angle_rad_.reset();
  latest_wall_valid_ = false;
  latest_wall_update_ = now();
  std_msgs::msg::Bool valid_message;
  valid_message.data = false;
  valid_pub_->publish(valid_message);
  std_msgs::msg::Float64 angle_message;
  angle_message.data = std::numeric_limits<double>::quiet_NaN();
  angle_pub_->publish(angle_message);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.stamp = now();
  clear.header.frame_id = reference_link_;
  clear.ns = "commander_wall_alignment";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  marker_pub_->publish(markers);

  if (reason != last_invalid_reason_) {
    RCLCPP_WARN(logger_, "Wall alignment invalid: %s", reason.c_str());
    last_invalid_reason_ = reason;
  }
}

void ScenarioCommanderNode::publish_estimate(const WallEstimate & estimate)
{
  double angle_rad = estimate.angle_rad;
  tracked_angle_rad_ = angle_rad;
  if (filtered_angle_rad_) {
    const double delta = normalize_wall_angle(angle_rad - *filtered_angle_rad_);
    angle_rad = normalize_wall_angle(*filtered_angle_rad_ + smoothing_alpha_ * delta);
  }
  filtered_angle_rad_ = angle_rad;
  latest_wall_valid_ = true;
  latest_wall_angle_rad_ = angle_rad;
  latest_wall_update_ = now();
  last_invalid_reason_.clear();
  const double angle_degrees = angle_rad * kRadiansToDegrees;

  std_msgs::msg::Bool valid_message;
  valid_message.data = true;
  valid_pub_->publish(valid_message);
  std_msgs::msg::Float64 angle_message;
  angle_message.data = angle_degrees;
  angle_pub_->publish(angle_message);

  geometry_msgs::msg::Vector3Stamped metrics;
  metrics.header.stamp = now();
  metrics.header.frame_id = reference_link_;
  metrics.vector.x = angle_degrees;
  metrics.vector.y = estimate.distance;
  metrics.vector.z = estimate.rmse;
  metrics_pub_->publish(metrics);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker line;
  line.header = metrics.header;
  line.ns = "commander_wall_alignment";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = 0.035;
  line.color.r = 0.1F;
  line.color.g = 1.0F;
  line.color.b = 0.25F;
  line.color.a = 1.0F;
  line.points = {marker_point(estimate.start, 0.05), marker_point(estimate.end, 0.05)};
  line.lifetime = rclcpp::Duration::from_seconds(2.0 / processing_rate_hz_);
  markers.markers.push_back(line);

  visualization_msgs::msg::Marker closest = line;
  closest.id = 1;
  closest.type = visualization_msgs::msg::Marker::SPHERE;
  closest.pose.position = marker_point(estimate.closest, 0.05);
  closest.scale.x = closest.scale.y = closest.scale.z = 0.10;
  closest.color.r = 1.0F;
  closest.color.g = 0.75F;
  closest.color.b = 0.1F;
  closest.points.clear();
  markers.markers.push_back(closest);

  visualization_msgs::msg::Marker text = line;
  text.id = 2;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.pose.position = marker_point(estimate.closest, 0.25);
  text.scale.z = 0.18;
  text.color.r = text.color.g = text.color.b = text.color.a = 1.0F;
  text.points.clear();
  text.text =
    "wall error: " + std::to_string(angle_degrees) +
    " deg  d=" + std::to_string(estimate.distance) +
    " m  rms=" + std::to_string(estimate.rmse) + " m";
  markers.markers.push_back(text);
  marker_pub_->publish(markers);

  RCLCPP_DEBUG(
    logger_, "Wall angle %.6f deg, distance %.3f m, RMS %.4f m, length %.2f m, %zu inliers",
    angle_degrees, estimate.distance, estimate.rmse, estimate.length,
    estimate.inlier_count);
}
}  // namespace weldline_reflectivity_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<weldline_reflectivity_detector::ScenarioCommanderNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("scenario_commander_node"), "Startup failed: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
