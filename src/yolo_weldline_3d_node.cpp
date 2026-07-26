#include "weldline_reflectivity_detector/yolo_weldline_3d_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace weldline_reflectivity_detector
{
namespace
{
constexpr int kSensorQueueDepth = 10;

cv::Mat letterbox(const cv::Mat & image, const cv::Size & target, float & scale, int & pad_x, int & pad_y)
{
  scale = std::min(static_cast<float>(target.width) / static_cast<float>(image.cols),
                   static_cast<float>(target.height) / static_cast<float>(image.rows));
  const cv::Size resized_size(static_cast<int>(std::round(static_cast<float>(image.cols) * scale)),
                              static_cast<int>(std::round(static_cast<float>(image.rows) * scale)));
  pad_x = (target.width - resized_size.width) / 2;
  pad_y = (target.height - resized_size.height) / 2;
  cv::Mat output(target, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::resize(image, output(cv::Rect(pad_x, pad_y, resized_size.width, resized_size.height)), resized_size);
  return output;
}

float intersection_over_union(const cv::Rect & a, const cv::Rect & b)
{
  const int intersection = (a & b).area();
  const int union_area = a.area() + b.area() - intersection;
  return union_area > 0 ? static_cast<float>(intersection) / static_cast<float>(union_area) : 0.0F;
}

std::vector<int> nms_indices(const std::vector<cv::Rect> & boxes, const std::vector<float> & scores,
                             const float threshold)
{
  std::vector<int> order(boxes.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&scores](int lhs, int rhs) { return scores[lhs] > scores[rhs]; });
  std::vector<int> kept;
  for (const int index : order) {
    bool suppressed = false;
    for (const int kept_index : kept) {
      if (intersection_over_union(boxes[static_cast<size_t>(index)], boxes[static_cast<size_t>(kept_index)]) > threshold) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) kept.push_back(index);
  }
  return kept;
}
}  // namespace

YoloWeldline3DNode::~YoloWeldline3DNode() = default;

YoloWeldline3DNode::YoloWeldline3DNode(const rclcpp::NodeOptions & options)
: Node("yolo_weldline_3d_node", options), color_sub_(this, ""), depth_sub_(this, ""), info_sub_(this, "")
{
  const auto weights = declare_parameter<std::string>("weights", "weights/best.onnx");
  // These deployment values are read by launch.sh before rclcpp starts; declaring them
  // here keeps the single YAML file valid when it is passed to this node.
  (void)declare_parameter<int>("ros_domain_id", 0);
  (void)declare_parameter<std::string>("usb_port_id", "");
  const auto color_topic = declare_parameter<std::string>("color_topic", "/camera/camera/color/image_raw");
  const auto depth_topic = declare_parameter<std::string>("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  const auto info_topic = declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
  output_frame_ = declare_parameter<std::string>("output_frame", "map");
  const auto rate_hz = declare_parameter<double>("processing_rate_hz", 30.0);
  confidence_threshold_ = static_cast<float>(declare_parameter<double>("confidence_threshold", 0.40));
  nms_threshold_ = static_cast<float>(declare_parameter<double>("nms_threshold", 0.25));
  const auto target_class_id = declare_parameter<int64_t>("target_class_id", -1);
  depth_window_ = std::max(1, static_cast<int>(declare_parameter<int>("depth_window", 7)));
  use_line_detection_ = declare_parameter<bool>("use_line_detection", true);
  planar_goal_ = declare_parameter<bool>("nav2_planar_goal", false);
  const auto goal_topic = declare_parameter<std::string>("nav2_goal_topic", "/goal_pose");
  const auto point_topic = declare_parameter<std::string>("debug_point_topic", "/weldline_yolo/debug/center_point");
  const auto marker_topic = declare_parameter<std::string>("marker_topic", "/weldline_yolo/debug/markers");
  const auto annotated_topic = declare_parameter<std::string>("annotated_topic", "/weldline_yolo/debug/annotated_image");
  if (rate_hz <= 0.0 || rate_hz > 120.0) throw std::invalid_argument("processing_rate_hz must be in (0, 120]");
  if (target_class_id < -1 || target_class_id > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("target_class_id must be -1 or a non-negative class id");
  }
  target_class_id_ = static_cast<int>(target_class_id);

  try {
    ort_options_.SetIntraOpNumThreads(1);
    ort_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    ort_session_ = std::make_unique<Ort::Session>(ort_env_, weights.c_str(), ort_options_);
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_name = ort_session_->GetInputNameAllocated(0, allocator);
    const auto output_name = ort_session_->GetOutputNameAllocated(0, allocator);
    input_name_ = input_name.get();
    output_name_ = output_name.get();

    const auto input_shape = ort_session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.size() != 4) {
      throw std::runtime_error("ONNX input must be a 4D NCHW tensor");
    }
    if (input_shape[1] > 0 && input_shape[1] != 3) {
      throw std::runtime_error("ONNX input channel count must be 3");
    }
    if (input_shape[2] > 0) network_height_ = static_cast<int>(input_shape[2]);
    if (input_shape[3] > 0) network_width_ = static_cast<int>(input_shape[3]);
    if (network_width_ <= 0 || network_height_ <= 0) {
      throw std::runtime_error("ONNX input width/height must be positive");
    }
    input_shape_ = {1, 3, network_height_, network_width_};
    input_tensor_.resize(static_cast<size_t>(input_shape_[0] * input_shape_[1] * input_shape_[2] * input_shape_[3]));
  } catch (const Ort::Exception & error) {
    throw std::runtime_error("Unable to load ONNX weights with ONNX Runtime '" + weights + "': " + error.what());
  }

  const auto sensor_qos = rclcpp::SensorDataQoS();
  color_sub_.subscribe(this, color_topic, sensor_qos.get_rmw_qos_profile());
  depth_sub_.subscribe(this, depth_topic, sensor_qos.get_rmw_qos_profile());
  info_sub_.subscribe(this, info_topic, sensor_qos.get_rmw_qos_profile());
  synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(kSensorQueueDepth), color_sub_, depth_sub_, info_sub_);
  synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.10));
  synchronizer_->registerCallback(std::bind(&YoloWeldline3DNode::on_synchronized_frame, this,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, rclcpp::QoS(5).reliable());
  point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(point_topic, rclcpp::QoS(5).reliable());
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(5).reliable());
  annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic, rclcpp::QoS(5).best_effort());
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz));
  processing_timer_ = create_wall_timer(period, std::bind(&YoloWeldline3DNode::process_latest_frame, this));
  RCLCPP_INFO(logger_, "Ready: C++ ONNX Runtime RGB-D localization at %.1f Hz; Nav2 goal topic: %s",
              rate_hz, goal_topic.c_str());
}

void YoloWeldline3DNode::on_synchronized_frame(const sensor_msgs::msg::Image::ConstSharedPtr & color,
                                                 const sensor_msgs::msg::Image::ConstSharedPtr & depth,
                                                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  std::scoped_lock lock(frame_mutex_);
  latest_frame_ = FrameBundle{color, depth, info};  // latest-only: bounded latency under camera overload
}

void YoloWeldline3DNode::process_latest_frame()
{
  std::optional<FrameBundle> frame;
  { std::scoped_lock lock(frame_mutex_); frame = latest_frame_; latest_frame_.reset(); }
  if (!frame) return;
  try {
    const auto color_bridge = cv_bridge::toCvShare(frame->color, "bgr8");
    auto header = frame->depth ? frame->depth->header : frame->color->header; if (header.frame_id.empty()) header = frame->color->header;
    const auto depth_bridge = cv_bridge::toCvShare(frame->depth);
    cv::Mat depth;
    if (frame->depth->encoding == "16UC1" || frame->depth->encoding == "mono16") depth_bridge->image.convertTo(depth, CV_32FC1, 0.001);
    else if (frame->depth->encoding == "32FC1") depth = depth_bridge->image;
    else {
      RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000, "Unsupported depth encoding: %s", frame->depth->encoding.c_str());
      publish_status_image(frame->color, color_bridge->image, "UNSUPPORTED DEPTH ENCODING: " + frame->depth->encoding);
      publish_empty_debug(frame->color->header);
      return;
    }
    const auto detection = infer(color_bridge->image);
    if (!detection) {
      publish_status_image(frame->color, color_bridge->image, "NO DETECTION");
      publish_empty_debug(header);
      return;
    }
    const auto center_camera = project_pixel(detection->center, depth, *frame->info);
    if (!center_camera) {
      cv::Mat annotated = color_bridge->image.clone();
      cv::rectangle(annotated, detection->box, cv::Scalar(60, 220, 255), 2);
      cv::circle(annotated, detection->center, 5, cv::Scalar(0, 220, 255), cv::FILLED);
      publish_status_image(frame->color, annotated, "DETECTION OK / NO VALID DEPTH");
      publish_empty_debug(header);
      return;
    }
    const auto center = transform_point(*center_camera, header);
    if (!center) {
      cv::Mat annotated = color_bridge->image.clone();
      cv::rectangle(annotated, detection->box, cv::Scalar(60, 220, 255), 2);
      cv::circle(annotated, detection->center, 5, cv::Scalar(0, 220, 255), cv::FILLED);
      publish_status_image(frame->color, annotated, "DETECTION OK / TF UNAVAILABLE");
      publish_empty_debug(header);
      return;
    }
    std::optional<geometry_msgs::msg::PointStamped> start, end;
    if (const auto p = project_pixel(detection->line_start, depth, *frame->info)) start = transform_point(*p, header);
    if (const auto p = project_pixel(detection->line_end, depth, *frame->info)) end = transform_point(*p, header);

    geometry_msgs::msg::PoseStamped goal;
    goal.header = center->header;
    goal.pose.position = center->point;
    if (planar_goal_) goal.pose.position.z = 0.0;
    double yaw = 0.0;
    if (start && end) yaw = std::atan2(end->point.y - start->point.y, end->point.x - start->point.x);
    tf2::Quaternion orientation; orientation.setRPY(0.0, 0.0, yaw);
    goal.pose.orientation = tf2::toMsg(orientation);
    goal_pub_->publish(goal);
    publish_debug(header, *center, start, end);

    cv::Mat annotated = color_bridge->image.clone();
    cv::rectangle(annotated, detection->box, cv::Scalar(60, 220, 255), 2);
    cv::line(annotated, detection->line_start, detection->line_end, cv::Scalar(0, 255, 40), 2);
    cv::circle(annotated, detection->center, 5, cv::Scalar(0, 220, 255), cv::FILLED);
    cv::putText(annotated, cv::format("%.3f, %.3f, %.3f m", center->point.x, center->point.y, center->point.z), detection->center,
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    annotated_pub_->publish(*cv_bridge::CvImage(frame->color->header, "bgr8", annotated).toImageMsg());
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(logger_, *get_clock(), 2000, "Frame processing failure: %s", error.what());
    if (frame && frame->color) {
      try {
        const auto color_bridge = cv_bridge::toCvShare(frame->color, "bgr8");
        publish_status_image(frame->color, color_bridge->image, "PROCESSING ERROR");
        publish_empty_debug(frame->color->header);
      } catch (const std::exception &) {}
    }
  }
}

std::optional<YoloWeldline3DNode::Detection> YoloWeldline3DNode::infer(const cv::Mat & bgr)
{
  float scale = 1.0F; int pad_x = 0; int pad_y = 0;
  const auto input = letterbox(bgr, {network_width_, network_height_}, scale, pad_x, pad_y);
  cv::Mat output = forward_network(input);
  cv::Mat rows;
  if (output.dims == 3) {
    const int d1 = output.size[1], d2 = output.size[2];
    if (d1 < d2) rows = cv::Mat(d2, d1, CV_32F, output.ptr<float>()).clone();
    else rows = cv::Mat(d1, d2, CV_32F, output.ptr<float>()).clone();
  } else if (output.dims == 2) rows = output;
  else return std::nullopt;
  std::vector<cv::Rect> boxes; std::vector<float> scores;
  for (int i = 0; i < rows.rows; ++i) {
    const float * row = rows.ptr<float>(i); if (rows.cols < 5) continue;
    float score = row[4];
    int class_id = -1;
    if (rows.cols == 6) {
      class_id = static_cast<int>(std::round(row[5]));
    } else if (rows.cols > 6) {
      const bool has_objectness = rows.cols != 84;  // YOLOv8 COCO is [x,y,w,h,80 classes]; YOLOv5 is [x,y,w,h,obj,80 classes].
      const int class_start = has_objectness ? 5 : 4;
      const int class_count = rows.cols - class_start;
      if (class_count <= 0) continue;
      const float objectness = has_objectness ? row[4] : 1.0F;
      if (target_class_id_ >= 0) {
        if (target_class_id_ >= class_count) continue;
        class_id = target_class_id_;
        score = objectness * row[class_start + target_class_id_];
      } else {
        const auto * begin = row + class_start;
        const auto * end = row + rows.cols;
        const auto * best = std::max_element(begin, end);
        class_id = static_cast<int>(std::distance(begin, best));
        score = objectness * (*best);
      }
    }
    if (target_class_id_ >= 0 && class_id >= 0 && class_id != target_class_id_) continue;
    if (target_class_id_ > 0 && class_id < 0) continue;
    if (score < confidence_threshold_) continue;
    float x1 = row[0], y1 = row[1], x2 = row[2], y2 = row[3];
    if (x2 <= x1 || y2 <= y1) { const float cx = x1, cy = y1; x1 = cx - x2 / 2.0F; y1 = cy - y2 / 2.0F; x2 = cx + x2 / 2.0F; y2 = cy + y2 / 2.0F; }
    x1 = (x1 - static_cast<float>(pad_x)) / scale;
    y1 = (y1 - static_cast<float>(pad_y)) / scale;
    x2 = (x2 - static_cast<float>(pad_x)) / scale;
    y2 = (y2 - static_cast<float>(pad_y)) / scale;
    const int left = std::clamp(static_cast<int>(std::round(x1)), 0, bgr.cols - 1);
    const int top = std::clamp(static_cast<int>(std::round(y1)), 0, bgr.rows - 1);
    const int right = std::clamp(static_cast<int>(std::round(x2)), left + 1, bgr.cols);
    const int bottom = std::clamp(static_cast<int>(std::round(y2)), top + 1, bgr.rows);
    boxes.emplace_back(left, top, right - left, bottom - top); scores.push_back(score);
  }
  std::vector<int> kept = nms_indices(boxes, scores, nms_threshold_);
  if (kept.empty()) return std::nullopt;
  const int index = *std::max_element(kept.begin(), kept.end(), [&scores](int a, int b) { return scores[a] < scores[b]; });
  const auto & box = boxes[static_cast<size_t>(index)];
  Detection result{box, {box.x + box.width / 2, box.y + box.height / 2}, {}, {}, scores[static_cast<size_t>(index)]};
  result.line_start = {box.x, result.center.y}; result.line_end = {box.x + box.width, result.center.y};
  if (use_line_detection_) {
    cv::Mat gray, edges; cv::cvtColor(bgr(box), gray, cv::COLOR_BGR2GRAY); cv::Canny(gray, edges, 40, 120);
    std::vector<cv::Vec4i> lines; cv::HoughLinesP(edges, lines, 1, CV_PI / 180.0, 25, std::max(10, box.width / 4), 12);
    if (!lines.empty()) { const auto best = *std::max_element(lines.begin(), lines.end(), [](const auto & a, const auto & b) { return cv::norm(cv::Point(a[0] - a[2], a[1] - a[3])) < cv::norm(cv::Point(b[0] - b[2], b[1] - b[3])); }); result.line_start = {box.x + best[0], box.y + best[1]}; result.line_end = {box.x + best[2], box.y + best[3]}; result.center = (result.line_start + result.line_end) * 0.5; }
  }
  return result;
}

cv::Mat YoloWeldline3DNode::forward_network(const cv::Mat & input)
{
  std::scoped_lock lock(network_mutex_);
  if (!ort_session_) throw std::runtime_error("ONNX Runtime session is not initialized");

  const int area = network_width_ * network_height_;
  for (int y = 0; y < network_height_; ++y) {
    const auto * row = input.ptr<cv::Vec3b>(y);
    for (int x = 0; x < network_width_; ++x) {
      const auto & bgr = row[x];
      const int index = y * network_width_ + x;
      input_tensor_[static_cast<size_t>(0 * area + index)] = static_cast<float>(bgr[2]) / 255.0F;
      input_tensor_[static_cast<size_t>(1 * area + index)] = static_cast<float>(bgr[1]) / 255.0F;
      input_tensor_[static_cast<size_t>(2 * area + index)] = static_cast<float>(bgr[0]) / 255.0F;
    }
  }

  auto input_tensor = Ort::Value::CreateTensor<float>(
    ort_memory_info_, input_tensor_.data(), input_tensor_.size(), input_shape_.data(), input_shape_.size());
  const char * input_names[] = {input_name_.c_str()};
  const char * output_names[] = {output_name_.c_str()};
  auto outputs = ort_session_->Run(
    Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
  if (outputs.empty() || !outputs.front().IsTensor()) {
    throw std::runtime_error("ONNX Runtime did not return a tensor output");
  }
  const auto shape = outputs.front().GetTensorTypeAndShapeInfo().GetShape();
  if (shape.size() != 2 && shape.size() != 3) {
    throw std::runtime_error("ONNX output must be shaped [N, 5+], [1, N, 5+], or [1, 5+, N]");
  }
  const float * data = outputs.front().GetTensorData<float>();
  const int first = static_cast<int>(shape[shape.size() - 2]);
  const int second = static_cast<int>(shape[shape.size() - 1]);
  if (second >= 5) {
    return cv::Mat(first, second, CV_32F, const_cast<float *>(data)).clone();
  }
  if (first >= 5) {
    cv::Mat channels_first(first, second, CV_32F, const_cast<float *>(data));
    cv::Mat rows;
    cv::transpose(channels_first, rows);
    return rows;
  }
  throw std::runtime_error("ONNX output does not contain detection rows with at least 5 values");
}

std::optional<geometry_msgs::msg::Point> YoloWeldline3DNode::project_pixel(const cv::Point & pixel, const cv::Mat & depth, const sensor_msgs::msg::CameraInfo & info) const
{
  if (pixel.x < 0 || pixel.y < 0 || pixel.x >= depth.cols || pixel.y >= depth.rows || info.k[0] == 0.0 || info.k[4] == 0.0) return std::nullopt;
  const int radius = depth_window_ / 2; std::vector<float> samples;
  for (int y = std::max(0, pixel.y - radius); y <= std::min(depth.rows - 1, pixel.y + radius); ++y) for (int x = std::max(0, pixel.x - radius); x <= std::min(depth.cols - 1, pixel.x + radius); ++x) { const float z = depth.at<float>(y, x); if (std::isfinite(z) && z > 0.05F) samples.push_back(z); }
  if (samples.empty()) return std::nullopt;
  std::nth_element(samples.begin(), samples.begin() + static_cast<long>(samples.size() / 2), samples.end()); const double z = samples[samples.size() / 2];
  geometry_msgs::msg::Point point; point.x = (pixel.x - info.k[2]) * z / info.k[0]; point.y = (pixel.y - info.k[5]) * z / info.k[4]; point.z = z; return point;
}

std::optional<geometry_msgs::msg::PointStamped> YoloWeldline3DNode::transform_point(const geometry_msgs::msg::Point & point, const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::PointStamped source, result; source.header = header; source.point = point;
  if (output_frame_.empty() || output_frame_ == header.frame_id) return source;
  try { tf_buffer_->transform(source, result, output_frame_, tf2::durationFromSec(0.02)); return result; }
  catch (const tf2::TransformException & error) { RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 2000, "No transform %s -> %s: %s", header.frame_id.c_str(), output_frame_.c_str(), error.what()); return std::nullopt; }
}

void YoloWeldline3DNode::publish_status_image(const sensor_msgs::msg::Image::ConstSharedPtr & source, const cv::Mat & bgr, const std::string & status) const
{
  cv::Mat annotated = bgr.clone();
  cv::rectangle(annotated, cv::Rect(0, 0, annotated.cols, 40), cv::Scalar(0, 0, 0), cv::FILLED);
  const cv::Scalar color = status == "NO DETECTION" ? cv::Scalar(0, 200, 255) : cv::Scalar(0, 120, 255);
  cv::putText(annotated, status, {10, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv::LINE_AA);
  annotated_pub_->publish(*cv_bridge::CvImage(source->header, "bgr8", annotated).toImageMsg());
}

void YoloWeldline3DNode::publish_empty_debug(const std_msgs::msg::Header & header) const
{
  geometry_msgs::msg::PointStamped point;
  point.header = header;
  point.point.x = std::numeric_limits<double>::quiet_NaN();
  point.point.y = std::numeric_limits<double>::quiet_NaN();
  point.point.z = std::numeric_limits<double>::quiet_NaN();
  point_pub_->publish(point);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = header;
  clear.ns = "weldline";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  marker_pub_->publish(markers);
}

void YoloWeldline3DNode::publish_debug(const std_msgs::msg::Header &, const geometry_msgs::msg::PointStamped & center, const std::optional<geometry_msgs::msg::PointStamped> & start, const std::optional<geometry_msgs::msg::PointStamped> & end) const
{
  point_pub_->publish(center);
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker sphere; sphere.header = center.header; sphere.ns = "weldline"; sphere.id = 0; sphere.type = visualization_msgs::msg::Marker::SPHERE; sphere.action = visualization_msgs::msg::Marker::ADD; sphere.pose.position = center.point; sphere.pose.orientation.w = 1.0; sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05; sphere.color.r = 1.0F; sphere.color.g = 0.8F; sphere.color.a = 1.0F; sphere.lifetime = rclcpp::Duration::from_seconds(0.2); markers.markers.push_back(sphere);
  if (start && end) { visualization_msgs::msg::Marker line = sphere; line.id = 1; line.type = visualization_msgs::msg::Marker::LINE_STRIP; line.scale.x = 0.012; line.color.r = 0.0F; line.color.g = 1.0F; line.color.b = 0.25F; line.points = {start->point, end->point}; markers.markers.push_back(line); }
  marker_pub_->publish(markers);
}
}  // namespace weldline_reflectivity_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<weldline_reflectivity_detector::YoloWeldline3DNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("yolo_weldline_3d_node"), "Startup failed: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
