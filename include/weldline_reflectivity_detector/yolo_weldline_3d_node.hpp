#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/dnn.hpp>

namespace weldline_reflectivity_detector
{
class YoloWeldline3DNode final : public rclcpp::Node
{
public:
  explicit YoloWeldline3DNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~YoloWeldline3DNode() override;

private:
  struct TensorRtContext;
  struct Detection { cv::Rect box; cv::Point center; cv::Point line_start; cv::Point line_end; float confidence; };
  struct FrameBundle {
    sensor_msgs::msg::Image::ConstSharedPtr color;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  };
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  void on_synchronized_frame(const sensor_msgs::msg::Image::ConstSharedPtr & color,
                             const sensor_msgs::msg::Image::ConstSharedPtr & depth,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);
  void process_latest_frame();
  std::optional<Detection> infer(const cv::Mat & bgr);
  cv::Mat forward_network(const cv::Mat & input);
  std::optional<geometry_msgs::msg::Point> project_pixel(const cv::Point & pixel, const cv::Mat & depth,
                                                         const sensor_msgs::msg::CameraInfo & info) const;
  std::optional<geometry_msgs::msg::PointStamped> transform_point(const geometry_msgs::msg::Point & point,
                                                                    const std_msgs::msg::Header & header);
  void publish_status_image(const sensor_msgs::msg::Image::ConstSharedPtr & source, const cv::Mat & bgr,
                            const std::string & status) const;
  void publish_empty_debug(const std_msgs::msg::Header & header) const;
  void publish_debug(const std_msgs::msg::Header & header,
                     const geometry_msgs::msg::PointStamped & center,
                     const std::optional<geometry_msgs::msg::PointStamped> & start,
                     const std::optional<geometry_msgs::msg::PointStamped> & end) const;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  mutable std::mutex frame_mutex_;
  std::optional<FrameBundle> latest_frame_;
  rclcpp::TimerBase::SharedPtr processing_timer_;

  cv::dnn::Net network_;
  std::unique_ptr<TensorRtContext> tensorrt_;
  mutable std::mutex network_mutex_;
  std::string inference_backend_;
  int network_width_{640};
  int network_height_{640};
  float confidence_threshold_{0.40F};
  float nms_threshold_{0.25F};
  int depth_window_{7};
  bool use_line_detection_{true};
  bool planar_goal_{false};
  std::string output_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("yolo_weldline_3d_node")};
};
}  // namespace weldline_reflectivity_detector
