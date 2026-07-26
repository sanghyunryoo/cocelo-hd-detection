#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "weldline_reflectivity_detector/scenario_planner.hpp"
#include "weldline_reflectivity_detector/point_cloud_xyz.hpp"
#include "weldline_reflectivity_detector/wall_alignment.hpp"

namespace weldline_reflectivity_detector
{
class ScenarioCommanderNode final : public rclcpp::Node
{
public:
  explicit ScenarioCommanderNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct CachedMap
  {
    std_msgs::msg::Header header;
    std::vector<Point3D> points;
  };

  void on_global_map(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void process_wall_alignment();
  void publish_invalid(const std::string & reason);
  void publish_estimate(const WallEstimate & estimate);
  void initialize_scenario(const std::string & scenario_file);
  void on_detector_goal(geometry_msgs::msg::PoseStamped::ConstSharedPtr message);
  void process_scenario_control();
  bool lookup_robot_pose(Pose2D & pose, std::string & reason);
  void publish_planner_output(const PlannerOutput & output);

  std::string global_map_topic_;
  std::string reference_link_;
  std::string scenario_file_;
  double processing_rate_hz_{5.0};
  double tf_timeout_sec_{0.05};
  bool use_cloud_stamp_for_tf_{false};
  double minimum_range_{0.25};
  double search_radius_{3.0};
  double minimum_height_{-2.0};
  double maximum_height_{2.0};
  double xy_voxel_size_{0.04};
  double minimum_vertical_span_{0.35};
  int minimum_voxel_points_{3};
  int point_stride_{1};
  double smoothing_alpha_{1.0};
  WallEstimatorConfig estimator_config_;

  std::mutex map_mutex_;
  std::shared_ptr<const CachedMap> cached_map_;
  std::optional<double> tracked_angle_rad_;
  std::optional<double> filtered_angle_rad_;
  bool latest_wall_valid_{false};
  double latest_wall_angle_rad_{0.0};
  std::optional<rclcpp::Time> latest_wall_update_;
  std::string last_invalid_reason_;

  std::unique_ptr<ScenarioPlanner> scenario_planner_;
  std::deque<Pose2D> detector_goal_candidates_;
  std::optional<PlannerState> last_planner_state_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detector_goal_sub_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::TimerBase::SharedPtr scenario_control_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr metrics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scenario_status_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("scenario_commander_node")};
};
}  // namespace weldline_reflectivity_detector
