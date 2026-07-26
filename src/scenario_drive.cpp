#include "weldline_reflectivity_detector/scenario_commander_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace weldline_reflectivity_detector
{
namespace
{
constexpr double kRadiansToDegrees = 57.2957795130823208768;

double monotonic_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double mean_yaw(const std::deque<Pose2D> & poses, const YawMode mode)
{
  double sine_sum = 0.0;
  double cosine_sum = 0.0;
  const double multiplier = mode == YawMode::kParallel ? 2.0 : 1.0;
  for (const auto & pose : poses) {
    sine_sum += std::sin(multiplier * pose.yaw_rad);
    cosine_sum += std::cos(multiplier * pose.yaw_rad);
  }
  const double mean = std::atan2(sine_sum, cosine_sum) / multiplier;
  return mode == YawMode::kParallel ?
         normalize_parallel_angle(mean) : normalize_angle(mean);
}

double yaw_difference(const double lhs, const double rhs, const YawMode mode)
{
  return mode == YawMode::kParallel ?
         normalize_parallel_angle(lhs - rhs) : normalize_angle(lhs - rhs);
}
}  // namespace

void ScenarioCommanderNode::initialize_scenario(const std::string & scenario_file)
{
  scenario_planner_ = std::make_unique<ScenarioPlanner>(
    load_scenario_file(scenario_file));
  const auto & scenario = scenario_planner_->scenario();
  if (scenario.control.robot_frame != reference_link_) {
    throw std::invalid_argument(
            "scenario.control.robot_frame must match commander reference_link");
  }

  const auto command_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  fsm_command_pub_ = create_publisher<std_msgs::msg::String>(
    scenario.control.command_topic, command_qos);
  scenario_status_pub_ = create_publisher<std_msgs::msg::String>(
    scenario.control.status_topic, rclcpp::QoS(10).reliable());
  detector_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    scenario.detector.goal_topic, rclcpp::QoS(10).reliable(),
    std::bind(&ScenarioCommanderNode::on_detector_goal, this, std::placeholders::_1));

  const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / scenario.control.rate_hz));
  scenario_control_timer_ = create_wall_timer(
    control_period, std::bind(&ScenarioCommanderNode::process_scenario_control, this));

  RCLCPP_INFO(
    logger_,
    "Scenario loaded: file=%s frame=%s robot=%s waypoints=%zu detector_model=%s cmd=%s @ %.1f Hz",
    scenario_file.c_str(), scenario.frame_id.c_str(), scenario.control.robot_frame.c_str(),
    scenario.waypoints.size(), scenario.detector.model_path.c_str(),
    scenario.control.command_topic.c_str(), scenario.control.rate_hz);
}

void ScenarioCommanderNode::on_detector_goal(
  geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
{
  if (scenario_planner_->detector_waypoint_resolved()) {
    return;
  }
  if (message->header.frame_id.empty()) {
    RCLCPP_WARN(logger_, "Rejected detector waypoint with empty frame_id");
    return;
  }

  const auto & scenario = scenario_planner_->scenario();
  const rclcpp::Time goal_stamp(message->header.stamp);
  if (goal_stamp.nanoseconds() <= 0) {
    RCLCPP_WARN(logger_, "Rejected detector waypoint with zero timestamp");
    return;
  }
  const double goal_age_sec = (now() - goal_stamp).seconds();
  if (goal_age_sec < -0.05 || goal_age_sec > scenario.detector.max_goal_age_sec) {
    RCLCPP_WARN_THROTTLE(
      logger_, *get_clock(), 2000, "Rejected stale/future detector waypoint: age=%.3f s",
      goal_age_sec);
    return;
  }

  geometry_msgs::msg::PoseStamped transformed = *message;
  try {
    if (message->header.frame_id != scenario.frame_id) {
      const auto transform = tf_buffer_->lookupTransform(
        scenario.frame_id, message->header.frame_id, rclcpp::Time(message->header.stamp),
        rclcpp::Duration::from_seconds(scenario.control.tf_timeout_sec));
      tf2::doTransform(*message, transformed, transform);
    }
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      logger_, *get_clock(), 2000, "Cannot transform detector waypoint %s -> %s: %s",
      message->header.frame_id.c_str(), scenario.frame_id.c_str(), error.what());
    return;
  }

  const auto & orientation = transformed.pose.orientation;
  const tf2::Quaternion quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w);
  if (quaternion.length2() <= std::numeric_limits<double>::epsilon() ||
    !std::isfinite(transformed.pose.position.x) ||
    !std::isfinite(transformed.pose.position.y))
  {
    RCLCPP_WARN(logger_, "Rejected detector waypoint with invalid pose");
    return;
  }
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion.normalized()).getRPY(roll, pitch, yaw);
  detector_goal_candidates_.push_back(
    {transformed.pose.position.x, transformed.pose.position.y, yaw});

  const auto sample_count = static_cast<std::size_t>(scenario.detector.stable_samples);
  while (detector_goal_candidates_.size() > sample_count) {
    detector_goal_candidates_.pop_front();
  }
  if (detector_goal_candidates_.size() < sample_count) {
    return;
  }

  const double mean_x = std::accumulate(
    detector_goal_candidates_.begin(), detector_goal_candidates_.end(), 0.0,
    [](const double sum, const Pose2D & pose) {return sum + pose.x;}) /
    static_cast<double>(sample_count);
  const double mean_y = std::accumulate(
    detector_goal_candidates_.begin(), detector_goal_candidates_.end(), 0.0,
    [](const double sum, const Pose2D & pose) {return sum + pose.y;}) /
    static_cast<double>(sample_count);
  const auto yaw_mode = scenario.waypoints.front().yaw_mode;
  const double averaged_yaw = mean_yaw(detector_goal_candidates_, yaw_mode);

  double maximum_position_spread = 0.0;
  double maximum_yaw_spread = 0.0;
  for (const auto & candidate : detector_goal_candidates_) {
    maximum_position_spread = std::max(
      maximum_position_spread, std::hypot(candidate.x - mean_x, candidate.y - mean_y));
    maximum_yaw_spread = std::max(
      maximum_yaw_spread,
      std::abs(yaw_difference(candidate.yaw_rad, averaged_yaw, yaw_mode)));
  }
  if (maximum_position_spread > scenario.detector.max_position_spread_m ||
    maximum_yaw_spread > scenario.detector.max_yaw_spread_rad)
  {
    RCLCPP_DEBUG(
      logger_, "Detector waypoint not stable: position spread=%.3f m yaw spread=%.3f deg",
      maximum_position_spread, maximum_yaw_spread * kRadiansToDegrees);
    return;
  }

  scenario_planner_->set_detector_waypoint({mean_x, mean_y, averaged_yaw});
  detector_goal_sub_.reset();
  RCLCPP_INFO(
    logger_,
    "Locked detector waypoint from %zu stable samples: x=%.4f y=%.4f yaw=%.3f deg",
    sample_count, mean_x, mean_y, averaged_yaw * kRadiansToDegrees);
}

bool ScenarioCommanderNode::lookup_robot_pose(Pose2D & pose, std::string & reason)
{
  const auto & scenario = scenario_planner_->scenario();
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      scenario.frame_id, scenario.control.robot_frame, tf2::TimePointZero,
      tf2::durationFromSec(scenario.control.tf_timeout_sec));
  } catch (const tf2::TransformException & error) {
    reason = std::string("missing ") + scenario.frame_id + " -> " +
      scenario.control.robot_frame + " TF: " + error.what();
    return false;
  }

  const rclcpp::Time stamp(transform.header.stamp);
  if (stamp.nanoseconds() <= 0) {
    reason = "map-to-robot TF has zero timestamp";
    return false;
  }
  const double age_sec = (now() - stamp).seconds();
  if (age_sec < -0.05 || age_sec > scenario.control.max_tf_age_sec) {
    std::ostringstream stream;
    stream << "map-to-robot TF is stale/future by " << std::fixed << std::setprecision(3) <<
      age_sec << " s";
    reason = stream.str();
    return false;
  }

  const auto & rotation = transform.transform.rotation;
  const tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
  if (quaternion.length2() <= std::numeric_limits<double>::epsilon()) {
    reason = "map-to-robot TF has invalid rotation";
    return false;
  }
  double roll = 0.0;
  double pitch = 0.0;
  tf2::Matrix3x3(quaternion.normalized()).getRPY(roll, pitch, pose.yaw_rad);
  pose.x = transform.transform.translation.x;
  pose.y = transform.transform.translation.y;
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw_rad)) {
    reason = "map-to-robot TF contains non-finite pose";
    return false;
  }
  return true;
}

void ScenarioCommanderNode::process_scenario_control()
{
  const auto & scenario = scenario_planner_->scenario();
  PlannerInput input;
  std::string pose_reason;
  input.pose_valid = lookup_robot_pose(input.robot_pose, pose_reason);
  input.monotonic_time_sec = monotonic_seconds();

  if (latest_wall_valid_ && latest_wall_update_) {
    const double wall_age_sec = (now() - *latest_wall_update_).seconds();
    input.wall_valid =
      wall_age_sec >= 0.0 && wall_age_sec <= scenario.control.max_wall_age_sec;
    input.wall_error_rad = latest_wall_angle_rad_;
  }

  auto output = scenario_planner_->update(input);
  if (!input.pose_valid && output.state == PlannerState::kWaitingPose) {
    output.detail = pose_reason;
  } else if (latest_wall_valid_ && !input.wall_valid &&
    output.state == PlannerState::kWaitingWall)
  {
    output.detail = "wall alignment estimate is stale";
  }
  publish_planner_output(output);
}

void ScenarioCommanderNode::publish_planner_output(const PlannerOutput & output)
{
  std_msgs::msg::String command;
  command.data = format_fsm_command(output.vx, output.vy, output.wz);
  fsm_command_pub_->publish(command);

  const auto & scenario = scenario_planner_->scenario();
  std::ostringstream status_stream;
  status_stream << "state=" << planner_state_name(output.state) <<
    " waypoint=" << (output.waypoint_index + 1U) << "/" << scenario.waypoints.size() <<
    " id=" << output.waypoint_id << std::fixed << std::setprecision(4) <<
    " distance_m=" << output.distance_error_m <<
    " yaw_error_deg=" << output.yaw_error_rad * kRadiansToDegrees <<
    " wall_error_deg=" << output.wall_error_rad * kRadiansToDegrees <<
    " command=\"" << command.data << "\" detail=\"" << output.detail << '"';
  std_msgs::msg::String status;
  status.data = status_stream.str();
  scenario_status_pub_->publish(status);

  if (!last_planner_state_ || *last_planner_state_ != output.state) {
    RCLCPP_INFO(logger_, "Scenario: %s", status.data.c_str());
    last_planner_state_ = output.state;
  }
}
}  // namespace weldline_reflectivity_detector
