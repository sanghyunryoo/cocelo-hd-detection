#include "weldline_reflectivity_detector/scenario_planner.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace weldline_reflectivity_detector
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;

void require_map(const YAML::Node & node, const std::string & context)
{
  if (!node || !node.IsMap()) {
    throw std::invalid_argument(context + " must be a YAML map");
  }
}

void reject_unknown_keys(
  const YAML::Node & node, const std::set<std::string> & allowed,
  const std::string & context)
{
  require_map(node, context);
  for (const auto & item : node) {
    const auto key = item.first.as<std::string>();
    if (allowed.count(key) == 0U) {
      throw std::invalid_argument(context + " contains unknown key '" + key + "'");
    }
  }
}

template<typename T>
T required(const YAML::Node & node, const std::string & key, const std::string & context)
{
  if (!node[key]) {
    throw std::invalid_argument(context + " is missing required key '" + key + "'");
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            context + "." + key + " has the wrong type: " + error.what());
  }
}

void require_finite(const double value, const std::string & name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " must be finite");
  }
}

double positive(const double value, const std::string & name)
{
  require_finite(value, name);
  if (value <= 0.0) {
    throw std::invalid_argument(name + " must be greater than zero");
  }
  return value;
}

double nonnegative(const double value, const std::string & name)
{
  require_finite(value, name);
  if (value < 0.0) {
    throw std::invalid_argument(name + " must be non-negative");
  }
  return value;
}

double waypoint_yaw_error(const ScenarioWaypoint & waypoint, const double robot_yaw)
{
  const double error = waypoint.pose.yaw_rad - robot_yaw;
  return waypoint.yaw_mode == YawMode::kParallel ?
         normalize_parallel_angle(error) : normalize_angle(error);
}

double clamp_angular(const double value, const double maximum)
{
  return std::clamp(value, -maximum, maximum);
}
}  // namespace

double normalize_angle(double angle_rad)
{
  while (angle_rad >= kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double normalize_parallel_angle(const double angle_rad)
{
  double normalized = normalize_angle(angle_rad);
  if (normalized >= kPi / 2.0) {
    normalized -= kPi;
  } else if (normalized < -kPi / 2.0) {
    normalized += kPi;
  }
  return normalized;
}

ScenarioDefinition load_scenario_file(const std::string & path)
{
  if (path.empty()) {
    throw std::invalid_argument("scenario_file must not be empty");
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument("Cannot load scenario file '" + path + "': " + error.what());
  }

  reject_unknown_keys(
    root, {"version", "frame_id", "detector", "control", "waypoints"}, "scenario");

  ScenarioDefinition scenario;
  scenario.version = required<int>(root, "version", "scenario");
  if (scenario.version != 1) {
    throw std::invalid_argument("scenario.version must be exactly 1");
  }
  scenario.frame_id = required<std::string>(root, "frame_id", "scenario");
  if (scenario.frame_id.empty()) {
    throw std::invalid_argument("scenario.frame_id must not be empty");
  }

  const auto detector = root["detector"];
  reject_unknown_keys(
    detector,
    {"model", "goal_topic", "stable_samples", "max_position_spread_m",
      "max_yaw_spread_deg", "max_goal_age_sec"},
    "scenario.detector");
  const auto model = required<std::string>(detector, "model", "scenario.detector");
  std::filesystem::path model_path(model);
  if (model_path.is_relative()) {
    model_path = std::filesystem::path(path).parent_path() / model_path;
  }
  model_path = std::filesystem::absolute(model_path).lexically_normal();
  if (!std::filesystem::is_regular_file(model_path)) {
    throw std::invalid_argument(
            "scenario.detector.model does not exist: " + model_path.string());
  }
  scenario.detector.model_path = model_path.string();
  scenario.detector.goal_topic =
    required<std::string>(detector, "goal_topic", "scenario.detector");
  scenario.detector.stable_samples =
    required<int>(detector, "stable_samples", "scenario.detector");
  scenario.detector.max_position_spread_m = positive(
    required<double>(detector, "max_position_spread_m", "scenario.detector"),
    "scenario.detector.max_position_spread_m");
  scenario.detector.max_yaw_spread_rad = positive(
    required<double>(detector, "max_yaw_spread_deg", "scenario.detector") *
    kDegreesToRadians, "scenario.detector.max_yaw_spread_deg");
  scenario.detector.max_goal_age_sec = positive(
    required<double>(detector, "max_goal_age_sec", "scenario.detector"),
    "scenario.detector.max_goal_age_sec");
  if (scenario.detector.goal_topic.empty() || scenario.detector.stable_samples < 2 ||
    scenario.detector.stable_samples > 100)
  {
    throw std::invalid_argument(
            "detector goal_topic must not be empty and stable_samples must be in [2, 100]");
  }

  const auto control = root["control"];
  reject_unknown_keys(
    control,
    {"enabled", "rate_hz", "command_topic", "status_topic", "robot_frame",
      "tf_timeout_sec", "max_tf_age_sec", "max_wall_age_sec", "position_kp",
      "max_linear_speed_mps", "wall_alignment_kp", "waypoint_yaw_kp",
      "max_angular_speed_rps", "wall_motion_gate_deg",
      "yaw_wall_consistency_deg"},
    "scenario.control");
  scenario.control.enabled = required<bool>(control, "enabled", "scenario.control");
  scenario.control.rate_hz = positive(
    required<double>(control, "rate_hz", "scenario.control"), "scenario.control.rate_hz");
  scenario.control.command_topic =
    required<std::string>(control, "command_topic", "scenario.control");
  scenario.control.status_topic =
    required<std::string>(control, "status_topic", "scenario.control");
  scenario.control.robot_frame =
    required<std::string>(control, "robot_frame", "scenario.control");
  scenario.control.tf_timeout_sec = nonnegative(
    required<double>(control, "tf_timeout_sec", "scenario.control"),
    "scenario.control.tf_timeout_sec");
  scenario.control.max_tf_age_sec = positive(
    required<double>(control, "max_tf_age_sec", "scenario.control"),
    "scenario.control.max_tf_age_sec");
  scenario.control.max_wall_age_sec = positive(
    required<double>(control, "max_wall_age_sec", "scenario.control"),
    "scenario.control.max_wall_age_sec");
  scenario.control.position_kp = positive(
    required<double>(control, "position_kp", "scenario.control"),
    "scenario.control.position_kp");
  scenario.control.max_linear_speed_mps = positive(
    required<double>(control, "max_linear_speed_mps", "scenario.control"),
    "scenario.control.max_linear_speed_mps");
  scenario.control.wall_alignment_kp = positive(
    required<double>(control, "wall_alignment_kp", "scenario.control"),
    "scenario.control.wall_alignment_kp");
  scenario.control.waypoint_yaw_kp = positive(
    required<double>(control, "waypoint_yaw_kp", "scenario.control"),
    "scenario.control.waypoint_yaw_kp");
  scenario.control.max_angular_speed_rps = positive(
    required<double>(control, "max_angular_speed_rps", "scenario.control"),
    "scenario.control.max_angular_speed_rps");
  scenario.control.wall_motion_gate_rad = positive(
    required<double>(control, "wall_motion_gate_deg", "scenario.control") *
    kDegreesToRadians, "scenario.control.wall_motion_gate_deg");
  scenario.control.yaw_wall_consistency_rad = positive(
    required<double>(control, "yaw_wall_consistency_deg", "scenario.control") *
    kDegreesToRadians, "scenario.control.yaw_wall_consistency_deg");
  if (scenario.control.rate_hz > 100.0 || scenario.control.command_topic.empty() ||
    scenario.control.status_topic.empty() || scenario.control.robot_frame.empty())
  {
    throw std::invalid_argument(
            "control rate_hz must be <= 100 and topic/frame names must not be empty");
  }

  const auto waypoints = root["waypoints"];
  if (!waypoints || !waypoints.IsSequence() || waypoints.size() < 2U) {
    throw std::invalid_argument("scenario.waypoints must contain detector + fixed waypoints");
  }

  std::set<std::string> waypoint_ids;
  for (std::size_t index = 0U; index < waypoints.size(); ++index) {
    const auto node = waypoints[index];
    const std::string context = "scenario.waypoints[" + std::to_string(index) + "]";
    reject_unknown_keys(
      node,
      {"id", "source", "x", "y", "yaw_deg", "yaw_mode", "position_tolerance_m",
        "yaw_tolerance_deg", "wall_tolerance_deg", "hold_time_sec"},
      context);

    ScenarioWaypoint waypoint;
    waypoint.id = required<std::string>(node, "id", context);
    if (waypoint.id.empty() || !waypoint_ids.insert(waypoint.id).second) {
      throw std::invalid_argument(context + ".id must be non-empty and unique");
    }
    const auto source = required<std::string>(node, "source", context);
    if (source == "detector") {
      waypoint.source = WaypointSource::kDetector;
    } else if (source == "fixed") {
      waypoint.source = WaypointSource::kFixed;
    } else {
      throw std::invalid_argument(context + ".source must be detector or fixed");
    }
    const auto yaw_mode = required<std::string>(node, "yaw_mode", context);
    if (yaw_mode == "directional") {
      waypoint.yaw_mode = YawMode::kDirectional;
    } else if (yaw_mode == "parallel") {
      waypoint.yaw_mode = YawMode::kParallel;
    } else {
      throw std::invalid_argument(context + ".yaw_mode must be directional or parallel");
    }
    waypoint.position_tolerance_m = positive(
      required<double>(node, "position_tolerance_m", context),
      context + ".position_tolerance_m");
    waypoint.yaw_tolerance_rad = positive(
      required<double>(node, "yaw_tolerance_deg", context) * kDegreesToRadians,
      context + ".yaw_tolerance_deg");
    waypoint.wall_tolerance_rad = positive(
      required<double>(node, "wall_tolerance_deg", context) * kDegreesToRadians,
      context + ".wall_tolerance_deg");
    waypoint.hold_time_sec = positive(
      required<double>(node, "hold_time_sec", context), context + ".hold_time_sec");

    if (waypoint.source == WaypointSource::kDetector) {
      if (index != 0U) {
        throw std::invalid_argument("Only the first waypoint may use source: detector");
      }
      if (node["x"] || node["y"] || node["yaw_deg"]) {
        throw std::invalid_argument(
                "Detector waypoint pose must come from the detector, not fixed x/y/yaw");
      }
    } else {
      waypoint.pose.x = required<double>(node, "x", context);
      waypoint.pose.y = required<double>(node, "y", context);
      waypoint.pose.yaw_rad = required<double>(node, "yaw_deg", context) *
        kDegreesToRadians;
      require_finite(waypoint.pose.x, context + ".x");
      require_finite(waypoint.pose.y, context + ".y");
      require_finite(waypoint.pose.yaw_rad, context + ".yaw_deg");
      waypoint.pose.yaw_rad = normalize_angle(waypoint.pose.yaw_rad);
      waypoint.resolved = true;
    }
    if (waypoint.wall_tolerance_rad > scenario.control.wall_motion_gate_rad) {
      throw std::invalid_argument(
              context + ".wall_tolerance_deg must not exceed control.wall_motion_gate_deg");
    }
    scenario.waypoints.push_back(std::move(waypoint));
  }

  if (scenario.waypoints.front().source != WaypointSource::kDetector) {
    throw std::invalid_argument("The first waypoint must use source: detector");
  }
  return scenario;
}

const char * planner_state_name(const PlannerState state)
{
  switch (state) {
    case PlannerState::kDisabled: return "DISABLED";
    case PlannerState::kWaitingDetector: return "WAITING_DETECTOR";
    case PlannerState::kWaitingPose: return "WAITING_POSE";
    case PlannerState::kWaitingWall: return "WAITING_WALL";
    case PlannerState::kAligningWall: return "ALIGNING_WALL";
    case PlannerState::kDriving: return "DRIVING";
    case PlannerState::kAligningWaypointYaw: return "ALIGNING_WAYPOINT_YAW";
    case PlannerState::kHeadingConflict: return "HEADING_CONFLICT";
    case PlannerState::kVerifyingWaypoint: return "VERIFYING_WAYPOINT";
    case PlannerState::kAdvancing: return "ADVANCING";
    case PlannerState::kHoldingFinal: return "HOLDING_FINAL";
  }
  return "UNKNOWN";
}

ScenarioPlanner::ScenarioPlanner(ScenarioDefinition scenario)
: scenario_(std::move(scenario))
{
  if (scenario_.waypoints.empty()) {
    throw std::invalid_argument("ScenarioPlanner requires at least one waypoint");
  }
}

void ScenarioPlanner::set_detector_waypoint(const Pose2D & pose)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw_rad)) {
    throw std::invalid_argument("Detector waypoint pose must be finite");
  }
  auto & waypoint = scenario_.waypoints.front();
  if (waypoint.source != WaypointSource::kDetector) {
    throw std::logic_error("Scenario first waypoint is not detector-driven");
  }
  waypoint.pose = pose;
  waypoint.pose.yaw_rad = normalize_angle(waypoint.pose.yaw_rad);
  waypoint.resolved = true;
}

bool ScenarioPlanner::detector_waypoint_resolved() const
{
  return scenario_.waypoints.front().resolved;
}

const ScenarioDefinition & ScenarioPlanner::scenario() const
{
  return scenario_;
}

std::size_t ScenarioPlanner::current_waypoint_index() const
{
  return current_waypoint_;
}

PlannerOutput ScenarioPlanner::stopped_output(
  const PlannerState state, const std::string & detail) const
{
  PlannerOutput output;
  output.state = state;
  output.waypoint_index = current_waypoint_;
  output.waypoint_id = scenario_.waypoints[current_waypoint_].id;
  output.detail = detail;
  return output;
}

PlannerOutput ScenarioPlanner::update(const PlannerInput & input)
{
  if (!scenario_.control.enabled) {
    within_tolerance_since_sec_.reset();
    return stopped_output(PlannerState::kDisabled, "scenario control disabled");
  }
  if (!detector_waypoint_resolved()) {
    within_tolerance_since_sec_.reset();
    return stopped_output(
      PlannerState::kWaitingDetector, "waiting for stable detector waypoint");
  }
  if (!input.pose_valid) {
    within_tolerance_since_sec_.reset();
    return stopped_output(PlannerState::kWaitingPose, "map-to-robot pose unavailable");
  }
  if (!input.wall_valid || !std::isfinite(input.wall_error_rad)) {
    within_tolerance_since_sec_.reset();
    return stopped_output(PlannerState::kWaitingWall, "fresh wall alignment unavailable");
  }

  const auto & waypoint = scenario_.waypoints[current_waypoint_];
  PlannerOutput output;
  output.waypoint_index = current_waypoint_;
  output.waypoint_id = waypoint.id;
  output.wall_error_rad = normalize_parallel_angle(input.wall_error_rad);

  const double dx_map = waypoint.pose.x - input.robot_pose.x;
  const double dy_map = waypoint.pose.y - input.robot_pose.y;
  output.distance_error_m = std::hypot(dx_map, dy_map);
  output.yaw_error_rad = waypoint_yaw_error(waypoint, input.robot_pose.yaw_rad);

  const auto wall_rotation = [&]() {
      output.wz = clamp_angular(
        scenario_.control.wall_alignment_kp * output.wall_error_rad,
        scenario_.control.max_angular_speed_rps);
    };

  if (std::abs(output.wall_error_rad) > scenario_.control.wall_motion_gate_rad) {
    within_tolerance_since_sec_.reset();
    output.state = PlannerState::kAligningWall;
    output.detail = "translation gated until wall alignment is restored";
    wall_rotation();
    return output;
  }

  if (output.distance_error_m > waypoint.position_tolerance_m) {
    within_tolerance_since_sec_.reset();
    const double cosine = std::cos(input.robot_pose.yaw_rad);
    const double sine = std::sin(input.robot_pose.yaw_rad);
    const double dx_robot = cosine * dx_map + sine * dy_map;
    const double dy_robot = -sine * dx_map + cosine * dy_map;
    output.vx = scenario_.control.position_kp * dx_robot;
    output.vy = scenario_.control.position_kp * dy_robot;
    const double speed = std::hypot(output.vx, output.vy);
    if (speed > scenario_.control.max_linear_speed_mps) {
      const double scale = scenario_.control.max_linear_speed_mps / speed;
      output.vx *= scale;
      output.vy *= scale;
    }
    wall_rotation();
    output.state = PlannerState::kDriving;
    output.detail = "tracking waypoint position with continuous wall correction";
    return output;
  }

  if (std::abs(output.wall_error_rad) > waypoint.wall_tolerance_rad) {
    within_tolerance_since_sec_.reset();
    output.state = PlannerState::kAligningWall;
    output.detail = "position reached; enforcing strict wall tolerance";
    wall_rotation();
    return output;
  }

  if (std::abs(output.yaw_error_rad) > waypoint.yaw_tolerance_rad) {
    within_tolerance_since_sec_.reset();
    const double mismatch = std::abs(
      normalize_parallel_angle(output.yaw_error_rad - output.wall_error_rad));
    if (mismatch > scenario_.control.yaw_wall_consistency_rad) {
      output.state = PlannerState::kHeadingConflict;
      output.detail = "scenario yaw is inconsistent with the measured wall heading";
      return output;
    }
    output.state = PlannerState::kAligningWaypointYaw;
    output.detail = "position reached; enforcing strict scenario yaw";
    output.wz = clamp_angular(
      scenario_.control.waypoint_yaw_kp * output.yaw_error_rad,
      scenario_.control.max_angular_speed_rps);
    return output;
  }

  if (!within_tolerance_since_sec_) {
    within_tolerance_since_sec_ = input.monotonic_time_sec;
  }
  const double held_sec = std::max(
    0.0, input.monotonic_time_sec - *within_tolerance_since_sec_);
  if (held_sec < waypoint.hold_time_sec) {
    output.state = PlannerState::kVerifyingWaypoint;
    output.detail = "all strict tolerances satisfied; verifying continuous hold";
    return output;
  }

  const bool is_final = current_waypoint_ + 1U >= scenario_.waypoints.size();
  if (is_final) {
    output.state = PlannerState::kHoldingFinal;
    output.detail = "final waypoint held; continuing closed-loop monitoring";
    return output;
  }

  ++current_waypoint_;
  within_tolerance_since_sec_.reset();
  output.state = PlannerState::kAdvancing;
  output.waypoint_index = current_waypoint_;
  output.waypoint_id = scenario_.waypoints[current_waypoint_].id;
  output.detail = "strict waypoint hold passed; advancing to next waypoint";
  return output;
}

std::string format_fsm_command(const double vx, const double vy, const double wz)
{
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
    throw std::invalid_argument("FSM command components must be finite");
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << "cmd " << vx << ' ' << vy << ' ' << wz;
  return stream.str();
}
}  // namespace weldline_reflectivity_detector
