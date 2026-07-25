#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace weldline_reflectivity_detector
{
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw_rad{0.0};
};

enum class WaypointSource
{
  kDetector,
  kFixed
};

enum class YawMode
{
  kDirectional,
  kParallel
};

struct ScenarioWaypoint
{
  std::string id;
  WaypointSource source{WaypointSource::kFixed};
  YawMode yaw_mode{YawMode::kParallel};
  Pose2D pose;
  bool resolved{false};
  double position_tolerance_m{0.05};
  double yaw_tolerance_rad{0.01};
  double wall_tolerance_rad{0.01};
  double hold_time_sec{1.0};
};

struct DetectorWaypointConfig
{
  std::string model_path;
  std::string goal_topic;
  int stable_samples{5};
  double max_position_spread_m{0.03};
  double max_yaw_spread_rad{0.03};
  double max_goal_age_sec{0.5};
};

struct ScenarioControlConfig
{
  bool enabled{true};
  double rate_hz{20.0};
  std::string command_topic{"/fsm_cmd"};
  std::string status_topic{"/commander/scenario/status"};
  std::string robot_frame{"base_link"};
  double tf_timeout_sec{0.05};
  double max_tf_age_sec{0.5};
  double max_wall_age_sec{0.75};
  double position_kp{0.6};
  double max_linear_speed_mps{0.20};
  double wall_alignment_kp{1.5};
  double waypoint_yaw_kp{1.2};
  double max_angular_speed_rps{0.45};
  double wall_motion_gate_rad{0.035};
  double yaw_wall_consistency_rad{0.087};
};

struct ScenarioDefinition
{
  int version{1};
  std::string frame_id{"map"};
  DetectorWaypointConfig detector;
  ScenarioControlConfig control;
  std::vector<ScenarioWaypoint> waypoints;
};

ScenarioDefinition load_scenario_file(const std::string & path);

enum class PlannerState
{
  kDisabled,
  kWaitingDetector,
  kWaitingPose,
  kWaitingWall,
  kAligningWall,
  kDriving,
  kAligningWaypointYaw,
  kHeadingConflict,
  kVerifyingWaypoint,
  kAdvancing,
  kHoldingFinal
};

const char * planner_state_name(PlannerState state);

struct PlannerInput
{
  bool pose_valid{false};
  Pose2D robot_pose;
  bool wall_valid{false};
  double wall_error_rad{0.0};
  double monotonic_time_sec{0.0};
};

struct PlannerOutput
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  PlannerState state{PlannerState::kWaitingDetector};
  std::size_t waypoint_index{0U};
  std::string waypoint_id;
  double distance_error_m{0.0};
  double yaw_error_rad{0.0};
  double wall_error_rad{0.0};
  std::string detail;
};

class ScenarioPlanner
{
public:
  explicit ScenarioPlanner(ScenarioDefinition scenario);

  void set_detector_waypoint(const Pose2D & pose);
  bool detector_waypoint_resolved() const;
  const ScenarioDefinition & scenario() const;
  std::size_t current_waypoint_index() const;
  PlannerOutput update(const PlannerInput & input);

private:
  PlannerOutput stopped_output(PlannerState state, const std::string & detail) const;

  ScenarioDefinition scenario_;
  std::size_t current_waypoint_{0U};
  std::optional<double> within_tolerance_since_sec_;
};

double normalize_angle(double angle_rad);
double normalize_parallel_angle(double angle_rad);
std::string format_fsm_command(double vx, double vy, double wz);
}  // namespace weldline_reflectivity_detector
