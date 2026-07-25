#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "weldline_reflectivity_detector/scenario_planner.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

weldline_reflectivity_detector::ScenarioDefinition make_scenario()
{
  using weldline_reflectivity_detector::ScenarioDefinition;
  using weldline_reflectivity_detector::ScenarioWaypoint;
  using weldline_reflectivity_detector::WaypointSource;
  using weldline_reflectivity_detector::YawMode;

  ScenarioDefinition scenario;
  scenario.frame_id = "map";
  scenario.control.rate_hz = 20.0;
  scenario.control.wall_motion_gate_rad = 2.0 * kPi / 180.0;
  scenario.control.yaw_wall_consistency_rad = 3.0 * kPi / 180.0;
  scenario.control.position_kp = 1.0;
  scenario.control.max_linear_speed_mps = 0.2;
  scenario.control.wall_alignment_kp = 1.5;
  scenario.control.waypoint_yaw_kp = 1.0;
  scenario.control.max_angular_speed_rps = 0.5;

  ScenarioWaypoint detector;
  detector.id = "detected";
  detector.source = WaypointSource::kDetector;
  detector.yaw_mode = YawMode::kParallel;
  detector.position_tolerance_m = 0.04;
  detector.yaw_tolerance_rad = 0.5 * kPi / 180.0;
  detector.wall_tolerance_rad = 0.35 * kPi / 180.0;
  detector.hold_time_sec = 1.0;

  ScenarioWaypoint fixed = detector;
  fixed.id = "final";
  fixed.source = WaypointSource::kFixed;
  fixed.pose = {1.0, 0.0, 0.0};
  fixed.resolved = true;
  scenario.waypoints = {detector, fixed};
  return scenario;
}
}  // namespace

TEST(ScenarioPlanner, FormatsExactFsmCommandContract)
{
  using weldline_reflectivity_detector::format_fsm_command;
  EXPECT_EQ(format_fsm_command(0.125, -0.25, 0.5), "cmd 0.125000 -0.250000 0.500000");
}

TEST(ScenarioPlanner, StopsUntilDetectorWaypointIsStableAndResolved)
{
  using weldline_reflectivity_detector::PlannerInput;
  using weldline_reflectivity_detector::PlannerState;
  using weldline_reflectivity_detector::ScenarioPlanner;

  ScenarioPlanner planner(make_scenario());
  PlannerInput input;
  input.pose_valid = true;
  input.wall_valid = true;
  const auto output = planner.update(input);
  EXPECT_EQ(output.state, PlannerState::kWaitingDetector);
  EXPECT_DOUBLE_EQ(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.wz, 0.0);
}

TEST(ScenarioPlanner, GatesTranslationAndRotatesByWallError)
{
  using weldline_reflectivity_detector::PlannerInput;
  using weldline_reflectivity_detector::PlannerState;
  using weldline_reflectivity_detector::ScenarioPlanner;

  ScenarioPlanner planner(make_scenario());
  planner.set_detector_waypoint({1.0, 0.0, 0.0});
  PlannerInput input;
  input.pose_valid = true;
  input.wall_valid = true;
  input.wall_error_rad = 10.0 * kPi / 180.0;
  const auto output = planner.update(input);
  EXPECT_EQ(output.state, PlannerState::kAligningWall);
  EXPECT_DOUBLE_EQ(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
  EXPECT_NEAR(output.wz, 1.5 * input.wall_error_rad, 1e-12);
}

TEST(ScenarioPlanner, ConvertsMapPositionErrorIntoRobotVelocity)
{
  using weldline_reflectivity_detector::PlannerInput;
  using weldline_reflectivity_detector::PlannerState;
  using weldline_reflectivity_detector::ScenarioPlanner;

  ScenarioPlanner planner(make_scenario());
  planner.set_detector_waypoint({1.0, 0.0, 0.0});
  PlannerInput input;
  input.pose_valid = true;
  input.robot_pose.yaw_rad = kPi / 2.0;
  input.wall_valid = true;
  input.wall_error_rad = 1.0 * kPi / 180.0;
  const auto output = planner.update(input);
  EXPECT_EQ(output.state, PlannerState::kDriving);
  EXPECT_NEAR(output.vx, 0.0, 1e-12);
  EXPECT_NEAR(output.vy, -0.2, 1e-12);
  EXPECT_GT(output.wz, 0.0);
}

TEST(ScenarioPlanner, RequiresContinuousStrictHoldBeforeAdvancing)
{
  using weldline_reflectivity_detector::PlannerInput;
  using weldline_reflectivity_detector::PlannerState;
  using weldline_reflectivity_detector::ScenarioPlanner;

  ScenarioPlanner planner(make_scenario());
  planner.set_detector_waypoint({0.0, 0.0, 0.0});
  PlannerInput input;
  input.pose_valid = true;
  input.wall_valid = true;
  input.monotonic_time_sec = 10.0;
  EXPECT_EQ(planner.update(input).state, PlannerState::kVerifyingWaypoint);

  input.monotonic_time_sec = 10.5;
  input.wall_error_rad = 1.0 * kPi / 180.0;
  EXPECT_EQ(planner.update(input).state, PlannerState::kAligningWall);

  input.wall_error_rad = 0.0;
  input.monotonic_time_sec = 11.0;
  EXPECT_EQ(planner.update(input).state, PlannerState::kVerifyingWaypoint);
  input.monotonic_time_sec = 12.01;
  const auto advanced = planner.update(input);
  EXPECT_EQ(advanced.state, PlannerState::kAdvancing);
  EXPECT_EQ(advanced.waypoint_index, 1U);
}

TEST(ScenarioPlanner, RejectsWaypointYawThatConflictsWithWall)
{
  using weldline_reflectivity_detector::PlannerInput;
  using weldline_reflectivity_detector::PlannerState;
  using weldline_reflectivity_detector::ScenarioPlanner;

  ScenarioPlanner planner(make_scenario());
  planner.set_detector_waypoint({0.0, 0.0, 0.0});
  PlannerInput input;
  input.pose_valid = true;
  input.robot_pose.yaw_rad = 20.0 * kPi / 180.0;
  input.wall_valid = true;
  input.wall_error_rad = 0.0;
  const auto output = planner.update(input);
  EXPECT_EQ(output.state, PlannerState::kHeadingConflict);
  EXPECT_DOUBLE_EQ(output.wz, 0.0);
}

TEST(ScenarioPlanner, FinalWaypointNeverExitsClosedLoopHold)
{
  using weldline_reflectivity_detector::PlannerInput;
  using weldline_reflectivity_detector::PlannerState;
  using weldline_reflectivity_detector::ScenarioPlanner;

  auto scenario = make_scenario();
  scenario.waypoints.resize(1U);
  ScenarioPlanner planner(std::move(scenario));
  planner.set_detector_waypoint({0.0, 0.0, 0.0});
  PlannerInput input;
  input.pose_valid = true;
  input.wall_valid = true;
  input.monotonic_time_sec = 1.0;
  EXPECT_EQ(planner.update(input).state, PlannerState::kVerifyingWaypoint);
  input.monotonic_time_sec = 2.1;
  EXPECT_EQ(planner.update(input).state, PlannerState::kHoldingFinal);

  input.robot_pose.x = -0.2;
  input.monotonic_time_sec = 2.2;
  const auto correction = planner.update(input);
  EXPECT_EQ(correction.state, PlannerState::kDriving);
  EXPECT_GT(correction.vx, 0.0);
}

TEST(ScenarioLoader, LoadsInstalledExampleAndBestOnnxMetadata)
{
  using weldline_reflectivity_detector::WaypointSource;
  using weldline_reflectivity_detector::load_scenario_file;

  const auto scenario = load_scenario_file(TEST_SCENARIO_FILE);
  ASSERT_GE(scenario.waypoints.size(), 2U);
  EXPECT_EQ(scenario.frame_id, "map");
  EXPECT_EQ(scenario.control.command_topic, "/fsm_cmd");
  EXPECT_EQ(scenario.waypoints.front().source, WaypointSource::kDetector);
  EXPECT_FALSE(scenario.waypoints.front().resolved);
  EXPECT_NE(scenario.detector.model_path.find("best.onnx"), std::string::npos);
}
