from math import pi
from pathlib import Path

from weldline_reflectivity_detector.scenario import Pose2D, ScenarioPlanner, format_fsm_command, load_scenario
from weldline_reflectivity_detector.wall_alignment import Point2D, WallConfig, estimate_wall


def test_load_example_and_format_command():
    scenario = load_scenario(str(Path(__file__).parents[1] / "config/scenario.yaml"))
    assert scenario.frame_id == "map"
    assert format_fsm_command(1.0, 0.0, -0.5) == "cmd 1.000000 0.000000 -0.500000"


def test_planner_waits_for_detector_then_drives():
    planner = ScenarioPlanner(load_scenario(str(Path(__file__).parents[1] / "config/scenario.yaml")))
    assert planner.update(Pose2D(), 0.0, 0.0).state == "WAITING_DETECTOR"
    planner.set_detector_waypoint(Pose2D(1.0, 0.0, 0.0))
    output = planner.update(Pose2D(), 0.0, 0.0)
    assert output.state == "DRIVING" and output.vx > 0.0


def test_wall_fit_recovers_parallel_angle():
    points = [Point2D(float(index) / 10.0, 1.0) for index in range(-20, 21)]
    estimate = estimate_wall(points, WallConfig(min_inliers=10, min_length=1.0), None)
    assert estimate is not None
    assert abs(estimate.angle) < 1e-6
