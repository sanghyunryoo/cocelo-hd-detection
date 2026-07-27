from math import pi
from pathlib import Path

from weldline_reflectivity_detector.mission import MissionController, load_mission
from weldline_reflectivity_detector.scenario import Pose2D, format_fsm_command, wall_heading_error
from weldline_reflectivity_detector.wall_alignment import Point2D, WallConfig, estimate_wall


def _mission():
    return MissionController(load_mission(str(Path(__file__).parents[1] / "config/scenario.yaml")))


def _jump_to(controller, state):
    controller.index = next(index for index, step in enumerate(controller.config.steps) if step.name == state)


def test_load_mission_and_format_command():
    mission = _mission().config
    assert mission.frame_id == "map"
    assert len(mission.steps) == 16
    assert mission.steps[9].name == "TURN_180"
    assert format_fsm_command(1.0, 0.0, -0.5) == "cmd 1.000000 0.000000 -0.500000"


def test_mission_waits_for_detector_then_drives():
    controller = _mission()
    assert controller.update(Pose2D(), 0.0).state == "WAIT_WELDLINE_GOAL"
    controller.set_detected_goal(Pose2D(1.0, 0.0, 0.0))
    assert controller.update(Pose2D(), 0.0).state == "MOVE_WELDLINE"
    output = controller.update(Pose2D(), 0.0)
    assert output.state == "MOVE_WELDLINE" and output.vx > 0.0


def test_turn_uses_slam_yaw_and_not_parallel_wall_heading():
    controller = _mission()
    _jump_to(controller, "TURN_LEFT_90")
    output = controller.update(Pose2D(0.0, 0.0, 0.0), None)
    assert output.wz > 0.0
    assert controller.update(Pose2D(0.0, 0.0, pi / 2.0), -pi / 2.0).state == "MOVE_DESTINATION_1"


def test_sit_requires_explicit_resume_before_rl():
    controller = _mission()
    _jump_to(controller, "SIT")
    assert controller.update(Pose2D(), 0.0).fsm_command == "SIT"
    assert controller.update(Pose2D(), 0.0).state == "SIT"
    controller.request_resume()
    output = controller.update(Pose2D(), 0.0)
    assert output.fsm_command == "RL" and controller.state == "MOVE_DESTINATION_2"


def test_full_mission_state_sequence_completes():
    controller = _mission()
    controller.set_detected_goal(Pose2D(0.0, 0.0, 0.0))
    controller.update(Pose2D(0.0, 0.0, 0.0), 0.0)
    controller.update(Pose2D(0.0, 0.0, 0.0), 0.0)
    controller.update(Pose2D(0.0, 0.0, 0.0), 0.0)
    controller.update(Pose2D(0.0, 0.0, pi / 2.0), -pi / 2.0)
    controller.update(Pose2D(0.75, 0.0, pi / 2.0), 0.0)
    controller.update(Pose2D(0.75, 0.0, 0.0), 0.0)
    assert controller.update(Pose2D(0.75, 0.0, 0.0), 0.0).fsm_command == "SIT"
    controller.request_resume()
    assert controller.update(Pose2D(0.75, 0.0, 0.0), 0.0).fsm_command == "RL"
    controller.update(Pose2D(1.50, 0.0, 0.0), 0.0)
    controller.update(Pose2D(1.50, 0.0, 0.0), 0.0)
    controller.update(Pose2D(2.00, 0.0, 0.0), 0.0)
    controller.update(Pose2D(2.00, 0.0, 0.0), 0.0)
    controller.update(Pose2D(2.00, 0.0, 0.0), 0.0)
    controller.update(Pose2D(2.00, 0.0, pi), 0.0)
    for pose in (Pose2D(2.00, 0.0, pi), Pose2D(1.50, 0.0, pi), Pose2D(1.50, 0.0, pi), Pose2D(0.75, 0.0, pi), Pose2D(0.75, 0.0, pi), Pose2D(0.0, 0.0, pi)):
        controller.update(pose, 0.0)
    assert controller.state == "COMPLETE"


def test_wall_fit_recovers_parallel_angle():
    points = [Point2D(float(index) / 10.0, 1.0) for index in range(-20, 21)]
    estimate = estimate_wall(points, WallConfig(min_inliers=10, min_length=1.0), None)
    assert estimate is not None
    assert abs(estimate.angle) < 1e-6


def test_front_sector_rejects_a_wall_behind_the_robot():
    points = [Point2D(float(index) / 10.0 - 4.0, 1.0) for index in range(-20, 21)]
    assert estimate_wall(points, WallConfig(min_inliers=10, min_length=1.0, sector="front")) is None


def test_simulator_wall_error_is_zero_when_parallel():
    assert wall_heading_error(0.0, 0.0) == 0.0
    assert abs(wall_heading_error(3.0 * pi / 4.0, -pi / 4.0)) < 1e-9


def test_wall_alignment_rotates_toward_parallel():
    controller = _mission()
    _jump_to(controller, "ALIGN_WALL_1")
    assert controller.update(Pose2D(), 0.1).wz > 0.0
