"""File-defined weld-line mission state machine."""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, pi, sin
from pathlib import Path
from typing import Optional

import yaml

from .scenario import ControlConfig, DetectorConfig, Pose2D, normalize_angle, normalize_parallel_angle


@dataclass
class Goal:
    name: str
    source: str
    pose: Optional[Pose2D] = None


@dataclass
class Step:
    name: str
    action: str
    goal: str = ""
    tolerance: float = 0.05
    angle_rad: float = 0.0
    expected_wall_error: Optional[float] = None
    wall_tolerance: float = 0.01
    requires_resume: bool = False


@dataclass
class MissionConfig:
    frame_id: str
    detector: DetectorConfig
    control: ControlConfig
    goals: dict[str, Goal]
    steps: list[Step]


@dataclass
class MissionOutput:
    state: str
    detail: str
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    fsm_command: Optional[str] = None


def _required(mapping: dict, key: str, context: str):
    if key not in mapping:
        raise ValueError(f"{context} is missing required key '{key}'")
    return mapping[key]


def _positive(value: float, name: str) -> float:
    value = float(value)
    if not isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


def _check_keys(mapping: dict, allowed: set[str], context: str) -> None:
    if not isinstance(mapping, dict):
        raise ValueError(f"{context} must be a YAML map")
    unknown = set(mapping) - allowed
    if unknown:
        raise ValueError(f"{context} has unknown key '{sorted(unknown)[0]}'")


def load_mission(path: str) -> MissionConfig:
    source = Path(path).resolve()
    with source.open(encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    _check_keys(root, {"version", "frame_id", "detector", "control", "goals", "mission"}, "mission")
    if _required(root, "version", "mission") != 2:
        raise ValueError("mission.version must be 2")
    frame_id = str(_required(root, "frame_id", "mission"))
    if not frame_id:
        raise ValueError("mission.frame_id must not be empty")

    raw_detector = _required(root, "detector", "mission")
    _check_keys(raw_detector, {"goal_topic", "stable_samples", "max_position_spread_m", "max_yaw_spread_deg", "max_goal_age_sec"}, "mission.detector")
    detector = DetectorConfig(
        "", str(_required(raw_detector, "goal_topic", "mission.detector")),
        int(_required(raw_detector, "stable_samples", "mission.detector")),
        _positive(_required(raw_detector, "max_position_spread_m", "mission.detector"), "max_position_spread_m"),
        _positive(_required(raw_detector, "max_yaw_spread_deg", "mission.detector"), "max_yaw_spread_deg") * pi / 180.0,
        _positive(_required(raw_detector, "max_goal_age_sec", "mission.detector"), "max_goal_age_sec"),
    )
    if not detector.goal_topic or not 2 <= detector.stable_samples <= 100:
        raise ValueError("detector goal_topic or stable_samples is invalid")

    raw_control = _required(root, "control", "mission")
    allowed_control = {"enabled", "rate_hz", "command_topic", "status_topic", "robot_frame", "position_kp", "max_linear_speed_mps", "wall_alignment_kp", "waypoint_yaw_kp", "max_angular_speed_rps"}
    _check_keys(raw_control, allowed_control, "mission.control")
    control = ControlConfig(
        bool(_required(raw_control, "enabled", "mission.control")),
        _positive(_required(raw_control, "rate_hz", "mission.control"), "rate_hz"),
        str(_required(raw_control, "command_topic", "mission.control")),
        str(_required(raw_control, "status_topic", "mission.control")),
        str(_required(raw_control, "robot_frame", "mission.control")),
        0.05, 1.0, 1.0,
        _positive(_required(raw_control, "position_kp", "mission.control"), "position_kp"),
        _positive(_required(raw_control, "max_linear_speed_mps", "mission.control"), "max_linear_speed_mps"),
        _positive(_required(raw_control, "wall_alignment_kp", "mission.control"), "wall_alignment_kp"),
        _positive(_required(raw_control, "waypoint_yaw_kp", "mission.control"), "waypoint_yaw_kp"),
        _positive(_required(raw_control, "max_angular_speed_rps", "mission.control"), "max_angular_speed_rps"),
        0.0, 0.0,
    )

    goals: dict[str, Goal] = {}
    for name, raw_goal in _required(root, "goals", "mission").items():
        _check_keys(raw_goal, {"source", "x", "y", "yaw_deg"}, f"mission.goals.{name}")
        source_kind = str(_required(raw_goal, "source", f"mission.goals.{name}"))
        if source_kind == "detector":
            if any(key in raw_goal for key in ("x", "y", "yaw_deg")):
                raise ValueError(f"mission.goals.{name} detector goal must not contain a pose")
            goals[name] = Goal(name, source_kind)
        elif source_kind == "fixed":
            goals[name] = Goal(name, source_kind, Pose2D(float(_required(raw_goal, "x", f"mission.goals.{name}")), float(_required(raw_goal, "y", f"mission.goals.{name}")), normalize_angle(float(raw_goal.get("yaw_deg", 0.0)) * pi / 180.0)))
        else:
            raise ValueError(f"mission.goals.{name}.source must be detector or fixed")
    if not goals:
        raise ValueError("mission.goals must not be empty")

    steps: list[Step] = []
    for index, raw_step in enumerate(_required(root, "mission", "mission")):
        context = f"mission.mission[{index}]"
        _check_keys(raw_step, {"name", "action", "goal", "position_tolerance_m", "angle_deg", "angle_tolerance_deg", "wall_angle_deg", "wall_tolerance_deg", "requires_resume"}, context)
        action = str(_required(raw_step, "action", context))
        name = str(_required(raw_step, "name", context))
        if action not in {"wait_detection", "move", "turn", "align_wall", "sit"}:
            raise ValueError(f"{context}.action is invalid")
        goal = str(raw_step.get("goal", ""))
        if action == "move" and goal not in goals:
            raise ValueError(f"{context}.goal must name a defined goal")
        if action in {"wait_detection", "move"} and not goal:
            raise ValueError(f"{context}.goal is required")
        tolerance_key = "position_tolerance_m" if action == "move" else "angle_tolerance_deg"
        default = 0.05 if action == "move" else 1.0
        tolerance = _positive(raw_step.get(tolerance_key, default), tolerance_key)
        if action != "move": tolerance *= pi / 180.0
        angle = float(raw_step.get("angle_deg", 0.0)) * pi / 180.0
        expected_wall_error = raw_step.get("wall_angle_deg")
        if expected_wall_error is not None and action != "turn":
            raise ValueError(f"{context}.wall_angle_deg is only valid for turn")
        wall_tolerance = _positive(raw_step.get("wall_tolerance_deg", raw_step.get("angle_tolerance_deg", 1.0)), "wall_tolerance_deg") * pi / 180.0
        steps.append(Step(name, action, goal, tolerance, angle, None if expected_wall_error is None else float(expected_wall_error) * pi / 180.0, wall_tolerance, bool(raw_step.get("requires_resume", False))))
    if not steps:
        raise ValueError("mission.mission must not be empty")
    return MissionConfig(frame_id, detector, control, goals, steps)


class MissionController:
    def __init__(self, config: MissionConfig):
        self.config, self.index = config, 0
        self.detected_goal: Optional[Pose2D] = None
        self.turn_target: Optional[float] = None
        self.command_sent = False
        self.resume_requested = False

    @property
    def state(self) -> str:
        return "COMPLETE" if self.index >= len(self.config.steps) else self.config.steps[self.index].name

    def set_detected_goal(self, pose: Pose2D) -> None:
        if not all(isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            raise ValueError("detected goal must be finite")
        self.detected_goal = pose

    def request_resume(self) -> None:
        self.resume_requested = True

    def _advance(self) -> None:
        self.index += 1
        self.turn_target = None
        self.command_sent = False

    def _stop(self, detail: str) -> MissionOutput:
        return MissionOutput(self.state, detail)

    def update(self, pose: Optional[Pose2D], wall_error: Optional[float]) -> MissionOutput:
        if not self.config.control.enabled:
            return MissionOutput("DISABLED", "mission control disabled")
        if self.index >= len(self.config.steps):
            return MissionOutput("COMPLETE", "mission complete")
        step = self.config.steps[self.index]
        if step.action == "wait_detection":
            if self.detected_goal is None:
                return self._stop("waiting for weld-line goal pose")
            self._advance()
            return self._stop("weld-line goal pose accepted")
        if pose is None:
            return self._stop("waiting for robot pose")
        if step.action == "move":
            goal = self.detected_goal if self.config.goals[step.goal].source == "detector" else self.config.goals[step.goal].pose
            if goal is None:
                return self._stop(f"waiting for goal '{step.goal}'")
            dx, dy = goal.x - pose.x, goal.y - pose.y
            if (dx * dx + dy * dy) ** 0.5 <= step.tolerance:
                self._advance()
                return self._stop(f"reached goal '{step.goal}'")
            vx = self.config.control.position_kp * (cos(pose.yaw) * dx + sin(pose.yaw) * dy)
            vy = self.config.control.position_kp * (-sin(pose.yaw) * dx + cos(pose.yaw) * dy)
            speed = (vx * vx + vy * vy) ** 0.5
            if speed > self.config.control.max_linear_speed:
                vx, vy = vx * self.config.control.max_linear_speed / speed, vy * self.config.control.max_linear_speed / speed
            return MissionOutput(step.name, f"moving to '{step.goal}'", vx, vy)
        if step.action == "turn":
            self.turn_target = normalize_angle(pose.yaw + step.angle_rad) if self.turn_target is None else self.turn_target
            error = normalize_angle(self.turn_target - pose.yaw)
            if abs(error) <= step.tolerance and step.expected_wall_error is not None:
                if wall_error is None or not isfinite(wall_error):
                    return self._stop("turn reached; waiting for front-wall angle")
                if abs(normalize_parallel_angle(wall_error - step.expected_wall_error)) > step.wall_tolerance:
                    return self._stop("turn reached; waiting for expected front-wall angle")
            if abs(error) <= step.tolerance:
                self._advance()
                return self._stop("turn completed")
            wz = max(-self.config.control.max_angular_speed, min(self.config.control.max_angular_speed, self.config.control.waypoint_yaw_kp * error))
            return MissionOutput(step.name, "turning", wz=wz)
        if step.action == "align_wall":
            if wall_error is None or not isfinite(wall_error):
                return self._stop("waiting for front-wall angle")
            if abs(wall_error) <= step.tolerance:
                self._advance()
                return self._stop("wall alignment completed")
            wz = max(-self.config.control.max_angular_speed, min(self.config.control.max_angular_speed, self.config.control.wall_alignment_kp * wall_error))
            return MissionOutput(step.name, "aligning to front wall", wz=wz)
        if not self.command_sent:
            self.command_sent = True
            return MissionOutput(step.name, "sent SIT; waiting for RL resume", fsm_command="SIT")
        if step.requires_resume and not self.resume_requested:
            return self._stop("SIT completed; waiting for /commander/mission/resume")
        self.resume_requested = False
        self._advance()
        return MissionOutput(step.name, "resuming RL mode", fsm_command="RL")
