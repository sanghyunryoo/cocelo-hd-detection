"""Scenario configuration and the deterministic waypoint controller."""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, hypot, isfinite, pi, sin
from pathlib import Path
from typing import Optional

import yaml


def normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def normalize_parallel_angle(angle: float) -> float:
    angle = normalize_angle(angle)
    return angle - pi if angle >= pi / 2 else angle + pi if angle < -pi / 2 else angle


def wall_heading_error(robot_yaw: float, wall_heading: float) -> float:
    """Signed robot-to-wall tangent error; zero means parallel to the wall."""
    return normalize_parallel_angle(wall_heading - robot_yaw)


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class Waypoint:
    identifier: str
    source: str
    yaw_mode: str
    pose: Pose2D = field(default_factory=Pose2D)
    resolved: bool = False
    position_tolerance: float = 0.05
    yaw_tolerance: float = 0.01
    wall_tolerance: float = 0.01
    hold_time: float = 1.0


@dataclass
class DetectorConfig:
    model_path: str
    goal_topic: str
    stable_samples: int
    max_position_spread: float
    max_yaw_spread: float
    max_goal_age: float


@dataclass
class ControlConfig:
    enabled: bool
    rate_hz: float
    command_topic: str
    status_topic: str
    robot_frame: str
    tf_timeout: float
    max_tf_age: float
    max_wall_age: float
    position_kp: float
    max_linear_speed: float
    wall_alignment_kp: float
    waypoint_yaw_kp: float
    max_angular_speed: float
    wall_motion_gate: float
    yaw_wall_consistency: float


@dataclass
class Scenario:
    frame_id: str
    detector: DetectorConfig
    control: ControlConfig
    waypoints: list[Waypoint]


def _need(mapping: dict, key: str, context: str):
    if key not in mapping:
        raise ValueError(f"{context} is missing required key '{key}'")
    return mapping[key]


def _keys(mapping: dict, allowed: set[str], context: str) -> None:
    if not isinstance(mapping, dict):
        raise ValueError(f"{context} must be a YAML map")
    unknown = set(mapping) - allowed
    if unknown:
        raise ValueError(f"{context} contains unknown key '{sorted(unknown)[0]}'")


def _positive(value: float, name: str, allow_zero: bool = False) -> float:
    value = float(value)
    if not isfinite(value) or value < 0.0 or (not allow_zero and value == 0.0):
        raise ValueError(f"{name} must be {'non-negative' if allow_zero else 'positive'}")
    return value


def load_scenario(path: str) -> Scenario:
    source = Path(path).resolve()
    with source.open(encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    _keys(root, {"version", "frame_id", "detector", "control", "waypoints"}, "scenario")
    if _need(root, "version", "scenario") != 1:
        raise ValueError("scenario.version must be exactly 1")
    frame_id = str(_need(root, "frame_id", "scenario"))
    if not frame_id:
        raise ValueError("scenario.frame_id must not be empty")

    detector = _need(root, "detector", "scenario")
    _keys(detector, {"model", "goal_topic", "stable_samples", "max_position_spread_m", "max_yaw_spread_deg", "max_goal_age_sec"}, "scenario.detector")
    model_path = (source.parent / str(_need(detector, "model", "scenario.detector"))).resolve()
    if not model_path.is_file():
        raise ValueError(f"scenario.detector.model does not exist: {model_path}")
    detector_config = DetectorConfig(
        str(model_path), str(_need(detector, "goal_topic", "scenario.detector")),
        int(_need(detector, "stable_samples", "scenario.detector")),
        _positive(_need(detector, "max_position_spread_m", "scenario.detector"), "max_position_spread_m"),
        _positive(_need(detector, "max_yaw_spread_deg", "scenario.detector"), "max_yaw_spread_deg") * pi / 180.0,
        _positive(_need(detector, "max_goal_age_sec", "scenario.detector"), "max_goal_age_sec"),
    )
    if not detector_config.goal_topic or not 2 <= detector_config.stable_samples <= 100:
        raise ValueError("detector goal topic must be set and stable_samples must be in [2, 100]")

    control = _need(root, "control", "scenario")
    allowed = {"enabled", "rate_hz", "command_topic", "status_topic", "robot_frame", "tf_timeout_sec", "max_tf_age_sec", "max_wall_age_sec", "position_kp", "max_linear_speed_mps", "wall_alignment_kp", "waypoint_yaw_kp", "max_angular_speed_rps", "wall_motion_gate_deg", "yaw_wall_consistency_deg"}
    _keys(control, allowed, "scenario.control")
    control_config = ControlConfig(
        bool(_need(control, "enabled", "scenario.control")),
        _positive(_need(control, "rate_hz", "scenario.control"), "rate_hz"),
        str(_need(control, "command_topic", "scenario.control")), str(_need(control, "status_topic", "scenario.control")),
        str(_need(control, "robot_frame", "scenario.control")),
        _positive(_need(control, "tf_timeout_sec", "scenario.control"), "tf_timeout_sec", True),
        _positive(_need(control, "max_tf_age_sec", "scenario.control"), "max_tf_age_sec"),
        _positive(_need(control, "max_wall_age_sec", "scenario.control"), "max_wall_age_sec"),
        _positive(_need(control, "position_kp", "scenario.control"), "position_kp"),
        _positive(_need(control, "max_linear_speed_mps", "scenario.control"), "max_linear_speed_mps"),
        _positive(_need(control, "wall_alignment_kp", "scenario.control"), "wall_alignment_kp"),
        _positive(_need(control, "waypoint_yaw_kp", "scenario.control"), "waypoint_yaw_kp"),
        _positive(_need(control, "max_angular_speed_rps", "scenario.control"), "max_angular_speed_rps"),
        _positive(_need(control, "wall_motion_gate_deg", "scenario.control"), "wall_motion_gate_deg") * pi / 180.0,
        _positive(_need(control, "yaw_wall_consistency_deg", "scenario.control"), "yaw_wall_consistency_deg") * pi / 180.0,
    )
    if control_config.rate_hz > 100 or not all((control_config.command_topic, control_config.status_topic, control_config.robot_frame)):
        raise ValueError("invalid scenario.control topics or rate")

    raw_waypoints = _need(root, "waypoints", "scenario")
    if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
        raise ValueError("scenario.waypoints must contain detector and fixed waypoints")
    waypoints: list[Waypoint] = []
    identifiers: set[str] = set()
    for index, raw in enumerate(raw_waypoints):
        context = f"scenario.waypoints[{index}]"
        _keys(raw, {"id", "source", "x", "y", "yaw_deg", "yaw_mode", "position_tolerance_m", "yaw_tolerance_deg", "wall_tolerance_deg", "hold_time_sec"}, context)
        identifier, source_kind, yaw_mode = str(_need(raw, "id", context)), str(_need(raw, "source", context)), str(_need(raw, "yaw_mode", context))
        if not identifier or identifier in identifiers or source_kind not in {"detector", "fixed"} or yaw_mode not in {"parallel", "directional"}:
            raise ValueError(f"{context} has invalid id, source, or yaw_mode")
        identifiers.add(identifier)
        waypoint = Waypoint(identifier, source_kind, yaw_mode, position_tolerance=_positive(_need(raw, "position_tolerance_m", context), "position_tolerance_m"), yaw_tolerance=_positive(_need(raw, "yaw_tolerance_deg", context), "yaw_tolerance_deg") * pi / 180.0, wall_tolerance=_positive(_need(raw, "wall_tolerance_deg", context), "wall_tolerance_deg") * pi / 180.0, hold_time=_positive(_need(raw, "hold_time_sec", context), "hold_time_sec"))
        if source_kind == "detector":
            if index != 0 or any(key in raw for key in ("x", "y", "yaw_deg")):
                raise ValueError("only the first waypoint can be detector-driven")
        else:
            waypoint.pose = Pose2D(float(_need(raw, "x", context)), float(_need(raw, "y", context)), normalize_angle(float(_need(raw, "yaw_deg", context)) * pi / 180.0))
            waypoint.resolved = True
        if waypoint.wall_tolerance > control_config.wall_motion_gate:
            raise ValueError(f"{context}.wall_tolerance_deg exceeds wall_motion_gate_deg")
        waypoints.append(waypoint)
    return Scenario(frame_id, detector_config, control_config, waypoints)


@dataclass
class PlannerOutput:
    state: str
    waypoint_id: str
    index: int
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    detail: str = ""


class ScenarioPlanner:
    def __init__(self, scenario: Scenario):
        self.scenario, self.index, self.within_since = scenario, 0, None

    @property
    def detector_resolved(self) -> bool:
        return self.scenario.waypoints[0].resolved

    def set_detector_waypoint(self, pose: Pose2D) -> None:
        if not all(map(isfinite, (pose.x, pose.y, pose.yaw))):
            raise ValueError("detector waypoint must be finite")
        waypoint = self.scenario.waypoints[0]
        waypoint.pose, waypoint.resolved = Pose2D(pose.x, pose.y, normalize_angle(pose.yaw)), True

    def update(self, pose: Optional[Pose2D], wall_error: Optional[float], now: float) -> PlannerOutput:
        waypoint = self.scenario.waypoints[self.index]
        def stopped(state: str, detail: str) -> PlannerOutput:
            self.within_since = None
            return PlannerOutput(state, waypoint.identifier, self.index, detail=detail)
        if not self.scenario.control.enabled: return stopped("DISABLED", "scenario control disabled")
        if not self.detector_resolved: return stopped("WAITING_DETECTOR", "waiting for stable detector waypoint")
        if pose is None: return stopped("WAITING_POSE", "map-to-robot pose unavailable")
        if wall_error is None or not isfinite(wall_error): return stopped("WAITING_WALL", "fresh wall alignment unavailable")
        control, wall_error = self.scenario.control, normalize_parallel_angle(wall_error)
        dx, dy = waypoint.pose.x - pose.x, waypoint.pose.y - pose.y
        distance = hypot(dx, dy)
        yaw_error = normalize_parallel_angle(waypoint.pose.yaw - pose.yaw) if waypoint.yaw_mode == "parallel" else normalize_angle(waypoint.pose.yaw - pose.yaw)
        output = PlannerOutput("", waypoint.identifier, self.index)
        wall_wz = max(-control.max_angular_speed, min(control.max_angular_speed, control.wall_alignment_kp * wall_error))
        if abs(wall_error) > control.wall_motion_gate:
            return PlannerOutput("ALIGNING_WALL", waypoint.identifier, self.index, wz=wall_wz, detail="translation gated until wall alignment is restored")
        if distance > waypoint.position_tolerance:
            output.vx, output.vy = control.position_kp * (cos(pose.yaw) * dx + sin(pose.yaw) * dy), control.position_kp * (-sin(pose.yaw) * dx + cos(pose.yaw) * dy)
            speed = hypot(output.vx, output.vy)
            if speed > control.max_linear_speed:
                output.vx, output.vy = output.vx * control.max_linear_speed / speed, output.vy * control.max_linear_speed / speed
            output.state, output.wz, output.detail = "DRIVING", wall_wz, "tracking waypoint position with continuous wall correction"
            return output
        if abs(wall_error) > waypoint.wall_tolerance:
            return PlannerOutput("ALIGNING_WALL", waypoint.identifier, self.index, wz=wall_wz, detail="position reached; enforcing strict wall tolerance")
        if abs(yaw_error) > waypoint.yaw_tolerance:
            if abs(normalize_parallel_angle(yaw_error - wall_error)) > control.yaw_wall_consistency:
                return PlannerOutput("HEADING_CONFLICT", waypoint.identifier, self.index, detail="scenario yaw conflicts with wall heading")
            return PlannerOutput("ALIGNING_WAYPOINT_YAW", waypoint.identifier, self.index, wz=max(-control.max_angular_speed, min(control.max_angular_speed, control.waypoint_yaw_kp * yaw_error)), detail="position reached; enforcing scenario yaw")
        self.within_since = now if self.within_since is None else self.within_since
        if now - self.within_since < waypoint.hold_time:
            return PlannerOutput("VERIFYING_WAYPOINT", waypoint.identifier, self.index, detail="verifying continuous hold")
        if self.index + 1 >= len(self.scenario.waypoints):
            return PlannerOutput("HOLDING_FINAL", waypoint.identifier, self.index, detail="final waypoint held")
        self.index, self.within_since = self.index + 1, None
        return PlannerOutput("ADVANCING", self.scenario.waypoints[self.index].identifier, self.index, detail="advancing to next waypoint")


def format_fsm_command(vx: float, vy: float, wz: float) -> str:
    if not all(map(isfinite, (vx, vy, wz))): raise ValueError("FSM command must be finite")
    return f"cmd {vx:.6f} {vy:.6f} {wz:.6f}"
