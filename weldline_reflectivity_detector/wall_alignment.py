"""Robust 2D wall fitting used by the scenario commander."""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin
from random import Random
from typing import Optional

import numpy as np


@dataclass
class Point2D:
    x: float
    y: float


@dataclass
class WallConfig:
    distance_threshold: float = 0.04
    iterations: int = 300
    max_candidates: int = 6
    min_inliers: int = 30
    min_length: float = 0.8
    max_rmse: float = 0.04
    selection: str = "tracked"
    sector: str = "any"
    sector_half_angle: float = 70.0 * pi / 180.0
    tracking_max_angle: float = 20.0 * pi / 180.0


@dataclass
class WallEstimate:
    angle: float
    distance: float
    rmse: float
    length: float
    inlier_count: int
    start: Point2D
    end: Point2D
    closest: Point2D


def normalize_wall_angle(angle: float) -> float:
    angle = (angle + pi / 2.0) % pi - pi / 2.0
    return angle


def _sector(bearing: float, sector: str, half_angle: float) -> bool:
    centers = {"any": None, "front": 0.0, "left": pi / 2.0, "rear": pi, "right": -pi / 2.0}
    if sector not in centers: raise ValueError("invalid wall sector")
    return centers[sector] is None or abs((bearing - centers[sector] + pi) % (2 * pi) - pi) <= half_angle


def _fit(points: np.ndarray, indices: np.ndarray) -> Optional[dict]:
    if len(indices) < 2: return None
    selected = points[indices]
    center = selected.mean(axis=0)
    _, values, vectors = np.linalg.svd(selected - center, full_matrices=False)
    if values[0] <= np.finfo(float).eps: return None
    direction = vectors[0]
    projection = (selected - center) @ direction
    projection.sort()
    trim = int(0.02 * len(projection))
    low, high = projection[trim], projection[-1 - trim]
    residual = (selected - center) @ np.array([-direction[1], direction[0]])
    closest_projection = np.clip(-center @ direction, low, high)
    return {"indices": indices, "center": center, "direction": direction, "start": center + low * direction, "end": center + high * direction, "closest": center + closest_projection * direction, "length": high - low, "rmse": float(np.sqrt(np.mean(residual ** 2)))}


def _line(points: np.ndarray, remaining: np.ndarray, config: WallConfig, rng: Random) -> Optional[dict]:
    if len(remaining) < config.min_inliers: return None
    best = np.array([], dtype=int)
    for _ in range(config.iterations):
        first, second = rng.sample(remaining.tolist(), 2)
        origin, vector = points[first], points[second] - points[first]
        length = np.linalg.norm(vector)
        if length < 2.0 * config.distance_threshold: continue
        direction = vector / length
        residual = np.abs((points[remaining] - origin) @ np.array([-direction[1], direction[0]]))
        inliers = remaining[residual <= config.distance_threshold]
        if len(inliers) > len(best): best = inliers
    for _ in range(2):
        fit = _fit(points, best)
        if fit is None: return None
        residual = np.abs((points[remaining] - fit["center"]) @ np.array([-fit["direction"][1], fit["direction"][0]]))
        best = remaining[residual <= config.distance_threshold]
    return _fit(points, best) if len(best) >= config.min_inliers else None


def estimate_wall(points: list[Point2D], config: WallConfig, preferred_angle: Optional[float] = None) -> Optional[WallEstimate]:
    if config.min_inliers < 2 or config.distance_threshold <= 0 or config.iterations <= 0 or config.selection not in {"tracked", "strongest", "nearest"}:
        raise ValueError("invalid wall estimator configuration")
    cloud = np.asarray([(point.x, point.y) for point in points], dtype=float)
    if len(cloud) < config.min_inliers: return None
    remaining, candidates, rng = np.arange(len(cloud)), [], Random(0xC0CE10)
    for _ in range(config.max_candidates):
        fit = _line(cloud, remaining, config, rng)
        if fit is None: break
        closest = fit["closest"]
        if fit["length"] >= config.min_length and fit["rmse"] <= config.max_rmse and _sector(atan2(closest[1], closest[0]), config.sector, config.sector_half_angle): candidates.append(fit)
        remaining = remaining[~np.isin(remaining, fit["indices"])]
        if len(remaining) < config.min_inliers: break
    if not candidates: return None
    if config.selection == "nearest": chosen = min(candidates, key=lambda item: (np.linalg.norm(item["closest"]), -len(item["indices"])))
    elif config.selection == "tracked" and preferred_angle is not None:
        nearest = min(candidates, key=lambda item: abs(normalize_wall_angle(atan2(item["direction"][1], item["direction"][0]) - preferred_angle)))
        chosen = nearest if abs(normalize_wall_angle(atan2(nearest["direction"][1], nearest["direction"][0]) - preferred_angle)) <= config.tracking_max_angle else max(candidates, key=lambda item: len(item["indices"]) * item["length"])
    else: chosen = max(candidates, key=lambda item: len(item["indices"]) * item["length"])
    point = lambda value: Point2D(float(value[0]), float(value[1]))
    return WallEstimate(normalize_wall_angle(atan2(chosen["direction"][1], chosen["direction"][0])), float(np.linalg.norm(chosen["closest"])), chosen["rmse"], chosen["length"], len(chosen["indices"]), point(chosen["start"]), point(chosen["end"]), point(chosen["closest"]))
