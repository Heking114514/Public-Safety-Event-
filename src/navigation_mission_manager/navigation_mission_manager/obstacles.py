from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


Point = tuple[float, float]
Polygon = tuple[Point, ...]


def obstacle_memory_expired(
    observed_at: float | None, now: float, retention_s: float
) -> bool:
    return (
        observed_at is not None
        and retention_s > 0.0
        and now - observed_at >= retention_s
    )


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def front_obstacle_polygon(
    pose: Pose2D,
    measured_range: float,
    sensor_forward_offset: float,
    obstacle_width: float,
    obstacle_depth: float,
) -> Polygon:
    values = (
        pose.x,
        pose.y,
        pose.yaw,
        measured_range,
        sensor_forward_offset,
        obstacle_width,
        obstacle_depth,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("obstacle projection values must be finite")
    if measured_range <= 0.0 or sensor_forward_offset < 0.0:
        raise ValueError("obstacle range and sensor offset are invalid")
    if obstacle_width <= 0.0 or obstacle_depth <= 0.0:
        raise ValueError("obstacle dimensions must be positive")

    forward = (math.cos(pose.yaw), math.sin(pose.yaw))
    side = (-forward[1], forward[0])
    distance = sensor_forward_offset + measured_range + 0.5 * obstacle_depth
    center = (pose.x + forward[0] * distance, pose.y + forward[1] * distance)
    return tuple(
        (
            center[0]
            + forward[0] * depth * 0.5 * obstacle_depth
            + side[0] * width * 0.5 * obstacle_width,
            center[1]
            + forward[1] * depth * 0.5 * obstacle_depth
            + side[1] * width * 0.5 * obstacle_width,
        )
        for depth, width in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
    )


def valid_polygon(points: Iterable[Point]) -> Polygon:
    polygon = tuple((float(x), float(y)) for x, y in points)
    if len(polygon) < 3 or not all(
        math.isfinite(value) for point in polygon for value in point
    ):
        raise ValueError("dynamic obstacle polygon is invalid")
    return polygon


def merge_obstacle(
    obstacles: Iterable[Polygon], candidate: Polygon, center_tolerance: float
) -> list[Polygon]:
    if center_tolerance <= 0.0:
        raise ValueError("obstacle merge tolerance must be positive")
    center = polygon_center(candidate)
    merged = list(obstacles)
    for index, obstacle in enumerate(merged):
        existing = polygon_center(obstacle)
        distance = math.hypot(existing[0] - center[0], existing[1] - center[1])
        if distance <= center_tolerance:
            merged[index] = candidate
            return merged
    merged.append(candidate)
    return merged


def polygon_center(polygon: Polygon) -> Point:
    if not polygon:
        raise ValueError("polygon must not be empty")
    return (
        sum(point[0] for point in polygon) / len(polygon),
        sum(point[1] for point in polygon) / len(polygon),
    )
