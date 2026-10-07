from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Optional


ODOMETRY_TIMEOUT_S = 0.6


@dataclass(frozen=True)
class PlanSelection:
    mode: str
    target_ids: tuple[str, ...] = ()
    required_lane_ids: tuple[str, ...] = ()
    completed_lane_ids: tuple[str, ...] = ()
    return_home: bool = True
    allow_partial: bool = False


@dataclass(frozen=True)
class VehiclePose:
    x: float
    y: float
    yaw: float
    received_at: float

    def fresh(self, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else now
        return current - self.received_at <= ODOMETRY_TIMEOUT_S


def quaternion_yaw(quaternion: Any) -> Optional[float]:
    values = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    if not all(math.isfinite(value) for value in values):
        return None
    norm = sum(value * value for value in values)
    if norm <= 1.0e-12:
        return None
    x, y, z, w = (value / math.sqrt(norm) for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def map_to_arena(
    x: float, y: float, yaw: float, config: dict[str, Any]
) -> tuple[float, float, float]:
    origin_x, origin_y = map(float, config["frame"]["map_origin_m"])
    origin_yaw = math.radians(float(config["frame"]["map_x_heading_deg"]))
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return (
        origin_x + cosine * x - sine * y,
        origin_y + sine * x + cosine * y,
        math.remainder(yaw + origin_yaw, 2.0 * math.pi),
    )


def path_points(path: Any) -> list[tuple[float, float]]:
    return [
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in path.poses
    ]


def maneuver_name(value: int) -> str:
    return {
        0: "直行",
        1: "左转",
        2: "右转",
        3: "起步对正",
        4: "完成",
        5: "倒车退回",
    }.get(int(value), f"未知({value})")


def front_obstacle_polygon(
    pose: VehiclePose,
    measured_range: float,
    camera_forward_offset: float,
    obstacle_width: float = 0.24,
    obstacle_depth: float = 0.12,
) -> list[tuple[float, float]]:
    values = (
        pose.x,
        pose.y,
        pose.yaw,
        measured_range,
        camera_forward_offset,
        obstacle_width,
        obstacle_depth,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("障碍投影参数必须为有限值")
    if measured_range <= 0.0 or camera_forward_offset < 0.0:
        raise ValueError("障碍距离必须为正且相机偏移不能为负")
    if obstacle_width <= 0.0 or obstacle_depth <= 0.0:
        raise ValueError("障碍尺寸必须为正")

    forward = (math.cos(pose.yaw), math.sin(pose.yaw))
    side = (-forward[1], forward[0])
    center_distance = camera_forward_offset + measured_range + 0.5 * obstacle_depth
    center = (
        pose.x + forward[0] * center_distance,
        pose.y + forward[1] * center_distance,
    )
    return [
        (
            center[0]
            + forward[0] * depth_sign * 0.5 * obstacle_depth
            + side[0] * width_sign * 0.5 * obstacle_width,
            center[1]
            + forward[1] * depth_sign * 0.5 * obstacle_depth
            + side[1] * width_sign * 0.5 * obstacle_width,
        )
        for depth_sign, width_sign in (
            (-1.0, -1.0),
            (1.0, -1.0),
            (1.0, 1.0),
            (-1.0, 1.0),
        )
    ]


def merge_obstacle(
    obstacles: list[list[tuple[float, float]]],
    candidate: list[tuple[float, float]],
    center_tolerance: float = 0.18,
) -> list[list[tuple[float, float]]]:
    if len(candidate) < 3 or center_tolerance <= 0.0:
        raise ValueError("障碍多边形和合并距离无效")
    candidate_center = _polygon_center(candidate)
    merged = list(obstacles)
    for index, obstacle in enumerate(merged):
        center = _polygon_center(obstacle)
        distance = math.hypot(
            center[0] - candidate_center[0], center[1] - candidate_center[1]
        )
        if distance <= center_tolerance:
            merged[index] = candidate
            return merged
    merged.append(candidate)
    return merged[-20:]


def _polygon_center(polygon: list[tuple[float, float]]) -> tuple[float, float]:
    if len(polygon) < 3:
        raise ValueError("障碍多边形至少需要三个点")
    if not all(math.isfinite(value) for point in polygon for value in point):
        raise ValueError("障碍多边形坐标必须为有限值")
    return (
        sum(point[0] for point in polygon) / len(polygon),
        sum(point[1] for point in polygon) / len(polygon),
    )
