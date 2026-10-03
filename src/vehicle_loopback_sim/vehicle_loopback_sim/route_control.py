"""Pure helpers for loopback route tracking."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float
    turn_junction: bool


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Projection:
    remaining: float
    cross_track: float
    heading: float
    length: float
    valid: bool


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def target_distance(left: Pose2D | Waypoint, right: Waypoint) -> float:
    return math.hypot(right.x - left.x, right.y - left.y)


def project(start: Waypoint | Pose2D, target: Waypoint, pose: Pose2D) -> Projection:
    dx = target.x - start.x
    dy = target.y - start.y
    length = math.hypot(dx, dy)
    if length <= 1.0e-6:
        heading = math.atan2(target.y - pose.y, target.x - pose.x)
        return Projection(
            math.hypot(target.x - pose.x, target.y - pose.y),
            0.0,
            heading,
            0.0,
            False,
        )
    ux = dx / length
    uy = dy / length
    from_start_x = pose.x - start.x
    from_start_y = pose.y - start.y
    progress = ux * from_start_x + uy * from_start_y
    cross_track = ux * from_start_y - uy * from_start_x
    return Projection(length - progress, cross_track, math.atan2(dy, dx), length, True)


def parse_path(message) -> list[Waypoint]:
    waypoints: list[Waypoint] = []
    for stamped in message.poses:
        pose = stamped.pose
        if not (
            math.isfinite(pose.position.x)
            and math.isfinite(pose.position.y)
            and math.isfinite(pose.position.z)
        ):
            continue
        waypoints.append(
            Waypoint(
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
                abs(float(pose.position.z) - 1.0e-3) <= 1.0e-6,
            )
        )
    return waypoints
