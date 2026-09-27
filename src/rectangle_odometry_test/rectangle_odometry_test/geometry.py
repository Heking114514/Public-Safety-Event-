"""Pure geometry for the rectangular motion test."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RectangleWaypoint:
    pose: Pose2D
    turn_junction: bool


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def local_to_world(origin: Pose2D, local_x: float, local_y: float) -> Pose2D:
    cosine = math.cos(origin.yaw)
    sine = math.sin(origin.yaw)
    return Pose2D(
        origin.x + cosine * local_x - sine * local_y,
        origin.y + sine * local_x + cosine * local_y,
        origin.yaw,
    )


def local_pose_to_world(
    origin: Pose2D, local_x: float, local_y: float, local_yaw: float
) -> Pose2D:
    position = local_to_world(origin, local_x, local_y)
    return Pose2D(position.x, position.y, wrap_angle(origin.yaw + local_yaw))


def build_clockwise_rectangle(
    origin: Pose2D, side_length_m: float
) -> list[RectangleWaypoint]:
    """Build targets for a clockwise closed rectangle.

    The first target is one side length ahead of the current vehicle pose.
    The final target is the starting position after the fourth side. The
    existing navigator completes at that position without adding a fifth leg.
    """
    if not math.isfinite(side_length_m) or side_length_m <= 0.0:
        raise ValueError("side_length_m must be finite and positive")

    local_targets = (
        (side_length_m, 0.0, 0.0, True),
        (side_length_m, -side_length_m, -0.5 * math.pi, True),
        (0.0, -side_length_m, math.pi, True),
        (0.0, 0.0, 0.5 * math.pi, False),
    )
    waypoints: list[RectangleWaypoint] = []
    for local_x, local_y, local_yaw, turn_junction in local_targets:
        position = local_to_world(origin, local_x, local_y)
        waypoints.append(
            RectangleWaypoint(
                Pose2D(
                    position.x,
                    position.y,
                    wrap_angle(origin.yaw + local_yaw),
                ),
                turn_junction,
            )
        )
    return waypoints


def build_clockwise_rounded_rectangle(
    origin: Pose2D,
    side_length_m: float,
    corner_radius_m: float,
    corner_samples: int,
    straight_step_m: float = 0.05,
) -> list[RectangleWaypoint]:
    """Build a closed clockwise route with continuous quarter-circle corners.

    The origin is a tangent point on the first straight, not a sharp corner.
    Every waypoint is ordinary, so the existing navigator tracks through the
    corner instead of stopping and pivoting at a marked junction.
    """
    if not math.isfinite(side_length_m) or side_length_m <= 0.0:
        raise ValueError("side_length_m must be finite and positive")
    if not math.isfinite(corner_radius_m) or corner_radius_m <= 0.0:
        raise ValueError("corner_radius_m must be finite and positive")
    if corner_radius_m * 2.0 >= side_length_m:
        raise ValueError("corner_radius_m must be less than half the side length")
    if corner_samples < 2:
        raise ValueError("corner_samples must be at least 2")
    if not math.isfinite(straight_step_m) or straight_step_m <= 0.0:
        raise ValueError("straight_step_m must be finite and positive")

    radius = corner_radius_m
    straight = side_length_m - 2.0 * radius
    waypoints: list[RectangleWaypoint] = []

    def append(local_x: float, local_y: float, local_yaw: float) -> None:
        waypoints.append(
            RectangleWaypoint(
                local_pose_to_world(origin, local_x, local_y, local_yaw),
                False,
            )
        )

    def append_straight(
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
        local_yaw: float,
    ) -> None:
        length = math.hypot(end_x - start_x, end_y - start_y)
        samples = max(1, int(math.ceil(length / straight_step_m)))
        for sample in range(1, samples + 1):
            ratio = float(sample) / float(samples)
            append(
                start_x + (end_x - start_x) * ratio,
                start_y + (end_y - start_y) * ratio,
                local_yaw,
            )

    def append_arc(
        center_x: float, center_y: float, start_theta: float, end_theta: float
    ) -> None:
        step = (end_theta - start_theta) / float(corner_samples)
        for sample in range(1, corner_samples + 1):
            theta = start_theta + step * sample
            append(
                center_x + radius * math.cos(theta),
                center_y + radius * math.sin(theta),
                theta - 0.5 * math.pi,
            )

    append_straight(0.0, 0.0, straight, 0.0, 0.0)
    append_arc(straight, -radius, 0.5 * math.pi, 0.0)

    append_straight(
        straight + radius, -radius,
        straight + radius, -(straight + radius),
        -0.5 * math.pi,
    )
    append_arc(straight, -(straight + radius), 0.0, -0.5 * math.pi)

    append_straight(straight, -side_length_m, 0.0, -side_length_m, math.pi)
    append_arc(0.0, -(straight + radius), -0.5 * math.pi, -math.pi)

    append_straight(
        -radius, -(straight + radius),
        -radius, -radius,
        0.5 * math.pi,
    )
    append_arc(0.0, -radius, math.pi, 0.5 * math.pi)

    return waypoints
