"""Ideal planar motion model for loopback navigation tests."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Twist2D:
    linear_x: float = 0.0
    angular_z: float = 0.0


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def limit_rate(
    desired: float,
    current: float,
    limit_per_s: float,
    elapsed_s: float,
) -> float:
    if limit_per_s <= 0.0 or elapsed_s <= 0.0:
        return desired
    step = limit_per_s * elapsed_s
    return current + clamp(desired - current, -step, step)


def limited_twist(
    desired: Twist2D,
    current: Twist2D,
    elapsed_s: float,
    max_linear_speed: float,
    max_angular_speed: float,
    max_linear_accel: float,
    max_angular_accel: float,
) -> Twist2D:
    linear = desired.linear_x
    angular = desired.angular_z
    if max_linear_speed > 0.0:
        linear = clamp(linear, -max_linear_speed, max_linear_speed)
    if max_angular_speed > 0.0:
        angular = clamp(angular, -max_angular_speed, max_angular_speed)
    linear = limit_rate(linear, current.linear_x, max_linear_accel, elapsed_s)
    angular = limit_rate(angular, current.angular_z, max_angular_accel, elapsed_s)
    return Twist2D(linear, angular)


def integrate(pose: Pose2D, twist: Twist2D, elapsed_s: float) -> Pose2D:
    if elapsed_s <= 0.0:
        return pose
    if abs(twist.angular_z) <= 1.0e-9:
        x = pose.x + twist.linear_x * elapsed_s * math.cos(pose.yaw)
        y = pose.y + twist.linear_x * elapsed_s * math.sin(pose.yaw)
        return Pose2D(x, y, pose.yaw)

    next_yaw = pose.yaw + twist.angular_z * elapsed_s
    radius = twist.linear_x / twist.angular_z
    x = pose.x + radius * (math.sin(next_yaw) - math.sin(pose.yaw))
    y = pose.y - radius * (math.cos(next_yaw) - math.cos(pose.yaw))
    return Pose2D(x, y, normalize_angle(next_yaw))


def integrate_base_link(
    pose: Pose2D,
    twist: Twist2D,
    elapsed_s: float,
    base_offset_x_m: float,
    base_offset_y_m: float,
) -> Pose2D:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    wheel_pose = Pose2D(
        pose.x - cosine * base_offset_x_m + sine * base_offset_y_m,
        pose.y - sine * base_offset_x_m - cosine * base_offset_y_m,
        pose.yaw,
    )
    next_wheel = integrate(wheel_pose, twist, elapsed_s)
    next_cosine = math.cos(next_wheel.yaw)
    next_sine = math.sin(next_wheel.yaw)
    return Pose2D(
        next_wheel.x + next_cosine * base_offset_x_m - next_sine * base_offset_y_m,
        next_wheel.y + next_sine * base_offset_x_m + next_cosine * base_offset_y_m,
        next_wheel.yaw,
    )


def wheel_speeds(twist: Twist2D, wheel_track_m: float) -> tuple[float, float]:
    half_track = 0.5 * max(0.0, wheel_track_m)
    left = twist.linear_x - twist.angular_z * half_track
    right = twist.linear_x + twist.angular_z * half_track
    return left, right
