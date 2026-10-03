import math

from vehicle_loopback_sim.motion import (
    Pose2D,
    Twist2D,
    integrate,
    integrate_base_link,
    limited_twist,
    normalize_angle,
    wheel_speeds,
)


def test_integrates_straight_motion_in_heading_direction():
    pose = integrate(Pose2D(1.0, 2.0, math.pi / 2.0), Twist2D(0.2, 0.0), 0.5)

    assert math.isclose(pose.x, 1.0, abs_tol=1.0e-12)
    assert math.isclose(pose.y, 2.1, abs_tol=1.0e-12)
    assert math.isclose(pose.yaw, math.pi / 2.0, abs_tol=1.0e-12)


def test_integrates_arc_motion_exactly():
    pose = integrate(Pose2D(), Twist2D(0.2, 1.0), 0.5)

    assert math.isclose(pose.x, 0.2 * math.sin(0.5), rel_tol=1.0e-12)
    assert math.isclose(pose.y, 0.2 * (1.0 - math.cos(0.5)), rel_tol=1.0e-12)
    assert math.isclose(pose.yaw, 0.5, abs_tol=1.0e-12)


def test_integrates_base_link_with_forward_axle_offset():
    pose = integrate_base_link(Pose2D(), Twist2D(0.0, 1.0), 0.5, 0.05, 0.0)

    assert math.isclose(pose.x, 0.05 * (math.cos(0.5) - 1.0), abs_tol=1.0e-12)
    assert math.isclose(pose.y, 0.05 * math.sin(0.5), abs_tol=1.0e-12)
    assert math.isclose(pose.yaw, 0.5, abs_tol=1.0e-12)


def test_normalizes_angle_to_pi_range():
    assert math.isclose(normalize_angle(3.5 * math.pi), -0.5 * math.pi)
    assert math.isclose(normalize_angle(-3.5 * math.pi), 0.5 * math.pi)


def test_limited_twist_applies_speed_and_acceleration_limits():
    twist = limited_twist(
        Twist2D(1.0, -2.0),
        Twist2D(0.0, 0.0),
        0.5,
        max_linear_speed=0.4,
        max_angular_speed=1.0,
        max_linear_accel=0.2,
        max_angular_accel=0.6,
    )

    assert math.isclose(twist.linear_x, 0.1)
    assert math.isclose(twist.angular_z, -0.3)


def test_wheel_speeds_follow_differential_drive_signs():
    left, right = wheel_speeds(Twist2D(0.10, 1.0), 0.12)

    assert math.isclose(left, 0.04)
    assert math.isclose(right, 0.16)
