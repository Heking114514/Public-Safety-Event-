import math

from rectangle_odometry_test.geometry import (
    Pose2D,
    build_clockwise_rectangle,
    build_clockwise_rounded_rectangle,
    wrap_angle,
)


def test_clockwise_rectangle_is_closed_and_has_four_turns():
    waypoints = build_clockwise_rectangle(Pose2D(0.0, 0.0, 0.0), 1.8)

    assert len(waypoints) == 4
    assert [(round(item.pose.x, 3), round(item.pose.y, 3)) for item in waypoints] == [
        (1.8, 0.0),
        (1.8, -1.8),
        (0.0, -1.8),
        (0.0, 0.0),
    ]
    assert [item.turn_junction for item in waypoints] == [
        True,
        True,
        True,
        False,
    ]
    assert math.isclose(waypoints[0].pose.yaw, 0.0)
    assert math.isclose(waypoints[1].pose.yaw, -math.pi / 2.0)
    assert math.isclose(abs(waypoints[2].pose.yaw), math.pi)
    assert math.isclose(waypoints[3].pose.yaw, math.pi / 2.0)


def test_rectangle_follows_origin_heading():
    origin = Pose2D(2.0, 3.0, math.pi / 2.0)
    waypoints = build_clockwise_rectangle(origin, 1.8)

    assert math.isclose(waypoints[0].pose.x, 2.0, abs_tol=1.0e-9)
    assert math.isclose(waypoints[0].pose.y, 4.8, abs_tol=1.0e-9)
    assert math.isclose(waypoints[1].pose.x, 3.8, abs_tol=1.0e-9)
    assert math.isclose(waypoints[1].pose.y, 4.8, abs_tol=1.0e-9)


def test_rounded_rectangle_is_closed_without_stop_markers():
    waypoints = build_clockwise_rounded_rectangle(
        Pose2D(0.0, 0.0, 0.0), 1.8, 0.25, 16, 0.05
    )

    assert len(waypoints) == 168
    assert not any(item.turn_junction for item in waypoints)
    assert math.isclose(waypoints[-1].pose.x, 0.0, abs_tol=1.0e-9)
    assert math.isclose(waypoints[-1].pose.y, 0.0, abs_tol=1.0e-9)
    assert math.isclose(waypoints[-1].pose.yaw, 0.0, abs_tol=1.0e-9)
    assert math.isclose(waypoints[0].pose.x, 0.05, abs_tol=1.0e-9)
    assert math.isclose(waypoints[0].pose.y, 0.0, abs_tol=1.0e-9)


def test_rounded_rectangle_step_heading_stays_below_stop_threshold():
    waypoints = build_clockwise_rounded_rectangle(
        Pose2D(0.0, 0.0, 0.0), 1.8, 0.25, 16, 0.05
    )

    max_delta = 0.0
    max_distance = 0.0
    for left, right in zip(waypoints, waypoints[1:]):
        max_delta = max(max_delta, abs(wrap_angle(right.pose.yaw - left.pose.yaw)))
        max_distance = max(
            max_distance,
            math.hypot(right.pose.x - left.pose.x, right.pose.y - left.pose.y),
        )
    assert max_delta < 0.11
    assert max_distance <= 0.051
