import math
from types import SimpleNamespace

from new_map_planner_ui.model import (
    VehiclePose,
    front_obstacle_polygon,
    map_to_arena,
    maneuver_name,
    merge_obstacle,
    quaternion_yaw,
)


def quaternion(yaw):
    return SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(0.5 * yaw),
        w=math.cos(0.5 * yaw),
    )


def test_quaternion_yaw_rejects_invalid_and_recovers_heading():
    assert quaternion_yaw(SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)) is None
    assert math.isclose(quaternion_yaw(quaternion(1.2)), 1.2, abs_tol=1.0e-9)


def test_map_origin_and_heading_match_real_launch_frame():
    config = {
        "frame": {"map_origin_m": [1.6, 4.3], "map_x_heading_deg": -90.0}
    }
    x, y, yaw = map_to_arena(0.2, 0.5, 0.0, config)
    assert math.isclose(x, 2.1, abs_tol=1.0e-9)
    assert math.isclose(y, 4.1, abs_tol=1.0e-9)
    assert math.isclose(yaw, -0.5 * math.pi, abs_tol=1.0e-9)


def test_all_public_maneuver_values_have_operator_labels():
    assert [maneuver_name(value) for value in range(6)] == [
        "直行",
        "左转",
        "右转",
        "起步对正",
        "完成",
        "倒车退回",
    ]


def test_front_range_projects_an_oriented_real_obstacle():
    pose = VehiclePose(1.0, 2.0, 0.0, 0.0)
    polygon = front_obstacle_polygon(
        pose, measured_range=0.4, camera_forward_offset=0.1,
        obstacle_width=0.2, obstacle_depth=0.1
    )
    expected = [
        (1.5, 1.9),
        (1.6, 1.9),
        (1.6, 2.1),
        (1.5, 2.1),
    ]
    assert all(
        math.isclose(actual_value, expected_value, abs_tol=1.0e-9)
        for actual, expected_point in zip(polygon, expected)
        for actual_value, expected_value in zip(actual, expected_point)
    )


def test_repeated_obstacle_observations_replace_nearby_estimate():
    first = [[(1.0, 1.0), (1.1, 1.0), (1.1, 1.1), (1.0, 1.1)]]
    nearby = [(1.05, 1.0), (1.15, 1.0), (1.15, 1.1), (1.05, 1.1)]
    distant = [(2.0, 2.0), (2.1, 2.0), (2.1, 2.1), (2.0, 2.1)]

    assert merge_obstacle(first, nearby) == [nearby]
    assert merge_obstacle(first, distant) == first + [distant]
