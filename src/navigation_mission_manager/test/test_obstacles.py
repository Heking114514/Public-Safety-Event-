import math

from navigation_mission_manager.obstacles import (
    Pose2D,
    front_obstacle_polygon,
    merge_obstacle,
    obstacle_memory_expired,
    polygon_center,
)


def test_front_obstacle_projection_uses_sensor_offset_and_heading():
    polygon = front_obstacle_polygon(Pose2D(1.0, 2.0, 0.0), 0.5, 0.1, 0.2, 0.1)
    center = polygon_center(polygon)

    assert math.isclose(center[0], 1.65)
    assert math.isclose(center[1], 2.0)


def test_nearby_obstacle_observation_replaces_existing_shape():
    first = ((0.0, 0.0), (0.1, 0.0), (0.1, 0.1), (0.0, 0.1))
    second = ((0.1, 0.0), (0.2, 0.0), (0.2, 0.1), (0.1, 0.1))

    assert merge_obstacle((first,), second, 0.2) == [second]


def test_dynamic_obstacle_memory_expires_only_after_positive_retention():
    assert not obstacle_memory_expired(None, 20.0, 10.0)
    assert not obstacle_memory_expired(10.0, 20.0, 0.0)
    assert not obstacle_memory_expired(10.0, 19.9, 10.0)
    assert obstacle_memory_expired(10.0, 20.0, 10.0)
