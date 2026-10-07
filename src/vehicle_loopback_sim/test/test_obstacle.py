import math

from vehicle_loopback_sim.motion import Pose2D
from vehicle_loopback_sim.obstacle import (
    ObstacleDetection,
    ObstacleDetectionConfig,
    SimulatedObstacleDetector,
    StableObstacleGate,
    box_obstacle,
)


def _box(min_x, min_y, max_x, max_y):
    return [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ]


def test_detects_forward_obstacle_inside_trigger_range():
    detector = SimulatedObstacleDetector(
        [_box(0.80, -0.10, 0.95, 0.10)],
        ObstacleDetectionConfig(
            sensor_x_m=0.0,
            sensor_y_m=0.0,
            trigger_range_m=1.0,
            trigger_jitter_m=0.5,
            lateral_tolerance_m=0.06,
            random_seed=5,
        ),
    )

    detection = detector.detect(Pose2D())

    assert detection is not None
    assert detection.obstacle_index == 0
    assert math.isclose(detection.range_m, 0.80)


def test_ignores_side_and_behind_obstacles():
    detector = SimulatedObstacleDetector(
        [
            _box(0.40, 0.20, 0.60, 0.40),
            _box(-0.60, -0.04, -0.40, 0.04),
        ],
        ObstacleDetectionConfig(
            sensor_x_m=0.0,
            sensor_y_m=0.0,
            trigger_range_m=1.0,
            trigger_jitter_m=0.5,
            lateral_tolerance_m=0.06,
        ),
    )

    assert detector.detect(Pose2D()) is None


def test_does_not_start_with_already_too_close_static_obstacle():
    detector = SimulatedObstacleDetector(
        [_box(0.24, -0.04, 0.34, 0.04)],
        ObstacleDetectionConfig(
            sensor_x_m=0.0,
            sensor_y_m=0.0,
            trigger_range_m=1.0,
            trigger_jitter_m=0.5,
            lateral_tolerance_m=0.06,
        ),
    )

    assert detector.detect(Pose2D()) is None
    assert detector.detect(Pose2D(-0.40, 0.0, 0.0)) is None


def test_fixed_trigger_range_detects_close_manual_obstacle():
    detector = SimulatedObstacleDetector(
        [_box(0.15, -0.10, 0.25, 0.10)],
        ObstacleDetectionConfig(
            sensor_x_m=0.055,
            sensor_y_m=-0.015,
            trigger_range_m=0.25,
            trigger_jitter_m=0.0,
            lateral_tolerance_m=0.08,
            min_range_m=0.03,
            activation_floor_m=0.03,
        ),
    )

    detection = detector.detect(Pose2D())

    assert detection is not None
    assert math.isclose(detection.range_m, 0.095)


def test_waits_until_vehicle_reaches_trigger_distance():
    detector = SimulatedObstacleDetector(
        [_box(1.20, -0.04, 1.30, 0.04)],
        ObstacleDetectionConfig(
            sensor_x_m=0.0,
            sensor_y_m=0.0,
            trigger_range_m=1.0,
            trigger_jitter_m=0.5,
            lateral_tolerance_m=0.06,
            random_seed=5,
        ),
    )

    assert detector.detect(Pose2D()) is None
    detection = detector.detect(Pose2D(0.25, 0.0, 0.0))

    assert detection is not None
    assert math.isclose(detection.range_m, 0.95)


def test_camera_forward_offset_reduces_reported_range():
    detector = SimulatedObstacleDetector(
        [_box(1.00, -0.04, 1.10, 0.04)],
        ObstacleDetectionConfig(
            sensor_x_m=0.10,
            sensor_y_m=0.0,
            trigger_range_m=1.0,
            trigger_jitter_m=0.5,
            lateral_tolerance_m=0.06,
        ),
    )

    detection = detector.detect(Pose2D())

    assert detection is not None
    assert math.isclose(detection.range_m, 0.90)


def test_box_obstacle_uses_clicked_point_as_center():
    obstacle = box_obstacle(1.0, 2.0, width_m=0.24, depth_m=0.12)

    assert obstacle == (
        (0.94, 1.88),
        (1.06, 1.88),
        (1.06, 2.12),
        (0.94, 2.12),
    )


def test_box_obstacle_can_be_crosswise_to_vehicle_heading():
    obstacle = box_obstacle(
        1.0,
        2.0,
        width_m=0.24,
        depth_m=0.12,
        yaw_rad=0.5 * math.pi,
    )

    expected = (
        (1.12, 1.94),
        (1.12, 2.06),
        (0.88, 2.06),
        (0.88, 1.94),
    )
    for point, expected_point in zip(obstacle, expected):
        assert math.isclose(point[0], expected_point[0], abs_tol=1.0e-12)
        assert math.isclose(point[1], expected_point[1], abs_tol=1.0e-12)


def test_stable_obstacle_gate_waits_for_continuous_detection():
    gate = StableObstacleGate(stable_time_s=2.0)

    assert gate.update(ObstacleDetection(3, 0.40, 0.50), 10.0) is None
    assert gate.update(ObstacleDetection(3, 0.38, 0.50), 11.9) is None
    detection = gate.update(ObstacleDetection(3, 0.36, 0.50), 12.0)

    assert detection is not None
    assert detection.obstacle_index == 3
    assert math.isclose(detection.range_m, 0.36)


def test_stable_obstacle_gate_resets_on_loss_or_new_obstacle():
    gate = StableObstacleGate(stable_time_s=2.0)

    assert gate.update(ObstacleDetection(3, 0.40, 0.50), 10.0) is None
    assert gate.update(None, 11.0) is None
    assert gate.update(ObstacleDetection(3, 0.36, 0.50), 12.0) is None
    assert gate.update(ObstacleDetection(4, 0.34, 0.50), 13.0) is None
    detection = gate.update(ObstacleDetection(4, 0.32, 0.50), 15.0)

    assert detection is not None
    assert detection.obstacle_index == 4
