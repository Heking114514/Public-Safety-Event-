import ast
import math
from pathlib import Path

import pytest

from vehicle_loopback_sim.arena_layout import ArenaTransform
from vehicle_loopback_sim.mission import _parse_modes


MISSION_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "vehicle_loopback_sim"
    / "mission.py"
)


def test_parse_modes_accepts_single_mode_and_sequence():
    assert _parse_modes("layer2") == ["layer2"]
    assert _parse_modes("layer1, layer2,layer3") == [
        "layer1",
        "layer2",
        "layer3",
    ]


def test_parse_modes_rejects_empty_or_unknown_mode():
    with pytest.raises(ValueError):
        _parse_modes("  ")
    with pytest.raises(ValueError):
        _parse_modes("layer1,sidequest")


def test_arena_transform_round_trips_map_pose():
    transform = ArenaTransform(1.6, 4.3, -0.5 * math.pi)
    mapped = transform.point_to_map(2.1, 3.1)
    restored = transform.point_from_map(mapped["x_m"], mapped["y_m"])

    assert math.isclose(restored["x_m"], 2.1, abs_tol=1.0e-12)
    assert math.isclose(restored["y_m"], 3.1, abs_tol=1.0e-12)
    assert math.isclose(
        transform.yaw_from_map(transform.yaw_to_map(0.75)),
        0.75,
        abs_tol=1.0e-12,
    )


def test_mission_logger_calls_use_rclpy_signature():
    tree = ast.parse(MISSION_SOURCE.read_text(encoding="utf-8"))
    logger_methods = {"debug", "info", "warning", "warn", "error", "fatal"}
    bad_calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr not in logger_methods or len(node.args) <= 1:
            continue
        bad_calls.append(node.lineno)

    assert bad_calls == []
