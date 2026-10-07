import ast
import math
from pathlib import Path

import pytest

from vehicle_loopback_sim.arena_layout import (
    ArenaTransform,
    layout_from_config,
    nearest_lane_geometry,
)
from vehicle_loopback_sim.mission import _parse_ids, _parse_modes, _parse_status_topics


MISSION_SOURCE = (
    Path(__file__).resolve().parents[1] / "vehicle_loopback_sim" / "mission.py"
)


def test_parse_modes_accepts_single_mode_and_sequence():
    assert _parse_modes("targets") == ["targets"]
    assert _parse_modes("targets, coverage,mission") == [
        "targets",
        "coverage",
        "mission",
    ]
    assert _parse_modes("layer1,layer2,layer3") == [
        "layer1",
        "layer2",
        "layer3",
    ]


def test_parse_modes_rejects_empty_or_unknown_mode():
    with pytest.raises(ValueError):
        _parse_modes("  ")
    with pytest.raises(ValueError):
        _parse_modes("targets,sidequest")


def test_parse_ids_deduplicates_comma_separated_values():
    assert _parse_ids("12, 1,12") == ["12", "1"]
    assert _parse_ids("") == []


def test_parse_status_topics_uses_fallback_and_removes_duplicates():
    assert _parse_status_topics("", "/status") == ["/status"]
    assert (
        _parse_status_topics(
            "/new/status, /old/status, /new/status",
            "/unused",
        )
        == ["/new/status", "/old/status"]
    )


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


def test_new_map_layout_resolves_named_nodes_and_lanes():
    map_path = (
        Path(__file__).resolve().parents[2]
        / "new_map_planner"
        / "config"
        / "arena.yaml"
    )
    layout = layout_from_config(map_path)

    assert len(layout["tasks"]) == 12
    assert len(layout["inspection_edges"]) == 29
    assert layout["start"] == {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
    assert any(edge["label"] == "ROAD_TOP_RIGHT" for edge in layout["inspection_edges"])
    assert any(
        edge["label"] == "ROAD_TOP_RIGHT" and math.isclose(edge["width_m"], 0.20)
        for edge in layout["inspection_edges"]
    )


def test_nearest_lane_geometry_snaps_to_lane_direction_and_width():
    layout = {
        "inspection_edges": [
            {
                "label": "horizontal",
                "from": {"x_m": 0.0, "y_m": 0.0},
                "to": {"x_m": 1.0, "y_m": 0.0},
                "width_m": 0.20,
            },
            {
                "label": "vertical",
                "from": {"x_m": 2.0, "y_m": 0.0},
                "to": {"x_m": 2.0, "y_m": 1.0},
                "width_m": 0.30,
            },
        ]
    }

    horizontal = nearest_lane_geometry(layout, 0.45, 0.08, 0.10)
    vertical = nearest_lane_geometry(layout, 1.93, 0.75, 0.10)

    assert horizontal is not None
    assert horizontal["label"] == "horizontal"
    assert math.isclose(horizontal["x_m"], 0.45)
    assert math.isclose(horizontal["y_m"], 0.0)
    assert math.isclose(horizontal["yaw_rad"], 0.0)
    assert math.isclose(horizontal["width_m"], 0.20)

    assert vertical is not None
    assert vertical["label"] == "vertical"
    assert math.isclose(vertical["x_m"], 2.0)
    assert math.isclose(vertical["y_m"], 0.75)
    assert math.isclose(vertical["yaw_rad"], 0.5 * math.pi)
    assert math.isclose(vertical["width_m"], 0.30)


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
