#!/usr/bin/env python3

import csv
import importlib.util
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "arena_route_planner.py"
CONFIG_PATH = PACKAGE_ROOT / "config" / "arena_map.yaml"
NAVIGATION_CONFIG_PATH = PACKAGE_ROOT / "config" / "waypoint_navigation.yaml"
SPEC = importlib.util.spec_from_file_location("arena_route_planner", SCRIPT_PATH)
PLANNER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = PLANNER
SPEC.loader.exec_module(PLANNER)


def test_default_map_contains_all_nodes_in_free_space():
    config = PLANNER.load_config(CONFIG_PATH)
    grid = PLANNER.build_grid(config)
    for point in (config.start,) + config.tasks:
        assert grid.point_is_free(point)


def test_monitor_topics_and_tracking_offsets_match_navigator():
    config = PLANNER.load_config(CONFIG_PATH)
    navigation = yaml.safe_load(NAVIGATION_CONFIG_PATH.read_text(encoding="utf-8"))[
        "waypoint_navigator"
    ]["ros__parameters"]
    monitor = config.monitoring
    assert monitor.route_frame == navigation["route_frame"]
    assert monitor.odom_topic == navigation["odom_topic"]
    assert monitor.fusion_status_topic == navigation["fusion_status_topic"]
    assert monitor.navigation_status_topic == navigation["status_topic"]
    assert monitor.current_waypoint_topic == navigation["current_waypoint_topic"]
    assert monitor.navigation_path_topic == navigation["path_topic"]
    assert monitor.route_plan_input_topic == navigation["route_plan_input_topic"]
    assert monitor.actuator_health_topic == navigation["actuator_health_topic"]
    assert monitor.tracking_offset_x == navigation["tracking_point_offset_x"]
    assert monitor.tracking_offset_y == navigation["tracking_point_offset_y"]


def test_shortest_plan_is_closed_collision_free_and_visits_every_task():
    config = PLANNER.load_config(CONFIG_PATH, "shortest")
    result = PLANNER.plan_route(config)

    assert result.node_order[0] == 0
    assert result.node_order[-1] == 0
    assert sorted(result.node_order[1:-1]) == list(range(1, 13))
    assert set(result.actual_first_visit_order) == set(range(1, 13))
    assert math.hypot(*result.route_ros[0]) < 1e-9
    assert math.hypot(*result.route_ros[-1]) < 1e-9
    assert all(
        result.grid.segment_is_free(start, end)
        for start, end in zip(result.route_field, result.route_field[1:])
    )


def test_numbered_mode_preserves_requested_task_order():
    config = PLANNER.load_config(CONFIG_PATH, "numbered")
    result = PLANNER.plan_route(config)
    assert result.node_order == list(range(13)) + [0]


def test_custom_target_subset_keeps_fixed_start_and_return():
    config = PLANNER.load_config(CONFIG_PATH, "shortest")
    selected = replace(
        config,
        tasks=(config.tasks[0], (1.1, 3.5), config.tasks[11]),
        task_labels=("1", "P1", "12"),
    )

    result = PLANNER.plan_route(selected)

    assert result.route_field[0] == pytest.approx(config.start)
    assert result.route_field[-1] == pytest.approx(config.start)
    assert sorted(result.node_order[1:-1]) == [1, 2, 3]
    assert set(result.actual_first_visit_order) == {1, 2, 3}
    assert {label for label in result.waypoint_labels.values()} >= {
        "START",
        "TASK_1",
        "TASK_P1",
        "TASK_12",
        "RETURN",
    }


def test_plan_rejects_empty_target_selection():
    config = PLANNER.load_config(CONFIG_PATH)
    empty = replace(config, tasks=(), task_labels=())

    with pytest.raises(PLANNER.PlanningError, match="至少选择"):
        PLANNER.plan_route(empty)


def test_arena_ros_coordinate_transform_round_trip():
    config = PLANNER.load_config(CONFIG_PATH)
    arena_point = (0.35, 2.45)
    ros_point = PLANNER._arena_to_ros(arena_point, config)
    restored = PLANNER.ros_to_arena(ros_point, config)
    assert restored == pytest.approx(arena_point)


def test_published_route_headings_follow_segments_and_end_at_zero():
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]
    yaws = PLANNER.route_yaws(points)

    assert yaws == pytest.approx([0.0, math.pi / 2.0, -3.0 * math.pi / 4.0, 0.0])
    assert PLANNER.route_fingerprint(points, yaws) == (
        (0.0, 0.0, 0.0),
        (1.0, 0.0, round(math.pi / 2.0, 6)),
        (1.0, 1.0, round(-3.0 * math.pi / 4.0, 6)),
        (0.0, 0.0, 0.0),
    )


def test_tracking_point_correction_matches_controller_formula():
    corrected = PLANNER.base_position_from_tracking_point(
        1.0,
        2.0,
        math.pi * 0.5,
        0.0,
        0.087,
        0.040,
    )
    assert corrected == pytest.approx((1.127, 1.953))


def test_export_writes_navigation_csv_and_report(tmp_path):
    config = PLANNER.load_config(CONFIG_PATH, "shortest")
    result = PLANNER.plan_route(config)
    paths = PLANNER.export_result(result, tmp_path)

    assert all(path.is_file() for path in paths.values())
    with paths["route_csv"].open(encoding="utf-8") as route_file:
        rows = list(csv.reader(line for line in route_file if not line.startswith("#")))
    assert rows[0] == ["x", "y", "yaw", "speed", "tolerance", "stop_time"]
    assert len(rows) == len(result.route_ros) + 1
    report = json.loads(paths["report_json"].read_text(encoding="utf-8"))
    assert report["all_tasks_visited"] is True
    assert report["returns_to_start"] is True
