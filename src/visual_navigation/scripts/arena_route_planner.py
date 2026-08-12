#!/usr/bin/env python3

"""Generate and visualize a collision-checked route through arena tasks 1-12."""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import yaml
except ImportError as exception:  # pragma: no cover - depends on host packages.
    raise SystemExit("PyYAML is required: sudo apt install python3-yaml") from exception


Cell = Tuple[int, int]
Point = Tuple[float, float]
EventCallback = Optional[Callable[[Dict[str, Any]], None]]
TARGET_HIT_RADIUS_PX = 16.0
MAX_ACTIVE_TARGETS = 16


class PlanningError(RuntimeError):
    """Raised when the configured arena cannot produce a valid route."""


class PlanningCancelled(PlanningError):
    """Raised when the GUI cancels a running plan."""


@dataclass(frozen=True)
class MonitoringConfig:
    enabled: bool
    route_frame: str
    odom_topic: str
    fusion_status_topic: str
    navigation_status_topic: str
    current_waypoint_topic: str
    navigation_path_topic: str
    route_plan_input_topic: str
    actuator_health_topic: str
    require_actuator_health: bool
    activation_timeout: float
    fusion_status_timeout: float
    actuator_health_timeout: float
    odom_timeout: float
    tracking_offset_x: float
    tracking_offset_y: float
    trace_minimum_spacing: float
    trace_maximum_points: int


@dataclass(frozen=True)
class PlannerConfig:
    source_path: Path
    width: float
    height: float
    resolution: float
    free_regions: Tuple[Tuple[float, float, float, float], ...]
    obstacles: Tuple[Tuple[float, float, float, float], ...]
    inflation_radius: float
    start: Point
    start_heading: float
    tasks: Tuple[Point, ...]
    task_labels: Tuple[str, ...]
    mode: str
    allow_diagonal: bool
    waypoint_spacing: float
    route_speed: float
    waypoint_tolerance: float
    task_tolerance: float
    task_stop_time: float
    astar_animation_batch: int
    monitoring: MonitoringConfig
    output_directory: Path
    output_names: Dict[str, str]


@dataclass
class GridMap:
    config: PlannerConfig
    base_occupied: List[List[bool]]
    occupied: List[List[bool]]

    @property
    def rows(self) -> int:
        return len(self.occupied)

    @property
    def columns(self) -> int:
        return len(self.occupied[0])

    def in_bounds(self, cell: Cell) -> bool:
        column, row = cell
        return 0 <= column < self.columns and 0 <= row < self.rows

    def is_free(self, cell: Cell) -> bool:
        column, row = cell
        return self.in_bounds(cell) and not self.occupied[row][column]

    def world_to_cell(self, point: Point) -> Cell:
        x, y = point
        column = min(self.columns - 1, max(0, int(math.floor(x / self.config.resolution))))
        row = min(self.rows - 1, max(0, int(math.floor(y / self.config.resolution))))
        return column, row

    def cell_to_world(self, cell: Cell) -> Point:
        column, row = cell
        resolution = self.config.resolution
        return (column + 0.5) * resolution, (row + 0.5) * resolution

    def point_is_free(self, point: Point) -> bool:
        x, y = point
        if x < 0.0 or y < 0.0 or x >= self.config.width or y >= self.config.height:
            return False
        return self.is_free(self.world_to_cell(point))

    def segment_is_free(self, start: Point, end: Point) -> bool:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        sample_spacing = max(self.config.resolution * 0.20, 0.005)
        sample_count = max(1, int(math.ceil(distance / sample_spacing)))
        for index in range(sample_count + 1):
            ratio = index / sample_count
            point = (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
            if not self.point_is_free(point):
                return False
        return True


@dataclass
class PlanResult:
    grid: GridMap
    node_points: Tuple[Point, ...]
    node_order: List[int]
    route_field: List[Point]
    route_ros: List[Point]
    waypoint_labels: Dict[int, str]
    total_length: float
    pair_distances: List[List[float]]
    actual_first_visit_order: List[int]


def _as_rectangle(value: Sequence[Any], name: str) -> Tuple[float, float, float, float]:
    if len(value) != 4:
        raise PlanningError(f"{name} must contain four coordinates")
    rectangle = tuple(float(item) for item in value)
    x_min, y_min, x_max, y_max = rectangle
    if not all(math.isfinite(item) for item in rectangle) or x_min >= x_max or y_min >= y_max:
        raise PlanningError(f"{name} is not a valid rectangle")
    return rectangle


def _as_point(value: Sequence[Any], name: str) -> Point:
    if len(value) != 2:
        raise PlanningError(f"{name} must contain x and y")
    point = float(value[0]), float(value[1])
    if not all(math.isfinite(item) for item in point):
        raise PlanningError(f"{name} contains a non-finite coordinate")
    return point


def load_config(path: Path, mode_override: Optional[str] = None) -> PlannerConfig:
    path = path.expanduser().resolve()
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exception:
        raise PlanningError(f"cannot read configuration {path}: {exception}") from exception
    except yaml.YAMLError as exception:
        raise PlanningError(f"invalid YAML in {path}: {exception}") from exception

    if not isinstance(document, dict):
        raise PlanningError("configuration root must be a mapping")
    try:
        arena = document["arena"]
        start_section = document["start"]
        task_section = document["tasks"]
        planning = document["planning"]
        monitoring = document.get("monitoring", {})
        output = document["output"]
        width = float(arena["width_m"])
        height = float(arena["height_m"])
        resolution = float(arena["resolution_m"])
    except (KeyError, TypeError, ValueError) as exception:
        raise PlanningError(f"missing or invalid required configuration value: {exception}") from exception

    if width <= 0.0 or height <= 0.0 or resolution <= 0.0:
        raise PlanningError("arena dimensions and resolution must be positive")
    if width / resolution > 1000 or height / resolution > 1000:
        raise PlanningError("configured grid is too large; use a coarser resolution")

    free_regions = tuple(
        _as_rectangle(value, f"arena.free_regions[{index}]")
        for index, value in enumerate(arena.get("free_regions", []))
    )
    if not free_regions:
        raise PlanningError("arena.free_regions cannot be empty")
    obstacles = tuple(
        _as_rectangle(value, f"arena.obstacles[{index}]")
        for index, value in enumerate(arena.get("obstacles", []))
    )
    start = _as_point(start_section["position_m"], "start.position_m")

    expected_task_names = [str(index) for index in range(1, 13)]
    missing_tasks = [name for name in expected_task_names if name not in task_section]
    if missing_tasks:
        raise PlanningError(f"tasks are missing numbered points: {', '.join(missing_tasks)}")
    tasks = tuple(_as_point(task_section[name], f"tasks.{name}") for name in expected_task_names)

    mode = mode_override or str(planning.get("mode", "shortest"))
    if mode not in ("shortest", "numbered"):
        raise PlanningError("planning.mode must be 'shortest' or 'numbered'")

    output_directory = Path(str(output.get("directory", "../routes/arena_generated")))
    if not output_directory.is_absolute():
        output_directory = (path.parent / output_directory).resolve()
    output_names = {
        "route_csv": str(output.get("route_csv", "arena_route.csv")),
        "map_pgm": str(output.get("map_pgm", "arena_map.pgm")),
        "map_yaml": str(output.get("map_yaml", "arena_map.yaml")),
        "preview_png": str(output.get("preview_png", "arena_route_preview.png")),
        "report_json": str(output.get("report_json", "arena_route_report.json")),
    }
    for key, filename in output_names.items():
        if Path(filename).name != filename:
            raise PlanningError(f"output.{key} must be a filename, not a path")

    return PlannerConfig(
        source_path=path,
        width=width,
        height=height,
        resolution=resolution,
        free_regions=free_regions,
        obstacles=obstacles,
        inflation_radius=max(0.0, float(arena.get("inflation_radius_m", 0.0))),
        start=start,
        start_heading=math.radians(float(start_section.get("heading_deg", -90.0))),
        tasks=tasks,
        task_labels=tuple(expected_task_names),
        mode=mode,
        allow_diagonal=bool(planning.get("allow_diagonal", True)),
        waypoint_spacing=max(resolution, float(planning.get("waypoint_spacing_m", 0.35))),
        route_speed=max(0.0, float(planning.get("route_speed_mps", 0.50))),
        waypoint_tolerance=max(0.001, float(planning.get("waypoint_tolerance_m", 0.06))),
        task_tolerance=max(0.001, float(planning.get("task_tolerance_m", 0.08))),
        task_stop_time=max(0.0, float(planning.get("task_stop_time_s", 0.50))),
        astar_animation_batch=max(1, int(planning.get("astar_animation_batch", 32))),
        monitoring=MonitoringConfig(
            enabled=bool(monitoring.get("enabled", True)),
            route_frame=str(monitoring.get("route_frame", "map")),
            odom_topic=str(monitoring.get("odom_topic", "/odometry/fused")),
            fusion_status_topic=str(
                monitoring.get("fusion_status_topic", "/odometry/fusion_status")
            ),
            navigation_status_topic=str(
                monitoring.get("navigation_status_topic", "/waypoint_navigation/status")
            ),
            current_waypoint_topic=str(
                monitoring.get(
                    "current_waypoint_topic", "/waypoint_navigation/current_waypoint"
                )
            ),
            navigation_path_topic=str(
                monitoring.get("navigation_path_topic", "/waypoint_path")
            ),
            route_plan_input_topic=str(
                monitoring.get(
                    "route_plan_input_topic", "/waypoint_navigation/route_plan_input"
                )
            ),
            actuator_health_topic=str(
                monitoring.get(
                    "actuator_health_topic", "/cup_car_serial/connected"
                )
            ),
            require_actuator_health=bool(
                monitoring.get("require_actuator_health", True)
            ),
            activation_timeout=max(
                0.5, float(monitoring.get("activation_timeout_s", 3.0))
            ),
            fusion_status_timeout=max(
                0.05, float(monitoring.get("fusion_status_timeout_s", 0.60))
            ),
            actuator_health_timeout=max(
                0.05, float(monitoring.get("actuator_health_timeout_s", 0.80))
            ),
            odom_timeout=max(0.05, float(monitoring.get("odom_timeout_s", 0.50))),
            tracking_offset_x=float(monitoring.get("tracking_point_offset_x", 0.087)),
            tracking_offset_y=float(monitoring.get("tracking_point_offset_y", 0.040)),
            trace_minimum_spacing=max(
                0.001, float(monitoring.get("trace_minimum_spacing_m", 0.02))
            ),
            trace_maximum_points=max(
                2, int(monitoring.get("trace_maximum_points", 5000))
            ),
        ),
        output_directory=output_directory,
        output_names=output_names,
    )


def _point_in_rectangle(point: Point, rectangle: Tuple[float, float, float, float]) -> bool:
    x, y = point
    x_min, y_min, x_max, y_max = rectangle
    return x_min <= x < x_max and y_min <= y < y_max


def build_grid(config: PlannerConfig, event_callback: EventCallback = None) -> GridMap:
    columns = int(math.ceil(config.width / config.resolution))
    rows = int(math.ceil(config.height / config.resolution))
    base_occupied = [[True for _ in range(columns)] for _ in range(rows)]

    for row in range(rows):
        for column in range(columns):
            point = ((column + 0.5) * config.resolution, (row + 0.5) * config.resolution)
            inside_arena = any(_point_in_rectangle(point, region) for region in config.free_regions)
            inside_obstacle = any(_point_in_rectangle(point, obstacle) for obstacle in config.obstacles)
            base_occupied[row][column] = not inside_arena or inside_obstacle

    occupied = [row[:] for row in base_occupied]
    radius_cells = int(math.ceil(config.inflation_radius / config.resolution))
    if radius_cells > 0:
        offsets = []
        for row_offset in range(-radius_cells, radius_cells + 1):
            for column_offset in range(-radius_cells, radius_cells + 1):
                distance = math.hypot(column_offset, row_offset) * config.resolution
                if distance <= config.inflation_radius + 1e-9:
                    offsets.append((column_offset, row_offset))
        for row in range(rows):
            for column in range(columns):
                if base_occupied[row][column]:
                    continue
                for column_offset, row_offset in offsets:
                    test_column = column + column_offset
                    test_row = row + row_offset
                    if (
                        test_column < 0
                        or test_column >= columns
                        or test_row < 0
                        or test_row >= rows
                        or base_occupied[test_row][test_column]
                    ):
                        occupied[row][column] = True
                        break

    grid = GridMap(config=config, base_occupied=base_occupied, occupied=occupied)
    if event_callback:
        event_callback({"type": "grid_ready", "grid": grid})
    return grid


def _heuristic(left: Cell, right: Cell, allow_diagonal: bool) -> float:
    delta_x = abs(left[0] - right[0])
    delta_y = abs(left[1] - right[1])
    if not allow_diagonal:
        return float(delta_x + delta_y)
    diagonal = min(delta_x, delta_y)
    straight = max(delta_x, delta_y) - diagonal
    return math.sqrt(2.0) * diagonal + straight


def _neighbours(grid: GridMap, cell: Cell) -> Iterable[Tuple[Cell, float]]:
    column, row = cell
    cardinal = ((1, 0), (-1, 0), (0, 1), (0, -1))
    for column_delta, row_delta in cardinal:
        neighbour = column + column_delta, row + row_delta
        if grid.is_free(neighbour):
            yield neighbour, 1.0
    if not grid.config.allow_diagonal:
        return
    for column_delta, row_delta in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        neighbour = column + column_delta, row + row_delta
        if not grid.is_free(neighbour):
            continue
        # Both adjacent cardinal cells must be free to prevent corner cutting.
        if not grid.is_free((column + column_delta, row)):
            continue
        if not grid.is_free((column, row + row_delta)):
            continue
        yield neighbour, math.sqrt(2.0)


def astar(
    grid: GridMap,
    start: Cell,
    goal: Cell,
    event_callback: EventCallback = None,
    cancel_check: Optional[Callable[[], bool]] = None,
) -> Tuple[List[Cell], float]:
    if not grid.is_free(start):
        raise PlanningError(f"A* start cell {start} is occupied")
    if not grid.is_free(goal):
        raise PlanningError(f"A* goal cell {goal} is occupied")

    counter = 0
    frontier: List[Tuple[float, float, int, Cell]] = []
    start_heuristic = _heuristic(start, goal, grid.config.allow_diagonal)
    heapq.heappush(frontier, (start_heuristic, start_heuristic, counter, start))
    cost_so_far: Dict[Cell, float] = {start: 0.0}
    predecessor: Dict[Cell, Cell] = {}
    closed = set()
    animation_batch: List[Cell] = []

    while frontier:
        if cancel_check and cancel_check():
            raise PlanningCancelled("planning cancelled")
        _, _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        closed.add(current)
        animation_batch.append(current)
        if event_callback and len(animation_batch) >= grid.config.astar_animation_batch:
            event_callback({"type": "astar_expanded", "cells": animation_batch, "current": current})
            animation_batch = []
        if current == goal:
            path = [current]
            while current != start:
                current = predecessor[current]
                path.append(current)
            path.reverse()
            if event_callback and animation_batch:
                event_callback({"type": "astar_expanded", "cells": animation_batch, "current": goal})
            return path, cost_so_far[goal] * grid.config.resolution

        for neighbour, step_cost in _neighbours(grid, current):
            new_cost = cost_so_far[current] + step_cost
            if new_cost + 1e-12 >= cost_so_far.get(neighbour, math.inf):
                continue
            cost_so_far[neighbour] = new_cost
            predecessor[neighbour] = current
            heuristic = _heuristic(neighbour, goal, grid.config.allow_diagonal)
            counter += 1
            heapq.heappush(frontier, (new_cost + heuristic, heuristic, counter, neighbour))

    raise PlanningError(f"no grid path exists between {start} and {goal}")


def held_karp_order(
    distances: Sequence[Sequence[float]], event_callback: EventCallback = None
) -> List[int]:
    task_count = len(distances) - 1
    if task_count <= 0:
        return [0, 0]
    full_mask = (1 << task_count) - 1
    costs: Dict[Tuple[int, int], float] = {}
    parents: Dict[Tuple[int, int], Optional[int]] = {}
    for task in range(task_count):
        mask = 1 << task
        costs[(mask, task)] = distances[0][task + 1]
        parents[(mask, task)] = None

    for subset_size in range(2, task_count + 1):
        for mask in range(1, full_mask + 1):
            if mask.bit_count() != subset_size:
                continue
            for last in range(task_count):
                last_bit = 1 << last
                if not mask & last_bit:
                    continue
                previous_mask = mask ^ last_bit
                best_cost = math.inf
                best_previous = None
                for previous in range(task_count):
                    if not previous_mask & (1 << previous):
                        continue
                    candidate = costs[(previous_mask, previous)] + distances[previous + 1][last + 1]
                    if candidate < best_cost - 1e-12:
                        best_cost = candidate
                        best_previous = previous
                costs[(mask, last)] = best_cost
                parents[(mask, last)] = best_previous
        if event_callback:
            event_callback({"type": "tsp_layer", "subset_size": subset_size, "total": task_count})

    final_task = min(
        range(task_count),
        key=lambda task: costs[(full_mask, task)] + distances[task + 1][0],
    )
    reversed_tasks = []
    mask = full_mask
    task: Optional[int] = final_task
    while task is not None:
        reversed_tasks.append(task + 1)
        previous = parents[(mask, task)]
        mask ^= 1 << task
        task = previous
    return [0] + list(reversed(reversed_tasks)) + [0]


def _simplify_path(grid: GridMap, points: Sequence[Point]) -> List[Point]:
    if len(points) <= 2:
        return list(points)
    result = [points[0]]
    index = 0
    while index < len(points) - 1:
        candidate = len(points) - 1
        while candidate > index + 1 and not grid.segment_is_free(points[index], points[candidate]):
            candidate -= 1
        result.append(points[candidate])
        index = candidate
    return result


def _resample_path(points: Sequence[Point], maximum_spacing: float) -> List[Point]:
    if not points:
        return []
    result = [points[0]]
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        segment_count = max(1, int(math.ceil(distance / maximum_spacing)))
        for index in range(1, segment_count + 1):
            ratio = index / segment_count
            result.append(
                (
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                )
            )
    return result


def _arena_to_ros(point: Point, config: PlannerConfig) -> Point:
    delta_x = point[0] - config.start[0]
    delta_y = point[1] - config.start[1]
    cosine = math.cos(config.start_heading)
    sine = math.sin(config.start_heading)
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
    )


def ros_to_arena(point: Point, config: PlannerConfig) -> Point:
    """Convert a start-relative ROS map position back into arena coordinates."""
    cosine = math.cos(config.start_heading)
    sine = math.sin(config.start_heading)
    return (
        config.start[0] + cosine * point[0] - sine * point[1],
        config.start[1] + sine * point[0] + cosine * point[1],
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def route_yaws(points: Sequence[Point]) -> List[float]:
    """Give each published waypoint a valid heading; the closed route ends at yaw zero."""
    if not points:
        return []
    yaws: List[float] = []
    for index, point in enumerate(points):
        if index == len(points) - 1:
            yaws.append(0.0)
            continue
        next_index = index + 1
        while next_index < len(points):
            target = points[next_index]
            if math.hypot(target[0] - point[0], target[1] - point[1]) > 1e-12:
                break
            next_index += 1
        if next_index < len(points):
            target = points[next_index]
            yaws.append(math.atan2(target[1] - point[1], target[0] - point[0]))
        else:
            yaws.append(yaws[-1] if yaws else 0.0)
    return yaws


def route_fingerprint(points: Sequence[Point], yaws: Sequence[float]) -> Tuple[Tuple[float, ...], ...]:
    if len(points) != len(yaws):
        raise ValueError("route points and headings must have equal lengths")
    return tuple(
        (round(point[0], 6), round(point[1], 6), round(normalize_angle(yaw), 6))
        for point, yaw in zip(points, yaws)
    )


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> Optional[float]:
    values = x, y, z, w
    if not all(math.isfinite(value) for value in values):
        return None
    squared_norm = sum(value * value for value in values)
    if squared_norm <= 1e-12 or not math.isfinite(squared_norm):
        return None
    x, y, z, w = (value / math.sqrt(squared_norm) for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def base_position_from_tracking_point(
    tracking_x: float,
    tracking_y: float,
    yaw: float,
    reference_yaw: float,
    tracking_offset_x: float,
    tracking_offset_y: float,
) -> Point:
    """Match visual_navigation::BasePositionFromTrackingPoint exactly."""
    values = (
        tracking_x,
        tracking_y,
        yaw,
        reference_yaw,
        tracking_offset_x,
        tracking_offset_y,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("tracking point correction requires finite values")
    current_offset_x = (
        math.cos(yaw) * tracking_offset_x - math.sin(yaw) * tracking_offset_y
    )
    current_offset_y = (
        math.sin(yaw) * tracking_offset_x + math.cos(yaw) * tracking_offset_y
    )
    reference_offset_x = (
        math.cos(reference_yaw) * tracking_offset_x
        - math.sin(reference_yaw) * tracking_offset_y
    )
    reference_offset_y = (
        math.sin(reference_yaw) * tracking_offset_x
        + math.cos(reference_yaw) * tracking_offset_y
    )
    return (
        tracking_x - current_offset_x + reference_offset_x,
        tracking_y - current_offset_y + reference_offset_y,
    )


def _polyline_length(points: Sequence[Point]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def _distance_to_segment(point: Point, start: Point, end: Point) -> float:
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    squared_length = delta_x * delta_x + delta_y * delta_y
    if squared_length <= 1e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = ((point[0] - start[0]) * delta_x + (point[1] - start[1]) * delta_y) / squared_length
    ratio = min(1.0, max(0.0, ratio))
    projection = start[0] + ratio * delta_x, start[1] + ratio * delta_y
    return math.hypot(point[0] - projection[0], point[1] - projection[1])


def _first_visit_order(route: Sequence[Point], tasks: Sequence[Point], tolerance: float) -> List[int]:
    remaining = set(range(1, len(tasks) + 1))
    order: List[int] = []
    for start, end in zip(route, route[1:]):
        reached = [
            task_id
            for task_id in remaining
            if _distance_to_segment(tasks[task_id - 1], start, end) <= tolerance
        ]
        reached.sort()
        for task_id in reached:
            remaining.remove(task_id)
            order.append(task_id)
    return order


def plan_route(
    config: PlannerConfig,
    event_callback: EventCallback = None,
    cancel_check: Optional[Callable[[], bool]] = None,
) -> PlanResult:
    if not config.tasks:
        raise PlanningError("至少选择或添加一个目标点")
    if len(config.tasks) != len(config.task_labels):
        raise PlanningError("任务点数量与标签数量不一致")
    grid = build_grid(config, event_callback)
    node_points = (config.start,) + config.tasks
    node_cells = tuple(grid.world_to_cell(point) for point in node_points)
    for node_index, (point, cell) in enumerate(zip(node_points, node_cells)):
        if not grid.point_is_free(point):
            name = "start" if node_index == 0 else f"task {node_index}"
            raise PlanningError(
                f"{name} at {point} is occupied after {config.inflation_radius:.3f} m inflation"
            )
        if not grid.is_free(cell):
            raise PlanningError(f"node {node_index} maps to occupied grid cell {cell}")

    node_count = len(node_points)
    pair_total = node_count * (node_count - 1) // 2
    pair_index = 0
    distances = [[0.0 for _ in range(node_count)] for _ in range(node_count)]
    pair_paths: Dict[Tuple[int, int], List[Cell]] = {}
    for left in range(node_count):
        for right in range(left + 1, node_count):
            if cancel_check and cancel_check():
                raise PlanningCancelled("planning cancelled")
            pair_index += 1
            if event_callback:
                event_callback(
                    {
                        "type": "pair_start",
                        "left": left,
                        "right": right,
                        "index": pair_index,
                        "total": pair_total,
                    }
                )
            path, distance = astar(
                grid,
                node_cells[left],
                node_cells[right],
                event_callback=event_callback,
                cancel_check=cancel_check,
            )
            distances[left][right] = distance
            distances[right][left] = distance
            pair_paths[(left, right)] = path
            if event_callback:
                event_callback(
                    {
                        "type": "pair_done",
                        "left": left,
                        "right": right,
                        "index": pair_index,
                        "total": pair_total,
                        "path": [grid.cell_to_world(cell) for cell in path],
                        "distance": distance,
                    }
                )

    if config.mode == "numbered":
        node_order = list(range(node_count)) + [0]
    else:
        node_order = held_karp_order(distances, event_callback)

    route_field = [config.start]
    waypoint_labels: Dict[int, str] = {0: "START"}
    for left, right in zip(node_order, node_order[1:]):
        if left < right:
            cell_path = pair_paths[(left, right)]
        else:
            cell_path = list(reversed(pair_paths[(right, left)]))
        raw_points = [node_points[left]]
        raw_points.extend(grid.cell_to_world(cell) for cell in cell_path[1:-1])
        raw_points.append(node_points[right])
        segment = _resample_path(_simplify_path(grid, raw_points), config.waypoint_spacing)
        for point in segment[1:]:
            if route_field and math.hypot(point[0] - route_field[-1][0], point[1] - route_field[-1][1]) < 1e-9:
                continue
            route_field.append(point)
        waypoint_labels[len(route_field) - 1] = (
            "RETURN" if right == 0 else f"TASK_{config.task_labels[right - 1]}"
        )

    for start, end in zip(route_field, route_field[1:]):
        if not grid.segment_is_free(start, end):
            raise PlanningError(f"simplified route contains a collision between {start} and {end}")

    actual_first_visit_order = _first_visit_order(route_field, config.tasks, config.task_tolerance)
    expected_visits = set(range(1, len(config.tasks) + 1))
    if set(actual_first_visit_order) != expected_visits:
        missing = sorted(expected_visits - set(actual_first_visit_order))
        raise PlanningError(f"final route did not pass task points: {missing}")

    route_ros = [_arena_to_ros(point, config) for point in route_field]
    result = PlanResult(
        grid=grid,
        node_points=node_points,
        node_order=node_order,
        route_field=route_field,
        route_ros=route_ros,
        waypoint_labels=waypoint_labels,
        total_length=_polyline_length(route_field),
        pair_distances=distances,
        actual_first_visit_order=actual_first_visit_order,
    )
    if event_callback:
        event_callback({"type": "complete", "result": result})
    return result


def _write_pgm(grid: GridMap, path: Path) -> None:
    with path.open("w", encoding="ascii", newline="\n") as output:
        output.write("P2\n")
        output.write(f"# arena occupancy grid, resolution {grid.config.resolution:.6f} m\n")
        output.write(f"{grid.columns} {grid.rows}\n255\n")
        for row in reversed(grid.occupied):
            output.write(" ".join("0" if occupied else "254" for occupied in row))
            output.write("\n")


def _write_map_yaml(config: PlannerConfig, pgm_name: str, path: Path) -> None:
    path.write_text(
        "\n".join(
            (
                f"image: {pgm_name}",
                f"resolution: {config.resolution:.6f}",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            )
        ),
        encoding="utf-8",
    )


def _write_route_csv(result: PlanResult, path: Path) -> None:
    config = result.grid.config
    with path.open("w", encoding="utf-8", newline="") as output:
        output.write("# Generated by arena_route_planner.py. Coordinates use the start-relative map frame.\n")
        output.write("# x[m],y[m],yaw[rad],speed[m/s],tolerance[m],stop_time[s]\n")
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(("x", "y", "yaw", "speed", "tolerance", "stop_time"))
        final_index = len(result.route_ros) - 1
        for index, point in enumerate(result.route_ros):
            label = result.waypoint_labels.get(index)
            if label:
                output.write(f"# {label}\n")
            is_task = bool(label and label.startswith("TASK_"))
            yaw = "0.000000" if index == final_index else ""
            tolerance = config.task_tolerance if is_task else config.waypoint_tolerance
            stop_time = config.task_stop_time if is_task else 0.0
            writer.writerow(
                (
                    f"{point[0]:.6f}",
                    f"{point[1]:.6f}",
                    yaw,
                    f"{config.route_speed:.3f}",
                    f"{tolerance:.3f}",
                    f"{stop_time:.3f}",
                )
            )


def _write_preview(result: PlanResult, path: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plot
        from matplotlib.patches import Rectangle
    except ImportError as exception:  # pragma: no cover - depends on host packages.
        raise PlanningError("matplotlib is required to export the PNG preview") from exception

    config = result.grid.config
    figure, axes = plot.subplots(figsize=(6.4, 8.4), dpi=150)
    axes.set_facecolor("#20252b")
    for region in config.free_regions:
        axes.add_patch(
            Rectangle(
                (region[0], region[1]),
                region[2] - region[0],
                region[3] - region[1],
                facecolor="#f7f8fa",
                edgecolor="none",
            )
        )
    for obstacle in config.obstacles:
        axes.add_patch(
            Rectangle(
                (obstacle[0], obstacle[1]),
                obstacle[2] - obstacle[0],
                obstacle[3] - obstacle[1],
                facecolor="#303740",
                edgecolor="#111418",
                linewidth=1.0,
            )
        )
    route_x = [point[0] for point in result.route_field]
    route_y = [point[1] for point in result.route_field]
    axes.plot(route_x, route_y, color="#d92d20", linewidth=1.8, zorder=4)
    for task_label, point in zip(config.task_labels, config.tasks):
        axes.scatter([point[0]], [point[1]], s=90, color="#fdb022", edgecolor="#111418", zorder=5)
        axes.text(point[0], point[1], task_label, ha="center", va="center", fontsize=7, zorder=6)
    axes.scatter([config.start[0]], [config.start[1]], s=110, color="#1570ef", marker="D", zorder=6)
    axes.text(config.start[0], config.start[1], "S", color="white", ha="center", va="center", fontsize=7, zorder=7)
    axes.set_xlim(-0.08, config.width + 0.08)
    axes.set_ylim(-0.08, config.height + 0.08)
    axes.set_aspect("equal", adjustable="box")
    axes.set_xlabel("Arena X [m]")
    axes.set_ylabel("Arena Y [m]")
    axes.set_title(f"Arena route: {result.total_length:.2f} m")
    axes.grid(color="#98a2b3", alpha=0.25, linewidth=0.4)
    figure.tight_layout()
    figure.savefig(path)
    plot.close(figure)


def export_result(result: PlanResult, output_directory: Optional[Path] = None) -> Dict[str, Path]:
    config = result.grid.config
    directory = (output_directory or config.output_directory).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    paths = {key: directory / filename for key, filename in config.output_names.items()}
    _write_pgm(result.grid, paths["map_pgm"])
    _write_map_yaml(config, paths["map_pgm"].name, paths["map_yaml"])
    _write_route_csv(result, paths["route_csv"])
    _write_preview(result, paths["preview_png"])

    report = {
        "configuration": str(config.source_path),
        "planning_mode": config.mode,
        "resolution_m": config.resolution,
        "inflation_radius_m": config.inflation_radius,
        "route_frame": "start-relative map frame: +X initial forward, +Y initial left",
        "optimized_node_order": [
            "S" if node == 0 else config.task_labels[node - 1]
            for node in result.node_order
        ],
        "actual_first_visit_order": [
            config.task_labels[index - 1]
            for index in result.actual_first_visit_order
        ],
        "all_tasks_visited": set(result.actual_first_visit_order) == set(
            range(1, len(config.tasks) + 1)
        ),
        "returns_to_start": math.hypot(*result.route_ros[-1]) <= 1e-6,
        "route_length_m": round(result.total_length, 6),
        "waypoint_count": len(result.route_ros),
        "outputs": {key: str(path) for key, path in paths.items()},
    }
    paths["report_json"].write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return paths


def _default_config_path() -> Path:
    source_candidate = Path(__file__).resolve().parents[1] / "config" / "arena_map.yaml"
    if source_candidate.is_file():
        return source_candidate
    try:
        from ament_index_python.packages import get_package_share_directory

        installed_candidate = Path(get_package_share_directory("visual_navigation")) / "config" / "arena_map.yaml"
        if installed_candidate.is_file():
            return installed_candidate
    except (ImportError, LookupError):
        pass
    return source_candidate


class RosMonitor:
    """ROS adapter for GUI monitoring and atomic structured-route activation."""

    def __init__(
        self,
        config: MonitoringConfig,
        event_callback: Callable[[Dict[str, Any]], None],
    ) -> None:
        try:
            import rclpy
            from mission_control_interfaces.msg import Waypoint, WaypointRoute
            from nav_msgs.msg import Odometry, Path as NavigationPath
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from std_msgs.msg import Bool, Int32, String
        except ImportError as exception:
            raise PlanningError(
                "ROS 2 Python packages are unavailable; source /opt/ros/humble/setup.bash"
            ) from exception

        self.rclpy = rclpy
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init(args=[])

        monitor_config = config
        emit = event_callback
        owner = self
        self.config = config
        self._waypoint_type = Waypoint
        self._route_type = WaypointRoute
        self._activation_phase = "IDLE"
        self._activation_deadline = 0.0
        self._pending_fingerprint: Optional[Tuple[Tuple[float, ...], ...]] = None
        self._pending_stamp_ns = 0
        self._activation_result: Optional[Tuple[bool, str]] = None

        class ArenaMonitorNode(Node):
            def __init__(self) -> None:
                super().__init__("arena_route_monitor")
                self.reference_yaw: Optional[float] = None
                latched_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self.odom_subscription = self.create_subscription(
                    Odometry,
                    monitor_config.odom_topic,
                    self._handle_odometry,
                    qos_profile_sensor_data,
                )
                self.fusion_subscription = self.create_subscription(
                    String,
                    monitor_config.fusion_status_topic,
                    lambda message: emit(
                        {
                            "type": "fusion_status",
                            "value": message.data.strip(),
                            "arrival": time.monotonic(),
                        }
                    ),
                    10,
                )
                self.navigation_subscription = self.create_subscription(
                    String,
                    monitor_config.navigation_status_topic,
                    lambda message: emit(
                        {"type": "navigation_status", "value": message.data.strip()}
                    ),
                    latched_qos,
                )
                self.waypoint_subscription = self.create_subscription(
                    Int32,
                    monitor_config.current_waypoint_topic,
                    lambda message: emit(
                        {"type": "current_waypoint", "value": int(message.data)}
                    ),
                    latched_qos,
                )
                self.path_subscription = self.create_subscription(
                    NavigationPath,
                    monitor_config.navigation_path_topic,
                    self._handle_navigation_path,
                    latched_qos,
                )
                self.actuator_subscription = self.create_subscription(
                    Bool,
                    monitor_config.actuator_health_topic,
                    lambda message: emit(
                        {
                            "type": "actuator_health",
                            "connected": bool(message.data),
                            "arrival": time.monotonic(),
                        }
                    ),
                    latched_qos,
                )
                self.route_publisher = self.create_publisher(
                    WaypointRoute,
                    monitor_config.route_plan_input_topic,
                    QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE,
                    ),
                )

            def _handle_odometry(self, message: Any) -> None:
                frame = message.header.frame_id
                if frame and frame != monitor_config.route_frame:
                    emit(
                        {
                            "type": "odometry_invalid",
                            "reason": (
                                f"里程计坐标系 {frame} 与路线坐标系 "
                                f"{monitor_config.route_frame} 不一致"
                            ),
                        }
                    )
                    return
                position = message.pose.pose.position
                orientation = message.pose.pose.orientation
                yaw = quaternion_to_yaw(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
                if (
                    yaw is None
                    or not math.isfinite(position.x)
                    or not math.isfinite(position.y)
                ):
                    emit({"type": "odometry_invalid", "reason": "里程计位姿无效"})
                    return
                if self.reference_yaw is None:
                    self.reference_yaw = yaw
                base_x, base_y = base_position_from_tracking_point(
                    position.x,
                    position.y,
                    yaw,
                    self.reference_yaw,
                    monitor_config.tracking_offset_x,
                    monitor_config.tracking_offset_y,
                )
                emit(
                    {
                        "type": "odometry",
                        "map_pose": (base_x, base_y, yaw),
                        "relative_yaw": normalize_angle(yaw - self.reference_yaw),
                        "arrival": time.monotonic(),
                    }
                )

            def _handle_navigation_path(self, message: Any) -> None:
                frame = message.header.frame_id
                if frame and frame != monitor_config.route_frame:
                    emit(
                        {
                            "type": "navigation_path_invalid",
                            "reason": (
                                f"导航路线坐标系 {frame} 与 {monitor_config.route_frame} "
                                "不一致"
                            ),
                        }
                    )
                    return
                points = []
                yaws = []
                for stamped_pose in message.poses:
                    position = stamped_pose.pose.position
                    orientation = stamped_pose.pose.orientation
                    yaw = quaternion_to_yaw(
                        orientation.x, orientation.y, orientation.z, orientation.w
                    )
                    if (
                        not math.isfinite(position.x)
                        or not math.isfinite(position.y)
                        or yaw is None
                    ):
                        emit(
                            {"type": "navigation_path_invalid", "reason": "导航路线包含无效位姿"}
                        )
                        return
                    points.append((position.x, position.y))
                    yaws.append(yaw)
                stamp_ns = (
                    message.header.stamp.sec * 1_000_000_000
                    + message.header.stamp.nanosec
                )
                owner._handle_route_feedback(
                    stamp_ns, route_fingerprint(points, yaws)
                )
                emit({"type": "navigation_path", "points": points})

        try:
            self.node = ArenaMonitorNode()
        except Exception:
            if self.owns_context and rclpy.ok():
                rclpy.shutdown()
            raise

    def spin_once(self) -> None:
        if self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

    @property
    def activation_busy(self) -> bool:
        return self._activation_phase == "WAIT_ACK"

    @property
    def route_publisher_ready(self) -> bool:
        return self.node.route_publisher.get_subscription_count() > 0

    def publish_and_activate(self, result: PlanResult) -> None:
        if self.activation_busy:
            raise RuntimeError("路线正在发布，请等待导航器确认")
        if not result.route_ros:
            raise ValueError("规划结果没有航点")
        if not self.route_publisher_ready:
            raise RuntimeError("导航器尚未订阅规划路线话题")

        yaws = route_yaws(result.route_ros)
        message = self._route_type()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = self.config.route_frame
        for index, (point, yaw) in enumerate(zip(result.route_ros, yaws)):
            waypoint = self._waypoint_type()
            message.waypoints.append(waypoint)
            label = result.waypoint_labels.get(index, "")
            is_task = label.startswith("TASK_")
            waypoint.x = point[0]
            waypoint.y = point[1]
            waypoint.yaw = yaw
            waypoint.speed = result.grid.config.route_speed
            waypoint.tolerance = (
                result.grid.config.task_tolerance
                if is_task
                else result.grid.config.waypoint_tolerance
            )
            waypoint.stop_time = (
                result.grid.config.task_stop_time if is_task else 0.0
            )

        self._pending_fingerprint = route_fingerprint(result.route_ros, yaws)
        self._pending_stamp_ns = (
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        )
        self._activation_deadline = time.monotonic() + self.config.activation_timeout
        self._activation_phase = "WAIT_ACK"
        self._activation_result = None
        self.node.route_publisher.publish(message)

    def _handle_route_feedback(
        self, stamp_ns: int, fingerprint: Tuple[Tuple[float, ...], ...]
    ) -> None:
        if self._activation_phase != "WAIT_ACK":
            return
        if stamp_ns < self._pending_stamp_ns or fingerprint != self._pending_fingerprint:
            return
        self._finish_activation(True, "路线已发布，导航已启用")

    def poll_activation(self) -> None:
        if self.activation_busy and time.monotonic() > self._activation_deadline:
            self._finish_activation(False, "导航器未在规定时间内确认路线")

    def _finish_activation(self, success: bool, message: str) -> None:
        self._activation_phase = "SUCCEEDED" if success else "FAILED"
        self._activation_result = success, message
        self._pending_fingerprint = None

    def take_activation_result(self) -> Optional[Tuple[bool, str]]:
        result = self._activation_result
        self._activation_result = None
        return result

    def close(self) -> None:
        self.node.destroy_node()
        if self.owns_context and self.rclpy.ok():
            self.rclpy.shutdown()


class PlannerWindow:
    """Tkinter frontend; imports GUI modules lazily so headless mode stays usable."""

    def __init__(
        self,
        root: Any,
        config_path: Path,
        output_directory: Optional[Path],
        mode: str,
        enable_ros: bool = True,
    ):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.config_path = config_path
        self.output_directory = output_directory
        self.mode_variable = tk.StringVar(value=mode)
        self.status_variable = tk.StringVar(value="准备就绪")
        self.progress_variable = tk.StringVar(value="尚未开始规划")
        self.order_variable = tk.StringVar(value="-")
        self.length_variable = tk.StringVar(value="-")
        self.output_variable = tk.StringVar(value="-")
        self.monitor_variable = tk.StringVar(value="ROS监控未连接")
        self.pose_variable = tk.StringVar(value="-")
        self.fusion_variable = tk.StringVar(value="等待状态")
        self.navigation_variable = tk.StringVar(value="等待状态")
        self.waypoint_variable = tk.StringVar(value="-")
        self.actuator_variable = tk.StringVar(value="等待状态")
        self.publish_variable = tk.StringVar(value="请先完成规划")
        self.target_variable = tk.StringVar(value="默认点 12/12，手动点 0")
        self.events: queue.Queue = queue.Queue(maxsize=24)
        self.cancel_event = threading.Event()
        self.worker: Optional[threading.Thread] = None
        self.result: Optional[PlanResult] = None
        self.grid: Optional[GridMap] = None
        self.ros_monitor: Optional[RosMonitor] = None
        self.ros_enabled = enable_ros
        self.live_map_pose: Optional[Tuple[float, float, float]] = None
        self.live_arena_pose: Optional[Tuple[float, float, float]] = None
        self.last_odom_arrival: Optional[float] = None
        self.odometry_error = ""
        self.odom_ready = False
        self.fusion_status = ""
        self.last_fusion_arrival: Optional[float] = None
        self.navigation_status = ""
        self.current_waypoint: Optional[int] = None
        self.actuator_connected: Optional[bool] = None
        self.last_actuator_arrival: Optional[float] = None
        self.navigation_route_field: List[Point] = []
        self.plan_exported = False
        self.route_activated = False

        self.base_config = load_config(config_path, mode)
        self.config = self.base_config
        self.default_tasks = self.base_config.tasks
        self.default_task_labels = self.base_config.task_labels
        self.selected_default_indices = set(range(len(self.default_tasks)))
        self.custom_tasks: List[Point] = []
        self.root.title("赛场闭环路径规划器")
        self.root.geometry("1040x790")
        self.root.minsize(900, 700)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        toolbar = ttk.Frame(root, padding=(10, 8))
        toolbar.pack(fill=tk.X)
        self.start_button = ttk.Button(toolbar, text="开始规划", command=self.start)
        self.start_button.pack(side=tk.LEFT, padx=(0, 6))
        self.reset_button = ttk.Button(toolbar, text="重置", command=self.reset)
        self.reset_button.pack(side=tk.LEFT, padx=3)
        self.clear_trace_button = ttk.Button(toolbar, text="清除实车轨迹", command=self.clear_trace)
        self.clear_trace_button.pack(side=tk.LEFT, padx=3)
        ttk.Label(toolbar, text="模式").pack(side=tk.LEFT, padx=(18, 4))
        self.mode_box = ttk.Combobox(
            toolbar,
            textvariable=self.mode_variable,
            values=("shortest", "numbered"),
            width=10,
            state="readonly",
        )
        self.mode_box.pack(side=tk.LEFT)

        body = ttk.Frame(root, padding=(10, 0, 10, 10))
        body.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(body, bg="#20252b", highlightthickness=1, highlightbackground="#667085")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        panel = ttk.Frame(body, width=270, padding=(14, 8))
        panel.pack(side=tk.RIGHT, fill=tk.Y)
        panel.pack_propagate(False)
        self.publish_button = ttk.Button(
            panel,
            text="一键发布并启动导航",
            command=self.publish_route,
            state=tk.DISABLED,
        )
        self.publish_button.pack(fill=tk.X, pady=(4, 6))
        self._panel_row(panel, "路线发布", self.publish_variable)
        self._panel_row(panel, "状态", self.status_variable)
        self._panel_row(panel, "目标点", self.target_variable)
        self._panel_row(panel, "规划进度", self.progress_variable)
        self._panel_row(panel, "优化顺序", self.order_variable)
        self._panel_row(panel, "路线长度", self.length_variable)
        self._panel_row(panel, "输出目录", self.output_variable)
        ttk.Separator(panel).pack(fill=tk.X, pady=12)
        self._panel_row(panel, "实时监控", self.monitor_variable)
        self._panel_row(panel, "车体位姿", self.pose_variable)
        self._panel_row(panel, "融合状态", self.fusion_variable)
        self._panel_row(panel, "底盘连接", self.actuator_variable)
        self._panel_row(panel, "导航状态", self.navigation_variable)
        self._panel_row(panel, "当前目标", self.waypoint_variable)
        ttk.Separator(panel).pack(fill=tk.X, pady=12)
        ttk.Label(
            panel,
            text="shortest：目标点最短闭环\nnumbered：按当前标签顺序",
            justify=tk.LEFT,
            wraplength=240,
        ).pack(anchor=tk.W)

        self.canvas.bind("<Configure>", lambda _event: self._draw_base())
        self.canvas.bind("<Button-1>", self._handle_add_target)
        self.canvas.bind("<Button-2>", self._handle_remove_target)
        self.canvas.bind("<Button-3>", self._handle_remove_target)
        self.canvas.bind("<Control-Button-1>", self._handle_remove_target)
        self.grid = build_grid(self.config)
        self.actual_trace = deque(maxlen=self.config.monitoring.trace_maximum_points)
        self._refresh_target_status()
        self._start_ros_monitor()
        self.root.after(30, self._poll_events)
        self.root.after(20, self._spin_ros)
        self.root.after(100, self._refresh_monitor)

    def _panel_row(self, parent: Any, title: str, variable: Any) -> None:
        self.ttk.Label(parent, text=title).pack(anchor=self.tk.W, pady=(4, 1))
        self.ttk.Label(parent, textvariable=variable, wraplength=240, justify=self.tk.LEFT).pack(anchor=self.tk.W)

    def _navigation_running(self) -> bool:
        inactive_states = {"", "WAITING_FOR_ROUTE", "IDLE", "GOAL_REACHED"}
        return not (
            self.navigation_status in inactive_states
            or self.navigation_status.startswith("FAULT_")
        )

    def _editing_locked(self) -> bool:
        return bool(self.worker and self.worker.is_alive()) or self._navigation_running()

    def _selected_config(self, source: Optional[PlannerConfig] = None) -> PlannerConfig:
        source = source or self.base_config
        tasks = [
            self.default_tasks[index]
            for index in sorted(self.selected_default_indices)
        ]
        labels = [
            self.default_task_labels[index]
            for index in sorted(self.selected_default_indices)
        ]
        tasks.extend(self.custom_tasks)
        labels.extend(f"P{index}" for index in range(1, len(self.custom_tasks) + 1))
        return replace(source, tasks=tuple(tasks), task_labels=tuple(labels))

    def _refresh_target_status(self) -> None:
        self.target_variable.set(
            f"默认点 {len(self.selected_default_indices)}/{len(self.default_tasks)}，"
            f"手动点 {len(self.custom_tasks)}"
        )

    def _invalidate_plan(self, status: str) -> None:
        self.result = None
        self.plan_exported = False
        self.route_activated = False
        self.navigation_route_field = []
        self.config = self._selected_config()
        self.progress_variable.set("目标点已修改")
        self.order_variable.set("-")
        self.length_variable.set("-")
        self.output_variable.set("-")
        self.publish_variable.set("请重新规划")
        self.status_variable.set(status)
        self._refresh_target_status()
        self._update_publish_controls()
        self._draw_base()

    def _canvas_geometry(self) -> Tuple[float, float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        padding = 24.0
        scale = min(
            (width - 2.0 * padding) / self.config.width,
            (height - 2.0 * padding) / self.config.height,
        )
        offset_x = (width - self.config.width * scale) * 0.5
        offset_y = (height - self.config.height * scale) * 0.5
        return scale, offset_x, offset_y

    def _canvas_to_field(self, canvas_x: float, canvas_y: float) -> Point:
        scale, offset_x, offset_y = self._canvas_geometry()
        return (
            (canvas_x - offset_x) / scale,
            (self.canvas.winfo_height() - offset_y - canvas_y) / scale,
        )

    def _nearest_target(self, canvas_x: float, canvas_y: float) -> Optional[Tuple[str, int]]:
        best: Optional[Tuple[str, int]] = None
        best_distance = TARGET_HIT_RADIUS_PX
        for index, point in enumerate(self.default_tasks):
            x, y = self._transform(point)
            distance = math.hypot(canvas_x - x, canvas_y - y)
            if distance <= best_distance:
                best = "default", index
                best_distance = distance
        for index, point in enumerate(self.custom_tasks):
            x, y = self._transform(point)
            distance = math.hypot(canvas_x - x, canvas_y - y)
            if distance <= best_distance:
                best = "custom", index
                best_distance = distance
        return best

    def _handle_add_target(self, event: Any) -> str:
        if self._editing_locked():
            self.status_variable.set("规划或导航运行中，目标点已锁定")
            return "break"
        nearest = self._nearest_target(event.x, event.y)
        if nearest is not None:
            kind, index = nearest
            if kind == "default" and index not in self.selected_default_indices:
                self.selected_default_indices.add(index)
                self._invalidate_plan(f"已恢复默认点 {self.default_task_labels[index]}")
            else:
                self.status_variable.set("该目标点已选中")
            return "break"
        active_count = len(self.selected_default_indices) + len(self.custom_tasks)
        if active_count >= MAX_ACTIVE_TARGETS:
            self.status_variable.set(f"目标点最多 {MAX_ACTIVE_TARGETS} 个")
            return "break"
        point = self._canvas_to_field(event.x, event.y)
        if self.grid is None or not self.grid.point_is_free(point):
            self.status_variable.set("该位置不可通行，未添加目标点")
            return "break"
        self.custom_tasks.append(point)
        self._invalidate_plan(f"已添加手动点 P{len(self.custom_tasks)}")
        return "break"

    def _handle_remove_target(self, event: Any) -> str:
        if self._editing_locked():
            self.status_variable.set("规划或导航运行中，目标点已锁定")
            return "break"
        nearest = self._nearest_target(event.x, event.y)
        if nearest is None:
            self.status_variable.set("未选中可取消的目标点")
            return "break"
        kind, index = nearest
        if kind == "default":
            if index not in self.selected_default_indices:
                self.status_variable.set("该默认点已经取消")
                return "break"
            self.selected_default_indices.remove(index)
            self._invalidate_plan(f"已取消默认点 {self.default_task_labels[index]}")
        else:
            removed_label = f"P{index + 1}"
            self.custom_tasks.pop(index)
            self._invalidate_plan(f"已取消手动点 {removed_label}")
        return "break"

    def _update_edit_controls(self) -> None:
        locked = self._editing_locked()
        has_targets = bool(self.selected_default_indices or self.custom_tasks)
        self.start_button.configure(
            state=self.tk.DISABLED if locked or not has_targets else self.tk.NORMAL
        )
        self.reset_button.configure(
            state=self.tk.DISABLED if locked else self.tk.NORMAL
        )
        self.mode_box.configure(state="disabled" if locked else "readonly")

    def _start_ros_monitor(self) -> None:
        if not self.ros_enabled or not self.config.monitoring.enabled:
            self.monitor_variable.set("已禁用，仅使用离线规划")
            return
        try:
            self.ros_monitor = RosMonitor(
                self.config.monitoring, self._handle_monitor_event
            )
        except Exception as exception:
            self.ros_monitor = None
            self.monitor_variable.set(f"ROS不可用：{exception}")
            return
        self.monitor_variable.set(
            f"已连接，等待 {self.config.monitoring.odom_topic}"
        )

    def _spin_ros(self) -> None:
        if self.ros_monitor is not None:
            try:
                self.ros_monitor.spin_once()
            except Exception as exception:
                self.monitor_variable.set(f"ROS监控错误：{exception}")
                self.ros_monitor.close()
                self.ros_monitor = None
        self.root.after(20, self._spin_ros)

    def _handle_monitor_event(self, event: Dict[str, Any]) -> None:
        event_type = event["type"]
        if event_type == "odometry":
            map_x, map_y, map_yaw = event["map_pose"]
            arena_x, arena_y = ros_to_arena((map_x, map_y), self.config)
            arena_yaw = normalize_angle(
                self.config.start_heading + event["relative_yaw"]
            )
            self.live_map_pose = map_x, map_y, map_yaw
            self.live_arena_pose = arena_x, arena_y, arena_yaw
            self.last_odom_arrival = event["arrival"]
            self.odometry_error = ""
            point = arena_x, arena_y
            if (
                not self.actual_trace
                or math.hypot(
                    point[0] - self.actual_trace[-1][0],
                    point[1] - self.actual_trace[-1][1],
                )
                >= self.config.monitoring.trace_minimum_spacing
            ):
                self.actual_trace.append(point)
            self._draw_live_overlay()
        elif event_type == "odometry_invalid":
            self.odometry_error = event["reason"]
        elif event_type == "fusion_status":
            self.fusion_status = event["value"] or "EMPTY"
            self.last_fusion_arrival = event["arrival"]
            if self.fusion_status == "FULL":
                label = "正常：FULL"
            elif self.fusion_status.startswith("DEGRADED_"):
                label = f"降级：{self.fusion_status}"
            elif self.fusion_status.startswith("FAULT"):
                label = f"故障：{self.fusion_status}"
            else:
                label = f"未知：{self.fusion_status}"
            self.fusion_variable.set(label)
        elif event_type == "actuator_health":
            self.actuator_connected = event["connected"]
            self.last_actuator_arrival = event["arrival"]
            self.actuator_variable.set(
                "已连接" if self.actuator_connected else "未连接"
            )
        elif event_type == "navigation_status":
            self.navigation_status = event["value"] or "EMPTY"
            self.navigation_variable.set(self.navigation_status)
            self._draw_task_highlight()
        elif event_type == "current_waypoint":
            self.current_waypoint = event["value"]
            self._update_waypoint_display()
            self._draw_task_highlight()
        elif event_type == "navigation_path":
            self.navigation_route_field = [
                ros_to_arena(point, self.config) for point in event["points"]
            ]
            self._draw_final_route()
            self._draw_live_overlay()
            self._update_waypoint_display()
            self._draw_task_highlight()
        elif event_type == "navigation_path_invalid":
            self.navigation_variable.set(event["reason"])

    def _refresh_monitor(self) -> None:
        if self.ros_monitor is not None:
            self.ros_monitor.poll_activation()
            activation_result = self.ros_monitor.take_activation_result()
            if activation_result is not None:
                success, message = activation_result
                self.route_activated = success
                self.publish_variable.set(message)
                if not success:
                    self._show_error(message)
            if self.odometry_error:
                self.odom_ready = False
                self.monitor_variable.set(self.odometry_error)
            elif self.last_odom_arrival is None:
                self.odom_ready = False
                self.monitor_variable.set(
                    f"等待 {self.config.monitoring.odom_topic}"
                )
            else:
                age = time.monotonic() - self.last_odom_arrival
                self.odom_ready = age <= self.config.monitoring.odom_timeout
                if self.odom_ready:
                    if self.live_arena_pose is not None and not (
                        -0.25 <= self.live_arena_pose[0] <= self.config.width + 0.25
                        and -0.25 <= self.live_arena_pose[1] <= self.config.height + 0.25
                    ):
                        self.monitor_variable.set("里程计正常，但位姿超出赛场范围")
                    else:
                        self.monitor_variable.set("里程计正常")
                else:
                    self.monitor_variable.set(f"里程计超时（{age:.1f} s）")
            self._update_pose_display()
            self._draw_live_overlay()
            now = time.monotonic()
            if (
                self.last_fusion_arrival is not None
                and now - self.last_fusion_arrival
                > self.config.monitoring.fusion_status_timeout
            ):
                self.fusion_variable.set("状态超时")
            if (
                self.last_actuator_arrival is not None
                and now - self.last_actuator_arrival
                > self.config.monitoring.actuator_health_timeout
            ):
                self.actuator_variable.set("连接心跳超时")
        self._update_publish_controls()
        self._update_edit_controls()
        self.root.after(100, self._refresh_monitor)

    def _publish_readiness(self) -> Tuple[bool, str]:
        if not self.ros_enabled:
            return False, "ROS 已禁用"
        if self.ros_monitor is None:
            return False, "ROS 连接不可用"
        if self.ros_monitor.activation_busy:
            return False, "正在等待导航器确认"
        if self.result is None or not self.plan_exported:
            return False, "请先完成规划"
        if self.route_activated:
            return False, "路线已发布，导航已启用"
        if not self.ros_monitor.route_publisher_ready:
            return False, "等待导航程序启动"
        if not self.odom_ready:
            return False, "等待里程计就绪"
        fusion_fresh = (
            self.last_fusion_arrival is not None
            and time.monotonic() - self.last_fusion_arrival
            <= self.config.monitoring.fusion_status_timeout
        )
        if not fusion_fresh or not (
            self.fusion_status == "FULL"
            or self.fusion_status.startswith("DEGRADED_")
        ):
            return False, "等待融合定位正常"
        if (
            self.config.monitoring.require_actuator_health
            and (
                self.actuator_connected is not True
                or self.last_actuator_arrival is None
                or time.monotonic() - self.last_actuator_arrival
                > self.config.monitoring.actuator_health_timeout
            )
        ):
            return False, "等待底盘串口连接"
        inactive_states = {"WAITING_FOR_ROUTE", "IDLE", "GOAL_REACHED"}
        if self.navigation_status and not (
            self.navigation_status in inactive_states
            or self.navigation_status.startswith("FAULT_")
        ):
            return False, f"导航正在运行：{self.navigation_status}"
        return True, "可以发布规划路线"

    def _update_publish_controls(self) -> None:
        ready, reason = self._publish_readiness()
        self.publish_button.configure(
            state=self.tk.NORMAL if ready else self.tk.DISABLED
        )
        if not (
            self.ros_monitor is not None and self.ros_monitor.activation_busy
        ):
            self.publish_variable.set(reason)

    def publish_route(self) -> None:
        ready, reason = self._publish_readiness()
        if not ready or self.result is None or self.ros_monitor is None:
            self._show_error(reason)
            return
        try:
            self.ros_monitor.publish_and_activate(self.result)
        except (RuntimeError, ValueError) as exception:
            self._show_error(str(exception))
            return
        self.publish_variable.set(
            f"正在发布 {len(self.result.route_ros)} 个航点..."
        )
        self._update_publish_controls()

    def _update_pose_display(self) -> None:
        if self.live_arena_pose is None or self.live_map_pose is None:
            self.pose_variable.set("-")
            return
        arena_x, arena_y, arena_yaw = self.live_arena_pose
        map_x, map_y, _map_yaw = self.live_map_pose
        self.pose_variable.set(
            f"场地 x={arena_x:.3f}, y={arena_y:.3f} m\n"
            f"map x={map_x:.3f}, y={map_y:.3f} m\n"
            f"朝向 {math.degrees(arena_yaw):.1f}°"
        )

    def _next_task_id(self) -> Optional[str]:
        if self.current_waypoint is None:
            return None
        if self.navigation_status == "GOAL_REACHED":
            return None
        if self.navigation_route_field:
            start_index = max(0, self.current_waypoint)
            for point in self.navigation_route_field[start_index:]:
                for task_label, task_point in zip(
                    self.config.task_labels, self.config.tasks
                ):
                    if (
                        math.hypot(
                            point[0] - task_point[0], point[1] - task_point[1]
                        )
                        <= self.config.task_tolerance
                    ):
                        return task_label
        if self.result is None:
            return None
        for waypoint_index in range(
            max(0, self.current_waypoint), len(self.result.route_field)
        ):
            label = self.result.waypoint_labels.get(waypoint_index, "")
            if label.startswith("TASK_"):
                return label.split("_", 1)[1]
        return None

    def _update_waypoint_display(self) -> None:
        if self.current_waypoint is None:
            self.waypoint_variable.set("-")
            return
        task_id = self._next_task_id()
        suffix = f"，下一任务 {task_id}" if task_id is not None else ""
        self.waypoint_variable.set(f"航点索引 {self.current_waypoint}{suffix}")

    def _transform(self, point: Point) -> Tuple[float, float]:
        scale, offset_x, offset_y = self._canvas_geometry()
        return (
            offset_x + point[0] * scale,
            self.canvas.winfo_height() - offset_y - point[1] * scale,
        )

    def _cell_rectangle(self, cell: Cell) -> Tuple[float, float, float, float]:
        column, row = cell
        x0, y1 = self._transform((column * self.config.resolution, row * self.config.resolution))
        x1, y0 = self._transform(((column + 1) * self.config.resolution, (row + 1) * self.config.resolution))
        return x0, y0, x1, y1

    def _draw_base(self) -> None:
        if not self.grid:
            return
        self.canvas.delete("all")
        for row in range(self.grid.rows):
            for column in range(self.grid.columns):
                rectangle = self._cell_rectangle((column, row))
                if self.grid.base_occupied[row][column]:
                    colour = "#252b31"
                elif self.grid.occupied[row][column]:
                    colour = "#667085"
                else:
                    colour = "#f7f8fa"
                self.canvas.create_rectangle(*rectangle, fill=colour, outline=colour, tags=("base",))
        self._draw_markers()
        if self.result or self.navigation_route_field:
            self._draw_final_route()
        self._draw_task_highlight()
        self._draw_live_overlay()

    def _draw_markers(self) -> None:
        for index, (task_label, point) in enumerate(
            zip(self.default_task_labels, self.default_tasks)
        ):
            x, y = self._transform(point)
            selected = index in self.selected_default_indices
            fill = "#fdb022" if selected else "#98a2b3"
            text_fill = "#111418" if selected else "#ffffff"
            self.canvas.create_oval(x - 10, y - 10, x + 10, y + 10, fill=fill, outline="#111418", width=1, tags=("marker",))
            self.canvas.create_text(x, y, text=task_label, fill=text_fill, font=("TkDefaultFont", 8, "bold"), tags=("marker",))
        for index, point in enumerate(self.custom_tasks, 1):
            x, y = self._transform(point)
            self.canvas.create_oval(x - 10, y - 10, x + 10, y + 10, fill="#06aed4", outline="#084c61", width=2, tags=("marker",))
            self.canvas.create_text(x, y, text=f"P{index}", fill="#ffffff", font=("TkDefaultFont", 8, "bold"), tags=("marker",))
        x, y = self._transform(self.config.start)
        self.canvas.create_polygon(x, y - 11, x + 11, y, x, y + 11, x - 11, y, fill="#1570ef", outline="#0b4a9e", tags=("marker",))
        self.canvas.create_text(x, y, text="S", fill="white", font=("TkDefaultFont", 8, "bold"), tags=("marker",))

    def _draw_task_highlight(self) -> None:
        self.canvas.delete("task_highlight")
        task_label = self._next_task_id()
        if task_label is None or task_label not in self.config.task_labels:
            return
        task_index = self.config.task_labels.index(task_label)
        x, y = self._transform(self.config.tasks[task_index])
        self.canvas.create_oval(
            x - 15,
            y - 15,
            x + 15,
            y + 15,
            fill="",
            outline="#06aed4",
            width=4,
            tags=("task_highlight",),
        )
        self.canvas.tag_raise("marker")

    def _draw_live_overlay(self) -> None:
        self.canvas.delete("actual_trace")
        self.canvas.delete("live_vehicle")
        if len(self.actual_trace) > 1:
            self._draw_polyline(
                list(self.actual_trace), "#175cd3", 3, "actual_trace"
            )
        if self.live_arena_pose is None:
            return
        arena_x, arena_y, arena_yaw = self.live_arena_pose
        centre_x, centre_y = self._transform((arena_x, arena_y))
        arrow_point = (
            arena_x + 0.20 * math.cos(arena_yaw),
            arena_y + 0.20 * math.sin(arena_yaw),
        )
        arrow_x, arrow_y = self._transform(arrow_point)
        colour = "#12b76a" if self.odom_ready else "#98a2b3"
        outline = "#05603a" if self.odom_ready else "#475467"
        self.canvas.create_line(
            centre_x,
            centre_y,
            arrow_x,
            arrow_y,
            fill=outline,
            width=4,
            arrow=self.tk.LAST,
            arrowshape=(10, 12, 5),
            tags=("live_vehicle",),
        )
        self.canvas.create_oval(
            centre_x - 8,
            centre_y - 8,
            centre_x + 8,
            centre_y + 8,
            fill=colour,
            outline="#ffffff",
            width=2,
            tags=("live_vehicle",),
        )
        self.canvas.tag_raise("live_vehicle")

    def clear_trace(self) -> None:
        self.actual_trace.clear()
        self.canvas.delete("actual_trace")

    def _draw_polyline(self, points: Sequence[Point], colour: str, width: int, tag: str) -> None:
        if len(points) < 2:
            return
        coordinates: List[float] = []
        for point in points:
            coordinates.extend(self._transform(point))
        self.canvas.create_line(*coordinates, fill=colour, width=width, tags=(tag,), joinstyle=self.tk.ROUND)
        self.canvas.tag_raise("marker")

    def _draw_final_route(self) -> None:
        self.canvas.delete("final_route")
        route = (
            self.navigation_route_field
            if self.navigation_route_field
            else self.result.route_field if self.result else []
        )
        if not route:
            return
        self._draw_polyline(route, "#d92d20", 3, "final_route")

    def start(self) -> None:
        if self.worker and self.worker.is_alive():
            return
        try:
            loaded_config = load_config(self.config_path, self.mode_variable.get())
        except PlanningError as exception:
            self._show_error(str(exception))
            return
        self.base_config = loaded_config
        self.default_tasks = loaded_config.tasks
        self.default_task_labels = loaded_config.task_labels
        self.selected_default_indices.intersection_update(
            range(len(self.default_tasks))
        )
        self.config = self._selected_config(loaded_config)
        if not self.config.tasks:
            self._show_error("至少选择或添加一个目标点")
            return
        self.cancel_event = threading.Event()
        self.events = queue.Queue(maxsize=24)
        self.result = None
        self.plan_exported = False
        self.route_activated = False
        self.navigation_route_field = []
        self.grid = build_grid(self.config)
        self.start_button.configure(state=self.tk.DISABLED)
        self.status_variable.set("正在规划")
        self.progress_variable.set("生成栅格地图")
        self.order_variable.set("-")
        self.length_variable.set("-")
        self.output_variable.set("-")
        self.publish_variable.set("正在规划")
        self._update_publish_controls()
        self._draw_base()
        self.worker = threading.Thread(target=self._worker_main, daemon=True)
        self.worker.start()

    def _worker_main(self) -> None:
        def emit(event: Dict[str, Any]) -> None:
            if event["type"] in {"astar_expanded", "pair_done"}:
                return
            while not self.cancel_event.is_set():
                try:
                    self.events.put(event, timeout=0.1)
                    return
                except queue.Full:
                    continue
            raise PlanningCancelled("planning cancelled")

        try:
            result = plan_route(self.config, emit, self.cancel_event.is_set)
            paths = export_result(result, self.output_directory)
            emit({"type": "exported", "paths": paths})
        except PlanningCancelled:
            self.events.put({"type": "cancelled"})
        except Exception as exception:  # Keep GUI alive and report worker failures.
            self.events.put({"type": "error", "message": str(exception)})
        finally:
            self.events.put({"type": "worker_done"})

    def _poll_events(self) -> None:
        for _ in range(32):
            if self.events.empty():
                break
            self._process_one_event()
        self.root.after(50, self._poll_events)

    def _process_one_event(self) -> None:
        try:
            event = self.events.get_nowait()
        except queue.Empty:
            return
        event_type = event["type"]
        if event_type == "grid_ready":
            self.grid = event["grid"]
            self.progress_variable.set("栅格与障碍膨胀完成")
        elif event_type == "pair_start":
            left_name = (
                "S"
                if event["left"] == 0
                else self.config.task_labels[event["left"] - 1]
            )
            right_name = (
                "S"
                if event["right"] == 0
                else self.config.task_labels[event["right"] - 1]
            )
            self.progress_variable.set(
                f"A* {event['index']}/{event['total']}：{left_name} 到 {right_name}"
            )
        elif event_type == "tsp_layer":
            self.progress_variable.set(
                f"Held-Karp：已处理包含 {event['subset_size']}/{event['total']} 个任务点的状态"
            )
        elif event_type == "complete":
            self.result = event["result"]
            self.progress_variable.set("路线计算和碰撞检查通过")
            order = [
                "S" if node == 0 else self.config.task_labels[node - 1]
                for node in self.result.node_order
            ]
            self.order_variable.set(" → ".join(order))
            self.length_variable.set(f"{self.result.total_length:.2f} m，{len(self.result.route_ros)} 个航点")
            self._draw_final_route()
            self._update_waypoint_display()
            self._draw_task_highlight()
        elif event_type == "exported":
            route_path = event["paths"]["route_csv"]
            self.output_variable.set(str(route_path.parent))
            self.status_variable.set("规划完成，文件已导出")
            self.plan_exported = True
            self._update_publish_controls()
        elif event_type == "cancelled":
            self.status_variable.set("规划已取消")
        elif event_type == "error":
            self.status_variable.set("规划失败")
            self._show_error(event["message"])
        elif event_type == "worker_done":
            self._update_edit_controls()

    def reset(self) -> None:
        if self._editing_locked():
            return
        self.cancel_event.set()
        self.selected_default_indices = set(range(len(self.default_tasks)))
        self.custom_tasks.clear()
        self.config = self._selected_config()
        self.result = None
        self.plan_exported = False
        self.route_activated = False
        self.status_variable.set("准备就绪")
        self.progress_variable.set("尚未开始规划")
        self.order_variable.set("-")
        self.length_variable.set("-")
        self.output_variable.set("-")
        self.publish_variable.set("请先完成规划")
        self.navigation_route_field = []
        self._refresh_target_status()
        self._update_publish_controls()
        self._update_edit_controls()
        self._update_waypoint_display()
        self._draw_base()

    def _show_error(self, message: str) -> None:
        from tkinter import messagebox

        messagebox.showerror("路径规划错误", message)

    def _close(self) -> None:
        self.cancel_event.set()
        if self.ros_monitor is not None:
            self.ros_monitor.close()
            self.ros_monitor = None
        self.root.destroy()


def _run_gui(
    config_path: Path,
    output_directory: Optional[Path],
    mode: str,
    enable_ros: bool,
) -> int:
    try:
        import tkinter as tk
    except ImportError as exception:
        raise PlanningError("Tkinter is required for the GUI: sudo apt install python3-tk") from exception
    root = tk.Tk()
    PlannerWindow(root, config_path, output_directory, mode, enable_ros=enable_ros)
    root.mainloop()
    return 0


def _parse_arguments(arguments: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plan a collision-checked closed route through arena targets."
    )
    parser.add_argument("--config", type=Path, default=_default_config_path(), help="arena YAML file")
    parser.add_argument("--output-dir", type=Path, help="override the configured output directory")
    parser.add_argument(
        "--mode",
        choices=("shortest", "numbered"),
        help="override shortest Held-Karp order or numbered 1-12 order",
    )
    parser.add_argument("--headless", action="store_true", help="plan and export without opening the GUI")
    parser.add_argument(
        "--no-ros",
        action="store_true",
        help="disable live ROS monitoring while keeping the planning GUI",
    )
    return parser.parse_args(arguments)


def main(arguments: Optional[Sequence[str]] = None) -> int:
    options = _parse_arguments(arguments)
    try:
        initial_config = load_config(options.config, options.mode)
        mode = options.mode or initial_config.mode
        if not options.headless:
            return _run_gui(
                options.config.resolve(), options.output_dir, mode, not options.no_ros
            )
        result = plan_route(initial_config)
        paths = export_result(result, options.output_dir)
    except PlanningError as exception:
        print(f"arena_route_planner: {exception}", file=sys.stderr)
        return 2

    order = " -> ".join(
        "S" if node == 0 else initial_config.task_labels[node - 1]
        for node in result.node_order
    )
    print(f"Planning mode: {initial_config.mode}")
    print(f"Visit order: {order}")
    print(
        "Actual first visits: "
        + " -> ".join(
            initial_config.task_labels[index - 1]
            for index in result.actual_first_visit_order
        )
    )
    print(f"Route length: {result.total_length:.2f} m")
    print(f"Waypoints: {len(result.route_ros)}")
    print(f"Route CSV: {paths['route_csv']}")
    print(f"Map YAML: {paths['map_yaml']}")
    print(f"Preview: {paths['preview_png']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
