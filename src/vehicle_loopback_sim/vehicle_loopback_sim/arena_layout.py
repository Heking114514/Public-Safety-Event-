"""Arena-map helpers shared by the loopback GUI and mission starter."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import yaml


@dataclass(frozen=True)
class ArenaTransform:
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float

    def point_to_map(self, x_m: float, y_m: float) -> dict[str, float]:
        dx = x_m - self.origin_x_m
        dy = y_m - self.origin_y_m
        cosine = math.cos(self.origin_yaw_rad)
        sine = math.sin(self.origin_yaw_rad)
        return {
            "x_m": cosine * dx + sine * dy,
            "y_m": -sine * dx + cosine * dy,
        }

    def yaw_to_map(self, yaw_rad: float) -> float:
        return _normalize_angle(yaw_rad - self.origin_yaw_rad)

    def point_from_map(self, x_m: float, y_m: float) -> dict[str, float]:
        cosine = math.cos(self.origin_yaw_rad)
        sine = math.sin(self.origin_yaw_rad)
        return {
            "x_m": self.origin_x_m + cosine * x_m - sine * y_m,
            "y_m": self.origin_y_m + sine * x_m + cosine * y_m,
        }

    def yaw_from_map(self, yaw_rad: float) -> float:
        return _normalize_angle(yaw_rad + self.origin_yaw_rad)


@dataclass(frozen=True)
class ArenaStart:
    x_m: float
    y_m: float
    yaw_rad: float


def load_root(path: str | Path) -> Mapping[str, Any]:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    if not isinstance(root, Mapping):
        raise ValueError("arena config root must be a mapping")
    return root


def load_start(root: Mapping[str, Any]) -> ArenaStart:
    start = _mapping(root, "start")
    position = _point(start, "position_m")
    yaw_rad = math.radians(_number(start, "heading_deg"))
    return ArenaStart(position[0], position[1], yaw_rad)


def layout_from_config(path: str | Path) -> dict[str, Any]:
    root = load_root(path)
    arena = _mapping(root, "arena")
    start = load_start(root)
    transform = ArenaTransform(start.x_m, start.y_m, start.yaw_rad)
    nodes = [_point_value(point) for point in _sequence(_mapping(root, "inspection_graph"), "nodes")]
    return {
        "frame_id": "map",
        "width_m": _number(arena, "width_m"),
        "height_m": _number(arena, "height_m"),
        "start": {
            **transform.point_to_map(start.x_m, start.y_m),
            "yaw_rad": 0.0,
        },
        "free_regions": [
            _rect_polygon(transform, rect)
            for rect in _sequence(arena, "free_regions")
        ],
        "staging_regions": [
            _rect_polygon(transform, rect)
            for rect in arena.get("staging_regions", [])
        ],
        "obstacles": [
            _rect_polygon(transform, rect)
            for rect in _sequence(arena, "obstacles")
        ],
        "turn_junctions": [
            transform.point_to_map(*_point_value(point))
            for point in arena.get("turn_junctions", [])
        ],
        "tasks": _task_points(root, transform),
        "inspection_edges": _inspection_edges(root, nodes, transform),
    }


def arena_pose_to_map(
    transform: ArenaTransform,
    x_m: float,
    y_m: float,
    yaw_rad: float,
) -> dict[str, float]:
    point = transform.point_to_map(x_m, y_m)
    point["yaw_rad"] = transform.yaw_to_map(yaw_rad)
    return point


def transform_from_config(path: str | Path) -> ArenaTransform:
    start = load_start(load_root(path))
    return ArenaTransform(start.x_m, start.y_m, start.yaw_rad)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _mapping(parent: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = parent.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"arena config requires mapping '{key}'")
    return value


def _sequence(parent: Mapping[str, Any], key: str) -> list[Any]:
    value = parent.get(key)
    if not isinstance(value, list):
        raise ValueError(f"arena config requires sequence '{key}'")
    return value


def _number(parent: Mapping[str, Any], key: str) -> float:
    value = float(parent[key])
    if not math.isfinite(value):
        raise ValueError(f"arena config requires finite '{key}'")
    return value


def _point(parent: Mapping[str, Any], key: str) -> tuple[float, float]:
    return _point_value(parent[key])


def _point_value(value: Any) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError("arena point must contain x and y")
    x_m = float(value[0])
    y_m = float(value[1])
    if not math.isfinite(x_m) or not math.isfinite(y_m):
        raise ValueError("arena point coordinates must be finite")
    return x_m, y_m


def _rect_polygon(transform: ArenaTransform, rect: Any) -> list[dict[str, float]]:
    if not isinstance(rect, list) or len(rect) != 4:
        raise ValueError("arena rectangle must contain four coordinates")
    min_x, min_y, max_x, max_y = [float(value) for value in rect]
    corners = [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ]
    return [transform.point_to_map(x_m, y_m) for x_m, y_m in corners]


def _task_points(
    root: Mapping[str, Any],
    transform: ArenaTransform,
) -> list[dict[str, Any]]:
    tasks = _mapping(root, "tasks")
    result = []
    for label, value in sorted(tasks.items(), key=lambda item: _task_sort_key(item[0])):
        x_m, y_m = _point_value(value)
        result.append({"label": str(label), **transform.point_to_map(x_m, y_m)})
    return result


def _inspection_edges(
    root: Mapping[str, Any],
    nodes: list[tuple[float, float]],
    transform: ArenaTransform,
) -> list[dict[str, Any]]:
    graph = _mapping(root, "inspection_graph")
    result = []
    for edge in _sequence(graph, "edges"):
        if not isinstance(edge, list) or len(edge) != 4:
            raise ValueError("inspection edge must contain from, to, label and tunnel")
        begin_index = int(edge[0])
        end_index = int(edge[1])
        begin = nodes[begin_index]
        end = nodes[end_index]
        result.append(
            {
                "label": str(edge[2]),
                "tunnel": bool(edge[3]),
                "from": transform.point_to_map(*begin),
                "to": transform.point_to_map(*end),
            }
        )
    return result


def _task_sort_key(label: str) -> tuple[int, str]:
    text = str(label)
    try:
        return int(text), text
    except ValueError:
        return 999999, text
