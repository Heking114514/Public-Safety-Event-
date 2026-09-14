"""Actual road-interval coverage derived from the vehicle trajectory."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence


@dataclass(frozen=True)
class RoadEdge:
    label: str
    start: tuple[float, float]
    end: tuple[float, float]

    @property
    def length(self) -> float:
        return math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])


def _merge(
    intervals: Iterable[tuple[float, float]], gap_tolerance: float
) -> list[tuple[float, float]]:
    ordered = sorted(
        (max(0.0, min(left, right)), min(1.0, max(left, right)))
        for left, right in intervals
        if math.isfinite(left) and math.isfinite(right)
    )
    merged: list[tuple[float, float]] = []
    for left, right in ordered:
        if right <= left:
            continue
        if not merged or left > merged[-1][1] + gap_tolerance:
            merged.append((left, right))
            continue
        merged[-1] = (merged[-1][0], max(merged[-1][1], right))
    return merged


class RoadCoverageLedger:
    """Accumulates only trajectory segments travelling along configured roads."""

    def __init__(
        self,
        nodes: Sequence[Sequence[float]],
        edges: Sequence[Sequence[object]],
        lateral_tolerance: float = 0.08,
        maximum_heading_error_deg: float = 25.0,
        minimum_motion: float = 0.003,
        maximum_motion: Optional[float] = None,
    ) -> None:
        if not math.isfinite(lateral_tolerance) or lateral_tolerance <= 0.0:
            raise ValueError("lateral_tolerance must be positive and finite")
        if not 0.0 < maximum_heading_error_deg < 90.0:
            raise ValueError("maximum_heading_error_deg must be between 0 and 90")
        if not math.isfinite(minimum_motion) or minimum_motion <= 0.0:
            raise ValueError("minimum_motion must be positive and finite")
        if maximum_motion is not None and (
            not math.isfinite(maximum_motion) or maximum_motion <= minimum_motion
        ):
            raise ValueError("maximum_motion must be finite and exceed minimum_motion")
        self.lateral_tolerance = lateral_tolerance
        self.minimum_motion = minimum_motion
        self.maximum_motion = math.inf if maximum_motion is None else maximum_motion
        self.minimum_alignment = math.cos(math.radians(maximum_heading_error_deg))
        self.edges: dict[str, RoadEdge] = {}
        for item in edges:
            if len(item) < 3:
                raise ValueError("inspection edge requires from, to, and label")
            from_index = int(item[0])
            to_index = int(item[1])
            if from_index < 0 or to_index < 0:
                raise ValueError("inspection edge node index must be nonnegative")
            try:
                start_node = nodes[from_index]
                end_node = nodes[to_index]
            except IndexError as error:
                raise ValueError("inspection edge references a missing node") from error
            start = (float(start_node[0]), float(start_node[1]))
            end = (float(end_node[0]), float(end_node[1]))
            label = str(item[2])
            edge = RoadEdge(label, start, end)
            if not label or label in self.edges or edge.length <= 1.0e-9:
                raise ValueError("inspection edges need unique labels and nonzero length")
            if not all(math.isfinite(value) for value in (*start, *end)):
                raise ValueError("inspection edge coordinates must be finite")
            self.edges[label] = edge
        self._intervals: dict[str, list[tuple[float, float]]] = {
            label: [] for label in self.edges
        }

    def reset(self) -> None:
        for intervals in self._intervals.values():
            intervals.clear()

    def add_interval(self, label: str, start_fraction: float, end_fraction: float) -> None:
        edge = self.edges.get(label)
        if edge is None:
            raise ValueError(f"unknown inspection edge: {label}")
        gap = self.lateral_tolerance / edge.length
        self._intervals[label] = _merge(
            [*self._intervals[label], (start_fraction, end_fraction)], gap
        )

    def observe_motion(
        self, start: tuple[float, float], end: tuple[float, float]
    ) -> list[str]:
        values = (*start, *end)
        if not all(math.isfinite(value) for value in values):
            return []
        move_x = end[0] - start[0]
        move_y = end[1] - start[1]
        move_length = math.hypot(move_x, move_y)
        if move_length < self.minimum_motion or move_length > self.maximum_motion:
            return []
        move_unit = (move_x / move_length, move_y / move_length)
        updated: list[str] = []
        for label, edge in self.edges.items():
            edge_x = edge.end[0] - edge.start[0]
            edge_y = edge.end[1] - edge.start[1]
            edge_length = edge.length
            edge_unit = (edge_x / edge_length, edge_y / edge_length)
            alignment = abs(move_unit[0] * edge_unit[0] + move_unit[1] * edge_unit[1])
            if alignment < self.minimum_alignment:
                continue

            def projection_and_distance(point: tuple[float, float]) -> tuple[float, float]:
                relative_x = point[0] - edge.start[0]
                relative_y = point[1] - edge.start[1]
                along = (relative_x * edge_unit[0] + relative_y * edge_unit[1]) / edge_length
                lateral = abs(relative_x * edge_unit[1] - relative_y * edge_unit[0])
                return along, lateral

            start_fraction, start_distance = projection_and_distance(start)
            end_fraction, end_distance = projection_and_distance(end)
            fraction_margin = self.lateral_tolerance / edge_length
            if max(start_distance, end_distance) > self.lateral_tolerance:
                continue
            if max(start_fraction, end_fraction) < -fraction_margin or min(
                start_fraction, end_fraction
            ) > 1.0 + fraction_margin:
                continue
            before = tuple(self._intervals[label])
            self.add_interval(
                label,
                min(start_fraction, end_fraction) - fraction_margin,
                max(start_fraction, end_fraction) + fraction_margin,
            )
            if tuple(self._intervals[label]) != before:
                updated.append(label)
        return updated

    def intervals(self) -> list[tuple[str, float, float]]:
        return [
            (label, left, right)
            for label in self.edges
            for left, right in self._intervals[label]
        ]

    def covered_edges(self) -> list[str]:
        return [
            label
            for label, intervals in self._intervals.items()
            if len(intervals) == 1
            and intervals[0][0] <= 1.0e-9
            and intervals[0][1] >= 1.0 - 1.0e-9
        ]

    def covered_length(self) -> float:
        return sum(
            (right - left) * self.edges[label].length
            for label, intervals in self._intervals.items()
            for left, right in intervals
        )

    def _with_intervals(
        self, intervals: Iterable[tuple[str, float, float]]
    ) -> dict[str, list[tuple[float, float]]]:
        merged = {
            label: list(current) for label, current in self._intervals.items()
        }
        for label, left, right in intervals:
            edge = self.edges.get(label)
            if edge is None:
                continue
            gap = self.lateral_tolerance / edge.length
            merged[label] = _merge([*merged[label], (left, right)], gap)
        return merged

    def additional_length(
        self, intervals: Iterable[tuple[str, float, float]]
    ) -> float:
        candidates = self._with_intervals(intervals)
        predicted = sum(
            (right - left) * self.edges[label].length
            for label, merged in candidates.items()
            for left, right in merged
        )
        return max(0.0, predicted - self.covered_length())

    def accounted_edges(
        self, exempt_intervals: Iterable[tuple[str, float, float]]
    ) -> list[str]:
        """Return roads fully accounted for by driven plus blocked intervals."""
        candidates = self._with_intervals(exempt_intervals)
        return [
            label
            for label, intervals in candidates.items()
            if len(intervals) == 1
            and intervals[0][0] <= 1.0e-9
            and intervals[0][1] >= 1.0 - 1.0e-9
        ]

    def remaining_intervals(self, label: str) -> list[tuple[float, float]]:
        if label not in self.edges:
            raise ValueError(f"unknown inspection edge: {label}")
        remaining: list[tuple[float, float]] = []
        cursor = 0.0
        for left, right in self._intervals[label]:
            if left > cursor:
                remaining.append((cursor, left))
            cursor = max(cursor, right)
        if cursor < 1.0:
            remaining.append((cursor, 1.0))
        return remaining
