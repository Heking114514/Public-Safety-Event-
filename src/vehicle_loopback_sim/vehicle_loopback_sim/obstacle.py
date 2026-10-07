"""Simple front-obstacle recognition model for loopback simulation."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Iterable

from .motion import Pose2D


Point = tuple[float, float]
Polygon = tuple[Point, ...]


@dataclass(frozen=True)
class ObstacleDetectionConfig:
    sensor_x_m: float
    sensor_y_m: float
    trigger_range_m: float = 0.25
    trigger_jitter_m: float = 0.0
    lateral_tolerance_m: float = 0.08
    min_range_m: float = 0.03
    random_seed: int = 0
    activation_floor_m: float | None = None


@dataclass(frozen=True)
class ObstacleDetection:
    obstacle_index: int
    range_m: float
    trigger_range_m: float


class StableObstacleGate:
    """Delays reports until the same obstacle is detected continuously."""

    def __init__(self, stable_time_s: float) -> None:
        stable_time_s = float(stable_time_s)
        if not math.isfinite(stable_time_s):
            raise ValueError("stable_time_s must be finite")
        self.stable_time_s = max(0.0, stable_time_s)
        self._candidate_index: int | None = None
        self._candidate_since_s: float | None = None

    def reset(self) -> None:
        self._candidate_index = None
        self._candidate_since_s = None

    def update(
        self,
        detection: ObstacleDetection | None,
        now_s: float,
    ) -> ObstacleDetection | None:
        if detection is None:
            self.reset()
            return None
        if self.stable_time_s <= 0.0:
            self._candidate_index = detection.obstacle_index
            self._candidate_since_s = now_s
            return detection
        if (
            self._candidate_index != detection.obstacle_index
            or self._candidate_since_s is None
            or now_s < self._candidate_since_s
        ):
            self._candidate_index = detection.obstacle_index
            self._candidate_since_s = now_s
            return None
        if now_s - self._candidate_since_s < self.stable_time_s:
            return None
        return detection


class SimulatedObstacleDetector:
    """Reports map obstacles when they enter a forward camera corridor."""

    def __init__(
        self,
        obstacles: Iterable[Iterable[Point]],
        config: ObstacleDetectionConfig,
    ) -> None:
        self.config = config
        self.obstacles = tuple(
            tuple(point for point in obstacle) for obstacle in obstacles
        )
        rng = random.Random(None if config.random_seed < 0 else config.random_seed)
        low = max(config.min_range_m, config.trigger_range_m - config.trigger_jitter_m)
        high = max(low, config.trigger_range_m + config.trigger_jitter_m)
        if config.activation_floor_m is None:
            activation_floor_m = low
        else:
            activation_floor_m = max(config.min_range_m, config.activation_floor_m)
        self._activation_floor_m = max(0.0, activation_floor_m)
        self._trigger_ranges = tuple(rng.uniform(low, high) for _ in self.obstacles)
        self._previous_ranges: dict[int, float] = {}
        self._active_index: int | None = None

    def detect(self, pose: Pose2D) -> ObstacleDetection | None:
        ranges = {
            index: distance
            for index, obstacle in enumerate(self.obstacles)
            if (distance := self._corridor_range(pose, obstacle)) is not None
        }
        if self._active_index is not None:
            distance = ranges.get(self._active_index)
            if distance is not None:
                self._previous_ranges.update(ranges)
                return ObstacleDetection(
                    self._active_index,
                    distance,
                    self._trigger_ranges[self._active_index],
                )
            self._active_index = None

        best: ObstacleDetection | None = None
        for index, distance in ranges.items():
            trigger_range = self._trigger_ranges[index]
            previous = self._previous_ranges.get(index)
            approaching = previous is None or distance <= previous + 1.0e-3
            in_trigger_band = self._activation_floor_m <= distance <= trigger_range
            if not approaching or not in_trigger_band:
                continue
            detection = ObstacleDetection(index, distance, trigger_range)
            if best is None or detection.range_m < best.range_m:
                best = detection
        self._previous_ranges = ranges
        if best is not None:
            self._active_index = best.obstacle_index
        return best

    def _corridor_range(self, pose: Pose2D, obstacle: Polygon) -> float | None:
        if len(obstacle) < 3:
            return None
        points = [self._sensor_frame(pose, point) for point in obstacle]
        half_width = max(0.0, self.config.lateral_tolerance_m)
        candidates: list[float] = []

        for forward, lateral in points:
            if forward >= self.config.min_range_m and abs(lateral) <= half_width:
                candidates.append(forward)

        boundaries = (-half_width, 0.0, half_width)
        for begin, end in zip(points, points[1:] + points[:1]):
            f0, y0 = begin
            f1, y1 = end
            for boundary in boundaries:
                crossed = (y0 <= boundary <= y1) or (y1 <= boundary <= y0)
                if not crossed or math.isclose(y0, y1, abs_tol=1.0e-12):
                    continue
                ratio = (boundary - y0) / (y1 - y0)
                forward = f0 + ratio * (f1 - f0)
                if forward >= self.config.min_range_m:
                    candidates.append(forward)
            crossed_front = (f0 <= self.config.min_range_m <= f1) or (
                f1 <= self.config.min_range_m <= f0
            )
            if crossed_front and not math.isclose(f0, f1, abs_tol=1.0e-12):
                ratio = (self.config.min_range_m - f0) / (f1 - f0)
                lateral = y0 + ratio * (y1 - y0)
                if abs(lateral) <= half_width:
                    candidates.append(self.config.min_range_m)

        if not candidates:
            return None
        return min(candidates)

    def _sensor_frame(self, pose: Pose2D, point: Point) -> Point:
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        sensor_x = (
            pose.x + cosine * self.config.sensor_x_m - sine * self.config.sensor_y_m
        )
        sensor_y = (
            pose.y + sine * self.config.sensor_x_m + cosine * self.config.sensor_y_m
        )
        dx = point[0] - sensor_x
        dy = point[1] - sensor_y
        return (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
        )


def layout_obstacles(layout: dict) -> list[Polygon]:
    return [
        tuple((float(point["x_m"]), float(point["y_m"])) for point in polygon)
        for polygon in layout.get("obstacles", [])
    ]


def box_obstacle(
    center_x_m: float,
    center_y_m: float,
    width_m: float,
    depth_m: float,
    yaw_rad: float = 0.0,
) -> Polygon:
    half_width = 0.5 * width_m
    half_depth = 0.5 * depth_m
    forward_x = math.cos(yaw_rad)
    forward_y = math.sin(yaw_rad)
    lateral_x = -math.sin(yaw_rad)
    lateral_y = math.cos(yaw_rad)

    def corner(forward_offset: float, lateral_offset: float) -> Point:
        return (
            center_x_m
            + forward_x * forward_offset
            + lateral_x * lateral_offset,
            center_y_m
            + forward_y * forward_offset
            + lateral_y * lateral_offset,
        )

    return (
        corner(-half_depth, -half_width),
        corner(half_depth, -half_width),
        corner(half_depth, half_width),
        corner(-half_depth, half_width),
    )
