"""Geometry helpers for promoting a front-range event into a planning obstacle."""

from __future__ import annotations

import math
from typing import Iterable, Optional


Aabb = tuple[float, float, float, float]
Pose2d = tuple[float, float, float]


class TimedBoolEventGate:
    """Emit Bool state changes and restart true events after publisher silence."""

    def __init__(self, event_timeout: float) -> None:
        if not math.isfinite(event_timeout) or event_timeout <= 0.0:
            raise ValueError("event timeout must be positive and finite")
        self.event_timeout = event_timeout
        self.last_value: Optional[bool] = None
        self.last_received_at: Optional[float] = None

    def observe(self, value: bool, received_at: float) -> bool:
        if not math.isfinite(received_at) or received_at < 0.0:
            return False
        if self.last_received_at is not None and received_at < self.last_received_at:
            return False
        state = bool(value)
        timed_out = (
            state
            and self.last_value is True
            and self.last_received_at is not None
            and received_at - self.last_received_at > self.event_timeout
        )
        emit = self.last_value is None or state != self.last_value or timed_out
        self.last_value = state
        self.last_received_at = received_at
        return emit


def front_obstacle_aabb(
    vehicle_x: float,
    vehicle_y: float,
    vehicle_yaw: float,
    measured_range: float,
    camera_forward_offset: float = 0.070,
    obstacle_width: float = 0.24,
    obstacle_depth: float = 0.12,
) -> Aabb:
    """Return the axis-aligned bounds of an oriented obstacle estimate.

    The range points to the first visible surface. The obstacle extends farther
    in the viewing direction, so observing the same block from the opposite end
    produces approximately the same centre.
    """
    values = (
        vehicle_x,
        vehicle_y,
        vehicle_yaw,
        measured_range,
        camera_forward_offset,
        obstacle_width,
        obstacle_depth,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("obstacle projection values must be finite")
    if measured_range <= 0.0 or camera_forward_offset < 0.0:
        raise ValueError("range must be positive and camera offset nonnegative")
    if obstacle_width <= 0.0 or obstacle_depth <= 0.0:
        raise ValueError("obstacle dimensions must be positive")

    forward = (math.cos(vehicle_yaw), math.sin(vehicle_yaw))
    side = (-forward[1], forward[0])
    centre_distance = camera_forward_offset + measured_range + obstacle_depth * 0.5
    centre = (
        vehicle_x + forward[0] * centre_distance,
        vehicle_y + forward[1] * centre_distance,
    )
    corners = [
        (
            centre[0] + forward[0] * depth_sign * obstacle_depth * 0.5
            + side[0] * width_sign * obstacle_width * 0.5,
            centre[1] + forward[1] * depth_sign * obstacle_depth * 0.5
            + side[1] * width_sign * obstacle_width * 0.5,
        )
        for depth_sign in (-1.0, 1.0)
        for width_sign in (-1.0, 1.0)
    ]
    return (
        min(point[0] for point in corners),
        min(point[1] for point in corners),
        max(point[0] for point in corners),
        max(point[1] for point in corners),
    )


def merge_nearby_aabbs(
    obstacles: Iterable[Aabb], candidate: Aabb, gap: float = 0.08
) -> list[Aabb]:
    """Merge repeated views of the same obstacle without merging distant blocks."""
    if not math.isfinite(gap) or gap < 0.0:
        raise ValueError("merge gap must be finite and nonnegative")
    pending = _validated_aabb(candidate)
    untouched = [_validated_aabb(obstacle) for obstacle in obstacles]
    while True:
        separated: list[Aabb] = []
        merged_any = False
        for obstacle in untouched:
            if _aabbs_are_separated(pending, obstacle, gap):
                separated.append(obstacle)
                continue
            pending = (
                min(pending[0], obstacle[0]),
                min(pending[1], obstacle[1]),
                max(pending[2], obstacle[2]),
                max(pending[3], obstacle[3]),
            )
            merged_any = True
        untouched = separated
        if not merged_any:
            break
    untouched.append(pending)
    return untouched


def clip_aabb(
    obstacle: Aabb, arena_width: float, arena_height: float
) -> Optional[Aabb]:
    """Clip an obstacle to the arena, rejecting estimates wholly outside it."""
    bounds = _validated_aabb(obstacle)
    if (
        not math.isfinite(arena_width)
        or not math.isfinite(arena_height)
        or arena_width <= 0.0
        or arena_height <= 0.0
    ):
        raise ValueError("arena dimensions must be positive and finite")
    clipped = (
        max(0.0, min(arena_width, bounds[0])),
        max(0.0, min(arena_height, bounds[1])),
        max(0.0, min(arena_width, bounds[2])),
        max(0.0, min(arena_height, bounds[3])),
    )
    if clipped[2] - clipped[0] <= 1.0e-6 or clipped[3] - clipped[1] <= 1.0e-6:
        return None
    return clipped


def _validated_aabb(obstacle: Aabb) -> Aabb:
    if len(obstacle) != 4 or not all(math.isfinite(value) for value in obstacle):
        raise ValueError("obstacle bounds must contain four finite values")
    if obstacle[2] <= obstacle[0] or obstacle[3] <= obstacle[1]:
        raise ValueError("obstacle bounds must have positive area")
    return obstacle


def _aabbs_are_separated(left: Aabb, right: Aabb, gap: float) -> bool:
    return (
        left[2] + gap < right[0]
        or right[2] + gap < left[0]
        or left[3] + gap < right[1]
        or right[3] + gap < left[1]
    )


class ObstacleMappingState:
    """Hold one obstacle estimate until navigation explicitly requests replanning."""

    def __init__(
        self,
        arena_width: float,
        arena_height: float,
        maximum_range_age: float = 0.5,
        future_tolerance: float = 0.1,
        pairing_reorder_tolerance: float = 0.1,
    ) -> None:
        if not math.isfinite(maximum_range_age) or maximum_range_age <= 0.0:
            raise ValueError("maximum range age must be positive and finite")
        if not math.isfinite(future_tolerance) or future_tolerance < 0.0:
            raise ValueError("future tolerance must be finite and nonnegative")
        if (
            not math.isfinite(pairing_reorder_tolerance)
            or pairing_reorder_tolerance < 0.0
        ):
            raise ValueError("pairing reorder tolerance must be finite and nonnegative")
        # Validate dimensions once even before an estimate is available.
        clip_aabb((0.0, 0.0, arena_width, arena_height), arena_width, arena_height)
        self.arena_width = arena_width
        self.arena_height = arena_height
        self.maximum_range_age = maximum_range_age
        self.future_tolerance = future_tolerance
        self.pairing_reorder_tolerance = pairing_reorder_tolerance
        self.latest_range: Optional[tuple[float, float]] = None
        self.pending: Optional[Aabb] = None
        self.blocked = False
        self.event_started_at: Optional[float] = None
        self.recovery_active = False

    def update_blocked(
        self,
        blocked: bool,
        received_at: float,
        pose: Optional[Pose2d] = None,
        force_new_event: bool = False,
    ) -> bool:
        if not math.isfinite(received_at) or received_at < 0.0:
            return False
        rising_edge = bool(blocked) and (not self.blocked or force_new_event)
        self.blocked = bool(blocked)
        if not rising_edge:
            return False
        self.event_started_at = received_at
        self.pending = None
        return bool(
            self.recovery_active
            and pose is not None
            and self._capture(pose, received_at)
        )

    def update_range(
        self, measured_range: float, received_at: float, pose: Optional[Pose2d] = None
    ) -> bool:
        if (
            not math.isfinite(measured_range)
            or measured_range <= 0.0
            or not math.isfinite(received_at)
            or received_at < 0.0
        ):
            return False
        self.latest_range = (measured_range, received_at)
        return bool(
            self.recovery_active
            and pose is not None
            and self._capture(pose, received_at)
        )

    def begin_recovery(self, pose: Pose2d, received_at: float) -> bool:
        if not self.recovery_active:
            self.pending = None
        self.recovery_active = True
        return self._capture(pose, received_at)

    def cancel_recovery(self) -> None:
        self.recovery_active = False
        self.pending = None
        self.event_started_at = None

    def promote(
        self, obstacles: Iterable[Aabb], pose: Pose2d, received_at: float
    ) -> tuple[list[Aabb], bool]:
        if self.recovery_active:
            self._capture(pose, received_at)
        candidate = self.pending
        self.cancel_recovery()
        if candidate is None:
            return list(obstacles), False
        return merge_nearby_aabbs(obstacles, candidate), True

    def finalize_navigation_status(
        self,
        status: str,
        obstacles: Iterable[Aabb],
        pose: Pose2d,
        received_at: float,
    ) -> tuple[list[Aabb], bool]:
        """Preserve a measured obstacle when recovery terminates with a fault."""
        if str(status).startswith("FAULT_") and self.pending is not None:
            return self.promote(obstacles, pose, received_at)
        self.cancel_recovery()
        return list(obstacles), False

    def _capture(self, pose: Pose2d, received_at: float) -> bool:
        if (
            self.latest_range is None
            or self.event_started_at is None
            or not math.isfinite(received_at)
        ):
            return False
        measured_range, range_received_at = self.latest_range
        age = received_at - range_received_at
        if age > self.maximum_range_age or age < -self.future_tolerance:
            return False
        if range_received_at < self.event_started_at - self.pairing_reorder_tolerance:
            return False
        try:
            estimate = front_obstacle_aabb(*pose, measured_range)
            candidate = clip_aabb(
                estimate, self.arena_width, self.arena_height
            )
        except (TypeError, ValueError):
            return False
        if candidate is None:
            return False
        self.pending = candidate
        return True
