#!/usr/bin/env python3
"""Route acknowledgement and completion bookkeeping for arena missions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Optional, Sequence


ROUTE_DEPENDENT_FAULTS = frozenset(
    {
        "FAULT_NO_PATH_PROGRESS",
        "FAULT_NO_TURN_PROGRESS",
        "FAULT_WAYPOINT_BRAKE_TIMEOUT",
    }
)
ROUTE_LOST_STATUSES = frozenset({"IDLE", "WAITING_FOR_ROUTE"})
PLANNER_REQUEST_TIMEOUT_SECONDS = 30.0


def valid_odometry_stamp(
    frame_id: str,
    child_frame_id: str,
    pose_values: Sequence[float],
    stamp_sec: int,
    stamp_nanosec: int,
    current_time_ns: int,
    last_stamp_ns: int,
    maximum_age_seconds: float = 0.5,
    future_tolerance_seconds: float = 0.1,
) -> Optional[int]:
    if frame_id != "map" or child_frame_id != "base_link":
        return None
    if len(pose_values) != 7 or not all(math.isfinite(value) for value in pose_values):
        return None
    quaternion_norm = sum(value * value for value in pose_values[3:])
    if quaternion_norm <= 1.0e-12 or not math.isfinite(quaternion_norm):
        return None
    if stamp_sec < 0 or stamp_nanosec < 0 or stamp_nanosec >= 1_000_000_000:
        return None
    stamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
    if stamp_ns <= 0 or stamp_ns <= last_stamp_ns:
        return None
    if current_time_ns > 0:
        age_ns = current_time_ns - stamp_ns
        if age_ns > int(maximum_age_seconds * 1_000_000_000):
            return None
        if age_ns < -int(future_tolerance_seconds * 1_000_000_000):
            return None
    return stamp_ns


@dataclass
class RouteExecution:
    route_id: int
    mode: str
    visit_order: tuple[str, ...]
    covered_edges: tuple[str, ...]
    deferred_targets: tuple[str, ...]
    all_targets_reached: bool
    activation_requested_at: float = 0.0
    acknowledged_at: Optional[float] = None
    active_state_seen: bool = False
    new_progress_count: int = 0
    route_signature: tuple[tuple[int, int], ...] = ()


class AutoReplanBackoff:
    def __init__(self, initial_seconds: float = 2.0, maximum_seconds: float = 5.0) -> None:
        self.initial_seconds = initial_seconds
        self.maximum_seconds = maximum_seconds
        self.reset()

    def reset(self) -> None:
        self._next_seconds = self.initial_seconds

    def next_delay(self) -> float:
        delay = self._next_seconds
        self._next_seconds = min(self.maximum_seconds, self._next_seconds * 1.5)
        return delay


class RouteFailureCooldown:
    """Temporarily avoid a route that failed mechanically, never ban it forever."""

    def __init__(self, seconds: float = 5.0) -> None:
        self.seconds = max(0.0, float(seconds))
        self._blocked_until: dict[tuple[str, tuple[tuple[int, int], ...]], float] = {}

    def clear(self) -> None:
        self._blocked_until.clear()

    def record(
        self,
        status: str,
        mode: str,
        signature: tuple[tuple[int, int], ...],
        now: float,
    ) -> bool:
        if status not in ROUTE_DEPENDENT_FAULTS:
            return False
        self._blocked_until[(mode, signature)] = now + self.seconds
        return True

    def blocked(
        self,
        mode: str,
        signature: tuple[tuple[int, int], ...],
        now: float,
    ) -> bool:
        key = (mode, signature)
        deadline = self._blocked_until.get(key)
        if deadline is None:
            return False
        if now >= deadline:
            self._blocked_until.pop(key, None)
            return False
        return True


class LatestRequestQueue:
    """Single-flight bookkeeping where queued callers replace stale work."""

    def __init__(self) -> None:
        self._next_token = 1
        self.active_token: Optional[int] = None
        self._pending: Any = None
        self._has_pending = False

    @property
    def in_flight(self) -> bool:
        return self.active_token is not None

    def submit(self, item: Any) -> Optional[tuple[int, Any]]:
        if self.in_flight:
            self._pending = item
            self._has_pending = True
            return None
        return self._activate(item)

    def complete(
        self, token: int
    ) -> tuple[bool, Optional[tuple[int, Any]]]:
        if token != self.active_token:
            return False, None
        if self._has_pending:
            item = self._pending
            self._pending = None
            self._has_pending = False
            return True, self._activate(item)
        self.active_token = None
        return True, None

    def cancel(self) -> None:
        self.active_token = None
        self._pending = None
        self._has_pending = False

    def _activate(self, item: Any) -> tuple[int, Any]:
        token = self._next_token
        self._next_token += 1
        self.active_token = token
        return token, item


class MissionExecutionState:
    """Commit planner predictions only after the matching route really finishes."""

    def __init__(
        self,
        task_labels: Sequence[str],
        tunnel_labels: Sequence[str],
        road_labels: Sequence[str],
    ) -> None:
        self.task_labels = tuple(dict.fromkeys(map(str, task_labels)))
        self.tunnel_labels = tuple(dict.fromkeys(map(str, tunnel_labels)))
        self.road_labels = tuple(dict.fromkeys(map(str, road_labels)))
        self.reset()

    def reset(self) -> None:
        self.completed_tasks: set[str] = set()
        self.completed_tunnels: set[str] = set()
        self.completed_roads: set[str] = set()
        self.pending: Optional[RouteExecution] = None
        self.active: Optional[RouteExecution] = None

    def begin_route(self, execution: RouteExecution) -> None:
        if execution.route_id <= 0:
            raise ValueError("route id must be positive")
        if (
            not math.isfinite(execution.activation_requested_at)
            or execution.activation_requested_at < 0.0
        ):
            raise ValueError("activation request time must be finite and nonnegative")
        if self.pending is not None or self.active is not None:
            raise RuntimeError("cannot replace a pending or active mission route")
        self.pending = execution

    def acknowledge(self, route_id: int, received_at: float) -> bool:
        if (
            self.pending is None
            or self.active is not None
            or self.pending.route_id != route_id
            or not math.isfinite(received_at)
            or received_at < self.pending.activation_requested_at
        ):
            return False
        self.pending.acknowledged_at = received_at
        self.active = self.pending
        self.pending = None
        return True

    def discard_pending(self, route_id: int) -> Optional[RouteExecution]:
        if self.pending is None or self.pending.route_id != route_id:
            return None
        execution = self.pending
        self.pending = None
        return execution

    def abort_active(self) -> Optional[RouteExecution]:
        execution = self.active
        self.active = None
        return execution

    def abort_for_status(
        self, status: str, received_at: float
    ) -> Optional[RouteExecution]:
        execution = self.active
        if (
            execution is None
            or execution.acknowledged_at is None
            or not math.isfinite(received_at)
            or received_at < execution.activation_requested_at
            or (
                status not in ROUTE_LOST_STATUSES
                and not status.startswith("FAULT_")
            )
        ):
            return None
        self.active = None
        return execution

    def observe_status(
        self, status: str, received_at: float
    ) -> Optional[RouteExecution]:
        execution = self.active
        if (
            execution is None
            or execution.acknowledged_at is None
            or not math.isfinite(received_at)
            or received_at < execution.activation_requested_at
        ):
            return None
        if status == "GOAL_REACHED":
            return self._complete_goal() if execution.active_state_seen else None
        if status == "IDLE" or status == "WAITING_FOR_ROUTE" or status.startswith("FAULT_"):
            return None
        execution.active_state_seen = True
        return None

    def _complete_goal(self) -> RouteExecution:
        execution = self.active
        if execution is None:
            raise RuntimeError("no active route")
        before = (
            len(self.completed_tasks)
            + len(self.completed_tunnels)
            + len(self.completed_roads)
        )
        self.completed_tasks.update(
            label for label in execution.visit_order if label in self.task_labels
        )
        self.completed_tunnels.update(
            label for label in execution.visit_order if label in self.tunnel_labels
        )
        self.completed_roads.update(
            label for label in execution.covered_edges if label in self.road_labels
        )
        after = (
            len(self.completed_tasks)
            + len(self.completed_tunnels)
            + len(self.completed_roads)
        )
        execution.new_progress_count = after - before
        self.active = None
        return execution

    def remaining_tasks(self) -> list[str]:
        return [label for label in self.task_labels if label not in self.completed_tasks]

    def remaining_tunnels(self) -> list[str]:
        return [label for label in self.tunnel_labels if label not in self.completed_tunnels]

    def remaining_roads(self) -> list[str]:
        return [label for label in self.road_labels if label not in self.completed_roads]

    def covered_roads(self) -> list[str]:
        return [label for label in self.road_labels if label in self.completed_roads]

    def predicted_progress_count(self, execution: RouteExecution) -> int:
        predicted_tasks = set(execution.visit_order).intersection(self.remaining_tasks())
        predicted_tunnels = set(execution.visit_order).intersection(
            self.remaining_tunnels()
        )
        predicted_roads = set(execution.covered_edges).intersection(
            self.remaining_roads()
        )
        return len(predicted_tasks) + len(predicted_tunnels) + len(predicted_roads)

    def probe_may_activate(
        self, execution: RouteExecution, route_has_motion: bool
    ) -> bool:
        if not route_has_motion:
            return False
        if self.predicted_progress_count(execution) > 0:
            return True
        return (
            execution.mode in {"layered", "layer3"}
            and execution.all_targets_reached
            and not execution.deferred_targets
            and not self.remaining_tasks()
            and not self.remaining_tunnels()
            and not self.remaining_roads()
        )

    def continuation_mode(self, execution: RouteExecution) -> Optional[str]:
        if self.remaining_tasks():
            return "layer1"
        if self.remaining_tunnels():
            return "layer2"
        if self.remaining_roads() or execution.deferred_targets:
            return "layer3"
        if execution.mode not in {"layered", "layer3"}:
            return "layer3"
        return None

    def mission_complete(self, execution: RouteExecution) -> bool:
        return (
            execution.mode in {"layered", "layer3"}
            and execution.all_targets_reached
            and not execution.deferred_targets
            and not self.remaining_tasks()
            and not self.remaining_tunnels()
            and not self.remaining_roads()
        )
