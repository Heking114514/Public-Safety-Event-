"""Pure controller and planner for the rectangular primitive test."""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Sequence

from .geometry import Pose2D, build_clockwise_rectangle, wrap_angle


_EPSILON = 1.0e-9


@dataclass(frozen=True)
class Twist2D:
    linear_x_mps: float
    angular_z_rps: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.linear_x_mps):
            raise ValueError("linear_x_mps must be finite")
        if not math.isfinite(self.angular_z_rps):
            raise ValueError("angular_z_rps must be finite")


@dataclass(frozen=True)
class ActionPrimitive:
    linear_x_mps: float
    angular_z_rps: float
    duration_s: float

    def __post_init__(self) -> None:
        values = (self.linear_x_mps, self.angular_z_rps, self.duration_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("primitive values must be finite")
        if self.duration_s <= 0.0:
            raise ValueError("duration_s must be positive")


@dataclass(frozen=True)
class PlannedPrimitive:
    action: ActionPrimitive
    start: Pose2D
    end: Pose2D


CollisionChecker = Callable[[Pose2D], bool]


def pose_is_finite(pose: Pose2D) -> bool:
    return all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw))


def pose_distance(first: Pose2D, second: Pose2D) -> float:
    return math.hypot(first.x - second.x, first.y - second.y)


def integrate_pose(
    pose: Pose2D, linear_x_mps: float, angular_z_rps: float, duration_s: float
) -> Pose2D:
    """Integrate one constant unicycle command in metres and seconds."""

    if not all(
        math.isfinite(value)
        for value in (linear_x_mps, angular_z_rps, duration_s)
    ):
        raise ValueError("motion values must be finite")
    if duration_s < 0.0:
        raise ValueError("duration_s must not be negative")

    delta_yaw = angular_z_rps * duration_s
    if abs(angular_z_rps) < _EPSILON:
        return Pose2D(
            pose.x + linear_x_mps * duration_s * math.cos(pose.yaw),
            pose.y + linear_x_mps * duration_s * math.sin(pose.yaw),
            wrap_angle(pose.yaw + delta_yaw),
        )

    radius = linear_x_mps / angular_z_rps
    return Pose2D(
        pose.x + radius * (math.sin(pose.yaw + delta_yaw) - math.sin(pose.yaw)),
        pose.y - radius * (math.cos(pose.yaw + delta_yaw) - math.cos(pose.yaw)),
        wrap_angle(pose.yaw + delta_yaw),
    )


def differential_wheel_speeds(
    command: Twist2D, wheel_track_m: float
) -> tuple[float, float]:
    if not math.isfinite(wheel_track_m) or wheel_track_m <= 0.0:
        raise ValueError("wheel_track_m must be finite and positive")
    half_track = 0.5 * wheel_track_m
    return (
        command.linear_x_mps - command.angular_z_rps * half_track,
        command.linear_x_mps + command.angular_z_rps * half_track,
    )


def make_action_primitives(
    translation_step_m: float = 0.03,
    duration_s: float = 0.5,
    turn_steps_deg: Sequence[float] = (3.0, 6.0, 12.0),
    pivot_step_deg: float = 6.0,
) -> tuple[ActionPrimitive, ...]:
    """Create the script-style action set with an explicit time base."""

    if not math.isfinite(translation_step_m) or translation_step_m <= 0.0:
        raise ValueError("translation_step_m must be finite and positive")
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be finite and positive")
    if not all(math.isfinite(step) and step > 0.0 for step in turn_steps_deg):
        raise ValueError("turn_steps_deg must contain positive finite values")
    if not math.isfinite(pivot_step_deg) or pivot_step_deg <= 0.0:
        raise ValueError("pivot_step_deg must be finite and positive")

    linear_speeds = (
        translation_step_m / duration_s,
        -translation_step_m / duration_s,
    )
    angular_rates = [0.0]
    for step_deg in turn_steps_deg:
        rate = math.radians(step_deg) / duration_s
        angular_rates.extend((rate, -rate))

    actions = [
        ActionPrimitive(speed, rate, duration_s)
        for speed in linear_speeds
        for rate in angular_rates
    ]
    pivot_rate = math.radians(pivot_step_deg) / duration_s
    actions.extend(
        (
            ActionPrimitive(0.0, pivot_rate, duration_s),
            ActionPrimitive(0.0, -pivot_rate, duration_s),
        )
    )
    return tuple(actions)


def rectangle_goals(origin: Pose2D, side_length_m: float) -> tuple[Pose2D, ...]:
    return tuple(
        waypoint.pose
        for waypoint in build_clockwise_rectangle(origin, side_length_m)
    )


class PrimitivePlanner:
    """Bounded A* planner over constant differential-drive primitives."""

    def __init__(
        self,
        actions: Iterable[ActionPrimitive] | None = None,
        collision_checker: CollisionChecker | None = None,
        position_resolution_m: float = 0.01,
        yaw_resolution_rad: float = math.radians(5.0),
        collision_sample_period_s: float = 0.05,
        goal_position_tolerance_m: float = 0.04,
        goal_yaw_tolerance_rad: float = math.radians(8.0),
        search_margin_m: float = 0.6,
        max_expansions: int = 50000,
        heuristic_weight: float = 1.2,
    ) -> None:
        self.actions = tuple(actions) if actions is not None else make_action_primitives()
        if not self.actions:
            raise ValueError("at least one action primitive is required")
        if not all(isinstance(action, ActionPrimitive) for action in self.actions):
            raise TypeError("actions must contain ActionPrimitive values")
        self.collision_checker = collision_checker or (lambda pose: False)
        self.position_resolution_m = self._positive(
            position_resolution_m, "position_resolution_m"
        )
        self.yaw_resolution_rad = self._positive(
            yaw_resolution_rad, "yaw_resolution_rad"
        )
        self.collision_sample_period_s = self._positive(
            collision_sample_period_s, "collision_sample_period_s"
        )
        self.goal_position_tolerance_m = self._positive(
            goal_position_tolerance_m, "goal_position_tolerance_m"
        )
        self.goal_yaw_tolerance_rad = self._positive(
            goal_yaw_tolerance_rad, "goal_yaw_tolerance_rad"
        )
        self.search_margin_m = self._nonnegative(search_margin_m, "search_margin_m")
        if max_expansions <= 0:
            raise ValueError("max_expansions must be positive")
        self.max_expansions = int(max_expansions)
        if not math.isfinite(heuristic_weight) or heuristic_weight <= 0.0:
            raise ValueError("heuristic_weight must be finite and positive")
        self.heuristic_weight = heuristic_weight

    @staticmethod
    def _positive(value: float, name: str) -> float:
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    @staticmethod
    def _nonnegative(value: float, name: str) -> float:
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and nonnegative")
        return value

    def plan(self, start: Pose2D, goal: Pose2D) -> Optional[list[PlannedPrimitive]]:
        if not pose_is_finite(start) or not pose_is_finite(goal):
            raise ValueError("start and goal must be finite")
        if self.collision_checker(start):
            return None
        if self._goal_reached(start, goal):
            return []

        bounds = (
            min(start.x, goal.x) - self.search_margin_m,
            max(start.x, goal.x) + self.search_margin_m,
            min(start.y, goal.y) - self.search_margin_m,
            max(start.y, goal.y) + self.search_margin_m,
        )
        start_key = self._state_key(start)
        states = {start_key: start}
        costs = {start_key: 0.0}
        came_from: dict[tuple[int, int, int], tuple[int, int, int]] = {}
        action_from: dict[tuple[int, int, int], PlannedPrimitive] = {}
        counter = itertools.count()
        open_set = [(self._heuristic(start, goal), 0.0, next(counter), start_key)]
        expanded = 0

        while open_set and expanded < self.max_expansions:
            _, current_cost, _, current_key = heapq.heappop(open_set)
            if current_cost > costs[current_key] + _EPSILON:
                continue
            current = states[current_key]
            if self._goal_reached(current, goal):
                return self._reconstruct(current_key, came_from, action_from)

            expanded += 1
            for action in self.actions:
                end = self._rollout(current, action)
                if end is None or not self._within_bounds(end, bounds):
                    continue
                next_key = self._state_key(end)
                next_cost = current_cost + self._action_cost(action)
                if next_cost >= costs.get(next_key, math.inf) - _EPSILON:
                    continue
                states[next_key] = end
                costs[next_key] = next_cost
                came_from[next_key] = current_key
                action_from[next_key] = PlannedPrimitive(action, current, end)
                priority = next_cost + self.heuristic_weight * self._heuristic(
                    end, goal
                )
                heapq.heappush(
                    open_set, (priority, next_cost, next(counter), next_key)
                )
        return None

    def _rollout(self, start: Pose2D, action: ActionPrimitive) -> Optional[Pose2D]:
        samples = max(
            1, int(math.ceil(action.duration_s / self.collision_sample_period_s))
        )
        sample_duration = action.duration_s / float(samples)
        pose = start
        for _ in range(samples):
            pose = integrate_pose(
                pose,
                action.linear_x_mps,
                action.angular_z_rps,
                sample_duration,
            )
            if self.collision_checker(pose):
                return None
        return pose

    def _state_key(self, pose: Pose2D) -> tuple[int, int, int]:
        return (
            self._round_to_grid(pose.x, self.position_resolution_m),
            self._round_to_grid(pose.y, self.position_resolution_m),
            self._round_to_grid(wrap_angle(pose.yaw), self.yaw_resolution_rad),
        )

    @staticmethod
    def _round_to_grid(value: float, resolution: float) -> int:
        return int(math.floor(value / resolution + 0.5))

    def _goal_reached(self, pose: Pose2D, goal: Pose2D) -> bool:
        return (
            pose_distance(pose, goal) <= self.goal_position_tolerance_m
            and abs(wrap_angle(pose.yaw - goal.yaw))
            <= self.goal_yaw_tolerance_rad
        )

    @staticmethod
    def _within_bounds(
        pose: Pose2D, bounds: tuple[float, float, float, float]
    ) -> bool:
        min_x, max_x, min_y, max_y = bounds
        return min_x <= pose.x <= max_x and min_y <= pose.y <= max_y

    @staticmethod
    def _action_cost(action: ActionPrimitive) -> float:
        distance_cost = abs(action.linear_x_mps) * action.duration_s
        turn_cost = 0.02 * abs(action.angular_z_rps) * action.duration_s
        reverse_penalty = 1.6 if action.linear_x_mps < 0.0 else 1.0
        pivot_penalty = 0.015 if abs(action.linear_x_mps) < _EPSILON else 0.0
        return (
            max(distance_cost, 0.001) * reverse_penalty
            + turn_cost
            + pivot_penalty
        )

    @staticmethod
    def _heuristic(first: Pose2D, second: Pose2D) -> float:
        return pose_distance(first, second) + 0.03 * abs(
            wrap_angle(first.yaw - second.yaw)
        )

    @staticmethod
    def _reconstruct(
        end_key: tuple[int, int, int],
        came_from: dict[tuple[int, int, int], tuple[int, int, int]],
        action_from: dict[tuple[int, int, int], PlannedPrimitive],
    ) -> list[PlannedPrimitive]:
        plan: list[PlannedPrimitive] = []
        key = end_key
        while key in came_from:
            plan.append(action_from[key])
            key = came_from[key]
        plan.reverse()
        return plan


class PrimitiveController:
    """Execute a primitive plan from live odometry, replanning on deviation."""

    def __init__(
        self,
        planner: PrimitivePlanner,
        position_tolerance_m: float = 0.05,
        yaw_tolerance_rad: float = math.radians(5.0),
        replan_position_error_m: float = 0.10,
        replan_yaw_error_rad: float = math.radians(12.0),
        replan_cooldown_s: float = 0.30,
        failed_plan_retry_s: float = 0.50,
        goal_line_tolerance_m: float = 0.0,
        goal_lateral_tolerance_m: float = 0.0,
        turn_correction_start_rad: float = math.radians(6.0),
        turn_correction_min_rate_radps: float = math.radians(10.0),
        turn_correction_max_rate_radps: float = math.radians(25.0),
        turn_correction_gain: float = 1.0,
        straight_heading_deadband_rad: float = math.radians(1.5),
        straight_heading_gain: float = 1.5,
        straight_heading_max_rate_radps: float = math.radians(8.0),
    ) -> None:
        self.planner = planner
        self.position_tolerance_m = self._positive(position_tolerance_m)
        self.yaw_tolerance_rad = self._positive(yaw_tolerance_rad)
        self.replan_position_error_m = self._positive(replan_position_error_m)
        self.replan_yaw_error_rad = self._positive(replan_yaw_error_rad)
        self.replan_cooldown_s = self._nonnegative(replan_cooldown_s)
        self.failed_plan_retry_s = self._nonnegative(failed_plan_retry_s)
        self.goal_line_tolerance_m = (
            self.position_tolerance_m
            if goal_line_tolerance_m <= 0.0
            else max(
                self.position_tolerance_m,
                self._positive(goal_line_tolerance_m),
            )
        )
        self.goal_lateral_tolerance_m = (
            self.position_tolerance_m
            if goal_lateral_tolerance_m <= 0.0
            else max(
                self.position_tolerance_m,
                self._positive(goal_lateral_tolerance_m),
            )
        )
        self.turn_correction_start_rad = self._positive(
            turn_correction_start_rad
        )
        self.turn_correction_min_rate_radps = self._positive(
            turn_correction_min_rate_radps
        )
        self.turn_correction_max_rate_radps = self._positive(
            turn_correction_max_rate_radps
        )
        if (
            self.turn_correction_min_rate_radps
            > self.turn_correction_max_rate_radps
        ):
            raise ValueError(
                "turn correction minimum rate must not exceed maximum rate"
            )
        self.turn_correction_gain = self._positive(turn_correction_gain)
        self.straight_heading_deadband_rad = self._nonnegative(
            straight_heading_deadband_rad
        )
        self.straight_heading_gain = self._positive(straight_heading_gain)
        self.straight_heading_max_rate_radps = self._positive(
            straight_heading_max_rate_radps
        )
        self.goals: tuple[Pose2D, ...] = ()
        self.goal_index = 0
        self.plan: list[PlannedPrimitive] = []
        self.action_index = 0
        self.action_start_pose: Optional[Pose2D] = None
        self.action_started_at: Optional[float] = None
        self.last_plan_at = -math.inf
        self.last_plan_reason = ""
        self.plan_attempts = 0
        self.replan_count = 0
        self.state = "IDLE"

    @staticmethod
    def _positive(value: float) -> float:
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("value must be finite and positive")
        return value

    @staticmethod
    def _nonnegative(value: float) -> float:
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("value must be finite and nonnegative")
        return value

    @property
    def current_goal(self) -> Optional[Pose2D]:
        if self.goal_index >= len(self.goals):
            return None
        return self.goals[self.goal_index]

    @property
    def is_complete(self) -> bool:
        return self.state == "COMPLETE"

    def start(self, start_pose: Pose2D, goals: Sequence[Pose2D]) -> None:
        if not pose_is_finite(start_pose):
            raise ValueError("start_pose must be finite")
        if not goals:
            raise ValueError("at least one goal is required")
        if not all(pose_is_finite(goal) for goal in goals):
            raise ValueError("goals must be finite")
        self.goals = tuple(goals)
        self.goal_index = 0
        self._clear_plan()
        self.last_plan_at = -math.inf
        self.last_plan_reason = "start"
        self.plan_attempts = 0
        self.replan_count = 0
        self.state = "RUNNING"

    def stop(self) -> None:
        self.goals = ()
        self.goal_index = 0
        self._clear_plan()
        self.state = "IDLE"

    def invalidate_plan(self, reason: str = "external") -> None:
        self._clear_plan()
        self.last_plan_reason = reason
        if self.goals and self.state != "COMPLETE":
            self.state = "RUNNING"

    def update(self, now_s: float, odometry: Pose2D) -> Twist2D:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        if not pose_is_finite(odometry):
            raise ValueError("odometry must be finite")
        if not self.goals or self.state == "COMPLETE":
            return Twist2D(0.0, 0.0)

        self._advance_reached_goals(odometry)
        if self.state == "COMPLETE":
            return Twist2D(0.0, 0.0)
        if not self.plan or self.action_index >= len(self.plan):
            if not self._request_plan(now_s, odometry, "plan_exhausted"):
                return Twist2D(0.0, 0.0)

        active = self.plan[self.action_index]
        command = self._command_for_action(odometry, active)
        started_at = now_s if self.action_started_at is None else self.action_started_at
        elapsed = max(0.0, now_s - started_at)
        expected = integrate_pose(
            self.action_start_pose or odometry,
            command.linear_x_mps,
            command.angular_z_rps,
            min(elapsed, active.action.duration_s),
        )
        if (
            elapsed > self.replan_cooldown_s
            and now_s - self.last_plan_at >= self.replan_cooldown_s
            and self._deviated(odometry, expected)
        ):
            if not self._request_plan(now_s, odometry, "odometry_deviation"):
                return Twist2D(0.0, 0.0)
            active = self.plan[self.action_index]
            command = self._command_for_action(odometry, active)
            elapsed = 0.0

        if elapsed >= active.action.duration_s:
            self.action_index += 1
            self.action_start_pose = odometry
            self.action_started_at = now_s
            self._advance_reached_goals(odometry)
            if self.state == "COMPLETE":
                return Twist2D(0.0, 0.0)
            if self.action_index >= len(self.plan):
                if not self._request_plan(now_s, odometry, "action_complete"):
                    return Twist2D(0.0, 0.0)
                active = self.plan[self.action_index]
            command = self._command_for_action(odometry, active)

        return command

    def _request_plan(self, now_s: float, odometry: Pose2D, reason: str) -> bool:
        if (
            self.state == "PLAN_FAILED"
            and now_s - self.last_plan_at < self.failed_plan_retry_s
        ):
            return False
        goal = self.current_goal
        if goal is None:
            self.state = "COMPLETE"
            return False

        self.last_plan_at = now_s
        self.last_plan_reason = reason
        self.plan_attempts += 1
        result = self.planner.plan(odometry, goal)
        if not result:
            self._clear_plan()
            self._advance_reached_goals(odometry)
            if self.state != "COMPLETE":
                self.state = "PLAN_FAILED"
            return False

        self.plan = result
        self.action_index = 0
        self.action_start_pose = odometry
        self.action_started_at = now_s
        self.replan_count += 1
        self.state = "RUNNING"
        return True

    def _advance_reached_goals(self, odometry: Pose2D) -> None:
        while self.current_goal is not None and self._goal_reached(
            odometry, self.current_goal
        ):
            self.goal_index += 1
            self._clear_plan()
        if self.current_goal is None:
            self.state = "COMPLETE"

    def _goal_reached(self, odometry: Pose2D, goal: Pose2D) -> bool:
        if abs(wrap_angle(odometry.yaw - goal.yaw)) > self.yaw_tolerance_rad:
            return False
        if pose_distance(odometry, goal) <= self.position_tolerance_m:
            return True

        dx = odometry.x - goal.x
        dy = odometry.y - goal.y
        forward_x = math.cos(goal.yaw)
        forward_y = math.sin(goal.yaw)
        along_error = dx * forward_x + dy * forward_y
        lateral_error = -dx * forward_y + dy * forward_x
        return (
            abs(along_error) <= self.goal_line_tolerance_m
            and abs(lateral_error) <= self.goal_lateral_tolerance_m
        )

    def _deviated(self, odometry: Pose2D, expected: Pose2D) -> bool:
        return (
            pose_distance(odometry, expected) > self.replan_position_error_m
            or abs(wrap_angle(odometry.yaw - expected.yaw))
            > self.replan_yaw_error_rad
        )

    def _command_for_action(
        self, odometry: Pose2D, planned: PlannedPrimitive
    ) -> Twist2D:
        """Apply gentle yaw hold on straights and slower correction on turns."""

        command = Twist2D(
            planned.action.linear_x_mps, planned.action.angular_z_rps
        )
        goal = self.current_goal
        if goal is None:
            return command

        yaw_error = wrap_angle(goal.yaw - odometry.yaw)
        if abs(command.angular_z_rps) <= _EPSILON:
            if abs(yaw_error) <= self.straight_heading_deadband_rad:
                return command
            requested = self.straight_heading_gain * yaw_error
            requested = max(
                -self.straight_heading_max_rate_radps,
                min(self.straight_heading_max_rate_radps, requested),
            )
            return Twist2D(command.linear_x_mps, requested)

        if abs(yaw_error) <= self.turn_correction_start_rad:
            return command

        requested = self.turn_correction_gain * yaw_error
        requested = max(
            -self.turn_correction_max_rate_radps,
            min(self.turn_correction_max_rate_radps, requested),
        )
        if abs(requested) < self.turn_correction_min_rate_radps:
            requested = math.copysign(
                self.turn_correction_min_rate_radps, yaw_error
            )

        if command.angular_z_rps * yaw_error > 0.0:
            requested = math.copysign(
                min(
                    self.turn_correction_max_rate_radps,
                    max(abs(command.angular_z_rps), abs(requested)),
                ),
                yaw_error,
            )
        return Twist2D(command.linear_x_mps, requested)

    def _clear_plan(self) -> None:
        self.plan = []
        self.action_index = 0
        self.action_start_pose = None
        self.action_started_at = None


__all__ = [
    "ActionPrimitive",
    "PlannedPrimitive",
    "PrimitiveController",
    "PrimitivePlanner",
    "Twist2D",
    "differential_wheel_speeds",
    "integrate_pose",
    "make_action_primitives",
    "pose_distance",
    "pose_is_finite",
    "rectangle_goals",
]
