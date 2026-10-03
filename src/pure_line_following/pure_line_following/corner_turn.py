"""Interruptible first-corner shunt turn for pure visual line following."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .tracker import JUNCTION, LEFT_BRANCH, RIGHT_BRANCH, TRACK, TrackObservation


LEFT = "left"
RIGHT = "right"
AUTO = "auto"
NONE = "none"

IDLE = "idle"
APPROACH = "approach"
STOP = "stop"
CHECK = "check"
ARC = "arc"
ROTATE = "rotate"
CREEP = "creep"
REVERSE = "reverse"
DONE = "done"


@dataclass(frozen=True)
class MotionCommand:
    linear_x_mps: float = 0.0
    angular_z_rps: float = 0.0


@dataclass(frozen=True)
class CornerTurnDecision:
    observation: TrackObservation
    command: MotionCommand | None = None


@dataclass(frozen=True)
class CornerTurnConfig:
    enabled: bool = True
    once: bool = True
    direction: str = AUTO
    junction_prefer_direction: str = LEFT
    stable_frames: int = 3
    trigger_min_x_m: float = 0.16
    trigger_max_x_m: float = 0.50
    turn_entry_x_m: float = 0.32
    turn_entry_hysteresis_m: float = 0.035
    min_branch_extent_m: float = 0.12
    branch_extent_margin_m: float = 0.070
    min_confidence: float = 0.35
    linear_speed_mps: float = 0.018
    angular_speed_rps: float = 0.30
    min_turn_s: float = 1.20
    max_turn_s: float = 7.50
    approach_timeout_s: float = 12.0
    exit_lateral_error_m: float = 0.040
    exit_heading_error_rad: float = 0.22
    exit_stable_frames: int = 3
    stop_s: float = 0.18
    check_s: float = 0.05
    arc_segment_s: float = 0.22
    rotate_segment_s: float = 0.16
    creep_segment_s: float = 0.16
    reverse_segment_s: float = 0.18
    turn_entry_distance_m: float = 0.070
    target_yaw_rad: float = math.pi / 2.0
    exit_yaw_tolerance_rad: float = 0.20
    road_width_m: float = 0.20
    road_margin_m: float = 0.008
    road_back_m: float = 0.60
    road_forward_m: float = 0.56
    vehicle_length_m: float = 0.1485
    vehicle_width_m: float = 0.1535
    vehicle_center_x_m: float = 0.050
    wheel_track_m: float = 0.1466
    prediction_horizon_s: float = 0.24
    prediction_step_s: float = 0.04
    pose_correction_gain: float = 0.35
    late_turn_progress: float = 0.72
    late_linear_speed_mps: float = 0.018
    late_angular_speed_rps: float = 0.12
    rotate_angular_speed_rps: float = 0.20
    creep_linear_speed_mps: float = 0.010
    reverse_linear_speed_mps: float = -0.008
    reverse_angular_speed_rps: float = -0.04
    max_reverse_count: int = 3


@dataclass(frozen=True)
class _Pose:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


def _direction(value: str, default: str = AUTO) -> str:
    normalized = str(value).strip().lower()
    if normalized in {LEFT, RIGHT, AUTO, NONE}:
        return normalized
    return default


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _finite(value: float, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


class CornerTurnStateMachine:
    """Select the first branch, then execute a stop-check-act corner loop."""

    def __init__(self, params: CornerTurnConfig) -> None:
        self.params = params
        self._candidate_direction = ""
        self._candidate_frames = 0
        self._active_direction = ""
        self._phase = IDLE
        self._started_at = 0.0
        self._phase_started_at = 0.0
        self._maneuver_started_at = 0.0
        self._last_update_time: float | None = None
        self._pose = _Pose()
        self._exit_frames = 0
        self._done = False
        self._last_completed_direction = ""
        self._last_phase_command = MotionCommand()
        self._reverse_count = 0
        self._last_block_reason = ""

    @property
    def active(self) -> bool:
        return bool(self._active_direction)

    @property
    def turning(self) -> bool:
        return bool(self._active_direction and self._phase != APPROACH)

    @property
    def done(self) -> bool:
        return self._done

    def cancel_active(self) -> None:
        self._candidate_direction = ""
        self._candidate_frames = 0
        self._active_direction = ""
        self._phase = DONE if self._done else IDLE
        self._started_at = 0.0
        self._phase_started_at = 0.0
        self._maneuver_started_at = 0.0
        self._last_update_time = None
        self._pose = _Pose()
        self._exit_frames = 0
        self._last_phase_command = MotionCommand()
        self._reverse_count = 0
        self._last_block_reason = ""

    def reset(self) -> None:
        self.cancel_active()
        self._done = False
        self._phase = IDLE
        self._last_completed_direction = ""

    def update(
        self,
        observation: TrackObservation,
        now: float,
        travel_distance_m: float | None = None,
        previous_linear_mps: float = 0.0,
        previous_angular_rps: float = 0.0,
    ) -> CornerTurnDecision:
        del travel_distance_m
        now = _finite(now)
        if self._last_update_time is not None and self.turning:
            elapsed = _clamp(now - self._last_update_time, 0.0, 0.50)
            self._integrate_pose(
                _finite(previous_linear_mps),
                _finite(previous_angular_rps),
                elapsed,
            )
            self._correct_pose_from_observation(observation)
        self._last_update_time = now

        if not self.params.enabled or _direction(self.params.direction) == NONE:
            self.reset()
            return CornerTurnDecision(observation)

        if self._active_direction:
            return self._update_active(observation, now)

        if self._done and self.params.once:
            self._candidate_direction = ""
            self._candidate_frames = 0
            return CornerTurnDecision(observation)

        candidate = self._corner_direction(observation)
        if not candidate:
            self._candidate_direction = ""
            self._candidate_frames = 0
            return CornerTurnDecision(observation)

        if candidate == self._candidate_direction:
            self._candidate_frames += 1
        else:
            self._candidate_direction = candidate
            self._candidate_frames = 1

        if self._candidate_frames < max(1, int(self.params.stable_frames)):
            return CornerTurnDecision(observation)

        self._candidate_direction = ""
        self._candidate_frames = 0
        self._active_direction = candidate
        self._started_at = now
        self._phase_started_at = now
        self._phase = APPROACH
        self._exit_frames = 0

        if self._ready_to_turn(observation):
            self._begin_turn(now)
            return self._update_active(observation, now)
        return CornerTurnDecision(observation)

    def _update_active(
        self,
        observation: TrackObservation,
        now: float,
    ) -> CornerTurnDecision:
        if self._phase == APPROACH:
            return self._update_approach(observation, now)

        elapsed_turn = max(0.0, now - self._maneuver_started_at)
        if elapsed_turn >= self._turn_timeout_s():
            if not self._target_yaw_reached():
                self._last_block_reason = "turn_timeout_before_target"
                self._set_phase(CHECK, now, MotionCommand())
                return CornerTurnDecision(
                    self._branch_follow_observation(
                        observation,
                        self._active_direction,
                    ),
                    MotionCommand(),
                )
            self._complete_turn()
            return CornerTurnDecision(observation)

        if (
            elapsed_turn >= max(0.0, float(self.params.min_turn_s))
            and self._target_yaw_reached()
            and self._exit_observation_ready(observation)
        ):
            self._exit_frames += 1
            if self._exit_frames >= max(1, int(self.params.exit_stable_frames)):
                self._complete_turn()
                return CornerTurnDecision(observation)
        elif self._phase != CHECK:
            self._exit_frames = 0

        command = self._phase_command(observation, now)
        return CornerTurnDecision(
            self._branch_follow_observation(observation, self._active_direction),
            command,
        )

    def _update_approach(
        self,
        observation: TrackObservation,
        now: float,
    ) -> CornerTurnDecision:
        elapsed = max(0.0, now - self._started_at)
        approach_timeout = max(0.0, float(self.params.approach_timeout_s))
        if approach_timeout > 0.0 and elapsed >= approach_timeout:
            self.cancel_active()
            return CornerTurnDecision(observation)
        if not self._side_is_supported(observation, self._active_direction):
            self.cancel_active()
            return CornerTurnDecision(observation)
        if self._ready_to_turn(observation):
            self._begin_turn(now)
            return self._update_active(observation, now)
        return CornerTurnDecision(observation)

    def _begin_turn(self, now: float) -> None:
        entry = max(0.0, float(self.params.turn_entry_distance_m))
        self._phase = STOP
        self._phase_started_at = now
        self._maneuver_started_at = now
        self._pose = _Pose(-entry, 0.0, 0.0)
        self._exit_frames = 0
        self._reverse_count = 0
        self._last_phase_command = MotionCommand()
        self._last_block_reason = ""

    def _complete_turn(self) -> None:
        self._last_completed_direction = self._active_direction
        self._done = True
        self.cancel_active()

    def _phase_command(
        self,
        observation: TrackObservation,
        now: float,
    ) -> MotionCommand:
        phase_elapsed = max(0.0, now - self._phase_started_at)
        if self._phase == STOP:
            if phase_elapsed < max(0.0, float(self.params.stop_s)):
                return MotionCommand()
            self._set_phase(CHECK, now, MotionCommand())
            return MotionCommand()

        if self._phase == CHECK:
            if phase_elapsed < max(0.0, float(self.params.check_s)):
                return MotionCommand()
            return self._choose_next_action(observation, now)

        duration = {
            ARC: self.params.arc_segment_s,
            ROTATE: self.params.rotate_segment_s,
            CREEP: self.params.creep_segment_s,
            REVERSE: self.params.reverse_segment_s,
        }.get(self._phase, 0.0)
        if (
            phase_elapsed < max(0.0, float(duration))
            and self._command_is_safe(self._last_phase_command)
        ):
            return self._last_phase_command
        return self._choose_next_action(observation, now)

    def _choose_next_action(
        self,
        observation: TrackObservation,
        now: float,
    ) -> MotionCommand:
        if self._target_yaw_reached():
            if self._exit_observation_ready(observation):
                self._set_phase(CHECK, now, MotionCommand())
                return MotionCommand()
            creep = self._creep_command()
            if self._command_is_safe(creep):
                return self._set_phase(CREEP, now, creep)
            self._last_block_reason = "exit_creep_unsafe"
            self._set_phase(CHECK, now, MotionCommand())
            return MotionCommand()

        for phase, command in (
            (ARC, self._arc_command()),
            (ROTATE, self._rotate_command()),
            (CREEP, self._creep_command()),
        ):
            if self._command_is_safe(command):
                self._last_block_reason = ""
                return self._set_phase(phase, now, command)

        if self._reverse_count < max(0, int(self.params.max_reverse_count)):
            reverse = self._reverse_command()
            if self._command_is_safe(reverse):
                self._reverse_count += 1
                self._last_block_reason = ""
                return self._set_phase(REVERSE, now, reverse)

        self._last_block_reason = "all_predicted_paths_unsafe"
        self._set_phase(CHECK, now, MotionCommand())
        return MotionCommand()

    def _set_phase(
        self,
        phase: str,
        now: float,
        command: MotionCommand,
    ) -> MotionCommand:
        self._phase = phase
        self._phase_started_at = now
        self._last_phase_command = command
        return command

    def _ready_to_turn(self, observation: TrackObservation) -> bool:
        branch_x = float(observation.branch_x_m)
        if not math.isfinite(branch_x):
            return False
        entry_x = max(
            float(self.params.trigger_min_x_m),
            float(self.params.turn_entry_x_m),
        )
        if self._phase == APPROACH:
            entry_x += max(0.0, float(self.params.turn_entry_hysteresis_m))
        return branch_x <= entry_x

    def _corner_direction(self, observation: TrackObservation) -> str:
        if not observation.valid:
            return ""
        if observation.confidence < max(0.0, float(self.params.min_confidence)):
            return ""
        branch_x = float(observation.branch_x_m)
        if not math.isfinite(branch_x):
            return ""
        if branch_x < float(self.params.trigger_min_x_m):
            return ""
        if branch_x > float(self.params.trigger_max_x_m):
            return ""

        requested = _direction(self.params.direction)
        if requested in {LEFT, RIGHT}:
            return requested if self._side_is_supported(observation, requested) else ""

        if observation.state == LEFT_BRANCH:
            return LEFT if self._side_is_supported(observation, LEFT) else ""
        if observation.state == RIGHT_BRANCH:
            return RIGHT if self._side_is_supported(observation, RIGHT) else ""
        if observation.state != JUNCTION:
            return ""

        left_extent = float(observation.branch_left_extent_m)
        right_extent = float(observation.branch_right_extent_m)
        margin = max(0.0, float(self.params.branch_extent_margin_m))
        if left_extent >= right_extent + margin:
            return LEFT if self._side_is_supported(observation, LEFT) else ""
        if right_extent >= left_extent + margin:
            return RIGHT if self._side_is_supported(observation, RIGHT) else ""
        preferred = _direction(self.params.junction_prefer_direction, default=LEFT)
        if preferred == AUTO:
            preferred = LEFT
        if preferred == NONE:
            return ""
        return preferred if self._side_is_supported(observation, preferred) else ""

    def _side_is_supported(
        self,
        observation: TrackObservation,
        direction: str,
    ) -> bool:
        if not observation.valid:
            return False
        extent = (
            float(observation.branch_left_extent_m)
            if direction == LEFT
            else float(observation.branch_right_extent_m)
        )
        if not math.isfinite(extent):
            return False
        return extent >= max(0.0, float(self.params.min_branch_extent_m))

    def _branch_follow_observation(
        self,
        observation: TrackObservation,
        direction: str,
    ) -> TrackObservation:
        if not observation.valid:
            return observation

        sign = 1.0 if direction == LEFT else -1.0
        side_extent = (
            float(observation.branch_left_extent_m)
            if direction == LEFT
            else float(observation.branch_right_extent_m)
        )
        branch_x = float(observation.branch_x_m)
        if not math.isfinite(side_extent):
            side_extent = 0.0
        if not math.isfinite(branch_x) or branch_x <= 0.0:
            branch_x = float(self.params.trigger_max_x_m)

        requested = max(0.0, side_extent - 0.5 * float(observation.road_clearance_m))
        requested = max(requested, 0.5 * max(0.0, side_extent))
        requested = min(
            max(0.0, requested),
            max(0.04, 0.80 * max(0.0, side_extent)),
        )
        if requested <= 1.0e-6:
            return replace(observation, lateral_source="corner_shunt")

        lateral = sign * requested
        heading = math.atan2(lateral, max(0.08, branch_x))
        road_width = float(observation.road_clearance_m)
        if not math.isfinite(road_width) or road_width <= 0.0:
            road_width = self.params.road_width_m
        half_width = max(0.04, 0.5 * min(0.24, road_width))
        return replace(
            observation,
            state=LEFT_BRANCH if direction == LEFT else RIGHT_BRANCH,
            lateral_error_m=lateral,
            heading_error_rad=heading,
            road_left_y_m=lateral - half_width,
            road_right_y_m=lateral + half_width,
            road_clearance_m=2.0 * half_width,
            lateral_source="corner_branch_centerline",
        )

    def _exit_observation_ready(self, observation: TrackObservation) -> bool:
        return (
            observation.valid
            and observation.state == TRACK
            and observation.confidence >= max(0.0, float(self.params.min_confidence))
            and abs(float(observation.lateral_error_m))
            <= max(0.0, float(self.params.exit_lateral_error_m))
            and abs(float(observation.heading_error_rad))
            <= max(0.0, float(self.params.exit_heading_error_rad))
        )

    def _target_yaw_reached(self) -> bool:
        target = max(0.05, abs(float(self.params.target_yaw_rad)))
        tolerance = max(0.0, float(self.params.exit_yaw_tolerance_rad))
        return abs(self._pose.yaw_rad) >= max(0.0, target - tolerance)

    def _turn_progress(self) -> float:
        target = max(0.05, abs(float(self.params.target_yaw_rad)))
        return _clamp(abs(self._pose.yaw_rad) / target, 0.0, 1.0)

    def _turn_timeout_s(self) -> float:
        angular = max(
            abs(float(self.params.angular_speed_rps)),
            abs(float(self.params.late_angular_speed_rps)),
            abs(float(self.params.rotate_angular_speed_rps)),
            1.0e-6,
        )
        target = max(0.05, abs(float(self.params.target_yaw_rad)))
        estimated = (
            target / angular
            + max(0.0, float(self.params.stop_s))
            + max(0.0, float(self.params.check_s))
            + 0.80
        )
        return max(max(0.0, float(self.params.max_turn_s)), estimated)

    def _arc_command(self) -> MotionCommand:
        progress = self._turn_progress()
        angular = max(0.0, float(self.params.angular_speed_rps))
        linear = max(0.0, float(self.params.linear_speed_mps))
        if progress >= _clamp(float(self.params.late_turn_progress), 0.0, 1.0):
            angular = max(0.0, float(self.params.late_angular_speed_rps))
            linear = max(linear, max(0.0, float(self.params.late_linear_speed_mps)))
        remaining = max(0.0, abs(float(self.params.target_yaw_rad)) - abs(self._pose.yaw_rad))
        if remaining < 0.35:
            angular = min(angular, max(0.05, 0.50 * remaining))
        angular *= self._turn_sign()
        linear = self._inner_wheel_safe_linear(linear, angular)
        return MotionCommand(linear, angular)

    def _rotate_command(self) -> MotionCommand:
        angular = (
            max(0.0, float(self.params.rotate_angular_speed_rps))
            * self._turn_sign()
        )
        return MotionCommand(0.0, angular)

    def _creep_command(self) -> MotionCommand:
        return MotionCommand(max(0.0, float(self.params.creep_linear_speed_mps)), 0.0)

    def _reverse_command(self) -> MotionCommand:
        angular = float(self.params.reverse_angular_speed_rps) * self._turn_sign()
        return MotionCommand(float(self.params.reverse_linear_speed_mps), angular)

    def _inner_wheel_safe_linear(self, linear: float, angular: float) -> float:
        half_track = 0.5 * max(0.0, float(self.params.wheel_track_m))
        if half_track <= 0.0 or abs(angular) <= 1.0e-9:
            return linear
        return max(linear, half_track * abs(angular))

    def _turn_sign(self) -> float:
        return 1.0 if self._active_direction == LEFT else -1.0

    def _integrate_pose(
        self,
        linear_mps: float,
        angular_rps: float,
        elapsed_s: float,
    ) -> None:
        if elapsed_s <= 0.0:
            return
        x = self._pose.x_m
        y = self._pose.y_m
        yaw = self._pose.yaw_rad
        if abs(angular_rps) <= 1.0e-6:
            x += linear_mps * elapsed_s * math.cos(yaw)
            y += linear_mps * elapsed_s * math.sin(yaw)
        else:
            next_yaw = yaw + angular_rps * elapsed_s
            radius = linear_mps / angular_rps
            x += radius * (math.sin(next_yaw) - math.sin(yaw))
            y -= radius * (math.cos(next_yaw) - math.cos(yaw))
            yaw = next_yaw
        target = abs(float(self.params.target_yaw_rad)) + 0.35
        yaw = _clamp(yaw, -target, target)
        self._pose = _Pose(x, y, yaw)

    def _correct_pose_from_observation(self, observation: TrackObservation) -> None:
        gain = _clamp(float(self.params.pose_correction_gain), 0.0, 1.0)
        if gain <= 0.0 or not observation.valid:
            return
        if self._phase in {STOP, CHECK}:
            return
        if observation.state not in {TRACK, LEFT_BRANCH, RIGHT_BRANCH, JUNCTION}:
            return
        if not math.isfinite(float(observation.lateral_error_m)):
            return
        sign = self._turn_sign()
        observed_center = sign * float(observation.lateral_error_m)
        if abs(observed_center) > 0.10:
            return
        corrected_y = (1.0 - gain) * self._pose.y_m + gain * sign * observed_center
        self._pose = _Pose(self._pose.x_m, corrected_y, self._pose.yaw_rad)

    def _command_is_safe(self, command: MotionCommand) -> bool:
        if (
            abs(command.linear_x_mps) <= 1.0e-9
            and abs(command.angular_z_rps) <= 1.0e-9
        ):
            return True
        if not self._pose_is_safe(self._pose):
            return False
        horizon = max(0.0, float(self.params.prediction_horizon_s))
        step = max(0.01, float(self.params.prediction_step_s))
        samples = max(1, int(math.ceil(horizon / step)))
        pose = self._pose
        for _ in range(samples):
            pose = self._integrated_pose(
                pose,
                command.linear_x_mps,
                command.angular_z_rps,
                horizon / samples,
            )
            if not self._pose_is_safe(pose):
                return False
        return True

    def _integrated_pose(
        self,
        pose: _Pose,
        linear_mps: float,
        angular_rps: float,
        elapsed_s: float,
    ) -> _Pose:
        x = pose.x_m
        y = pose.y_m
        yaw = pose.yaw_rad
        if abs(angular_rps) <= 1.0e-6:
            x += linear_mps * elapsed_s * math.cos(yaw)
            y += linear_mps * elapsed_s * math.sin(yaw)
        else:
            next_yaw = yaw + angular_rps * elapsed_s
            radius = linear_mps / angular_rps
            x += radius * (math.sin(next_yaw) - math.sin(yaw))
            y -= radius * (math.cos(next_yaw) - math.cos(yaw))
            yaw = next_yaw
        return _Pose(x, y, yaw)

    def _pose_is_safe(self, pose: _Pose) -> bool:
        corners = self._footprint_corners(pose)
        for index, start in enumerate(corners):
            end = corners[(index + 1) % len(corners)]
            for step in range(13):
                amount = step / 12.0
                point = (
                    start[0] + (end[0] - start[0]) * amount,
                    start[1] + (end[1] - start[1]) * amount,
                )
                if not self._point_inside_safe_road(point):
                    return False
        center = (
            sum(point[0] for point in corners) / len(corners),
            sum(point[1] for point in corners) / len(corners),
        )
        return self._point_inside_safe_road(center)

    def _footprint_corners(self, pose: _Pose) -> tuple[tuple[float, float], ...]:
        half_length = 0.5 * max(0.0, float(self.params.vehicle_length_m))
        half_width = 0.5 * max(0.0, float(self.params.vehicle_width_m))
        center_x = float(self.params.vehicle_center_x_m)
        cos_yaw = math.cos(pose.yaw_rad)
        sin_yaw = math.sin(pose.yaw_rad)
        corners = []
        for local_x, local_y in (
            (center_x - half_length, -half_width),
            (center_x + half_length, -half_width),
            (center_x + half_length, half_width),
            (center_x - half_length, half_width),
        ):
            corners.append(
                (
                    pose.x_m + cos_yaw * local_x - sin_yaw * local_y,
                    pose.y_m + sin_yaw * local_x + cos_yaw * local_y,
                )
            )
        return tuple(corners)

    def _point_inside_safe_road(self, point: tuple[float, float]) -> bool:
        x, y = point
        y *= self._turn_sign()
        half = max(0.0, 0.5 * float(self.params.road_width_m))
        margin = _clamp(float(self.params.road_margin_m), 0.0, half - 1.0e-6)
        safe_half = half - margin
        back = max(0.0, float(self.params.road_back_m)) - margin
        forward = max(0.0, float(self.params.road_forward_m)) - margin
        in_entry = -back <= x <= safe_half and -safe_half <= y <= safe_half
        in_exit = -safe_half <= x <= safe_half and -safe_half <= y <= forward
        return in_entry or in_exit

    def status(self, now: float | None = None) -> dict[str, object]:
        if self._active_direction:
            state = self._phase
        else:
            state = DONE if self._done else IDLE
        elapsed = 0.0
        phase_elapsed = 0.0
        if self._active_direction and now is not None:
            elapsed = max(0.0, float(now) - self._started_at)
            phase_elapsed = max(0.0, float(now) - self._phase_started_at)
        return {
            "state": state,
            "maneuver_phase": self._phase,
            "direction": self._active_direction or self._last_completed_direction,
            "candidate_direction": self._candidate_direction,
            "candidate_frames": self._candidate_frames,
            "exit_frames": self._exit_frames,
            "elapsed_s": round(elapsed, 3),
            "phase_elapsed_s": round(phase_elapsed, 3),
            "turn_timeout_s": round(self._turn_timeout_s(), 3),
            "turn_progress": round(self._turn_progress(), 3),
            "pose_x_m": round(self._pose.x_m, 4),
            "pose_y_m": round(self._pose.y_m, 4),
            "pose_yaw_rad": round(self._pose.yaw_rad, 4),
            "pose_safe": bool(self._pose_is_safe(self._pose))
            if self._active_direction
            else True,
            "reverse_count": self._reverse_count,
            "block_reason": self._last_block_reason,
            "enabled": bool(self.params.enabled),
            "once": bool(self.params.once),
        }
