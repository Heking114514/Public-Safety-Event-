"""Deterministic offline checks for the loopback truth simulator.

The offline runner intentionally does not start ROS or production navigation
nodes. It checks the simulator time contract and the JSON truth status.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping, Sequence

from .motion import Pose2D, Twist2D, integrate_base_link, limited_twist


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _mapping(parent: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = parent.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"truth status requires object '{name}'")
    return value


@dataclass(frozen=True)
class PlaybackConfig:
    """Playback settings shared by offline checks and the C++ simulator."""

    mode: str = "closed_loop"
    speed: float = 1.0

    def validate(self) -> None:
        if self.mode not in {"closed_loop", "offline"}:
            raise ValueError("simulation_mode must be closed_loop or offline")
        speed = _number(self.speed, "playback_speed")
        if speed < 0.1 or speed > 10.0:
            raise ValueError("playback_speed must be between 0.1 and 10.0")
        if self.mode == "closed_loop" and not math.isclose(speed, 1.0, abs_tol=1.0e-9):
            raise ValueError(
                "closed_loop requires playback_speed=1.0; use simulation_mode=offline"
            )

    @property
    def acceptance(self) -> bool:
        self.validate()
        return self.mode == "closed_loop" and math.isclose(
            self.speed, 1.0, abs_tol=1.0e-9
        )


@dataclass(frozen=True)
class TruthState:
    pose: Pose2D
    twist: Twist2D
    fusion_status: str

    @classmethod
    def from_status_json(cls, text: str) -> "TruthState":
        try:
            payload = json.loads(text)
        except json.JSONDecodeError as error:
            raise ValueError("truth status must be valid JSON") from error
        if not isinstance(payload, Mapping):
            raise ValueError("truth status root must be an object")
        pose = _mapping(payload, "pose")
        twist = _mapping(payload, "twist")
        fusion_status = payload.get("fusion_status")
        if not isinstance(fusion_status, str) or not fusion_status:
            raise ValueError("truth status requires a non-empty fusion_status")
        return cls(
            pose=Pose2D(
                _number(pose.get("x_m"), "pose.x_m"),
                _number(pose.get("y_m"), "pose.y_m"),
                _number(pose.get("yaw_rad"), "pose.yaw_rad"),
            ),
            twist=Twist2D(
                _number(twist.get("linear_x_mps"), "twist.linear_x_mps"),
                _number(twist.get("angular_z_rps"), "twist.angular_z_rps"),
            ),
            fusion_status=fusion_status,
        )


@dataclass(frozen=True)
class TruthSample:
    sim_time_s: float
    state: TruthState


class TruthOracle:
    """Parse and sequence-check loopback truth status messages."""

    def __init__(self) -> None:
        self._samples: list[TruthSample] = []

    @property
    def samples(self) -> tuple[TruthSample, ...]:
        return tuple(self._samples)

    def observe(self, sim_time_s: float, status_text: str) -> TruthSample:
        stamp = _number(sim_time_s, "sim_time_s")
        if self._samples and stamp < self._samples[-1].sim_time_s:
            raise ValueError("truth sample time must be monotonic")
        sample = TruthSample(stamp, TruthState.from_status_json(status_text))
        self._samples.append(sample)
        return sample


class OfflineTruthRunner:
    """Run the same planar truth integration with deterministic wall steps."""

    def __init__(
        self,
        playback: PlaybackConfig = PlaybackConfig(),
        initial_pose: Pose2D = Pose2D(),
        base_offset_x_m: float = 0.03,
        base_offset_y_m: float = 0.0,
        max_linear_speed_mps: float = 0.2,
        max_angular_speed_rps: float = 0.9,
        max_linear_accel_mps2: float = 0.0,
        max_angular_accel_rps2: float = 0.0,
    ) -> None:
        playback.validate()
        self.playback = playback
        self.pose = initial_pose
        self.twist = Twist2D()
        self.sim_time_s = 0.0
        self.base_offset_x_m = base_offset_x_m
        self.base_offset_y_m = base_offset_y_m
        self.max_linear_speed_mps = max_linear_speed_mps
        self.max_angular_speed_rps = max_angular_speed_rps
        self.max_linear_accel_mps2 = max_linear_accel_mps2
        self.max_angular_accel_rps2 = max_angular_accel_rps2

    def step(self, desired: Twist2D, wall_step_s: float) -> TruthSample:
        wall_step = _number(wall_step_s, "wall_step_s")
        if wall_step <= 0.0:
            raise ValueError("wall_step_s must be positive")
        elapsed = wall_step * self.playback.speed
        self.twist = limited_twist(
            desired,
            self.twist,
            elapsed,
            self.max_linear_speed_mps,
            self.max_angular_speed_rps,
            self.max_linear_accel_mps2,
            self.max_angular_accel_rps2,
        )
        self.pose = integrate_base_link(
            self.pose,
            self.twist,
            elapsed,
            self.base_offset_x_m,
            self.base_offset_y_m,
        )
        self.sim_time_s += elapsed
        return TruthSample(
            self.sim_time_s,
            TruthState(self.pose, self.twist, "FULL"),
        )


def run_constant_command(
    commands: Sequence[Twist2D],
    wall_step_s: float,
    playback: PlaybackConfig,
) -> tuple[TruthSample, ...]:
    """Run a finite command sequence for small offline scenario tests."""

    runner = OfflineTruthRunner(playback=playback)
    return tuple(runner.step(command, wall_step_s) for command in commands)
