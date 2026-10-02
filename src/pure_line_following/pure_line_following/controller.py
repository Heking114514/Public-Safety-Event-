"""Odometry-independent differential-drive line-following controller."""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry_msgs.msg import Twist

from .tracker import (
    JUNCTION,
    LEFT_BRANCH,
    LOST,
    RIGHT_BRANCH,
    UNSAFE_EDGE,
    TrackObservation,
)


@dataclass(frozen=True)
class ControllerConfig:
    max_linear_speed_mps: float = 0.07
    min_linear_speed_mps: float = 0.035
    corner_linear_speed_mps: float = 0.035
    junction_linear_speed_mps: float = 0.055
    min_confidence: float = 0.35
    max_angular_speed_rps: float = 0.55
    lateral_gain: float = 3.6
    heading_gain: float = 0.8
    lateral_integral_gain: float = 1.0
    lateral_integral_limit_m_s: float = 0.06
    lateral_integral_max_error_m: float = 0.04
    lateral_integral_max_heading_rad: float = 0.22
    lateral_integral_leak_per_s: float = 1.2
    angular_deadband_rps: float = 0.02
    speed_turn_slowdown: float = 0.65
    rotate_in_place_angular_threshold_rps: float = 0.50
    max_angular_acceleration_rps2: float = 2.0
    max_angular_deceleration_rps2: float = 4.0
    junction_lateral_deadband_m: float = 0.0
    junction_lateral_gain_scale: float = 1.0
    junction_heading_deadband_rad: float = 0.0
    road_center_slow_error_m: float = 0.025
    road_center_stop_error_m: float = 0.09
    road_width_m: float = 0.20
    vehicle_width_m: float = 0.1535
    road_edge_margin_m: float = 0.010
    pivot_heading_error_rad: float = 0.32
    heading_ignore_road_width_m: float = 0.22
    min_observed_road_width_m: float = 0.12
    max_recovery_lateral_error_m: float = 0.12
    unsafe_recovery_max_angular_speed_rps: float = 0.35
    recovery_min_confidence: float = 0.05


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _usable_center_error(params: ControllerConfig) -> float:
    physical = 0.5 * (params.road_width_m - params.vehicle_width_m)
    physical = max(0.0, physical - params.road_edge_margin_m)
    if params.road_center_stop_error_m > 0.0:
        return params.road_center_stop_error_m
    return physical


def _has_recovery_geometry(
    observation: TrackObservation,
    params: ControllerConfig,
) -> bool:
    """Return whether an unsafe frame still contains both road boundaries."""

    observed_width = float(observation.road_clearance_m)
    left = float(observation.road_left_y_m)
    right = float(observation.road_right_y_m)
    minimum_width = max(0.0, params.min_observed_road_width_m)
    if not all(math.isfinite(value) for value in (observed_width, left, right)):
        return False
    # TrackObservation uses 0.0 for an unavailable boundary. Ground-grid
    # samples are not expected to land exactly on zero, so do not treat that
    # sentinel as usable recovery geometry.
    if abs(left) <= 1.0e-9 or abs(right) <= 1.0e-9:
        return False
    if observed_width < minimum_width:
        return False
    boundary_width = abs(right - left)
    if boundary_width < minimum_width:
        return False
    # The tracker reports both values from the same near-field slice. Reject
    # malformed observations rather than steering from a stale/mismatched pair.
    width_tolerance = max(0.02, 0.25 * observed_width)
    if abs(boundary_width - observed_width) > width_tolerance:
        return False
    if abs(float(observation.lateral_error_m)) > max(
        0.0,
        params.max_recovery_lateral_error_m,
    ):
        return False
    return True


def _angular_command(
    lateral_error: float,
    heading_error: float,
    params: ControllerConfig,
    lateral_integral_m_s: float = 0.0,
    maximum: float | None = None,
    lateral_gain_scale: float = 1.0,
) -> float:
    integral = _clamp(
        lateral_integral_m_s,
        -max(0.0, params.lateral_integral_limit_m_s),
        max(0.0, params.lateral_integral_limit_m_s),
    )
    angular = (
        params.lateral_gain
        * _clamp(float(lateral_gain_scale), 0.0, 1.0)
        * lateral_error
        + params.heading_gain * heading_error
        + params.lateral_integral_gain * integral
    )
    angular = _clamp(
        angular,
        -params.max_angular_speed_rps,
        params.max_angular_speed_rps,
    )
    if maximum is not None:
        angular = _clamp(angular, -maximum, maximum)
    if abs(angular) < params.angular_deadband_rps:
        return 0.0
    return angular


def command_for(
    observation: TrackObservation,
    params: ControllerConfig,
    lateral_integral_m_s: float = 0.0,
) -> Twist:
    """Convert one current-frame observation into a Twist command."""

    command = Twist()
    if observation.state == UNSAFE_EDGE:
        # An unsafe frame must never advance. A complete pair of road
        # boundaries can still support a slow, bounded pivot back toward the
        # road centre; a narrow or one-sided remnant cannot.
        if not _has_recovery_geometry(observation, params):
            return command
        command.angular.z = _angular_command(
            observation.lateral_error_m,
            observation.heading_error_rad,
            params,
            lateral_integral_m_s=0.0,
            maximum=min(
                params.max_angular_speed_rps,
                max(0.0, params.unsafe_recovery_max_angular_speed_rps),
            ),
        )
        return command

    if (
        not observation.valid
        or observation.state == LOST
        or observation.confidence < params.min_confidence
    ):
        return command

    lateral_error = observation.lateral_error_m
    heading_error = observation.heading_error_rad
    lateral_gain_scale = 1.0
    # At a crossroad the horizontal branch corrupts the visual heading more
    # than the near-field centre. Drive straight through it and only apply a
    # gentle lateral correction outside a deadband.
    if observation.state == JUNCTION:
        heading_error = 0.0
        if abs(lateral_error) <= params.junction_lateral_deadband_m:
            lateral_error = 0.0
        lateral_gain_scale = params.junction_lateral_gain_scale
    elif (
        math.isfinite(float(observation.road_clearance_m))
        and observation.road_clearance_m >= params.heading_ignore_road_width_m
    ):
        # A very wide near-field support is usually a crossroad/side branch or
        # a nearby white object joined to the road. Its centre can still be
        # useful, but its tangent is not the straight-ahead track direction.
        heading_error = 0.0

    angular = _angular_command(
        lateral_error,
        heading_error,
        params,
        lateral_integral_m_s=lateral_integral_m_s,
        lateral_gain_scale=lateral_gain_scale,
    )

    speed_limit = params.max_linear_speed_mps
    if observation.state in (LEFT_BRANCH, RIGHT_BRANCH):
        speed_limit = min(speed_limit, params.corner_linear_speed_mps)
    elif observation.state == JUNCTION:
        speed_limit = min(speed_limit, params.junction_linear_speed_mps)

    turn_fraction = abs(angular) / max(params.max_angular_speed_rps, 1.0e-6)
    speed_scale = max(0.0, 1.0 - params.speed_turn_slowdown * turn_fraction)
    linear = speed_limit * speed_scale

    # The nominal road is only 200 mm wide while the measured chassis envelope
    # is 153.5 mm. Once the centre error is more than a few centimetres, moving
    # forward is no longer a correction: it consumes the remaining clearance.
    # Pivot until the camera sees the centreline again.
    lateral_abs = abs(lateral_error)
    stop_error = _usable_center_error(params)
    if lateral_abs >= stop_error:
        linear = 0.0
    elif lateral_abs > params.road_center_slow_error_m:
        fraction = (
            lateral_abs - params.road_center_slow_error_m
        ) / max(
            stop_error - params.road_center_slow_error_m,
            1.0e-6,
        )
        linear *= max(0.0, 1.0 - fraction)

    if abs(heading_error) >= params.pivot_heading_error_rad:
        linear = 0.0

    if linear > 0.0:
        if abs(angular) < params.rotate_in_place_angular_threshold_rps:
            linear = max(params.min_linear_speed_mps, linear)
        else:
            linear = 0.0
    command.linear.x = _clamp(linear, 0.0, params.max_linear_speed_mps)
    command.angular.z = angular
    return command


def limit_angular_rate(
    desired: float,
    previous: float,
    elapsed_s: float,
    params: ControllerConfig,
) -> float:
    """Limit steering changes without delaying the zero-command safety path."""

    desired = _clamp(
        desired,
        -params.max_angular_speed_rps,
        params.max_angular_speed_rps,
    )
    previous = _clamp(
        previous,
        -params.max_angular_speed_rps,
        params.max_angular_speed_rps,
    )
    if elapsed_s <= 0.0:
        return desired
    changing_direction = previous != 0.0 and desired * previous < 0.0
    rate = (
        params.max_angular_deceleration_rps2
        if changing_direction or abs(desired) < abs(previous)
        else params.max_angular_acceleration_rps2
    )
    step = max(0.0, rate) * elapsed_s
    return _clamp(desired, previous - step, previous + step)
