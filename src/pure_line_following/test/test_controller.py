import math

from geometry_msgs.msg import Twist

from pure_line_following.controller import (
    ControllerConfig,
    command_for,
    limit_angular_rate,
)
from pure_line_following.tracker import (
    LOST,
    TRACK,
    UNSAFE_EDGE,
    TrackObservation,
)


def observation(
    state=TRACK,
    valid=True,
    lateral=0.0,
    heading=0.0,
    confidence=1.0,
    road_left=0.0,
    road_right=0.0,
    road_clearance=0.0,
):
    return TrackObservation(
        state=state,
        valid=valid,
        confidence=confidence,
        lateral_error_m=lateral,
        heading_error_rad=heading,
        green_fraction=0.5,
        component_area=100,
        control_pixels=50,
        control_rows=10,
        mask=None,
        ground_mask=None,
        component_mask=None,
        grid=None,
        road_left_y_m=road_left,
        road_right_y_m=road_right,
        road_clearance_m=road_clearance,
    )


def test_lost_is_zero():
    command = command_for(observation(LOST, valid=False), ControllerConfig())
    assert isinstance(command, Twist)
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_left_error_turns_left():
    command = command_for(
        observation(lateral=0.1, heading=0.05),
        ControllerConfig(),
    )
    assert command.angular.z > 0.0
    assert command.linear.x == 0.0


def test_small_left_error_moves_while_correcting():
    command = command_for(
        observation(lateral=0.008, heading=0.0),
        ControllerConfig(),
    )
    assert command.angular.z > 0.0
    assert command.linear.x > 0.0


def test_right_error_turns_right():
    command = command_for(
        observation(lateral=-0.1, heading=-0.05),
        ControllerConfig(),
    )
    assert command.angular.z < 0.0


def test_command_is_bounded():
    params = ControllerConfig(max_angular_speed_rps=0.4, max_linear_speed_mps=0.06)
    command = command_for(
        observation(lateral=1.0, heading=math.pi / 2.0),
        params,
    )
    assert abs(command.angular.z) <= params.max_angular_speed_rps
    assert 0.0 <= command.linear.x <= params.max_linear_speed_mps


def test_large_turn_stops_forward_motion():
    params = ControllerConfig(
        max_angular_speed_rps=0.6,
        rotate_in_place_angular_threshold_rps=0.5,
    )
    command = command_for(
        observation(lateral=0.2, heading=0.2),
        params,
    )
    assert command.linear.x == 0.0
    assert abs(command.angular.z) == params.max_angular_speed_rps


def test_centered_junction_keeps_forward_command_straight():
    command = command_for(
        observation(
            state="JUNCTION",
            lateral=0.0,
            heading=0.0,
        ),
        ControllerConfig(),
    )
    assert command.linear.x > 0.0
    assert command.angular.z == 0.0


def test_junction_small_lateral_error_keeps_moving_while_correcting():
    command = command_for(
        observation(
            state="JUNCTION",
            lateral=0.014,
            heading=0.0,
        ),
        ControllerConfig(),
    )
    assert command.linear.x > 0.0
    assert command.angular.z > 0.0


def test_junction_deadband_suppresses_small_lateral_wobble():
    command = command_for(
        observation(
            state="JUNCTION",
            lateral=0.014,
            heading=0.20,
        ),
        ControllerConfig(
            junction_lateral_deadband_m=0.025,
            junction_lateral_gain_scale=0.35,
        ),
    )
    assert command.linear.x > 0.0
    assert command.angular.z == 0.0


def test_junction_lateral_gain_is_reduced_after_deadband():
    normal = command_for(
        observation(
            state="JUNCTION",
            lateral=0.05,
            heading=0.0,
        ),
        ControllerConfig(),
    )
    damped = command_for(
        observation(
            state="JUNCTION",
            lateral=0.05,
            heading=0.0,
        ),
        ControllerConfig(
            junction_lateral_deadband_m=0.025,
            junction_lateral_gain_scale=0.35,
        ),
    )
    assert damped.linear.x > 0.0
    assert 0.0 < abs(damped.angular.z) < abs(normal.angular.z)


def test_junction_heading_is_ignored_for_straight_through():
    command = command_for(
        observation(
            state="JUNCTION",
            lateral=0.0,
            heading=0.35,
        ),
        ControllerConfig(),
    )
    assert command.linear.x > 0.0
    assert command.angular.z == 0.0


def test_wide_track_heading_is_ignored_for_straight_through():
    command = command_for(
        observation(
            state=TRACK,
            lateral=0.0,
            heading=0.35,
            road_clearance=0.28,
        ),
        ControllerConfig(),
    )
    assert command.linear.x > 0.0
    assert command.angular.z == 0.0


def test_lateral_integral_corrects_persistent_bias():
    command = command_for(
        observation(
            state=TRACK,
            lateral=0.0,
            heading=0.0,
            road_clearance=0.18,
        ),
        ControllerConfig(),
        lateral_integral_m_s=0.04,
    )
    assert command.linear.x > 0.0
    assert command.angular.z > 0.0


def test_angular_rate_is_limited():
    params = ControllerConfig(
        max_angular_speed_rps=0.85,
        max_angular_acceleration_rps2=2.0,
    )
    limited = limit_angular_rate(0.85, 0.0, 0.1, params)
    assert limited == 0.2


def test_low_confidence_is_zero():
    params = ControllerConfig(min_confidence=0.5)
    command = command_for(
        observation(confidence=0.25),
        params,
    )
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_unsafe_narrow_support_stops_completely():
    command = command_for(
        observation(
            state=UNSAFE_EDGE,
            valid=False,
            confidence=0.10,
            lateral=0.20,
            road_left=0.08,
            road_right=0.10,
            road_clearance=0.02,
        ),
        ControllerConfig(),
    )
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_unsafe_complete_support_pivots_without_forward_motion():
    params = ControllerConfig(unsafe_recovery_max_angular_speed_rps=0.30)
    command = command_for(
        observation(
            state=UNSAFE_EDGE,
            valid=False,
            confidence=0.10,
            lateral=0.05,
            heading=0.02,
            road_left=-0.10,
            road_right=0.10,
            road_clearance=0.20,
        ),
        params,
    )
    assert command.linear.x == 0.0
    assert 0.0 < command.angular.z <= 0.30


def test_unsafe_missing_boundary_does_not_steer_from_sentinel():
    command = command_for(
        observation(
            state=UNSAFE_EDGE,
            valid=False,
            lateral=-0.12,
            road_left=0.0,
            road_right=0.20,
            road_clearance=0.20,
        ),
        ControllerConfig(),
    )
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_unsafe_recovery_rejects_excessive_lateral_error():
    command = command_for(
        observation(
            state=UNSAFE_EDGE,
            valid=False,
            lateral=0.31,
            road_left=-0.10,
            road_right=0.10,
            road_clearance=0.20,
        ),
        ControllerConfig(),
    )
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0
