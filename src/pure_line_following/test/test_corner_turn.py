from pure_line_following.corner_turn import (
    ARC,
    CHECK,
    CornerTurnConfig,
    CornerTurnStateMachine,
    MotionCommand,
)


def test_default_vehicle_dimensions_match_shared_chassis():
    params = CornerTurnConfig()

    assert params.vehicle_length_m == 0.132
    assert params.vehicle_width_m == 0.132
from pure_line_following.tracker import (
    JUNCTION,
    LEFT_BRANCH,
    LOST,
    RIGHT_BRANCH,
    TRACK,
    TrackObservation,
)


def observation(
    state=TRACK,
    valid=True,
    confidence=1.0,
    lateral=0.0,
    heading=0.0,
    road_clearance=0.20,
    branch_x=0.0,
    branch_left=0.0,
    branch_right=0.0,
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
        road_clearance_m=road_clearance,
        branch_x_m=branch_x,
        branch_left_extent_m=branch_left,
        branch_right_extent_m=branch_right,
    )


def branch(direction=LEFT_BRANCH, x=0.31):
    if direction == RIGHT_BRANCH:
        return observation(
            state=RIGHT_BRANCH,
            branch_x=x,
            branch_right=0.18,
        )
    return observation(
        state=direction,
        branch_x=x,
        branch_left=0.18,
        branch_right=0.18 if direction == JUNCTION else 0.0,
    )


def advance(machine, frame, t, command):
    return machine.update(
        frame,
        t,
        previous_linear_mps=command.linear_x_mps,
        previous_angular_rps=command.angular_z_rps,
    )


def test_far_corner_is_approach_only_and_does_not_steer_early():
    machine = CornerTurnStateMachine(
        CornerTurnConfig(stable_frames=1, min_branch_extent_m=0.10)
    )

    decision = machine.update(branch(LEFT_BRANCH, x=0.45), 0.0)

    assert machine.active
    assert not machine.turning
    assert decision.command is None
    assert decision.observation.lateral_source != "corner_branch_centerline"


def test_close_left_corner_stops_before_shunt_actions():
    machine = CornerTurnStateMachine(
        CornerTurnConfig(
            stable_frames=1,
            min_branch_extent_m=0.10,
            stop_s=0.20,
        )
    )

    decision = machine.update(branch(LEFT_BRANCH, x=0.31), 0.0)

    assert machine.active
    assert machine.turning
    assert decision.command == MotionCommand()
    assert decision.observation.lateral_source == "corner_branch_centerline"
    assert machine.status(0.0)["state"] == "stop"


def test_left_corner_shunt_uses_inner_wheel_non_reversing_arc():
    params = CornerTurnConfig(
        stable_frames=1,
        min_branch_extent_m=0.10,
        stop_s=0.0,
        check_s=0.0,
        angular_speed_rps=0.30,
        linear_speed_mps=0.005,
        wheel_track_m=0.1466,
    )
    machine = CornerTurnStateMachine(params)
    command = machine.update(branch(LEFT_BRANCH, x=0.31), 0.0).command
    decision = advance(machine, branch(LEFT_BRANCH, x=0.31), 0.1, command)

    assert decision.command is not None
    assert decision.command.angular_z_rps > 0.0
    assert decision.command.linear_x_mps >= (
        0.5 * params.wheel_track_m * abs(decision.command.angular_z_rps)
    )
    assert machine.status(0.1)["state"] == ARC


def test_right_corner_shunt_turns_right():
    machine = CornerTurnStateMachine(
        CornerTurnConfig(
            stable_frames=1,
            min_branch_extent_m=0.10,
            stop_s=0.0,
            check_s=0.0,
            angular_speed_rps=0.30,
        )
    )

    command = machine.update(branch(RIGHT_BRANCH, x=0.31), 0.0).command
    decision = advance(machine, branch(RIGHT_BRANCH, x=0.31), 0.1, command)

    assert decision.command is not None
    assert decision.command.linear_x_mps > 0.0
    assert decision.command.angular_z_rps < 0.0
    assert decision.observation.state == RIGHT_BRANCH


def test_junction_prefers_configured_direction_when_balanced():
    machine = CornerTurnStateMachine(
        CornerTurnConfig(
            stable_frames=1,
            min_branch_extent_m=0.10,
            junction_prefer_direction="right",
            stop_s=0.0,
            check_s=0.0,
        )
    )

    command = machine.update(branch(JUNCTION, x=0.31), 0.0).command
    decision = advance(machine, branch(JUNCTION, x=0.31), 0.1, command)

    assert decision.command is not None
    assert decision.command.angular_z_rps < 0.0
    assert decision.observation.state == RIGHT_BRANCH


def test_unsafe_predicted_footprint_holds_position():
    machine = CornerTurnStateMachine(
        CornerTurnConfig(
            stable_frames=1,
            min_branch_extent_m=0.10,
            stop_s=0.0,
            check_s=0.0,
            vehicle_width_m=0.195,
            road_width_m=0.20,
            road_margin_m=0.008,
        )
    )

    command = machine.update(branch(LEFT_BRANCH, x=0.31), 0.0).command
    decision = advance(machine, branch(LEFT_BRANCH, x=0.31), 0.1, command)

    assert decision.command == MotionCommand()
    assert machine.status(0.1)["state"] == CHECK
    assert machine.status(0.1)["block_reason"] == "all_predicted_paths_unsafe"


def test_active_turn_continues_through_temporary_lost_track():
    machine = CornerTurnStateMachine(
        CornerTurnConfig(
            stable_frames=1,
            min_branch_extent_m=0.10,
            stop_s=0.0,
            check_s=0.0,
        )
    )
    command = machine.update(branch(LEFT_BRANCH, x=0.31), 0.0).command
    decision = advance(machine, observation(state=LOST, valid=False), 0.1, command)

    assert machine.active
    assert decision.command is not None
    assert not decision.observation.valid


def test_turn_exits_after_yaw_and_centered_track_are_ready():
    machine = CornerTurnStateMachine(
        CornerTurnConfig(
            stable_frames=1,
            min_branch_extent_m=0.10,
            stop_s=0.0,
            check_s=0.0,
            min_turn_s=0.0,
            max_turn_s=8.0,
            exit_stable_frames=2,
            vehicle_width_m=0.12,
        )
    )
    command = machine.update(branch(LEFT_BRANCH, x=0.31), 0.0).command
    t = 0.0
    for _ in range(80):
        t += 0.1
        decision = advance(
            machine,
            observation(state=LOST, valid=False),
            t,
            command or MotionCommand(),
        )
        command = decision.command
        if machine.status(t)["turn_progress"] >= 0.90:
            break

    assert machine.active
    centered = observation(state=TRACK, lateral=0.0, heading=0.0)
    first = advance(machine, centered, t + 0.1, command or MotionCommand())
    second = advance(machine, centered, t + 0.2, first.command or MotionCommand())

    assert first.observation.valid
    assert second.command is None
    assert machine.done


def test_timeout_before_target_holds_instead_of_releasing_control():
    machine = CornerTurnStateMachine(
        CornerTurnConfig(
            stable_frames=1,
            min_branch_extent_m=0.10,
            stop_s=0.0,
            check_s=0.0,
            min_turn_s=0.0,
            exit_stable_frames=1,
        )
    )
    command = machine.update(branch(LEFT_BRANCH, x=0.31), 0.0).command
    decision = None
    for step in range(1, 90):
        decision = advance(
            machine,
            observation(state=LOST, valid=False),
            step * 0.1,
            command or MotionCommand(),
        )
        command = decision.command

    assert machine.active
    assert not machine.done
    assert decision is not None
    assert decision.command == MotionCommand()
    assert machine.status(9.0)["block_reason"] == "turn_timeout_before_target"


def test_once_mode_ignores_later_corners_after_completion():
    machine = CornerTurnStateMachine(
        CornerTurnConfig(
            stable_frames=1,
            min_branch_extent_m=0.10,
            stop_s=0.0,
            check_s=0.0,
            min_turn_s=0.0,
            target_yaw_rad=0.05,
            exit_yaw_tolerance_rad=0.05,
            exit_stable_frames=1,
            once=True,
        )
    )

    command = machine.update(branch(LEFT_BRANCH, x=0.31), 0.0).command
    machine.update(
        observation(state=TRACK, lateral=0.0, heading=0.0),
        0.3,
        previous_linear_mps=(command or MotionCommand()).linear_x_mps,
        previous_angular_rps=(command or MotionCommand()).angular_z_rps,
    )
    assert machine.done

    decision = machine.update(branch(LEFT_BRANCH, x=0.31), 0.4)

    assert decision.command is None
    assert not machine.active
