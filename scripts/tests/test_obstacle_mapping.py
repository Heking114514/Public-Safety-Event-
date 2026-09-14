import math
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from obstacle_mapping import (  # noqa: E402
    ObstacleMappingState,
    TimedBoolEventGate,
    clip_aabb,
    front_obstacle_aabb,
    merge_nearby_aabbs,
)


def test_projects_visible_surface_in_front_of_vehicle() -> None:
    bounds = front_obstacle_aabb(
        1.0,
        2.0,
        0.0,
        0.4,
        camera_forward_offset=0.1,
        obstacle_width=0.2,
        obstacle_depth=0.1,
    )
    assert bounds == pytest.approx((1.5, 1.9, 1.6, 2.1))


def test_opposite_observations_recover_same_obstacle() -> None:
    from_left = front_obstacle_aabb(1.0, 2.0, 0.0, 0.4, 0.1, 0.2, 0.1)
    from_right = front_obstacle_aabb(2.1, 2.0, math.pi, 0.4, 0.1, 0.2, 0.1)
    assert from_left == pytest.approx(from_right)


def test_nearby_observations_merge_but_distant_obstacles_do_not() -> None:
    obstacles = [(1.0, 1.0, 1.2, 1.2), (2.0, 2.0, 2.2, 2.2)]
    merged = merge_nearby_aabbs(obstacles, (1.18, 1.0, 1.35, 1.2), gap=0.02)
    assert merged == pytest.approx([(2.0, 2.0, 2.2, 2.2), (1.0, 1.0, 1.35, 1.2)])


@pytest.mark.parametrize("bad_range", [0.0, -1.0, math.nan, math.inf])
def test_rejects_invalid_ranges(bad_range: float) -> None:
    with pytest.raises(ValueError):
        front_obstacle_aabb(0.0, 0.0, 0.0, bad_range)


def test_stale_range_is_not_promoted_into_the_planning_map() -> None:
    state = ObstacleMappingState(3.2, 4.4, maximum_range_age=0.5)
    state.update_range(0.4, 1.0)
    state.update_blocked(True, 1.0)
    state.cancel_recovery()
    state.update_blocked(False, 1.1)
    state.update_blocked(True, 1.3)

    # The range is still younger than maximum_range_age, but belongs to the
    # preceding Bool event and must not be reused for this rising edge.
    assert not state.begin_recovery((1.0, 2.0, 0.0), 1.31)
    obstacles, promoted = state.promote([], (1.0, 2.0, 0.0), 1.32)

    assert obstacles == []
    assert not promoted


def test_range_just_before_blocked_edge_pairs_despite_callback_reordering() -> None:
    state = ObstacleMappingState(3.2, 4.4, pairing_reorder_tolerance=0.1)
    state.update_range(0.4, 0.96)
    state.update_blocked(True, 1.0)

    assert state.begin_recovery((1.0, 2.0, 0.0), 1.02)
    obstacles, promoted = state.promote([], (0.8, 2.0, 0.0), 1.05)

    assert promoted
    assert len(obstacles) == 1


def test_true_heartbeats_are_deduplicated_but_true_after_silence_is_new_event() -> None:
    gate = TimedBoolEventGate(event_timeout=0.5)

    assert gate.observe(True, 1.0)
    assert not gate.observe(True, 1.2)
    assert not gate.observe(True, 1.6)
    assert gate.observe(True, 2.11)


def test_true_after_silence_can_promote_a_second_obstacle_without_false_edge() -> None:
    gate = TimedBoolEventGate(event_timeout=0.5)
    state = ObstacleMappingState(3.2, 4.4)
    obstacles = []

    assert gate.observe(True, 1.0)
    state.update_range(0.4, 1.0)
    state.update_blocked(True, 1.0, force_new_event=True)
    state.begin_recovery((0.5, 1.0, 0.0), 1.01)
    obstacles, promoted = state.promote(obstacles, (0.3, 1.0, 0.0), 1.02)
    assert promoted

    assert gate.observe(True, 1.6)
    state.update_range(0.4, 1.6)
    state.update_blocked(True, 1.6, force_new_event=True)
    state.begin_recovery((2.0, 3.0, 0.0), 1.61)
    obstacles, promoted = state.promote(obstacles, (1.8, 3.0, 0.0), 1.62)

    assert promoted
    assert len(obstacles) == 2


def test_obstacle_is_only_promoted_after_recovery_reaches_replan_state() -> None:
    state = ObstacleMappingState(3.2, 4.4)
    state.update_range(0.4, 1.0)
    state.update_blocked(True, 1.0)

    assert state.begin_recovery((1.0, 2.0, 0.0), 1.1)
    assert state.pending is not None
    obstacles, promoted = state.promote([], (0.8, 2.0, 0.0), 1.2)

    assert promoted
    assert len(obstacles) == 1
    assert obstacles[0] == pytest.approx((1.270, 1.88, 1.390, 2.12))
    assert state.pending is None


def test_expected_turn_wall_discards_pending_obstacle() -> None:
    state = ObstacleMappingState(3.2, 4.4)
    state.update_range(0.4, 1.0)
    state.update_blocked(True, 1.0)
    state.begin_recovery((1.0, 2.0, 0.0), 1.0)

    state.cancel_recovery()
    obstacles, promoted = state.promote([], (1.0, 2.0, 0.0), 1.1)

    assert obstacles == []
    assert not promoted


def test_navigation_fault_preserves_valid_pending_obstacle_for_replanning() -> None:
    state = ObstacleMappingState(3.2, 4.4)
    state.update_range(0.4, 1.0)
    state.update_blocked(True, 1.0)
    assert state.begin_recovery((1.0, 2.0, 0.0), 1.01)

    obstacles, promoted = state.finalize_navigation_status(
        "FAULT_OBSTACLE_RECOVERY_NO_PROGRESS",
        [],
        (0.9, 2.0, 0.0),
        1.1,
    )

    assert promoted
    assert len(obstacles) == 1
    assert state.pending is None
    assert not state.recovery_active


def test_navigation_fault_without_pending_obstacle_only_cancels_recovery() -> None:
    state = ObstacleMappingState(3.2, 4.4)
    state.begin_recovery((1.0, 2.0, 0.0), 1.0)
    existing = [(2.0, 2.0, 2.2, 2.2)]

    obstacles, promoted = state.finalize_navigation_status(
        "FAULT_NO_PATH_PROGRESS", existing, (1.0, 2.0, 0.0), 1.1
    )

    assert not promoted
    assert obstacles == existing
    assert not state.recovery_active


def test_outside_obstacle_estimate_is_rejected_instead_of_inverted() -> None:
    assert clip_aabb((3.3, 1.0, 3.5, 1.2), 3.2, 4.4) is None
