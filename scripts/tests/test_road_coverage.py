import math
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from road_coverage import RoadCoverageLedger  # noqa: E402


NODES = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
EDGES = [(0, 1, "AB", False), (1, 2, "BC", False)]


def test_partial_motion_from_each_end_merges_into_complete_road() -> None:
    ledger = RoadCoverageLedger(NODES, EDGES, lateral_tolerance=0.02)
    ledger.observe_motion((0.0, 0.01), (0.45, 0.01))
    remaining = ledger.remaining_intervals("AB")
    assert len(remaining) == 1
    assert remaining[0] == pytest.approx((0.47, 1.0))

    ledger.observe_motion((1.0, -0.01), (0.44, -0.01))
    assert ledger.remaining_intervals("AB") == []
    assert ledger.covered_edges() == ["AB"]


def test_perpendicular_crossing_does_not_mark_road_covered() -> None:
    ledger = RoadCoverageLedger(NODES, EDGES, lateral_tolerance=0.05)
    ledger.observe_motion((0.5, -0.1), (0.5, 0.1))
    assert ledger.intervals() == []


def test_reverse_motion_counts_the_same_as_forward_motion() -> None:
    forward = RoadCoverageLedger(NODES, EDGES, lateral_tolerance=0.01)
    reverse = RoadCoverageLedger(NODES, EDGES, lateral_tolerance=0.01)
    forward.observe_motion((0.2, 0.0), (0.7, 0.0))
    reverse.observe_motion((0.7, 0.0), (0.2, 0.0))
    assert forward.intervals() == reverse.intervals()


def test_nonfinite_or_tiny_motion_is_ignored() -> None:
    ledger = RoadCoverageLedger(NODES, EDGES)
    ledger.observe_motion((math.nan, 0.0), (1.0, 0.0))
    ledger.observe_motion((0.1, 0.0), (0.101, 0.0))
    assert ledger.intervals() == []


def test_large_pose_jump_is_not_reported_as_real_road_coverage() -> None:
    ledger = RoadCoverageLedger(
        NODES, EDGES, lateral_tolerance=0.01, maximum_motion=0.25
    )

    ledger.observe_motion((0.0, 0.0), (0.8, 0.0))

    assert ledger.intervals() == []


def test_reports_only_new_predicted_interval_length() -> None:
    ledger = RoadCoverageLedger(NODES, EDGES, lateral_tolerance=0.01)
    ledger.add_interval("AB", 0.0, 0.4)
    assert ledger.covered_length() == pytest.approx(0.4)
    assert ledger.additional_length([("AB", 0.2, 0.7)]) == pytest.approx(0.3)


def test_blocked_gap_only_completes_accounting_after_both_sides_are_driven() -> None:
    ledger = RoadCoverageLedger(NODES, EDGES, lateral_tolerance=0.001)
    blocked = [("AB", 0.45, 0.55)]

    ledger.add_interval("AB", 0.0, 0.45)
    assert ledger.covered_edges() == []
    assert ledger.accounted_edges(blocked) == []

    ledger.add_interval("AB", 0.55, 1.0)
    assert ledger.covered_edges() == []
    assert ledger.accounted_edges(blocked) == ["AB"]
