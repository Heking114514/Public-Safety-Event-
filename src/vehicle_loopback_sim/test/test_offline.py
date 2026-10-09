import json
import math

import pytest

from vehicle_loopback_sim.motion import Twist2D
from vehicle_loopback_sim.offline import (
    OfflineTruthRunner,
    PlaybackConfig,
    TruthOracle,
    TruthState,
)


def test_closed_loop_rejects_accelerated_playback():
    with pytest.raises(ValueError, match="closed_loop requires playback_speed=1.0"):
        PlaybackConfig("closed_loop", 3.0).validate()


def test_offline_mode_marks_acceleration_non_acceptance():
    playback = PlaybackConfig("offline", 3.0)
    playback.validate()
    assert not playback.acceptance


def test_truth_oracle_parses_status_and_checks_time_order():
    status = json.dumps(
        {
            "pose": {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.25},
            "twist": {"linear_x_mps": 0.1, "angular_z_rps": -0.2},
            "fusion_status": "FULL",
        }
    )
    oracle = TruthOracle()
    sample = oracle.observe(0.1, status)
    assert sample.state.pose.x == 1.0
    assert sample.state.twist.angular_z == -0.2
    with pytest.raises(ValueError, match="monotonic"):
        oracle.observe(0.09, status)


def test_truth_oracle_rejects_missing_fields():
    with pytest.raises(ValueError, match="pose.x_m"):
        TruthState.from_status_json(
            '{"pose": {}, "twist": {}, "fusion_status": "FULL"}'
        )


def test_offline_playback_scales_truth_time_and_distance():
    command = Twist2D(0.1, 0.0)
    normal = OfflineTruthRunner(PlaybackConfig("offline", 1.0))
    accelerated = OfflineTruthRunner(PlaybackConfig("offline", 2.0))
    for _ in range(10):
        normal.step(command, 0.1)
        accelerated.step(command, 0.1)
    assert math.isclose(normal.sim_time_s, 1.0)
    assert math.isclose(accelerated.sim_time_s, 2.0)
    assert math.isclose(accelerated.pose.x, 2.0 * normal.pose.x, rel_tol=1.0e-12)
