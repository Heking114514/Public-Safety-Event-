#!/usr/bin/env python3
"""验收脚本的证据回归：防止名称、缓存零速或错侧道路被写成通过。"""
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from recovery_evidence import RoadMap, measured_direction_stop_error, fixed_entry_stop_error
from run_pose_recovery_scenarios import Evidence, scenarios


class EvidenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.roads = RoadMap(PACKAGE.parent / "new_map_planner/config/arena.yaml")

    def test_recovery_phase_name_cannot_prove_internal_fixed_turn(self):
        evidence = Evidence(scenarios()[0], self.roads)
        evidence.observe_status("POSE_RECOVERY_TURN_INTO_EXIT")
        self.assertFalse(evidence.turn_seen)
        self.assertEqual(evidence.turn_phase_history, [])
        evidence.observe_status("POSE_RECOVERY_TURN_60")
        evidence.observe_status("POSE_RECOVERY_TURN_30")
        self.assertTrue(evidence.turn_seen)
        self.assertEqual(len(evidence.turn_phase_history), 2)

    def test_independent_projection_does_not_call_longitudinal_error_lateral(self):
        entry = self.roads.turn_entry((1.2, -.53), -1.5707963267948966,
                                      "x1_y3", -.070, 0.0)
        self.assertAlmostEqual(entry["road_center_error"], 0.0)
        self.assertAlmostEqual(entry["position_error"], .10)

    def test_lateral_entry_target_uses_left_or_right_turn_sign(self):
        left = self.roads.turn_entry((1.185, -.47), -1.5707963267948966,
            "x1_y3", -.030, -.015, outgoing_heading=0.0)
        right = self.roads.turn_entry((1.215, -.47), -1.5707963267948966,
            "x1_y3", -.030, -.015, outgoing_heading=3.141592653589793)
        self.assertAlmostEqual(left["position_error"], 0.0)
        self.assertAlmostEqual(right["position_error"], 0.0)
        wrong = self.roads.turn_entry((1.185, -.47), -1.5707963267948966,
            "x1_y3", -.030, -.015, outgoing_heading=3.141592653589793)
        self.assertAlmostEqual(wrong["position_error"], .030)

    def test_exit_requires_target_lane_outward_side_and_heading(self):
        candidate = dict(resume_node="x1_y3", resume_lane="ROAD_X1_Y3")
        self.assertEqual(self.roads.exit_error((.9, -.5), 3.141592653589793, candidate), "")
        self.assertTrue(self.roads.exit_error((1.5, -.5), 3.141592653589793, candidate))
        self.assertTrue(self.roads.exit_error((.9, -.4), 3.141592653589793, candidate))
        self.assertTrue(self.roads.exit_error((.9, -.5), 0.0, candidate))
        self.assertTrue(self.roads.exit_error((-.1, -.5), 3.141592653589793, candidate))

    @staticmethod
    def row(stamp, speed, angular=0.0, status="POSE_RECOVERY_DIRECTION_STOP"):
        return dict(stamp=stamp,linear=speed,angular=angular,status=status)

    def test_duplicate_and_stale_stop_samples_do_not_prove_direction_stop(self):
        for stop in ([1.1,1.1,1.1], [1.1,1.5,1.9], [1.1,1.12]):
            rows = [self.row(1.0,-.03)]
            rows += [self.row(stamp,0) for stamp in stop]
            rows += [self.row(2.0,.03)]
            self.assertTrue(measured_direction_stop_error(rows))

    def test_contiguous_measured_stop_allows_direction_change(self):
        rows = [self.row(1.0,-.03), self.row(1.1,0), self.row(1.16,0),
                self.row(1.22,0), self.row(1.23,.03)]
        self.assertEqual(measured_direction_stop_error(rows), "")

    def test_fixed_entry_rejects_stopping_at_another_point_and_rotating_in_place(self):
        entry = dict(t=2.0, target_x=1.2, target_y=-.47)
        rows = [dict(t=t, stamp=t, x=1.2, y=-.47, yaw=-1.5707963267948966,
                     linear=0.0, angular=0.0, status="POSE_RECOVERY_DIRECTION_STOP")
                for t in (1.60, 1.72, 1.84, 1.96)]
        args = (entry, -1.5707963267948966, .003, .010, .080, .22)
        self.assertEqual(fixed_entry_stop_error(rows, *args), "")
        stale = [dict(row, t=row["t"]-.30, stamp=row["stamp"]-.30) for row in rows]
        self.assertTrue(fixed_entry_stop_error(stale, *args))
        for row in rows:
            row["y"] = -.44
        self.assertTrue(fixed_entry_stop_error(rows, *args))
        for row in rows:
            row["y"], row["angular"] = -.47, .65
        self.assertTrue(fixed_entry_stop_error(rows, *args))

    def test_second_direction_change_is_checked_after_first_turn_entry(self):
        rows = [self.row(1.0,-.03), self.row(1.1,0), self.row(1.16,0),
                self.row(1.22,0), self.row(1.23,.03,"POSE_RECOVERY_TURN_60"),
                self.row(1.3,-.03,"POSE_RECOVERY_SECOND_TURN_POINT")]
        self.assertTrue(measured_direction_stop_error(rows))

    def test_scenario_names_distinguish_interrupted_and_completed_normal_turn(self):
        cases = {case["name"]: case for case in scenarios()}
        self.assertTrue(cases["after_normal_90deg_turn"]["after_turn"])
        self.assertTrue(cases["during_normal_90deg_turn"]["during_turn"])
        self.assertEqual(cases["t_u"]["expected_turns"], 2)
        self.assertEqual(cases["x_reverse"]["expected_turns"], 2)
        self.assertEqual(cases["long_retreat"]["minimum_reverse_m"], .85)
        self.assertEqual(self.roads.map_id, "official_arena_v1")


if __name__ == "__main__":
    unittest.main()
