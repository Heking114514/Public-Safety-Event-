#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from mission_execution import (  # noqa: E402
    AutoReplanBackoff,
    LatestRequestQueue,
    MissionExecutionState,
    PLANNER_REQUEST_TIMEOUT_SECONDS,
    PreviewSequenceState,
    RouteFailureCooldown,
    RouteExecution,
    valid_odometry_stamp,
)


def route(
    route_id=10,
    mode="layered",
    visits=("1", "TUNNEL_1"),
    roads=("ROAD_A",),
    deferred=("2", "ROAD_B"),
    complete=False,
    activation_requested_at=0.0,
):
    return RouteExecution(
        route_id=route_id,
        mode=mode,
        visit_order=tuple(visits),
        covered_edges=tuple(roads),
        deferred_targets=tuple(deferred),
        all_targets_reached=complete,
        activation_requested_at=activation_requested_at,
    )


class MissionExecutionStateTest(unittest.TestCase):
    def setUp(self):
        self.state = MissionExecutionState(
            ("1", "2"), ("TUNNEL_1",), ("ROAD_A", "ROAD_B")
        )

    def test_planner_predictions_are_not_committed_before_real_goal(self):
        self.state.begin_route(route())
        self.assertEqual(["1", "2"], self.state.remaining_tasks())
        self.assertEqual([], self.state.covered_roads())
        self.assertTrue(self.state.acknowledge(10, 2.0))
        self.state.observe_status("FOLLOWING", 2.1)
        self.assertEqual(["1", "2"], self.state.remaining_tasks())
        self.assertEqual([], self.state.covered_roads())

        completed = self.state.observe_status("GOAL_REACHED", 3.0)
        self.assertIsNotNone(completed)
        self.assertEqual(3, completed.new_progress_count)
        self.assertEqual(["2"], self.state.remaining_tasks())
        self.assertEqual(["ROAD_A"], self.state.covered_roads())
        self.assertFalse(self.state.mission_complete(completed))

    def test_latched_or_unarmed_goal_cannot_complete_new_route(self):
        self.state.begin_route(route())
        self.assertIsNone(self.state.observe_status("GOAL_REACHED", 1.0))
        self.assertTrue(self.state.acknowledge(10, 2.0))
        self.assertIsNone(self.state.observe_status("GOAL_REACHED", 2.1))
        self.assertEqual(["1", "2"], self.state.remaining_tasks())

        self.state.observe_status("WAITING_FOR_LOCALIZATION", 2.2)
        self.assertIsNotNone(self.state.observe_status("GOAL_REACHED", 3.0))

    def test_wrong_ack_and_fault_status_do_not_arm_completion(self):
        self.state.begin_route(route())
        self.assertFalse(self.state.acknowledge(99, 1.0))
        self.assertTrue(self.state.acknowledge(10, 2.0))
        self.state.observe_status("FAULT_NO_PATH_PROGRESS", 2.1)
        self.assertIsNone(self.state.observe_status("GOAL_REACHED", 2.2))

    def test_active_route_cannot_be_replaced(self):
        self.state.begin_route(route())
        with self.assertRaises(RuntimeError):
            self.state.begin_route(route(route_id=11))
        self.state.acknowledge(10, 1.0)
        with self.assertRaises(RuntimeError):
            self.state.begin_route(route(route_id=11))

    def test_ack_timeout_or_fault_discards_prediction_without_progress(self):
        self.state.begin_route(route())
        self.assertIsNotNone(self.state.discard_pending(10))
        self.assertFalse(self.state.acknowledge(10, 2.0))
        self.assertEqual(["1", "2"], self.state.remaining_tasks())

        self.state.begin_route(route(route_id=11))
        self.state.acknowledge(11, 3.0)
        self.assertIsNotNone(self.state.abort_active())
        self.assertEqual([], self.state.covered_roads())

    def test_route_lost_status_aborts_without_committing_predictions(self):
        for status in ("IDLE", "WAITING_FOR_ROUTE", "OBSTACLE_REPLAN_REQUIRED"):
            with self.subTest(status=status):
                self.state.reset()
                self.state.begin_route(route(activation_requested_at=10.0))
                self.assertTrue(self.state.acknowledge(10, 12.0))
                interrupted = self.state.abort_for_status(status, 12.1)
                self.assertIsNotNone(interrupted)
                self.assertIsNone(self.state.active)
                self.assertEqual(["1", "2"], self.state.remaining_tasks())
                self.assertEqual([], self.state.covered_roads())

    def test_status_between_activation_request_and_delayed_ack_is_replayed(self):
        self.state.begin_route(route(activation_requested_at=10.0))
        self.assertTrue(self.state.acknowledge(10, 12.0))

        # The status callback arrived before the ACK callback, but it belongs
        # to this request because it was received after activation was sent.
        self.state.observe_status("FOLLOWING", 11.0)
        completed = self.state.observe_status("GOAL_REACHED", 12.1)
        self.assertIsNotNone(completed)

    def test_fault_received_before_ack_callback_still_aborts_the_route(self):
        self.state.begin_route(route(activation_requested_at=10.0))
        self.assertTrue(self.state.acknowledge(10, 12.0))
        interrupted = self.state.abort_for_status(
            "FAULT_NO_PATH_PROGRESS", 11.5
        )
        self.assertIsNotNone(interrupted)
        self.assertIsNone(self.state.active)
        self.assertEqual(["1", "2"], self.state.remaining_tasks())

    def test_status_and_ack_from_before_activation_request_are_rejected(self):
        self.state.begin_route(route(activation_requested_at=10.0))
        self.assertFalse(self.state.acknowledge(10, 9.9))
        self.assertTrue(self.state.acknowledge(10, 12.0))
        self.state.observe_status("FOLLOWING", 9.9)
        self.assertIsNone(self.state.observe_status("GOAL_REACHED", 12.1))
        self.assertIsNone(self.state.abort_for_status("IDLE", 9.9))

    def test_cumulative_coverage_only_counts_new_progress(self):
        self.state.completed_roads.add("ROAD_A")
        execution = route(
            mode="layer3",
            visits=(),
            roads=("ROAD_A",),
            deferred=("ROAD_B",),
        )
        self.state.begin_route(execution)
        self.state.acknowledge(10, 1.0)
        self.state.observe_status("FOLLOWING", 1.1)
        completed = self.state.observe_status("GOAL_REACHED", 2.0)
        self.assertEqual(0, completed.new_progress_count)
        self.assertEqual(["ROAD_B"], self.state.remaining_roads())

    def test_probe_only_activates_new_work_or_required_final_return(self):
        repeated = route(
            mode="layer3", visits=(), roads=(), deferred=("ROAD_B",)
        )
        self.assertFalse(self.state.probe_may_activate(repeated, route_has_motion=True))
        useful = route(mode="layer3", visits=(), roads=("ROAD_A",), deferred=())
        self.assertTrue(self.state.probe_may_activate(useful, route_has_motion=True))
        self.assertFalse(self.state.probe_may_activate(useful, route_has_motion=False))

        self.state.completed_tasks.update(("1", "2"))
        self.state.completed_tunnels.add("TUNNEL_1")
        self.state.completed_roads.update(("ROAD_A", "ROAD_B"))
        return_home = route(
            mode="layer3", visits=(), roads=("ROAD_A", "ROAD_B"), deferred=(),
            complete=True,
        )
        self.assertTrue(self.state.probe_may_activate(return_home, route_has_motion=True))

    def test_partial_interval_is_useful_and_observed_roads_can_be_committed(self):
        partial = route(mode="layer3", visits=(), roads=(), deferred=("ROAD_A",))
        partial.predicted_interval_progress = True
        self.assertTrue(self.state.probe_may_activate(partial, route_has_motion=True))
        self.assertEqual(1, self.state.commit_observed_roads(["ROAD_A", "UNKNOWN"]))
        self.assertEqual(["ROAD_A"], self.state.covered_roads())

    def test_blocked_road_can_be_exempted_without_claiming_it_was_covered(self):
        self.assertEqual(1, self.state.commit_exempted_roads(["ROAD_A", "UNKNOWN"]))
        self.assertEqual([], self.state.covered_roads())
        self.assertEqual(["ROAD_B"], self.state.remaining_roads())

        self.assertEqual(1, self.state.commit_observed_roads(["ROAD_A"]))
        self.assertEqual(["ROAD_A"], self.state.covered_roads())
        self.assertEqual(set(), self.state.exempted_roads)

    def test_static_obstacle_exemptions_are_monotonic_during_a_mission(self):
        self.state.commit_exempted_roads(["ROAD_A"])

        self.state.commit_exempted_roads([])
        self.state.commit_exempted_roads(["ROAD_B"])

        self.assertEqual({"ROAD_A", "ROAD_B"}, self.state.exempted_roads)
        self.assertEqual([], self.state.covered_roads())
        self.assertEqual([], self.state.remaining_roads())

    def test_zero_motion_probe_finishes_after_blocked_road_is_accounted(self):
        self.state.completed_tasks.update(("1", "2"))
        self.state.completed_tunnels.add("TUNNEL_1")
        self.state.commit_exempted_roads(["ROAD_A", "ROAD_B"])
        probe = route(
            mode="layer3", visits=(), roads=(), deferred=(), complete=True
        )

        self.assertTrue(self.state.mission_complete(probe))
        self.assertIsNone(self.state.no_motion_continuation_mode(probe))

    def test_zero_motion_result_advances_to_the_next_unfinished_stage(self):
        self.state.completed_tasks.update(("1", "2"))
        empty_layer_one = route(
            mode="layer1", visits=(), roads=(), deferred=(), complete=True
        )

        self.assertEqual(
            "layer2", self.state.no_motion_continuation_mode(empty_layer_one)
        )

    def test_zero_motion_result_waits_when_current_stage_still_has_work(self):
        empty_layer_one = route(
            mode="layer1", visits=(), roads=(), deferred=("1",), complete=False
        )

        self.assertEqual(
            "layer1", self.state.no_motion_continuation_mode(empty_layer_one)
        )

    def test_auto_wait_backoff_keeps_retrying_and_is_bounded(self):
        backoff = AutoReplanBackoff(2.0, 5.0)
        self.assertEqual(
            [2.0, 3.0, 4.5, 5.0, 5.0],
            [backoff.next_delay() for _ in range(5)],
        )
        backoff.reset()
        self.assertEqual(2.0, backoff.next_delay())

    def test_only_route_faults_cool_down_and_the_same_route_is_retried(self):
        cooldown = RouteFailureCooldown(5.0)
        signature = ((1, 2), (3, 4))
        self.assertFalse(
            cooldown.record("FAULT_FUSION_STATUS", "layer3", signature, 10.0)
        )
        self.assertFalse(cooldown.blocked("layer3", signature, 10.1))

        self.assertTrue(
            cooldown.record("FAULT_NO_PATH_PROGRESS", "layer3", signature, 20.0)
        )
        self.assertTrue(cooldown.blocked("layer3", signature, 24.9))
        self.assertFalse(cooldown.blocked("layer3", signature, 25.0))

    def test_odometry_requires_fresh_monotonic_map_to_base_link_sample(self):
        pose = (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        now = 10_000_000_000
        accepted = valid_odometry_stamp(
            "map", "base_link", pose, 9, 800_000_000, now, 0
        )
        self.assertEqual(9_800_000_000, accepted)
        for arguments in (
            ("", "base_link", pose, 9, 900_000_000, now, accepted),
            ("map", "", pose, 9, 900_000_000, now, accepted),
            ("map", "base_link", pose, 9, 700_000_000, now, accepted),
            ("map", "base_link", pose, 8, 0, now, accepted),
            ("map", "base_link", pose, -1, 0, now, accepted),
            ("map", "base_link", pose, 9, 1_000_000_000, now, accepted),
            (
                "map",
                "base_link",
                (*pose[:2], float("nan"), *pose[3:]),
                9,
                900_000_000,
                now,
                accepted,
            ),
            ("map", "base_link", (*pose[:3], 0.0, 0.0, 0.0, 0.0), 9, 900_000_000, now, accepted),
        ):
            with self.subTest(arguments=arguments):
                self.assertIsNone(valid_odometry_stamp(*arguments))

    def test_open_stage_cannot_claim_whole_mission_before_return_stage(self):
        self.state.completed_tunnels.add("TUNNEL_1")
        self.state.completed_roads.update(("ROAD_A", "ROAD_B"))
        execution = route(
            mode="layer1",
            visits=("1", "2"),
            roads=("ROAD_A", "ROAD_B"),
            deferred=(),
            complete=True,
        )
        self.state.begin_route(execution)
        self.state.acknowledge(10, 1.0)
        self.state.observe_status("FOLLOWING", 1.1)
        completed = self.state.observe_status("GOAL_REACHED", 2.0)
        self.assertFalse(self.state.mission_complete(completed))
        self.assertEqual("layer3", self.state.continuation_mode(completed))

    def test_final_stage_requires_all_real_work_and_no_deferred_items(self):
        self.state.completed_tasks.update(("1", "2"))
        self.state.completed_tunnels.add("TUNNEL_1")
        self.state.completed_roads.update(("ROAD_A", "ROAD_B"))
        execution = route(
            mode="layer3", visits=(), roads=("ROAD_A", "ROAD_B"), deferred=(),
            complete=True,
        )
        self.state.begin_route(execution)
        self.state.acknowledge(10, 1.0)
        self.state.observe_status("FOLLOWING", 1.1)
        completed = self.state.observe_status("GOAL_REACHED", 2.0)
        self.assertTrue(self.state.mission_complete(completed))


class LatestRequestQueueTest(unittest.TestCase):
    def test_completion_or_timeout_wins_once_and_dispatches_only_latest(self):
        requests = LatestRequestQueue()
        first = requests.submit("first")
        self.assertIsNotNone(first)
        first_token, _first_item = first
        self.assertIsNone(requests.submit("stale"))
        self.assertIsNone(requests.submit("latest"))

        accepted, dispatch = requests.complete(first_token + 100)
        self.assertFalse(accepted)
        self.assertIsNone(dispatch)

        accepted, dispatch = requests.complete(first_token)
        self.assertTrue(accepted)
        self.assertIsNotNone(dispatch)
        next_token, next_item = dispatch
        self.assertEqual("latest", next_item)
        self.assertTrue(requests.in_flight)

        self.assertEqual((False, None), requests.complete(first_token))
        self.assertEqual((True, None), requests.complete(next_token))
        self.assertFalse(requests.in_flight)

    def test_cancel_discards_active_and_pending_requests(self):
        requests = LatestRequestQueue()
        requests.submit("active")
        requests.submit("pending")
        requests.cancel()
        self.assertFalse(requests.in_flight)
        dispatch = requests.submit("after-cancel")
        self.assertIsNotNone(dispatch)
        self.assertEqual("after-cancel", dispatch[1])

    def test_planner_timeout_budget_is_long_enough_for_normal_planning(self):
        self.assertEqual(30.0, PLANNER_REQUEST_TIMEOUT_SECONDS)


class PreviewSequenceStateTest(unittest.TestCase):
    def test_one_start_chains_all_three_preview_stages(self):
        preview = PreviewSequenceState()

        preview.start(1)
        self.assertEqual(2, preview.finish_stage(1))
        self.assertTrue(preview.waiting_for_plan)
        self.assertTrue(preview.plan_ready(2))
        self.assertEqual(3, preview.finish_stage(2))
        self.assertTrue(preview.plan_ready(3))
        self.assertIsNone(preview.finish_stage(3))

        self.assertTrue(preview.complete)
        self.assertFalse(preview.active)

    def test_pausing_while_next_stage_plans_prevents_autoplay(self):
        preview = PreviewSequenceState()
        preview.start(1)
        self.assertEqual(2, preview.finish_stage(1))

        preview.pause()

        self.assertFalse(preview.plan_ready(2))
        self.assertFalse(preview.complete)

    def test_preview_state_does_not_commit_real_mission_progress(self):
        mission = MissionExecutionState(["1"], ["TUNNEL_1"], ["ROAD_A"])
        preview = PreviewSequenceState()

        preview.start(1)
        preview.finish_stage(1)

        self.assertEqual(["1"], mission.remaining_tasks())
        self.assertEqual(["TUNNEL_1"], mission.remaining_tunnels())
        self.assertEqual(["ROAD_A"], mission.remaining_roads())


if __name__ == "__main__":
    unittest.main()
