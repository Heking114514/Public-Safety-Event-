"""Focused ROS-independent checks for the A/B truth checkpoint helper."""

import ast
from dataclasses import dataclass, field
import math
from pathlib import Path
import re
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "comparison_mission.py"


class FakeFuture:
    def __init__(self, response=None):
        self.response = response
        self.callbacks = []

    def result(self):
        return self.response

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def complete(self, response):
        self.response = response
        for callback in self.callbacks:
            callback(self)


class FakeClient:
    def __init__(self):
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        future = FakeFuture()
        self.futures.append(future)
        return future


class FakeMissionBase:
    def _odom_callback(self, message):
        self._latest_odom = message

    def _navigator_status_callback(self, source, message):
        self.goals.append((source, message.data))

    def _handle_response(self, future):
        self._planning = False
        self._waiting_for_goal = bool(future.result().success)


@pytest.fixture
def helpers(monkeypatch):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    names = {
        "parse_checkpoint_status", "stamp_ns", "CheckpointPlan",
        "motion_is_stationary", "StationaryHold", "ControllerCheckpoint",
        "ComparisonMissionStarter", "HOLD_SOURCE", "CONTROLLER_NODES",
    }
    selected = [
        node for node in tree.body
        if (isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names)
        or (isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id in names
                    for target in node.targets))
    ]
    module = ModuleType("comparison_checkpoint_helpers")
    module.__dict__.update({
        "math": math, "re": re, "dataclass": dataclass, "field": field,
        "LoopbackMissionStarter": FakeMissionBase, "String": NS,
        "SetMotionHold": NS(Request=NS), "ConfirmRfidCheckpoint": NS(Request=NS),
    })
    monkeypatch.setitem(sys.modules, module.__name__, module)
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), "exec"),
         module.__dict__)
    return module


def make_plan(sec=2, ids=("", "s1"), points=((0.0, 0.0), (1.0, 2.0))):
    return NS(
        plan_id=7,
        navigation_path=NS(
            header=NS(stamp=NS(sec=sec, nanosec=123), frame_id="map"),
            poses=[NS(pose=NS(position=NS(x=x, y=y, z=0.004))) for x, y in points],
        ),
        waypoint_supernode_ids=list(ids),
    )


def make_twist(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
    return NS(linear=NS(x=linear[0], y=linear[1], z=linear[2]),
              angular=NS(x=angular[0], y=angular[1], z=angular[2]))


def make_rig(helpers, monkeypatch, name="production"):
    clock = [10.0]
    monkeypatch.setattr(helpers, "monotonic", lambda: clock[0], raising=False)
    starter = object.__new__(helpers.ComparisonMissionStarter)
    starter._checkpoint_config = {
        "checkpoint_hold_s": 1.0, "checkpoint_radius_m": 0.05,
        "checkpoint_linear_epsilon": 0.005, "checkpoint_angular_epsilon": 0.02,
        "checkpoint_pose_timeout_s": 0.40,
    }
    starter._checkpoint_plan = helpers.CheckpointPlan(make_plan())
    starter._planning = starter._stopping = starter._external_manager = False
    starter._waiting_for_goal = True
    starter._planning_controller = "production"
    starter.goals = []
    starter.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round(clock[0] * 1e9)))
    starter.get_logger = lambda: NS(info=lambda text: None,
                                   warning=lambda text: None, error=lambda text: None)
    starter._publish_status = lambda text: None
    side = helpers.ControllerCheckpoint(
        name, f"/comparison/{name}/navigator_status", FakeClient(), FakeClient())
    side.ack_route_id = side.status_route_id = starter._checkpoint_plan.route_id
    side.status = "RFID_SCANNING:1"
    side.saw_activity = True
    starter._controllers = {side.status_topic: side}
    return NS(node=starter, side=side, clock=clock, helpers=helpers)


def truth_sample(rig, x=1.0, y=2.0, linear=0.0, angular=0.0, frame="map"):
    timestamp = round(rig.clock[0] * 1e9)
    message = NS(
        header=NS(stamp=NS(sec=timestamp // 1_000_000_000,
                           nanosec=timestamp % 1_000_000_000), frame_id=frame),
        pose=NS(pose=NS(position=NS(x=x, y=y))),
        twist=NS(twist=make_twist((linear, 0.0, 0.0), (0.0, 0.0, angular))),
    )
    rig.node._truth_callback(rig.side, message)
    return message


def hold_response(held=True, sources=("contrast_rfid",)):
    return NS(success=True, motion_held=held, active_sources=list(sources), message="ok")


@pytest.mark.parametrize("text,index", [
    ("RFID_SCANNING:0", 0), ("WAITING_FOR_RFID:12", 12),
    ("RFID_SCANNING:001", 1),
])
def test_checkpoint_status_parsing(helpers, text, index):
    assert helpers.parse_checkpoint_status(text) == index


@pytest.mark.parametrize("text", [
    "GOAL_REACHED", "FOLLOWING", "HELD_FOR_MISSION", "RFID_SCANNING:-1",
    "WAITING_FOR_RFID:1:old", "RFID_SCANNING:1 ", "RFID_SCANNING:",
])
def test_non_checkpoint_states_do_not_imply_arrival(helpers, text):
    assert helpers.parse_checkpoint_status(text) is None


def test_plan_preserves_path_and_uses_stamp_not_plan_id(helpers):
    original = make_plan()
    plan = helpers.CheckpointPlan(original)
    assert plan.plan_id == 7
    assert plan.path_stamp_ns == plan.route_id == 2_000_000_123
    assert plan.navigation_path is original.navigation_path
    assert plan.waypoint_supernode_ids == ("", "s1")
    assert plan.checkpoint(1) == (2_000_000_123, 1, "s1")
    assert [pose.pose.position.z for pose in original.navigation_path.poses] == [0.004] * 2
    assert all(plan.checkpoint(index) is None for index in (None, -1, 0, 2))


def test_plan_rejects_missing_metadata_and_invalid_stamp(helpers):
    with pytest.raises(ValueError, match="metadata"):
        helpers.CheckpointPlan(make_plan(ids=("s1",)))
    original = make_plan()
    original.navigation_path.header.stamp = NS(sec=0, nanosec=0)
    with pytest.raises(ValueError, match="stamped"):
        helpers.CheckpointPlan(original)
    with pytest.raises(ValueError, match="timestamp"):
        helpers.stamp_ns(NS(sec=-1, nanosec=0))
    with pytest.raises(ValueError, match="timestamp"):
        helpers.stamp_ns(NS(sec=1, nanosec=1_000_000_000))


@pytest.mark.parametrize("twist", [
    make_twist(linear=(0.0, 0.01, 0.0)),
    make_twist(angular=(0.0, 0.03, 0.0)),
    make_twist(linear=(float("nan"), 0.0, 0.0)),
])
def test_stop_requires_both_full_linear_and_angular_vectors(helpers, twist):
    assert not helpers.motion_is_stationary(twist, 0.005, 0.02)
    assert helpers.motion_is_stationary(make_twist(), 0.005, 0.02)


def test_continuous_stop_requires_fresh_samples_for_full_hold(helpers):
    dwell = helpers.StationaryHold()
    dwell.observe(10.0, True, 0.4)
    assert not dwell.ready(11.0, 1.0, 0.4)
    for now in (10.25, 10.5, 10.75):
        dwell.observe(now, True, 0.4)
        assert not dwell.ready(now, 1.0, 0.4)
    dwell.observe(11.0, True, 0.4)
    assert dwell.ready(11.0, 1.0, 0.4)
    assert not dwell.ready(11.5, 1.0, 0.4)


@pytest.mark.parametrize("interrupt", ["motion", "gap", "duplicate", "clock_backwards"])
def test_stop_hold_resets_on_motion_or_invalid_sample_sequence(helpers, interrupt):
    dwell = helpers.StationaryHold()
    for now in (10.0, 10.25, 10.5):
        dwell.observe(now, True, 0.4)
    timestamp = {"motion": 10.75, "gap": 11.0,
                 "duplicate": 10.5, "clock_backwards": 10.4}[interrupt]
    dwell.observe(timestamp, interrupt != "motion", 0.4)
    assert not dwell.ready(timestamp, 1.0, 0.4)
    dwell.observe(timestamp + 0.25, True, 0.4)
    assert not dwell.ready(timestamp + 0.25, 1.0, 0.4)


@pytest.mark.parametrize("name", ["production", "contrast"])
def test_both_controllers_hold_confirm_release_and_deduplicate(helpers, monkeypatch, name):
    rig = make_rig(helpers, monkeypatch, name)
    truth_sample(rig, linear=0.05)
    rig.node._checkpoint_tick()
    assert len(rig.side.hold_client.requests) == 1
    acquire = rig.side.hold_client.requests[0]
    assert acquire.source == "contrast_rfid" and acquire.hold
    rig.node._checkpoint_tick()
    assert len(rig.side.hold_client.requests) == 1
    rig.side.hold_client.futures[-1].complete(hold_response())
    rig.node._navigator_status_callback(rig.side.status_topic, NS(data="HELD_FOR_MISSION"))
    for now in (10.25, 10.5, 10.75, 11.0):
        rig.clock[0] = now
        truth_sample(rig)
        rig.node._checkpoint_tick()
        assert not rig.side.confirm_client.requests
    rig.clock[0] = 11.25
    truth_sample(rig)
    rig.node._checkpoint_tick()
    confirm = rig.side.confirm_client.requests[0]
    assert (confirm.route_id, confirm.waypoint_index, confirm.supernode_id) == (
        rig.node._checkpoint_plan.route_id, 1, "s1")
    rig.node._checkpoint_tick()
    assert len(rig.side.confirm_client.requests) == 1
    rig.side.confirm_client.futures[-1].complete(NS(accepted=True, message="ok"))
    assert rig.side.key in rig.side.completed
    rig.node._checkpoint_tick()
    assert not rig.side.hold_client.requests[-1].hold
    rig.side.hold_client.futures[-1].complete(hold_response(sources=("other",)))
    assert rig.side.key is None
    rig.node._navigator_status_callback(rig.side.status_topic, NS(data="RFID_SCANNING:1"))
    assert len(rig.side.hold_client.requests) == 2


@pytest.mark.parametrize("constraint", ["far", "frame", "stale", "ack", "metadata"])
def test_scanning_status_alone_never_acquires_hold(helpers, monkeypatch, constraint):
    rig = make_rig(helpers, monkeypatch)
    truth_sample(rig, x=2.0 if constraint == "far" else 1.0,
                 frame="arena" if constraint == "frame" else "map")
    if constraint == "stale":
        rig.clock[0] += 1.0
    elif constraint == "ack":
        rig.side.ack_route_id -= 1
    elif constraint == "metadata":
        rig.side.status = "RFID_SCANNING:0"
    rig.node._checkpoint_tick()
    assert not rig.side.hold_client.requests


def test_route_replacement_waits_for_old_request_then_releases(helpers, monkeypatch):
    rig = make_rig(helpers, monkeypatch)
    truth_sample(rig)
    rig.node._checkpoint_tick()
    old_future = rig.side.pending
    old_key = rig.side.key
    rig.node._handle_response(FakeFuture(NS(success=True, plan=make_plan(sec=3))))
    rig.node._ack_callback(rig.side, NS(data=rig.node._checkpoint_plan.route_id))
    rig.node._checkpoint_tick()
    assert rig.side.releasing and rig.side.pending is old_future
    old_future.complete(hold_response())
    rig.node._checkpoint_tick()
    assert rig.side.key == old_key
    assert not rig.side.hold_client.requests[-1].hold
    assert not rig.side.confirm_client.requests
    rig.side.hold_client.futures[-1].complete(hold_response(held=False, sources=()))
    assert rig.side.key is None and not rig.side.completed


def test_old_ack_duplicate_truth_and_stale_response_cannot_change_context(helpers, monkeypatch):
    rig = make_rig(helpers, monkeypatch)
    message = truth_sample(rig)
    rig.node._ack_callback(rig.side, NS(data=rig.side.ack_route_id - 1))
    assert rig.side.status == "RFID_SCANNING:1"
    rig.node._checkpoint_tick()
    pending, key = rig.side.pending, rig.side.key
    rig.node._service_response(rig.side, key, "hold", FakeFuture(hold_response()))
    assert rig.side.pending is pending and not rig.side.held
    received = rig.side.truth_received_s
    rig.clock[0] += 0.25
    rig.node._truth_callback(rig.side, message)
    assert rig.side.truth_received_s == received
    plan = rig.node._checkpoint_plan
    rig.node._handle_response(FakeFuture(NS(success=True, plan=make_plan(sec=1))))
    assert rig.node._checkpoint_plan is plan


def test_goal_status_requires_current_truth_near_route_end_and_stationary(helpers, monkeypatch):
    rig = make_rig(helpers, monkeypatch)
    rig.side.status = "GOAL_REACHED"
    truth_sample(rig, x=2.0)
    rig.node._checkpoint_tick()
    assert not rig.node.goals
    rig.clock[0] += 0.25
    truth_sample(rig, angular=0.1)
    rig.node._checkpoint_tick()
    assert not rig.node.goals
    rig.clock[0] += 0.25
    truth_sample(rig)
    rig.side.saw_activity = False
    rig.node._checkpoint_tick()
    assert not rig.node.goals
    rig.side.saw_activity = True
    rig.node._checkpoint_tick()
    assert rig.node.goals == [(rig.side.status_topic, "GOAL_REACHED")]


def test_rejected_confirmation_keeps_hold_and_never_marks_completed(helpers, monkeypatch):
    rig = make_rig(helpers, monkeypatch)
    rig.side.key = rig.node._checkpoint_plan.checkpoint(1)
    rig.side.held = True
    rig.node._call_service(rig.side, "confirm")
    rig.side.confirm_client.futures[-1].complete(NS(accepted=False, message="wrong route"))
    assert rig.side.held and not rig.side.releasing and not rig.side.completed
    rig.node._checkpoint_tick()
    assert not rig.side.hold_client.requests


def test_stale_accepted_confirmation_does_not_complete_new_route(helpers, monkeypatch):
    rig = make_rig(helpers, monkeypatch)
    rig.side.key = rig.node._checkpoint_plan.checkpoint(1)
    rig.side.held = True
    rig.node._call_service(rig.side, "confirm")
    old_future = rig.side.pending
    rig.node._handle_response(FakeFuture(NS(success=True, plan=make_plan(sec=3))))
    rig.node._ack_callback(rig.side, NS(data=rig.node._checkpoint_plan.route_id))
    rig.node._checkpoint_tick()
    old_future.complete(NS(accepted=True, message="old route"))
    assert not rig.side.completed and rig.side.releasing
    rig.node._checkpoint_tick()
    assert not rig.side.hold_client.requests[-1].hold
    release_future = rig.side.pending
    rig.node._service_response(rig.side, rig.side.key, "confirm", old_future)
    assert rig.side.pending is release_future


def test_contrast_only_planning_uses_contrast_truth(helpers, monkeypatch):
    rig = make_rig(helpers, monkeypatch, name="contrast")
    rig.node._planning_controller = "contrast"
    message = truth_sample(rig)
    assert rig.node._latest_odom is message


def test_external_manager_does_not_start_next_route(helpers, monkeypatch):
    rig = make_rig(helpers, monkeypatch)
    rig.node._external_manager = True
    truth_sample(rig)
    rig.node._navigator_status_callback(rig.side.status_topic, NS(data="GOAL_REACHED"))
    assert rig.node.goals == []


def test_external_route_requires_matching_ack_and_ignores_stale_plan(helpers, monkeypatch):
    rig = make_rig(helpers, monkeypatch)
    rig.node._external_manager = True
    newer = make_plan(sec=3)
    rig.node._external_plan_callback(newer)
    truth_sample(rig)
    rig.node._navigator_status_callback(rig.side.status_topic, NS(data="RFID_SCANNING:1"))
    assert not rig.side.hold_client.requests
    rig.node._external_plan_callback(make_plan(sec=2))
    assert rig.node._checkpoint_plan.route_id == 3_000_000_123
    rig.node._ack_callback(rig.side, NS(data=3_000_000_123))
    rig.node._navigator_status_callback(rig.side.status_topic, NS(data="RFID_SCANNING:1"))
    assert len(rig.side.hold_client.requests) == 1
