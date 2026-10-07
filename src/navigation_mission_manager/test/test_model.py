from pathlib import Path

import yaml

from navigation_mission_manager.model import MissionLayout, PlanSpec, StagedMission


CONFIG = (
    Path(__file__).resolve().parents[2] / "new_map_planner" / "config" / "arena.yaml"
)


def _mission() -> StagedMission:
    with CONFIG.open(encoding="utf-8") as stream:
        return StagedMission(MissionLayout.from_config(yaml.safe_load(stream)))


def test_stages_advance_only_from_confirmed_progress():
    mission = _mission()
    mission.start()
    layer1 = mission.plan_spec()
    assert layer1.mode == "layer1"
    assert not layer1.return_home
    assert mission.commit(layer1, layer1.target_ids, ("ROAD_TOP_LEFT",), True) > 0

    layer2 = mission.plan_spec()
    assert layer2.mode == "layer2"
    assert not layer2.return_home
    assert mission.commit(layer2, (), layer2.required_lane_ids, True) > 0

    layer3 = mission.plan_spec()
    assert layer3.mode == "layer3"
    assert layer3.return_home
    assert "ROAD_TOP_LEFT" not in layer3.required_lane_ids
    mission.commit(layer3, (), layer3.required_lane_ids, True)
    assert mission.finished


def test_partial_route_replans_same_stage():
    mission = _mission()
    mission.start(("1", "2"))
    first = mission.plan_spec()
    mission.commit(first, ("1",), ("ROAD_TOP_LEFT",), False)
    assert mission.plan_spec().target_ids == ("2",)


def test_tunnel_crossed_early_is_still_required_in_tunnel_stage():
    mission = _mission()
    mission.start(("1",))
    layer1 = mission.plan_spec()
    tunnel = mission.layout.tunnel_lane_ids[0]
    mission.commit(layer1, ("1",), (tunnel,), True)
    assert tunnel in mission.plan_spec().required_lane_ids


def test_pause_blocks_planning_without_discarding_progress():
    mission = _mission()
    mission.start(("1",))
    mission.pause()
    try:
        mission.plan_spec()
    except RuntimeError:
        pass
    else:
        raise AssertionError("paused mission must not produce a plan")
    mission.resume()
    assert mission.plan_spec().mode == "layer1"


def test_only_empty_final_stage_can_complete_without_navigation():
    mission = StagedMission(
        MissionLayout(task_ids=("1",), tunnel_lane_ids=(), road_lane_ids=())
    )
    mission.start()
    first = mission.plan_spec()
    mission.commit(first, ("1",), (), True)
    mission.commit_empty(mission.plan_spec(), True)
    assert mission.finished


def test_wrong_stage_result_is_rejected():
    mission = _mission()
    mission.start()
    try:
        mission.commit(PlanSpec(mode="layer2"), (), (), True)
    except RuntimeError:
        pass
    else:
        raise AssertionError("result from another stage must be rejected")


def test_plan_spec_requires_progress_in_its_current_work_set():
    layer1 = PlanSpec(mode="layer1", target_ids=("1", "2"))
    assert layer1.has_progress(("2",), ())
    assert not layer1.has_progress(("3",), ("ROAD_TOP_LEFT",))

    layer2 = PlanSpec(mode="layer2", required_lane_ids=("TUNNEL_1",))
    assert layer2.has_progress((), ("TUNNEL_1",))
    assert not layer2.has_progress(("2",), ("ROAD_TOP_LEFT",))
