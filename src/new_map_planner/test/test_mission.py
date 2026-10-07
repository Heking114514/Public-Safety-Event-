from pathlib import Path

import yaml

from new_map_mission.state import MissionLayout, PlanSpec, StagedMission


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "arena.yaml"


def _layout() -> MissionLayout:
    with CONFIG_PATH.open(encoding="utf-8") as stream:
        return MissionLayout.from_config(yaml.safe_load(stream))


def test_official_layout_has_three_distinct_work_sets():
    layout = _layout()
    assert len(layout.task_ids) == 12
    assert len(layout.tunnel_lane_ids) == 4
    assert len(layout.road_lane_ids) == 24


def test_staged_mission_advances_only_after_confirmed_progress():
    mission = StagedMission(_layout())
    mission.start()
    layer1 = mission.plan_spec()
    mission.commit(layer1, layer1.target_ids, ("ROAD_TOP_LEFT",), True)
    layer2 = mission.plan_spec()
    mission.commit(layer2, (), layer2.required_lane_ids, True)
    layer3 = mission.plan_spec()
    mission.commit(layer3, (), layer3.required_lane_ids, True)
    assert mission.finished


def test_partial_result_stays_in_the_same_stage():
    mission = StagedMission(_layout())
    mission.start(("1", "2"))
    first = mission.plan_spec()
    mission.commit(first, ("1",), ("ROAD_TOP_LEFT",), False)
    assert mission.plan_spec().target_ids == ("2",)


def test_tunnel_crossed_during_layer1_is_still_required_by_layer2():
    mission = StagedMission(_layout())
    mission.start(("1",))
    layer1 = mission.plan_spec()
    tunnel = mission.layout.tunnel_lane_ids[0]
    mission.commit(layer1, ("1",), (tunnel,), True)
    assert tunnel in mission.plan_spec().required_lane_ids


def test_only_empty_final_stage_can_complete_without_navigation():
    mission = StagedMission(
        MissionLayout(task_ids=("1",), tunnel_lane_ids=(), road_lane_ids=())
    )
    mission.start()
    layer1 = mission.plan_spec()
    mission.commit(layer1, ("1",), (), True)
    mission.commit_empty(mission.plan_spec(), True)
    assert mission.finished


def test_empty_nonfinal_stage_cannot_be_committed():
    mission = StagedMission(_layout())
    mission.start()
    try:
        mission.commit_empty(PlanSpec(mode="layer1"), True)
    except RuntimeError:
        pass
    else:
        raise AssertionError("layer1 must not complete without navigation")
