from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable


STAGE_LABELS = {
    "layer1": "1/3 任务点",
    "layer2": "2/3 隧道",
    "layer3": "3/3 道路补漏与回程",
}


@dataclass(frozen=True)
class PlanSpec:
    mode: str
    target_ids: tuple[str, ...] = ()
    required_lane_ids: tuple[str, ...] = ()
    completed_lane_ids: tuple[str, ...] = ()
    return_home: bool = True
    allow_partial: bool = False

    def has_progress(
        self, visit_order: Iterable[str], covered_lane_ids: Iterable[str]
    ) -> bool:
        if self.mode == "layer1":
            return bool(set(map(str, visit_order)).intersection(self.target_ids))
        if self.required_lane_ids:
            return bool(
                set(map(str, covered_lane_ids)).intersection(self.required_lane_ids)
            )
        return True


@dataclass(frozen=True)
class MissionLayout:
    task_ids: tuple[str, ...]
    tunnel_lane_ids: tuple[str, ...]
    road_lane_ids: tuple[str, ...]

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "MissionLayout":
        inspection = tuple(map(str, config["missions"]["default_inspection_lanes"]))
        lane_tunnels = {
            str(lane["id"]): bool(lane["tunnel"]) for lane in config["lanes"]
        }
        return cls(
            task_ids=tuple(map(str, config["missions"]["default_targets"])),
            tunnel_lane_ids=tuple(lane for lane in inspection if lane_tunnels[lane]),
            road_lane_ids=tuple(lane for lane in inspection if not lane_tunnels[lane]),
        )


class StagedMission:
    """Own confirmed progress for the three competition mission stages."""

    def __init__(self, layout: MissionLayout) -> None:
        self.layout = layout
        self.reset()

    def reset(self) -> None:
        self.requested_tasks: tuple[str, ...] = self.layout.task_ids
        self.completed_tasks: set[str] = set()
        self.completed_tunnels: set[str] = set()
        self.confirmed_lanes: set[str] = set()
        self.started = False
        self.running = False
        self.paused = False
        self.finished = False

    def start(self, task_ids: Iterable[str] | None = None) -> None:
        self.reset()
        selected = (
            self.layout.task_ids if task_ids is None else tuple(map(str, task_ids))
        )
        known = set(self.layout.task_ids)
        self.requested_tasks = tuple(task for task in selected if task in known)
        if not self.requested_tasks:
            raise ValueError("三阶段任务至少需要一个任务点")
        self.started = True
        self.running = True

    def pause(self) -> None:
        if self.running:
            self.paused = True

    def resume(self) -> None:
        if self.running:
            self.paused = False

    def abort(self) -> None:
        self.running = False
        self.paused = True

    def remaining_tasks(self) -> tuple[str, ...]:
        return tuple(
            task for task in self.requested_tasks if task not in self.completed_tasks
        )

    def remaining_tunnels(self) -> tuple[str, ...]:
        return tuple(
            lane
            for lane in self.layout.tunnel_lane_ids
            if lane not in self.completed_tunnels
        )

    def remaining_roads(self) -> tuple[str, ...]:
        return tuple(
            lane
            for lane in self.layout.road_lane_ids
            if lane not in self.confirmed_lanes
        )

    def current_stage(self) -> str | None:
        if not self.started or self.finished:
            return None
        if self.remaining_tasks():
            return "layer1"
        if self.remaining_tunnels():
            return "layer2"
        return "layer3"

    def plan_spec(self) -> PlanSpec:
        if not self.running or self.paused or self.finished:
            raise RuntimeError("三阶段任务当前不可规划")
        stage = self.current_stage()
        if stage == "layer1":
            return PlanSpec(
                mode=stage,
                target_ids=self.remaining_tasks(),
                completed_lane_ids=tuple(sorted(self.confirmed_lanes)),
                return_home=False,
                allow_partial=True,
            )
        if stage == "layer2":
            unfinished_tunnels = set(self.remaining_tunnels())
            return PlanSpec(
                mode=stage,
                required_lane_ids=self.remaining_tunnels(),
                completed_lane_ids=tuple(
                    sorted(self.confirmed_lanes - unfinished_tunnels)
                ),
                return_home=False,
                allow_partial=True,
            )
        if stage == "layer3":
            return PlanSpec(
                mode=stage,
                required_lane_ids=self.remaining_roads(),
                completed_lane_ids=tuple(sorted(self.confirmed_lanes)),
                return_home=True,
                allow_partial=True,
            )
        raise RuntimeError("三阶段任务已经完成")

    def commit(
        self,
        spec: PlanSpec,
        visit_order: Iterable[str],
        covered_lane_ids: Iterable[str],
        plan_complete: bool,
    ) -> int:
        if not self.running or spec.mode != self.current_stage():
            raise RuntimeError("完成结果与当前任务阶段不匹配")
        before = self.progress_count()
        covered = set(map(str, covered_lane_ids))
        self.confirmed_lanes.update(covered)
        if spec.mode == "layer1":
            self.completed_tasks.update(
                set(map(str, visit_order)).intersection(spec.target_ids)
            )
        elif spec.mode == "layer2":
            self.completed_tunnels.update(covered.intersection(spec.required_lane_ids))
        elif spec.mode == "layer3" and plan_complete and not self.remaining_roads():
            self.finished = True
            self.running = False
        return self.progress_count() - before

    def commit_empty(self, spec: PlanSpec, plan_complete: bool) -> None:
        if (
            spec.mode != "layer3"
            or spec.required_lane_ids
            or not plan_complete
            or self.remaining_roads()
        ):
            raise RuntimeError("只有无需移动的最终回程阶段可以直接完成")
        self.finished = True
        self.running = False

    def progress_count(self) -> int:
        return (
            len(self.completed_tasks)
            + len(self.completed_tunnels)
            + len(self.confirmed_lanes)
        )

    def summary(self) -> str:
        if self.finished:
            return "三阶段任务完成"
        stage = self.current_stage()
        if stage is None:
            return "三阶段任务未开始"
        return (
            f"{STAGE_LABELS[stage]} | 任务 {len(self.remaining_tasks())} "
            f"隧道 {len(self.remaining_tunnels())} "
            f"道路 {len(self.remaining_roads())}"
        )
