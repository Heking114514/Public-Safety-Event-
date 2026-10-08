from __future__ import annotations

import queue
import signal
import time
import tkinter as tk
from pathlib import Path
from typing import Any, Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from mission_control_interfaces.msg import MissionState
from mission_control_interfaces.srv import ControlMission
from vehicle_config.physical import load_vehicle_physical_config

from .model import (
    PlanSelection,
    VehiclePose,
    front_obstacle_polygon,
    maneuver_name,
    map_to_arena,
    merge_obstacle,
)
from .ros_client import RosRuntime
from .view import PlannerView


MODE_NAMES = {
    "第一阶段": "staged",
    "第二阶段": "staged",
    "第三阶段": "staged",
}
STAGE_LABELS = {
    "layer1": "第一阶段",
    "layer2": "第二阶段",
    "layer3": "第三阶段",
}


class PlannerApplication:
    def __init__(self, root: tk.Tk, config: dict[str, Any]) -> None:
        self.root = root
        self.config = config
        self.physical = load_vehicle_physical_config()
        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.runtime = RosRuntime(self.events)
        self.node = self.runtime.node

        self.mode_var = tk.StringVar(value="第一阶段")
        self.status_var = tk.StringVar(value="等待融合里程计与规划服务")
        self.odom_var = tk.StringVar(value="ODOM: WAIT")
        self.planner_var = tk.StringVar(value="PLANNER: WAIT")
        self.navigator_var = tk.StringVar(value="NAV: UNKNOWN")
        self.ack_var = tk.StringVar(value="ACK: --")
        self.stage_var = tk.StringVar(value="三阶段任务未开始")
        self.metric_var = tk.StringVar(value="尚未规划")
        self.hold_var = tk.StringVar(value="驻停")
        self.obstacle_var = tk.StringVar(value="动态障碍 0")

        self.selected_targets = set(map(str, config["tasks"].keys()))
        self.completed_lanes: set[str] = set()
        self.completed_targets: set[str] = set()
        self.mission_state: Optional[MissionState] = None
        self.vehicle_pose: Optional[VehiclePose] = None
        self.current_pose_arena: Optional[tuple[float, float, float]] = None
        self.plan: Any = None
        self.active_plan: Any = None
        self.active_spec: Optional[PlanSelection] = None
        self.active_plan_id: Optional[int] = None
        self.acknowledged_plan_id: Optional[int] = None
        self.received_ack_ids: set[int] = set()
        self.activation_started_at: Optional[float] = None
        self.activation_timed_out = False
        self.pending_spec: Optional[PlanSelection] = None
        self.pending_activate = False
        self.navigation_status = "UNKNOWN"
        self.motion_held = False
        self.hold_sources: tuple[str, ...] = ()
        self.closing = False
        self.close_deadline: Optional[float] = None
        self.latest_front_range: Optional[tuple[float, float]] = None
        self.front_blocked = False
        self.front_blocked_at: Optional[float] = None
        self.pending_obstacle: Optional[list[tuple[float, float]]] = None
        self.dynamic_obstacles: list[list[tuple[float, float]]] = []

        self.view = PlannerView(root, self)
        self.mode_changed()
        self.root.after(50, self._poll)

    def _selected_mode(self) -> str:
        return MODE_NAMES[self.mode_var.get()]

    def target_selection_changed(self) -> None:
        self.selected_targets = set(self.view.selected_task_ids())

    def mode_changed(self) -> None:
        # The phase selector mirrors the mission manager. It does not submit a
        # separate route or let the UI skip a phase.
        self._set_stage_selector(self.mission_state.stage if self.mission_state else "")
        self.view.set_targets_enabled(False)
        self._update_stage_display()

    def _set_stage_selector(self, stage: str) -> None:
        label = STAGE_LABELS.get(str(stage), "第一阶段")
        if self.mode_var.get() != label:
            self.mode_var.set(label)

    def preview(self) -> None:
        if self._selected_mode() == "staged":
            self._send_plan(
                PlanSelection(
                    mode="layer1",
                    target_ids=tuple(sorted(self.selected_targets, key=int)),
                    return_home=False,
                    allow_partial=True,
                ),
                activate=False,
            )
            return
        spec = self._manual_spec()
        if spec is not None:
            self._send_plan(spec, activate=False)

    def navigate(self) -> None:
        if self._selected_mode() == "staged":
            state = self.mission_state.state if self.mission_state is not None else None
            command = (
                ControlMission.Request.RESUME
                if state in {MissionState.PAUSED, MissionState.FAULT}
                else ControlMission.Request.START
            )
            try:
                self.node.request_mission(
                    command, sorted(self.selected_targets, key=int)
                )
                self.status_var.set("任务命令已发送")
            except RuntimeError as exception:
                self.status_var.set(str(exception))
            return
        spec = self._manual_spec()
        if spec is not None:
            self.status_var.set("手动路线只能预览；实车执行请通过任务总控")

    def _manual_spec(self) -> Optional[PlanSelection]:
        mode = self._selected_mode()
        targets = tuple(sorted(self.selected_targets, key=lambda item: int(item)))
        if mode == "targets" and not targets:
            self.status_var.set("至少选择一个任务点")
            return None
        return PlanSelection(
            mode=mode,
            target_ids=targets if mode == "targets" else (),
            completed_lane_ids=tuple(sorted(self.completed_lanes)),
            return_home=True,
            allow_partial=mode != "targets",
        )

    def _send_plan(self, spec: PlanSelection, activate: bool) -> None:
        if self.vehicle_pose is None or not self.vehicle_pose.fresh():
            self.status_var.set("无法规划：融合里程计未就绪或已超时")
            return
        if self.active_plan_id is not None:
            self.status_var.set("当前路线仍在执行，不能替换")
            return
        try:
            self.node.request_plan(
                self.vehicle_pose,
                spec,
                self.dynamic_obstacles,
                activate,
            )
        except RuntimeError as exception:
            self.status_var.set(str(exception))
            return
        self.pending_spec = spec
        self.pending_activate = activate
        stage = "第一阶段" if spec.mode == "layer1" else ""
        self.status_var.set(f"正在预览{stage}…" if stage else "正在按最新实车位姿规划并验证…")
        self._refresh_controls()

    def toggle_hold(self) -> None:
        if self._selected_mode() == "staged":
            state = self.mission_state.state if self.mission_state is not None else None
            command = (
                ControlMission.Request.RESUME
                if state in {MissionState.PAUSED, MissionState.FAULT}
                else ControlMission.Request.PAUSE
            )
            try:
                self.node.request_mission(command, [])
            except RuntimeError as exception:
                self.status_var.set(str(exception))
            return
        try:
            own_hold = (
                "navigation_mission_manager/external/new_map_planner_ui"
                in self.hold_sources
            )
            self.node.request_hold(not own_hold)
        except RuntimeError as exception:
            self.status_var.set(str(exception))

    def _consume_event(self, kind: str, value: Any) -> None:
        if kind == "odometry":
            self.vehicle_pose = value
            self.current_pose_arena = map_to_arena(
                value.x, value.y, value.yaw, self.config
            )
            self.view.draw()
        elif kind == "navigation_status":
            self._handle_navigation_status(str(value))
        elif kind == "route_ack":
            route_id = int(value)
            self.received_ack_ids.add(route_id)
            if len(self.received_ack_ids) > 64:
                self.received_ack_ids = set(sorted(self.received_ack_ids)[-32:])
            if route_id == self.active_plan_id:
                self.acknowledged_plan_id = route_id
                self.ack_var.set(f"ACK: {route_id}")
                self.status_var.set("导航器已确认路线，实车执行中")
        elif kind == "hold_state":
            self.motion_held, self.hold_sources = value
            if self._selected_mode() != "staged":
                own_hold = (
                    "navigation_mission_manager/external/new_map_planner_ui"
                    in self.hold_sources
                )
                self.hold_var.set("解除驻停" if own_hold else "驻停")
        elif kind == "hold_response":
            success, held, sources, message = value
            self.motion_held = held
            self.hold_sources = sources
            own_hold = (
                "navigation_mission_manager/external/new_map_planner_ui"
                in self.hold_sources
            )
            self.hold_var.set("解除驻停" if own_hold else "驻停")
            self.status_var.set(message if success else f"驻停失败：{message}")
            if success and held and self.activation_timed_out:
                self._clear_active_plan()
        elif kind == "mission_state":
            self._handle_mission_state(value)
        elif kind == "mission_response":
            self.mission_state = value.state
            self.status_var.set(
                value.message if value.accepted else f"任务命令被拒绝：{value.message}"
            )
            self._update_stage_display()
        elif kind == "planner_plan":
            self._show_plan(value)
        elif kind == "plan_response":
            self._handle_plan_response(value)
        elif kind == "front_range":
            self.latest_front_range = value
            if self.front_blocked and self.pending_obstacle is None:
                self._capture_pending_obstacle(value[1])
        elif kind == "front_blocked":
            self._handle_front_blocked(*value)
        elif kind == "error":
            self.pending_spec = None
            self.pending_activate = False
            self.status_var.set(str(value))

    def _handle_mission_state(self, state: MissionState) -> None:
        self.mission_state = state
        self._set_stage_selector(state.stage)
        stage_label = STAGE_LABELS.get(str(state.stage), "未开始")
        self.stage_var.set(
            f"{stage_label} | 任务 {state.remaining_tasks} "
            f"隧道 {state.remaining_tunnels} 道路 {state.remaining_roads}"
        )
        self.motion_held = bool(state.motion_held)
        self.hold_var.set(
            "继续巡视"
            if state.state in {MissionState.PAUSED, MissionState.FAULT}
            else "暂停巡视"
        )
        if self._selected_mode() == "staged":
            self.status_var.set(f"{state.state_name}: {state.message}")

    def _show_plan(self, plan: Any) -> None:
        self.plan = plan
        self.view.update_actions(plan.maneuvers, maneuver_name)
        turns = sum(int(item.type) in (1, 2) for item in plan.maneuvers)
        self.metric_var.set(f"{plan.status}  {plan.length_m:.2f} m  {turns} 次转弯")
        self.view.draw()

    def _handle_front_blocked(self, blocked: bool, received_at: float) -> None:
        if self._selected_mode() == "staged":
            return
        rising_edge = blocked and not self.front_blocked
        self.front_blocked = blocked
        if rising_edge:
            self.front_blocked_at = received_at
        if not rising_edge:
            return
        self._capture_pending_obstacle(received_at)

    def _capture_pending_obstacle(self, received_at: float) -> None:
        if (
            self.vehicle_pose is None
            or not self.vehicle_pose.fresh(received_at)
            or self.latest_front_range is None
            or self.front_blocked_at is None
            or received_at - self.latest_front_range[1] > 0.5
            or received_at - self.latest_front_range[1] < -0.1
            or self.latest_front_range[1] < self.front_blocked_at - 0.1
        ):
            self.pending_obstacle = None
            return
        try:
            self.pending_obstacle = front_obstacle_polygon(
                self.vehicle_pose,
                self.latest_front_range[0],
                self.physical.camera_x_m,
            )
        except ValueError:
            self.pending_obstacle = None

    def clear_obstacles(self) -> None:
        if self.active_plan_id is not None:
            self.status_var.set("路线执行中，不能清除障碍地图")
            return
        self.dynamic_obstacles.clear()
        self.pending_obstacle = None
        self.obstacle_var.set("动态障碍 0")
        self.status_var.set("已清除动态障碍地图")
        self.view.draw()

    def _handle_plan_response(self, response: Any) -> None:
        spec = self.pending_spec
        activate = self.pending_activate
        self.pending_spec = None
        self.pending_activate = False
        if not response.success:
            self.status_var.set(f"规划失败：{response.message}")
            return
        self._show_plan(response.plan)
        self.metric_var.set(
            f"{self.plan.status}  {self.plan.length_m:.2f} m  "
            f"{sum(int(item.type) in (1, 2) for item in self.plan.maneuvers)} 次转弯  "
            f"{response.planning_time_ms:.1f} ms"
        )
        if response.activated:
            self.active_plan = self.plan
            self.active_spec = spec
            self.active_plan_id = int(self.plan.plan_id)
            self.activation_started_at = time.monotonic()
            self.activation_timed_out = False
            if self.active_plan_id in self.received_ack_ids:
                self.acknowledged_plan_id = self.active_plan_id
                self.ack_var.set(f"ACK: {self.active_plan_id}")
                self.status_var.set("导航器已确认路线，实车执行中")
            else:
                self.acknowledged_plan_id = None
                self.ack_var.set("ACK: WAIT")
                self.status_var.set("安全路线已发布，等待导航器确认")
        elif activate:
            self.status_var.set(f"路线未激活：{response.message}")
        else:
            prefix = "第一阶段预览完成" if spec and spec.mode == "layer1" else "预览完成"
            self.status_var.set(f"{prefix}：{response.message}")

    def _handle_navigation_status(self, status: str) -> None:
        previous = self.navigation_status
        self.navigation_status = status
        self.navigator_var.set(f"NAV: {status}")
        if self._selected_mode() == "staged":
            return
        if status == "EXPECTED_OBSTACLE_AT_TURN":
            self.pending_obstacle = None
            self.front_blocked_at = None
        elif (
            status == "OBSTACLE_REPLAN_REQUIRED"
            or status.startswith("FAULT_OBSTACLE_RECOVERY")
        ) and self.pending_obstacle is not None:
            self.dynamic_obstacles = merge_obstacle(
                self.dynamic_obstacles, self.pending_obstacle
            )
            self.pending_obstacle = None
            self.front_blocked_at = None
            self.obstacle_var.set(f"动态障碍 {len(self.dynamic_obstacles)}")
            self.view.draw()
        if status == "GOAL_REACHED" and self.active_plan is not None:
            if self.acknowledged_plan_id != self.active_plan_id:
                return
            self.completed_lanes.update(self.active_plan.covered_lane_ids)
            self.completed_targets.update(self.active_plan.visit_order)
            self._clear_active_plan()
            self.status_var.set("单次实车路线执行完成")
        elif status.startswith("FAULT_") or status in {
            "OBSTACLE_REPLAN_REQUIRED",
        }:
            if self.active_plan_id is not None:
                self.status_var.set(f"导航中止：{status}，本段不提交进度")
            self._clear_active_plan()
        elif status != previous and self.active_plan_id is not None:
            self.status_var.set(f"实车导航状态：{status}")

    def _clear_active_plan(self) -> None:
        self.active_plan = None
        self.active_spec = None
        self.active_plan_id = None
        self.acknowledged_plan_id = None
        self.activation_started_at = None
        self.activation_timed_out = False
        self.ack_var.set("ACK: --")

    def _update_stage_display(self) -> None:
        if self._selected_mode() == "staged":
            if self.mission_state is None:
                self.stage_var.set("等待自主任务节点")
            else:
                self.stage_var.set(
                    f"{STAGE_LABELS.get(str(self.mission_state.stage), '未开始')} | "
                    f"任务 {self.mission_state.remaining_tasks} "
                    f"隧道 {self.mission_state.remaining_tunnels} "
                    f"道路 {self.mission_state.remaining_roads}"
                )
        else:
            self.stage_var.set("单次路线模式")

    def _refresh_controls(self) -> None:
        odom_ready = self.vehicle_pose is not None and self.vehicle_pose.fresh()
        planner_ready = self.node.ready()
        direct_busy = self.node.planning or self.active_plan_id is not None
        self.odom_var.set("ODOM: READY" if odom_ready else "ODOM: WAIT")
        self.planner_var.set("PLANNER: READY" if planner_ready else "PLANNER: WAIT")
        staged_mode = self._selected_mode() == "staged"
        mission_state = self.mission_state.state if self.mission_state else None
        mission_owns_navigation = mission_state not in {
            None,
            MissionState.IDLE,
            MissionState.WAIT_READY,
            MissionState.COMPLETE,
        }
        base_ready = odom_ready and planner_ready and not direct_busy
        if staged_mode:
            startable = mission_state in {MissionState.IDLE, MissionState.COMPLETE}
            resumable = mission_state in {MissionState.PAUSED, MissionState.FAULT}
            preview_ready = base_ready and startable
            navigate_ready = (
                odom_ready
                and self.node.mission_ready()
                and not direct_busy
                and (startable or resumable)
            )
        else:
            preview_ready = base_ready and not mission_owns_navigation
            navigate_ready = preview_ready and not self.motion_held
        self.view.set_commands_enabled(preview_ready, navigate_ready)
        self.view.set_mode_enabled(not direct_busy and not mission_owns_navigation)
        self.view.set_targets_enabled(
            self._selected_mode() == "targets"
            and not direct_busy
            and not mission_owns_navigation
        )
        self.view.set_navigate_text(
            "继续巡视"
            if staged_mode
            and mission_state in {MissionState.PAUSED, MissionState.FAULT}
            else "开始三阶段巡视"
            if staged_mode
            else "下发并导航"
        )
        mission_hold_enabled = mission_state in {
            MissionState.PLANNING,
            MissionState.WAIT_ROUTE_ACK,
            MissionState.EXECUTING,
            MissionState.RECOVERING_OBSTACLE,
            MissionState.REPLANNING,
            MissionState.PAUSED,
            MissionState.FAULT,
        }
        self.view.set_hold_enabled(
            self.node.mission_ready() and mission_hold_enabled
            if staged_mode
            else self.node.hold_ready()
        )
        self.view.set_clear_obstacles_enabled(
            not staged_mode and not direct_busy and bool(self.dynamic_obstacles)
        )

    def _poll(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                self._consume_event(kind, value)
        except queue.Empty:
            pass
        if (
            self.active_plan_id is not None
            and self.acknowledged_plan_id is None
            and self.activation_started_at is not None
            and time.monotonic() - self.activation_started_at > 3.0
            and not self.activation_timed_out
        ):
            self.activation_timed_out = True
            self.ack_var.set("ACK: TIMEOUT")
            self.status_var.set("导航器未确认路线，正在申请驻停")
            if self.node.hold_ready():
                try:
                    self.node.request_hold(True)
                except RuntimeError as exception:
                    self.status_var.set(f"路线未确认且驻停失败：{exception}")
        self._refresh_controls()
        if not self.closing:
            self.root.after(100, self._poll)

    def close(self) -> None:
        if self.closing:
            return
        self.closing = True
        if (
            self._selected_mode() != "staged"
            and self.active_plan_id is not None
            and not self.motion_held
        ):
            if not self.node.hold_ready():
                self.closing = False
                self.status_var.set("驻停服务未就绪，窗口保持打开")
                return
            try:
                self.node.request_hold(True)
                self.close_deadline = time.monotonic() + 2.0
                self.status_var.set("正在确认实车驻停后关闭…")
                self.root.after(50, self._wait_for_close_hold)
                return
            except RuntimeError as exception:
                self.closing = False
                self.status_var.set(f"无法确认驻停：{exception}")
                return
        self._finish_close()

    def _wait_for_close_hold(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                self._consume_event(kind, value)
        except queue.Empty:
            pass
        if self.motion_held:
            self._finish_close()
            return
        if self.close_deadline is not None and time.monotonic() >= self.close_deadline:
            self.closing = False
            self.close_deadline = None
            self.status_var.set("未确认导航驻停，窗口保持打开")
            self.root.after(100, self._poll)
            return
        self.root.after(50, self._wait_for_close_hold)

    def _finish_close(self) -> None:
        self.runtime.close()
        self.root.destroy()


def main() -> None:
    package_share = Path(get_package_share_directory("new_map_planner"))
    with (package_share / "config" / "arena.yaml").open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    root = tk.Tk()
    application = PlannerApplication(root, config)
    signal.signal(
        signal.SIGINT, lambda _signal, _frame: root.after(0, application.close)
    )
    root.mainloop()
