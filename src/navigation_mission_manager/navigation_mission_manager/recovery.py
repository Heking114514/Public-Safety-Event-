from __future__ import annotations

from typing import Any

from mission_control_interfaces.msg import MissionState
from mission_control_interfaces.srv import ContinueRetreat


class RecoveryMixin:
    """Coordinate bounded retreat steps with repeated planning attempts."""

    def _on_navigator_status(self, message) -> None:
        status = str(message.data)
        self._navigator_status = status
        if status == "EXPECTED_OBSTACLE_AT_TURN":
            self._pending_obstacle = None
            return
        if status in {
            "OBSTACLE_BRAKING",
            "OBSTACLE_REPLAYING",
            "OBSTACLE_REVERSING",
        }:
            if self._active_route is not None or self._awaiting_retreat:
                self._set_state(MissionState.RECOVERING_OBSTACLE, status)
            return
        if status == "OBSTACLE_REPLAN_REQUIRED":
            self._handle_obstacle_replan()
            return
        if status == "GOAL_REACHED":
            self._handle_goal_reached()
            return
        navigation_owned = (
            self._active_route is not None or self._obstacle_recovery_active
        )
        if navigation_owned and (
            status.startswith("FAULT_") or status == "TURN_BLOCKED"
        ):
            self._fault(f"导航故障：{status}")
            return
        if self._active_route is not None and self._active_route.acknowledged:
            inactive = {"IDLE", "WAITING_FOR_ROUTE", "WAITING_FOR_LOCALIZATION"}
            if status not in inactive:
                self._active_route.motion_seen = True

    def _handle_obstacle_replan(self) -> None:
        if self._awaiting_retreat:
            self._awaiting_retreat = False
            self._schedule_plan(True)
            return
        if self._active_route is None or not self._active_route.acknowledged:
            return
        if not self._active_route.motion_seen:
            self._fault("控制器拒绝了尚未开始执行的规划路线")
            return
        self._clear_active_route()
        if not self._promote_pending_obstacle():
            self._fault("控制器请求避障重规划，但没有可用障碍位置")
            return
        self._obstacle_recovery_active = True
        self._retreat_extensions = 0
        self._schedule_plan(True)

    def _handle_plan_rejection(self, message: str) -> None:
        if self._obstacle_recovery_active:
            self._request_additional_retreat(message)
        elif self._completed_route_retreat_available:
            self._completed_route_retreat_available = False
            self._obstacle_recovery_active = True
            self._retreat_extensions = 0
            self._request_additional_retreat(message)
        else:
            self._plan_failed(message)

    def _request_additional_retreat(self, planner_message: str) -> None:
        if self._retreat_future is not None:
            return
        maximum = int(self.get_parameter("max_retreat_extensions").value)
        if self._retreat_extensions >= maximum:
            self._fault(f"绕障规划仍无路，已达到最大继续倒退次数：{planner_message}")
            return
        if not self._retreat_client.service_is_ready():
            self._fault("控制器继续倒退服务不可用")
            return

        request = ContinueRetreat.Request()
        request.minimum_distance_m = float(self.get_parameter("retreat_step_m").value)
        self._retreat_generation += 1
        generation = self._retreat_generation
        self._retreat_future = self._retreat_client.call_async(request)
        self._retreat_future.add_done_callback(
            lambda future: self._on_retreat_response(
                generation, planner_message, future
            )
        )
        self._set_state(
            MissionState.RECOVERING_OBSTACLE,
            f"当前恢复姿态无路，请求继续倒退：{planner_message}",
        )

    def _on_retreat_response(
        self, generation: int, planner_message: str, future: Any
    ) -> None:
        if generation != self._retreat_generation:
            return
        self._retreat_future = None
        try:
            response = future.result()
        except Exception as error:
            self._fault(f"继续倒退服务失败：{error}")
            return
        if not response.accepted:
            reason = "已执行的安全轨迹耗尽" if response.trace_exhausted else response.message
            self._fault(f"绕障规划仍无路且无法继续倒退：{reason}")
            return

        self._retreat_extensions += 1
        self._awaiting_retreat = True
        maximum = self.get_parameter("max_retreat_extensions").value
        self._set_state(
            MissionState.RECOVERING_OBSTACLE,
            "控制器继续沿历史轨迹倒退 " f"({self._retreat_extensions}/{maximum})",
        )

    def _clear_obstacle_recovery(self) -> None:
        self._retreat_generation += 1
        self._retreat_future = None
        self._awaiting_retreat = False
        self._obstacle_recovery_active = False
        self._retreat_extensions = 0
