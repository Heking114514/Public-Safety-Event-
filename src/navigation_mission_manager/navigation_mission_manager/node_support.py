from __future__ import annotations

from typing import Any

from mission_control_interfaces.msg import MissionState, MotionHoldState
from mission_control_interfaces.srv import SetMotionHold
from std_msgs.msg import String


class NodeSupportMixin:
    """Mission state publication and multi-source motion-hold helpers."""

    def _on_hold_state(self, message: MotionHoldState) -> None:
        self._motion_held = bool(message.held)

    def _request_hold(self, suffix: str, hold: bool, reason: str) -> None:
        if not self._hold.service_is_ready():
            self.get_logger().warning("motion hold service is not ready")
            return
        request = SetMotionHold.Request()
        request.source = f"navigation_mission_manager/{suffix}"
        request.hold = hold
        request.reason = reason
        future = self._hold.call_async(request)
        future.add_done_callback(self._on_hold_response)

    def request_shutdown_hold(self) -> Any | None:
        if not self._hold.service_is_ready():
            return None
        request = SetMotionHold.Request()
        request.source = "navigation_mission_manager/shutdown"
        request.hold = True
        request.reason = "mission manager is shutting down"
        return self._hold.call_async(request)

    def _on_hold_response(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"motion hold request failed: {error}")
            return
        self._motion_held = bool(response.motion_held)
        if not response.success:
            self.get_logger().error(f"motion hold rejected: {response.message}")

    def _set_state(self, state: int, detail: str) -> None:
        changed = state != self._state or detail != self._detail
        self._state = state
        self._detail = detail
        if changed:
            self._publish_state()

    def _state_message(self) -> MissionState:
        message = MissionState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = self._state
        message.state_name = self._state_names[self._state]
        message.stage = self._mission.current_stage() or ""
        message.message = self._detail
        message.active_route_id = (
            self._active_route.route_id if self._active_route is not None else 0
        )
        message.autonomous = self._mission.running and not self._mission.paused
        message.motion_held = self._motion_held
        message.remaining_tasks = len(self._mission.remaining_tasks())
        message.remaining_tunnels = len(self._mission.remaining_tunnels())
        message.remaining_roads = len(self._mission.remaining_roads())
        message.dynamic_obstacles = len(self._dynamic_obstacles)
        message.confirmed_events = self._events.reported_count
        message.pending_reports = self._events.pending_count
        message.completed_task_ids = sorted(self._mission.completed_tasks)
        message.completed_tunnel_ids = sorted(self._mission.completed_tunnels)
        message.completed_lane_ids = sorted(self._mission.confirmed_lanes)
        return message

    def _publish_state(self) -> None:
        self._state_publisher.publish(self._state_message())
        self._publish_text(f"{self._state_names[self._state]}:{self._detail}")

    def _publish_text(self, text: str) -> None:
        message = String()
        message.data = text
        self._text_publisher.publish(message)
