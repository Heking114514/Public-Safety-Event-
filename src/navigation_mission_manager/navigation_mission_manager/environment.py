from __future__ import annotations

import math
import time
from typing import Any

from geometry_msgs.msg import PolygonStamped
from mission_control_interfaces.action import ReportEvent
from mission_control_interfaces.msg import EventCandidate, MissionState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from .obstacles import (
    Pose2D,
    front_obstacle_polygon,
    merge_obstacle,
    obstacle_memory_expired,
    valid_polygon,
)
from .ros_support import VehiclePose, event_data, event_message, quaternion_yaw


class EnvironmentMixin:
    """Obstacle observations and event-report orchestration for the mission node."""

    def _on_odometry(self, message: Odometry) -> None:
        if message.header.frame_id != "map" or message.child_frame_id != "base_link":
            return
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        if stamp_ns <= 0 or stamp_ns <= self._last_odom_stamp_ns:
            return
        yaw = quaternion_yaw(message.pose.pose.orientation)
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        if yaw is None or not all(math.isfinite(value) for value in (x, y)):
            return
        self._last_odom_stamp_ns = stamp_ns
        self._pose = VehiclePose(x, y, yaw, time.monotonic())

    def _on_front_range(self, message: Range) -> None:
        value = float(message.range)
        if math.isfinite(value) and value > 0.0:
            self._latest_range = (value, time.monotonic())

    def _on_front_blocked(self, message: Bool) -> None:
        blocked = bool(message.data)
        rising = blocked and not self._front_blocked
        self._front_blocked = blocked
        if not rising or self._pose is None or self._latest_range is None:
            return
        timeout = float(self.get_parameter("obstacle_range_sync_s").value)
        if time.monotonic() - self._latest_range[1] > timeout:
            return
        try:
            polygon = front_obstacle_polygon(
                Pose2D(self._pose.x, self._pose.y, self._pose.yaw),
                self._latest_range[0],
                float(self.get_parameter("obstacle_sensor_forward_offset_m").value),
                float(self.get_parameter("obstacle_width_m").value),
                float(self.get_parameter("obstacle_depth_m").value),
            )
        except ValueError:
            return
        self._pending_obstacle = (polygon, time.monotonic())

    def _on_obstacle_polygon(self, message: PolygonStamped) -> None:
        if message.header.frame_id and message.header.frame_id != "map":
            return
        try:
            polygon = valid_polygon(
                (point.x, point.y) for point in message.polygon.points
            )
        except ValueError:
            return
        self._pending_obstacle = (polygon, time.monotonic())

    def _promote_pending_obstacle(self) -> bool:
        if self._pending_obstacle is None:
            return False
        polygon, observed_at = self._pending_obstacle
        timeout = float(self.get_parameter("obstacle_observation_timeout_s").value)
        if time.monotonic() - observed_at > timeout:
            self._pending_obstacle = None
            return False
        merged = merge_obstacle(
            self._dynamic_obstacles,
            polygon,
            float(self.get_parameter("obstacle_merge_distance_m").value),
        )
        limit = int(self.get_parameter("max_dynamic_obstacles").value)
        self._dynamic_obstacles = merged[-limit:]
        self._dynamic_obstacles_updated_at = time.monotonic()
        self._pending_obstacle = None
        return True

    def _expire_dynamic_obstacles(self, now: float) -> None:
        retention = float(self.get_parameter("dynamic_obstacle_retention_s").value)
        if self._dynamic_obstacles and obstacle_memory_expired(
            self._dynamic_obstacles_updated_at, now, retention
        ):
            self._dynamic_obstacles.clear()
            self._dynamic_obstacles_updated_at = None
            self._publish_text("dynamic_obstacles_expired")

    def _on_event_candidate(self, message: EventCandidate) -> None:
        if message.header.frame_id != "map" or not self._mission.running:
            return
        candidate = event_data(message)
        if candidate is not None and self._events.offer(candidate):
            self._detail = f"发现待播报事件：{candidate.event_type}"
            self._publish_state()

    def _service_event_reporting(self, now: float) -> None:
        event = self._events.current()
        if (
            event is None
            or self._report_goal_future is not None
            or self._report_result_future is not None
        ):
            return
        if self._state in {
            MissionState.PLANNING,
            MissionState.REPLANNING,
            MissionState.RECOVERING_OBSTACLE,
            MissionState.WAIT_ROUTE_ACK,
            MissionState.PAUSED,
            MissionState.FAULT,
        }:
            return
        requires_hold = bool(self.get_parameter("report_requires_hold").value)
        if requires_hold and not self._motion_held:
            if not self._report_hold_requested:
                self._report_hold_requested = True
                self._report_wait_started = now
                self._request_hold("report", True, "event report in progress")
            timeout = float(self.get_parameter("report_server_timeout_s").value)
            if self._report_wait_started and now - self._report_wait_started > timeout:
                self._report_failed("车辆未能为播报驻停")
            else:
                self._set_state(MissionState.WAIT_REPORTER, "等待车辆驻停后播报")
            return
        if not self._reporter.server_is_ready():
            if self._report_wait_started is None:
                self._report_wait_started = now
            timeout = float(self.get_parameter("report_server_timeout_s").value)
            if now - self._report_wait_started > timeout:
                self._report_failed("播报服务未就绪")
            else:
                self._set_state(MissionState.WAIT_REPORTER, "等待播报服务")
            return
        goal = ReportEvent.Goal()
        goal.event = event_message(event, self.get_clock().now().to_msg())
        self._report_generation += 1
        generation = self._report_generation
        self._report_goal_future = self._reporter.send_goal_async(goal)
        self._report_goal_future.add_done_callback(
            lambda future: self._on_report_goal(generation, future)
        )
        self._set_state(MissionState.REPORTING, f"正在播报：{event.event_type}")

    def _on_report_goal(self, generation: int, future: Any) -> None:
        if generation != self._report_generation:
            try:
                stale_handle = future.result()
                if stale_handle.accepted:
                    stale_handle.cancel_goal_async()
            except Exception:
                pass
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self._report_goal_future = None
            self._report_failed(f"播报请求失败：{error}")
            return
        self._report_goal_future = None
        if not goal_handle.accepted:
            self._report_failed("播报请求被拒绝")
            return
        self._report_goal_handle = goal_handle
        self._report_result_future = goal_handle.get_result_async()
        self._report_result_future.add_done_callback(
            lambda result: self._on_report_result(generation, result)
        )

    def _on_report_result(self, generation: int, future: Any) -> None:
        if generation != self._report_generation:
            return
        self._report_result_future = None
        self._report_goal_handle = None
        try:
            result = future.result().result
        except Exception as error:
            self._report_failed(f"播报结果失败：{error}")
            return
        if not result.success:
            self._report_failed(result.message or "播报失败")
            return
        event = self._events.mark_reported()
        self._reported_publisher.publish(
            event_message(event, self.get_clock().now().to_msg())
        )
        held_for_report = self._report_hold_requested
        self._report_wait_started = None
        self._report_hold_requested = False
        if held_for_report:
            self._request_hold("report", False, "event report completed")
        if self._mission.finished:
            self._set_state(MissionState.COMPLETE, "事件播报完成，任务已结束")
        elif self._mission.paused:
            self._set_state(MissionState.PAUSED, "事件播报完成，任务保持暂停")
        elif self._active_route is not None:
            self._set_state(MissionState.EXECUTING, "事件播报完成，继续当前路线")
        elif self._mission.running and not self._mission.paused:
            self._schedule_plan(False)
        else:
            self._set_state(MissionState.IDLE, "事件播报完成")

    def _report_failed(self, message: str) -> None:
        event = self._events.mark_attempt()
        self._report_goal_future = None
        self._report_goal_handle = None
        self._report_result_future = None
        self._report_wait_started = time.monotonic()
        maximum = int(self.get_parameter("max_report_retries").value)
        if event.attempts > maximum:
            self._fault(message)
        else:
            self._set_state(MissionState.WAIT_REPORTER, f"{message}；准备重试")

    def _cancel_report(self) -> None:
        self._report_generation += 1
        if self._report_goal_handle is not None:
            self._report_goal_handle.cancel_goal_async()
        self._report_goal_future = None
        self._report_goal_handle = None
        self._report_result_future = None
        self._report_wait_started = None
        if self._report_hold_requested:
            self._request_hold("report", False, "event report canceled")
        self._report_hold_requested = False
