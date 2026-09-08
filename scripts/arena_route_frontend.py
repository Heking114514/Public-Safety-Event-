#!/usr/bin/env python3
"""Tk frontend for the C++ arena_path_planner ROS 2 service."""

from __future__ import annotations

import math
import os
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from typing import Any, Optional, Sequence

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from arena_path_planner.srv import PlanArenaPath
from geometry_msgs.msg import Point, Point32, Polygon, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt64
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from arena_view import ArenaView
from mission_execution import (
    AutoReplanBackoff,
    LatestRequestQueue,
    MissionExecutionState,
    PLANNER_REQUEST_TIMEOUT_SECONDS,
    RouteFailureCooldown,
    RouteExecution,
    valid_odometry_stamp,
)


SERVICE_NAME = "/arena_path_planner/plan"
ODOMETRY_TOPIC = "/odometry/fused"
ODOMETRY_TIMEOUT_S = 0.5
ROUTE_ACK_TIMEOUT_MS = 1500
ROUTE_FAILURE_COOLDOWN_S = 5.0
ROUTE_ACK_TOPIC = "/waypoint_navigation/route_ack"
NAVIGATION_STATUS_TOPIC = "/waypoint_navigation/status"


def normalize_angle(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def quaternion_to_yaw(quaternion: Any) -> Optional[float]:
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        return None
    squared_norm = sum(value * value for value in values)
    if squared_norm <= 1.0e-12 or not math.isfinite(squared_norm):
        return None
    x, y, z, w = (value / math.sqrt(squared_norm) for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PlannerClient(Node):
    """Serializes service calls and replaces pending work with the latest request."""

    def __init__(
        self,
        responses: queue.Queue[tuple[int, Any]],
        odometry_updates: queue.Queue[Optional[tuple[float, float, float, float]]],
        navigation_events: queue.Queue[tuple[str, Any, float]],
    ) -> None:
        super().__init__("arena_route_frontend")
        self.client = self.create_client(PlanArenaPath, SERVICE_NAME)
        self.responses = responses
        self.odometry_updates = odometry_updates
        self.navigation_events = navigation_events
        self.lock = threading.Lock()
        self.request_queue = LatestRequestQueue()
        self.request_timeout_timer: Optional[threading.Timer] = None
        self.request_future: Optional[Any] = None
        self.requests_closed = False
        self.last_odometry_stamp_ns = 0
        self.odom_subscription = self.create_subscription(
            Odometry, ODOMETRY_TOPIC, self._handle_odometry, qos_profile_sensor_data
        )
        self.route_ack_subscription = self.create_subscription(
            UInt64,
            ROUTE_ACK_TOPIC,
            self._handle_route_ack,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.navigation_status_subscription = self.create_subscription(
            String,
            NAVIGATION_STATUS_TOPIC,
            self._handle_navigation_status,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def _handle_route_ack(self, message: UInt64) -> None:
        self._put_navigation_event("ack", int(message.data))

    def _handle_navigation_status(self, message: String) -> None:
        self._put_navigation_event("status", str(message.data))

    def _put_navigation_event(self, kind: str, value: Any) -> None:
        event = (kind, value, time.monotonic())
        try:
            self.navigation_events.put_nowait(event)
        except queue.Full:
            try:
                self.navigation_events.get_nowait()
            except queue.Empty:
                return
            self.navigation_events.put_nowait(event)

    def _handle_odometry(self, message: Odometry) -> None:
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        z = float(message.pose.pose.position.z)
        orientation = message.pose.pose.orientation
        current_time_ns = self.get_clock().now().nanoseconds
        stamp_ns = valid_odometry_stamp(
            message.header.frame_id,
            message.child_frame_id,
            (x, y, z, orientation.x, orientation.y, orientation.z, orientation.w),
            int(message.header.stamp.sec),
            int(message.header.stamp.nanosec),
            current_time_ns,
            self.last_odometry_stamp_ns,
            ODOMETRY_TIMEOUT_S,
        )
        if stamp_ns is None:
            return
        yaw = quaternion_to_yaw(message.pose.pose.orientation)
        if yaw is None:
            return
        self.last_odometry_stamp_ns = stamp_ns
        update = (x, y, yaw, time.monotonic())
        try:
            self.odometry_updates.put_nowait(update)
        except queue.Full:
            try:
                self.odometry_updates.get_nowait()
            except queue.Empty:
                pass
            self.odometry_updates.put_nowait(update)

    def service_ready(self) -> bool:
        return self.client.service_is_ready()

    @property
    def in_flight(self) -> bool:
        with self.lock:
            return self.request_queue.in_flight

    def request(
        self, generation: int, start: Pose, mode: str, activate: bool,
        obstacles: list[tuple[float, float, float, float]],
        targets: list[tuple[str, float, float]],
        covered_edges: list[str],
        remaining_visits: list[str],
    ) -> None:
        item = (
            generation, start, mode, activate, obstacles, targets,
            covered_edges, remaining_visits,
        )
        if not self.client.service_is_ready():
            self.responses.put((generation, RuntimeError("规划后端未启动")))
            return
        with self.lock:
            if self.requests_closed:
                return
            dispatch = self.request_queue.submit(item)
        if dispatch is not None:
            self._send(*dispatch)

    def _send(self, token: int, item: tuple[Any, ...]) -> None:
        with self.lock:
            if self.requests_closed or self.request_queue.active_token != token:
                return
        (
            generation, start, mode, activate, obstacles, targets,
            covered_edges, remaining_visits,
        ) = item
        request = PlanArenaPath.Request()
        request.start = start
        request.mode = mode
        request.activate_navigation = activate
        request.covered_edges = covered_edges
        request.remaining_visits = remaining_visits
        for label, x, y in targets:
            request.targets.append(Point(x=x, y=y))
            request.labels.append(label)
        for minimum_x, minimum_y, maximum_x, maximum_y in obstacles:
            polygon = Polygon()
            polygon.points = [
                Point32(x=float(minimum_x), y=float(minimum_y)),
                Point32(x=float(maximum_x), y=float(minimum_y)),
                Point32(x=float(maximum_x), y=float(maximum_y)),
                Point32(x=float(minimum_x), y=float(maximum_y)),
            ]
            request.dynamic_obstacles.append(polygon)
        try:
            future = self.client.call_async(request)
        except Exception as exception:
            self._finish_failed_send(token, generation, exception)
            return
        timeout_timer = threading.Timer(
            PLANNER_REQUEST_TIMEOUT_SECONDS,
            self._request_timed_out,
            args=(token, generation, future),
        )
        timeout_timer.daemon = True
        with self.lock:
            if self.requests_closed or self.request_queue.active_token != token:
                cancel_immediately = True
            else:
                self.request_future = future
                self.request_timeout_timer = timeout_timer
                cancel_immediately = False
        if cancel_immediately:
            try:
                future.cancel()
            except Exception:
                pass
            return
        future.add_done_callback(
            lambda completed: self._complete_request(token, generation, completed)
        )
        timeout_timer.start()

    def _claim_request_completion(
        self, token: int
    ) -> Optional[tuple[Optional[threading.Timer], Optional[Any], Optional[tuple[int, Any]]]]:
        with self.lock:
            accepted, next_dispatch = self.request_queue.complete(token)
            if not accepted:
                return None
            timeout_timer = self.request_timeout_timer
            future = self.request_future
            self.request_timeout_timer = None
            self.request_future = None
        return timeout_timer, future, next_dispatch

    def _continue_with_latest(self, next_dispatch: Optional[tuple[int, Any]]) -> None:
        if next_dispatch is not None:
            self._send(*next_dispatch)

    def _finish_failed_send(
        self, token: int, generation: int, exception: Exception
    ) -> None:
        claimed = self._claim_request_completion(token)
        if claimed is None:
            return
        timeout_timer, _future, next_dispatch = claimed
        if timeout_timer is not None:
            timeout_timer.cancel()
        self.responses.put((generation, exception))
        self._continue_with_latest(next_dispatch)

    def _complete_request(self, token: int, generation: int, future: Any) -> None:
        claimed = self._claim_request_completion(token)
        if claimed is None:
            return
        timeout_timer, _active_future, next_dispatch = claimed
        if timeout_timer is not None:
            timeout_timer.cancel()
        try:
            self.responses.put((generation, future.result()))
        except Exception as exception:  # rclpy propagates transport failures here.
            self.responses.put((generation, exception))
        self._continue_with_latest(next_dispatch)

    def _request_timed_out(self, token: int, generation: int, future: Any) -> None:
        claimed = self._claim_request_completion(token)
        if claimed is None:
            return
        _timeout_timer, _active_future, next_dispatch = claimed
        try:
            future.cancel()
        except Exception:
            pass
        self.responses.put(
            (
                generation,
                TimeoutError(
                    f"规划服务 {PLANNER_REQUEST_TIMEOUT_SECONDS:.0f} 秒未响应"
                ),
            )
        )
        self._continue_with_latest(next_dispatch)

    def cancel_requests(self) -> None:
        with self.lock:
            self.requests_closed = True
            self.request_queue.cancel()
            timeout_timer = self.request_timeout_timer
            future = self.request_future
            self.request_timeout_timer = None
            self.request_future = None
        if timeout_timer is not None:
            timeout_timer.cancel()
        if future is not None:
            try:
                future.cancel()
            except Exception:
                pass


class ArenaFrontend:
    def __init__(self, root: tk.Tk, config: dict[str, Any]) -> None:
        self.root = root
        self.config = config
        self.arena = config["arena"]
        self.start_config = config["start"]
        self.tasks = list(config["tasks"].items())
        self.width_m = float(self.arena["width_m"])
        self.height_m = float(self.arena["height_m"])
        self.vehicle_length_m = float(self.arena.get("vehicle_length_m", 0.217))
        self.vehicle_width_m = float(self.arena.get("vehicle_width_m", 0.210))
        self.safety_margin_m = float(self.arena.get("safety_margin_m", 0.040))
        self.vehicle_x = float(self.start_config["position_m"][0])
        self.vehicle_y = float(self.start_config["position_m"][1])
        self.vehicle_yaw = math.radians(float(self.start_config["heading_deg"]))
        self.default_pose = (self.vehicle_x, self.vehicle_y, self.vehicle_yaw)
        self.current_odometry: Optional[tuple[float, float, float, float]] = None
        self.odom_ready = False
        self.odom_trace: list[tuple[float, float]] = []
        self.last_odom_draw = 0.0
        self.layer_start = self.default_pose
        self.route: list[tuple[float, float, float]] = []
        self.layer_routes: dict[int, list[tuple[float, float, float]]] = {}
        self.layer_display_segments: dict[int, list[list[tuple[float, float, float]]]] = {}
        self.layer_mode = 1
        self.visible_layer = 1
        self.dynamic_obstacles: list[tuple[float, float, float, float]] = []
        self.remaining_labels = [str(label) for label, _position in self.tasks]
        self.planned_deferred_labels: list[str] = []
        self.retry_pending = False
        self.retry_used = False
        self.preview_covered_edges: list[str] = []
        self.covered_edges: list[str] = []
        graph_edges = config.get("inspection_graph", {}).get("edges", [])
        road_labels = [str(edge[2]) for edge in graph_edges if len(edge) >= 3]
        self.mission = MissionExecutionState(
            [str(label) for label, _position in self.tasks],
            list(config.get("tunnels", {}).keys()),
            road_labels,
        )
        self.mission_running = False
        self.auto_waiting = False
        self.auto_wait_mode = "layered"
        self.auto_replan_backoff = AutoReplanBackoff()
        self.generation = 0
        self.applied_generation = -1
        self.request_count = 0
        self.activation_requests: dict[int, tuple[str, float]] = {}
        self.mission_probe_requests: dict[int, str] = {}
        self.awaiting_route_ack_stamp: Optional[int] = None
        self.received_route_acks: dict[int, float] = {}
        self.expired_route_ids: set[int] = set()
        self.recent_navigation_statuses: list[tuple[str, float]] = []
        self.replan_after_id: Optional[str] = None
        self.ack_watchdog_after_id: Optional[str] = None
        self.route_failure_cooldown = RouteFailureCooldown(
            ROUTE_FAILURE_COOLDOWN_S
        )
        self.last_drag_request = 0.0
        self.dragging = False
        self.playing = False
        self.closing = False
        self.play_distance = 0.0
        self.play_last_time = time.monotonic()
        self.responses: queue.Queue[tuple[int, Any]] = queue.Queue()
        self.odometry_updates: queue.Queue[
            Optional[tuple[float, float, float, float]]
        ] = queue.Queue(maxsize=1)
        self.navigation_events: queue.Queue[tuple[str, Any, float]] = queue.Queue(
            maxsize=64
        )

        rclpy.init(args=[])
        self.node = PlannerClient(
            self.responses, self.odometry_updates, self.navigation_events
        )
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.mode_var = tk.StringVar(value="simulation")
        self.order_var = tk.StringVar(value="shortest")
        self.status_var = tk.StringVar(value="等待规划后端")
        self.metrics_var = tk.StringVar(value="路径 -- m    规划 -- ms    重规划 0")
        self.view = ArenaView(self.root, self, self.width_m, self.height_m)
        self.root.after(50, self._poll)
        self.root.after(200, self._request_initial_plan)

    def _to_canvas(self, x: float, y: float) -> tuple[float, float]:
        return self.view.to_canvas(x, y)

    def _to_world(self, x: float, y: float) -> tuple[float, float]:
        return self.view.to_world(x, y)

    def draw(self) -> None:
        self.view.draw()

    def _pose(self, values: Optional[tuple[float, float, float]] = None) -> Pose:
        x, y, yaw = values or (self.vehicle_x, self.vehicle_y, self.vehicle_yaw)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _odometry_is_ready(self, now: Optional[float] = None) -> bool:
        if self.current_odometry is None:
            return False
        current_time = time.monotonic() if now is None else now
        return current_time - self.current_odometry[3] <= ODOMETRY_TIMEOUT_S

    def _odometry_to_arena(
        self, odom_x: float, odom_y: float, odom_yaw: float
    ) -> tuple[float, float, float]:
        start_x, start_y, start_yaw = self.default_pose
        cosine = math.cos(start_yaw)
        sine = math.sin(start_yaw)
        return (
            start_x + cosine * odom_x - sine * odom_y,
            start_y + sine * odom_x + cosine * odom_y,
            normalize_angle(start_yaw + odom_yaw),
        )

    def _apply_current_odometry(self, force_draw: bool = False) -> None:
        if self.current_odometry is None or self.mode_var.get() != "vehicle":
            return
        self.vehicle_x, self.vehicle_y, self.vehicle_yaw = self._odometry_to_arena(
            self.current_odometry[0], self.current_odometry[1], self.current_odometry[2]
        )
        if not self.odom_trace or math.hypot(
            self.vehicle_x - self.odom_trace[-1][0],
            self.vehicle_y - self.odom_trace[-1][1],
        ) >= 0.01:
            self.odom_trace.append((self.vehicle_x, self.vehicle_y))
            if len(self.odom_trace) > 10000:
                self.odom_trace = self.odom_trace[-10000:]
        now = time.monotonic()
        if force_draw or now - self.last_odom_draw >= 0.1:
            self.last_odom_draw = now
            self.draw()

    def _consume_odometry(self) -> None:
        received = False
        latest: Optional[tuple[float, float, float, float]] = None
        try:
            while True:
                latest = self.odometry_updates.get_nowait()
                received = True
        except queue.Empty:
            pass
        if received:
            self.current_odometry = latest
            self._apply_current_odometry()

        ready = self._odometry_is_ready()
        if ready != self.odom_ready:
            self.odom_ready = ready
            if self.mode_var.get() == "vehicle":
                self.view.status_label.configure(
                    text="ODOM READY" if ready else "NO ODOM",
                    style="OdomReady.Status.TLabel" if ready else "NoOdom.Status.TLabel",
                )
                self.draw()

    def request_plan(self, activate: bool = False) -> None:
        if self.mission_running:
            self.status_var.set("自动任务运行中，忽略预览规划请求")
            return
        if activate and self.mode_var.get() != "vehicle":
            self.status_var.set("模拟模式禁止发布导航命令")
            return
        mode = {1: "layer1", 2: "layer2", 3: "layer3"}.get(
            self.layer_mode, "layer1"
        )
        self.generation += 1
        self.request_count += 1
        if activate:
            self.activation_requests[self.generation] = (mode, time.monotonic())
        self.status_var.set("正在规划..." if self.node.service_ready() else "规划后端未就绪")
        self.node.request(
            self.generation, self._pose(self.layer_start), mode, activate,
            list(self.dynamic_obstacles),
            [
                (str(label), float(position[0]), float(position[1]))
                for label, position in self.tasks
                if str(label) in self.remaining_labels
            ],
            list(self.preview_covered_edges),
            [],
        )

    def _request_initial_plan(self) -> None:
        if self.mission_running:
            self.status_var.set("自动任务运行中，不能替换当前路线")
            return
        self.layer_mode = 1
        self.visible_layer = 1
        self.layer_routes.clear()
        self.layer_display_segments.clear()
        self.preview_covered_edges.clear()
        self.layer_start = (self.vehicle_x, self.vehicle_y, self.vehicle_yaw)
        self.view.next_layer_button.configure(text="进入第二阶段", state=tk.DISABLED)
        self.request_plan()

    def advance_layer(self) -> None:
        """Start the next inspection phase only after operator confirmation."""
        if self.mission_running or self.layer_mode >= 3 or self.node.in_flight:
            return
        current_route = self.layer_routes.get(self.layer_mode, [])
        if len(current_route) < 2:
            self.status_var.set("当前阶段尚未规划完成")
            return
        self.layer_start = current_route[-1]
        self.layer_mode += 1
        self.visible_layer = self.layer_mode
        self.view.next_layer_button.configure(state=tk.DISABLED)
        self.status_var.set(f"正在规划第 {self.layer_mode} 阶段...")
        self.request_plan()

    def publish_navigation(self) -> None:
        if self.mission_running:
            self.status_var.set("自动任务已经在运行")
            return
        if self.mode_var.get() != "vehicle":
            self.status_var.set("当前是模拟模式：请先切换到 vehicle 再开始导航")
            return
        if not self.node.service_ready():
            self.status_var.set("无法开始导航：规划后端未就绪")
            return
        if not self._odometry_is_ready():
            self.status_var.set("无法开始导航：融合里程计未就绪或已超时")
            return
        if not self.mission_running:
            self.mission.reset()
            self.covered_edges.clear()
            self.expired_route_ids.clear()
            self.route_failure_cooldown.clear()
            self.remaining_labels = [str(label) for label, _position in self.tasks]
            self.planned_deferred_labels.clear()
        self.mission_running = True
        self.auto_waiting = False
        self.auto_replan_backoff.reset()
        self._set_mission_controls(True)
        self._request_mission_plan("layered")

    def _set_mission_controls(self, running: bool) -> None:
        self.view.publish_button.configure(
            state=tk.DISABLED if running else tk.NORMAL,
            text="自动任务运行中" if running else "开始导航",
        )
        self.view.mode_box.configure(state=tk.DISABLED if running else "readonly")
        self.view.order_box.configure(state=tk.DISABLED if running else "readonly")
        for control in (
            self.view.start_button,
            self.view.next_layer_button,
            self.view.reset_button,
            self.view.clear_button,
        ):
            control.configure(state=tk.DISABLED if running else tk.NORMAL)

    def _request_mission_plan(self, mode: str, probe: bool = False) -> None:
        if self.closing or not self.mission_running:
            return
        if not self.node.service_ready() or self.node.in_flight:
            if probe or self.auto_waiting:
                self._schedule_auto_wait_probe(mode, "等待规划后端后自动重试")
            else:
                self._schedule_mission_replan(mode, "等待规划后端后自动重试")
            return
        if not self._odometry_is_ready():
            if probe or self.auto_waiting:
                self._schedule_auto_wait_probe(mode, "等待新鲜里程计后自动续规划")
            else:
                self._schedule_mission_replan(mode, "等待新鲜里程计后自动续规划")
            return

        remaining_tasks = set(self.mission.remaining_tasks())
        targets = [
            (str(label), float(position[0]), float(position[1]))
            for label, position in self.tasks
            if str(label) in remaining_tasks
        ] if mode in {"layered", "layer1"} else []
        self.generation += 1
        self.request_count += 1
        if probe:
            self.mission_probe_requests[self.generation] = mode
        else:
            self.activation_requests[self.generation] = (mode, time.monotonic())
        self.status_var.set(
            "正在规划完整任务..." if mode == "layered" else f"正在自动续规划 {mode}..."
        )
        self.node.request(
            self.generation, self._pose(), mode, not probe,
            list(self.dynamic_obstacles),
            targets,
            self.mission.covered_roads(),
            self.mission.remaining_tasks() + self.mission.remaining_tunnels(),
        )

    def _schedule_mission_replan(
        self, mode: str, reason: str, delay_ms: int = 750
    ) -> None:
        if self.closing or not self.mission_running:
            return
        self.status_var.set(reason)
        if self.replan_after_id is not None:
            return

        def retry() -> None:
            self.replan_after_id = None
            self._request_mission_plan(mode)

        self.replan_after_id = self.root.after(delay_ms, retry)

    def _schedule_auto_wait_probe(self, mode: str, reason: str) -> None:
        if self.closing or not self.mission_running:
            return
        self.auto_waiting = True
        self.auto_wait_mode = mode
        delay = self.auto_replan_backoff.next_delay()
        self.status_var.set(f"AUTO_WAIT_REPLAN：{reason}，{delay:.1f} 秒后自动重规划")
        if self.replan_after_id is not None:
            return

        def probe() -> None:
            self.replan_after_id = None
            self._request_mission_plan(self.auto_wait_mode, probe=True)

        self.replan_after_id = self.root.after(int(delay * 1000), probe)

    def _finish_mission(self) -> None:
        self.mission_running = False
        self.auto_waiting = False
        self._cancel_ack_watchdog()
        self.awaiting_route_ack_stamp = None
        self.planned_deferred_labels.clear()
        self.view.remaining_var.set("0 个延迟目标")
        self._set_mission_controls(False)
        self.view.publish_button.configure(text="重新执行")
        self.view.next_layer_button.configure(state=tk.DISABLED)
        self.status_var.set("全部任务已实际执行完成")

    def _read_route(self, response: Any) -> None:
        self.route = []
        for pose in response.arena_path.poses:
            orientation = pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
            )
            self.route.append((pose.pose.position.x, pose.pose.position.y, yaw))

    def _handle_preview_response(self, response: Any) -> None:
        incomplete = bool(response.deferred_targets) or not bool(
            response.all_targets_reached
        )
        self.layer_routes[self.layer_mode] = list(self.route)
        self.layer_display_segments[self.layer_mode] = self._display_segments(
            self.layer_mode, self.route, list(response.visit_order)
        )
        self.visible_layer = self.layer_mode
        self.planned_deferred_labels = list(response.deferred_targets)
        for edge in response.covered_edges:
            if edge not in self.preview_covered_edges:
                self.preview_covered_edges.append(edge)
        self.status_var.set(
            f"第 {self.layer_mode} 阶段可达部分已规划，实车将自动续规划"
            if incomplete
            else f"第 {self.layer_mode} 阶段规划完成"
        )
        if self.layer_mode < 3:
            self.view.next_layer_button.configure(
                text=f"进入第 {self.layer_mode + 1} 阶段",
                state=tk.DISABLED if incomplete else tk.NORMAL,
            )
        else:
            self.view.next_layer_button.configure(text="三阶段已完成", state=tk.DISABLED)

    def _accept_route_ack(self, route_id: int, received_at: float) -> None:
        if route_id in self.expired_route_ids:
            return
        if not self.mission.acknowledge(route_id, received_at):
            return
        self._cancel_ack_watchdog()
        self.received_route_acks.pop(route_id, None)
        self.awaiting_route_ack_stamp = None
        active = self.mission.active
        deferred_count = len(active.deferred_targets) if active is not None else 0
        self.status_var.set(
            "阶段路线已确认并启动"
            + (f"，后续仍有 {deferred_count} 项待重规划" if deferred_count else "")
        )
        for status, status_at in self.recent_navigation_statuses:
            if active is not None and status_at >= active.activation_requested_at:
                self._observe_active_route_status(status, status_at)

    def _observe_active_route_status(self, status: str, received_at: float) -> None:
        interrupted = self.mission.abort_for_status(status, received_at)
        if interrupted is not None:
            if status.startswith("FAULT_"):
                self.route_failure_cooldown.record(
                    status,
                    interrupted.mode,
                    interrupted.route_signature,
                    received_at,
                )
                reason = f"导航报告 {status}，本轮不记进度并自动重规划"
            else:
                reason = f"导航进入 {status}，当前路线已丢失并自动重新下发"
            self._schedule_auto_wait_probe(interrupted.mode, reason)
            return
        execution = self.mission.observe_status(status, received_at)
        if execution is None:
            return
        self.covered_edges = self.mission.covered_roads()
        self.remaining_labels = self.mission.remaining_tasks()
        self.planned_deferred_labels = list(execution.deferred_targets)
        if self.mission.mission_complete(execution):
            self._finish_mission()
            return
        mode = self.mission.continuation_mode(execution) or "layer3"
        if execution.new_progress_count == 0:
            self._schedule_auto_wait_probe(
                mode, "上一条路线没有新增任务，不再重复执行原路线"
            )
            return
        else:
            self.auto_replan_backoff.reset()
            self.route_failure_cooldown.clear()
        remaining_count = (
            len(self.mission.remaining_tasks())
            + len(self.mission.remaining_tunnels())
            + len(self.mission.remaining_roads())
        )
        self.view.remaining_var.set(f"{remaining_count} 项待自动续规划")
        self._schedule_mission_replan(
            mode, "阶段路线已跑完，正在自动规划下一段", delay_ms=100
        )

    def _handle_navigation_event(
        self, kind: str, value: Any, received_at: float
    ) -> None:
        if kind == "ack":
            route_id = int(value)
            self.received_route_acks[route_id] = received_at
            if len(self.received_route_acks) > 32:
                oldest = min(self.received_route_acks, key=self.received_route_acks.get)
                self.received_route_acks.pop(oldest, None)
            self._accept_route_ack(route_id, received_at)
            return
        status = str(value)
        self.recent_navigation_statuses.append((status, received_at))
        self.recent_navigation_statuses = self.recent_navigation_statuses[-64:]
        self._observe_active_route_status(status, received_at)
        if (
            status != "GOAL_REACHED"
            and self.mission_running
            and self.mission.active is not None
        ):
            self.status_var.set(f"阶段导航状态：{status}")

    def _handle_plan_response(self, generation: int, response: Any) -> None:
        activation = self.activation_requests.pop(generation, None)
        activation_mode = activation[0] if activation is not None else None
        activation_requested_at = activation[1] if activation is not None else 0.0
        probe_mode = self.mission_probe_requests.pop(generation, None)
        mission_mode = activation_mode or probe_mode
        if generation < self.applied_generation or generation < self.generation:
            return
        self.applied_generation = generation
        if isinstance(response, Exception):
            if mission_mode is not None and self.mission_running:
                self._schedule_auto_wait_probe(
                    mission_mode, f"服务调用失败，自动重试：{response}"
                )
            else:
                self.status_var.set(f"服务调用失败：{response}")
                self.view.publish_button.configure(state=tk.NORMAL, text="开始导航")
            return
        if not response.success:
            if mission_mode is not None and self.mission_running:
                self._schedule_auto_wait_probe(
                    mission_mode, f"暂无安全路线，自动重试：{response.message}"
                )
            else:
                self.route = []
                self.status_var.set(f"规划失败：{response.message}")
                self.view.publish_button.configure(state=tk.NORMAL, text="开始导航")
                self.draw()
            return

        self._read_route(response)
        self.planned_deferred_labels = list(response.deferred_targets)
        self.view.remaining_var.set(
            f"{len(self.planned_deferred_labels)} 个延迟目标"
        )
        self.metrics_var.set(
            f"路径 {response.length:.2f} m    规划 {response.planning_time_ms:.0f} ms\n"
            f"点数 {len(self.route)}    重规划 {self.request_count}\n"
            f"延迟目标 {', '.join(response.deferred_targets) or '无'}"
        )
        self.play_distance = 0.0
        self.draw()
        if mission_mode is None:
            self._handle_preview_response(response)
            return

        stamp = response.navigation_path.header.stamp
        route_id = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        route_has_motion = any(
            math.hypot(right[0] - left[0], right[1] - left[1]) > 1.0e-6
            for left, right in zip(self.route, self.route[1:])
        )
        candidate = RouteExecution(
            route_id=max(1, route_id),
            mode=mission_mode,
            visit_order=tuple(map(str, response.visit_order)),
            covered_edges=tuple(map(str, response.covered_edges)),
            deferred_targets=tuple(map(str, response.deferred_targets)),
            all_targets_reached=bool(response.all_targets_reached),
            activation_requested_at=activation_requested_at,
            route_signature=tuple(
                (round(point[0] * 20), round(point[1] * 20))
                for point in self.route
            ),
        )
        if probe_mode is not None:
            if (
                self.mission.probe_may_activate(candidate, route_has_motion)
                and not self.route_failure_cooldown.blocked(
                    candidate.mode, candidate.route_signature, time.monotonic()
                )
            ):
                self.auto_waiting = False
                self._schedule_mission_replan(
                    probe_mode, "发现新的可执行进展，正在下发路线", delay_ms=50
                )
            else:
                self._schedule_auto_wait_probe(
                    probe_mode, "仍无新的可执行进展，保持自动等待"
                )
            return

        if not route_has_motion or route_id <= 0:
            no_work_left = (
                activation_mode in {"layered", "layer3"}
                and
                response.all_targets_reached
                and not response.deferred_targets
                and not self.mission.remaining_tasks()
                and not self.mission.remaining_tunnels()
                and not self.mission.remaining_roads()
            )
            if no_work_left:
                self._finish_mission()
            else:
                next_mode = self.mission.continuation_mode(candidate)
                if next_mode is not None and next_mode != activation_mode:
                    self._schedule_mission_replan(
                        next_mode, "当前阶段无需行驶，自动进入下一阶段", delay_ms=50
                    )
                else:
                    self._schedule_auto_wait_probe(
                        activation_mode, "本轮没有可执行路线，继续自动重试"
                    )
            return

        execution = candidate
        execution.route_id = route_id
        if not bool(response.navigation_activated):
            self._schedule_auto_wait_probe(
                activation_mode,
                "规划结果没有新的可执行任务，继续自动重规划",
            )
            return
        if self.mission.pending is not None or self.mission.active is not None:
            self.status_var.set("当前阶段路线仍在执行，忽略重复规划结果")
            return
        self.mission.begin_route(execution)
        self.awaiting_route_ack_stamp = route_id
        self._start_ack_watchdog(execution)
        self.status_var.set(
            "安全阶段路线已发布，等待导航确认"
            if execution.deferred_targets or not execution.all_targets_reached
            else "完整路线已发布，等待导航确认"
        )
        cached_ack = self.received_route_acks.pop(route_id, None)
        if cached_ack is not None:
            self._accept_route_ack(route_id, cached_ack)

    def _cancel_ack_watchdog(self) -> None:
        if self.ack_watchdog_after_id is None:
            return
        try:
            self.root.after_cancel(self.ack_watchdog_after_id)
        except tk.TclError:
            pass
        self.ack_watchdog_after_id = None

    def _start_ack_watchdog(self, execution: RouteExecution) -> None:
        self._cancel_ack_watchdog()

        def expired() -> None:
            self.ack_watchdog_after_id = None
            discarded = self.mission.discard_pending(execution.route_id)
            if discarded is None:
                return
            self.expired_route_ids.add(execution.route_id)
            if len(self.expired_route_ids) > 64:
                self.expired_route_ids.pop()
            self.awaiting_route_ack_stamp = None
            self._schedule_auto_wait_probe(
                execution.mode, "路线确认超时，忽略迟到确认并自动重新请求"
            )

        self.ack_watchdog_after_id = self.root.after(ROUTE_ACK_TIMEOUT_MS, expired)

    def _poll(self) -> None:
        if self.closing or not rclpy.ok():
            return
        self._consume_odometry()
        try:
            while True:
                generation, response = self.responses.get_nowait()
                self._handle_plan_response(generation, response)
        except queue.Empty:
            pass
        events: list[tuple[str, Any, float]] = []
        try:
            while True:
                events.append(self.navigation_events.get_nowait())
        except queue.Empty:
            pass
        for event in sorted(events, key=lambda item: item[2]):
            self._handle_navigation_event(*event)
        if (
            not self.node.service_ready()
            and not self.node.in_flight
            and not self.mission_running
        ):
            self.status_var.set("规划后端未启动")
        if self.playing:
            self._advance_playback()
        self.root.after(35, self._poll)

    def _mode_changed(self) -> None:
        if self.mission_running:
            self.mode_var.set("vehicle")
            self.status_var.set("自动任务尚未完成，不能切换模式")
            return
        simulation = self.mode_var.get() == "simulation"
        if simulation:
            self.view.status_label.configure(
                text="SIMULATION", style="Simulation.Status.TLabel"
            )
        else:
            self._apply_current_odometry(force_draw=True)
            ready = self._odometry_is_ready()
            self.odom_ready = ready
            self.view.status_label.configure(
                text="ODOM READY" if ready else "NO ODOM",
                style="OdomReady.Status.TLabel" if ready else "NoOdom.Status.TLabel",
            )
        self.view.play_button.configure(
            state=tk.NORMAL if simulation else tk.DISABLED
        )
        self.view.publish_button.configure(state=tk.NORMAL, text="开始导航")
        if not simulation:
            self.playing = False
            self.view.play_button.configure(text="播放")
        self.status_var.set(
            "模拟模式"
            if simulation
            else ("实车模式：可点击开始导航" if self.odom_ready else "实车模式：等待融合里程计")
        )
        self._request_initial_plan()

    def _display_segments(
        self,
        layer: int,
        route: list[tuple[float, float, float]],
        visit_order: list[str],
    ) -> list[list[tuple[float, float, float]]]:
        """Return only the newly inspected road portions for the purple layer.

        The route sent to navigation still contains A* connector paths. Those
        connectors are necessary for safe travel but are not new coverage and
        should not make the gap-fill overlay look like a full-map repaint.
        """
        if layer != 3 or len(route) < 2:
            return [route] if len(route) >= 2 else []
        graph = self.config.get("inspection_graph", {})
        nodes = graph.get("nodes", [])
        edges = {
            str(edge[2]): (nodes[int(edge[0])], nodes[int(edge[1])])
            for edge in graph.get("edges", [])
            if len(edge) >= 3 and int(edge[0]) < len(nodes) and int(edge[1]) < len(nodes)
        }
        selected = [edges[label] for label in visit_order if label in edges]
        if not selected:
            return [route]

        def segment_distance(point: tuple[float, float], edge: tuple[list[float], list[float]]) -> float:
            (ax, ay), (bx, by) = edge
            dx, dy = bx - ax, by - ay
            length_sq = dx * dx + dy * dy
            if length_sq <= 1.0e-12:
                return math.hypot(point[0] - ax, point[1] - ay)
            ratio = max(0.0, min(1.0, ((point[0] - ax) * dx + (point[1] - ay) * dy) / length_sq))
            return math.hypot(point[0] - (ax + ratio * dx), point[1] - (ay + ratio * dy))

        accepted: list[tuple[float, float, float]] = []
        segments: list[list[tuple[float, float, float]]] = []
        for left, right in zip(route, route[1:]):
            midpoint = ((left[0] + right[0]) * 0.5, (left[1] + right[1]) * 0.5)
            on_new_edge = min(segment_distance(midpoint, edge) for edge in selected) <= 0.10
            if on_new_edge:
                if not accepted:
                    accepted = [left]
                accepted.append(right)
            elif len(accepted) >= 2:
                segments.append(accepted)
                accepted = []
        if len(accepted) >= 2:
            segments.append(accepted)
        return segments or [route]

    def _drag_start(self, event: tk.Event) -> None:
        if event.state & 0x0004:
            return
        if self.mode_var.get() != "simulation":
            return
        cx, cy = self._to_canvas(self.vehicle_x, self.vehicle_y)
        self.dragging = math.hypot(event.x - cx, event.y - cy) <= 28.0
        if self.dragging:
            self.playing = False
            self.view.play_button.configure(text="播放")

    def _drag_motion(self, event: tk.Event) -> None:
        if not self.dragging:
            return
        x, y = self._to_world(event.x, event.y)
        self.vehicle_x = min(self.width_m - 1.0e-3, max(0.0, x))
        self.vehicle_y = min(self.height_m - 1.0e-3, max(0.0, y))
        self.draw()
        now = time.monotonic()
        if now - self.last_drag_request >= 0.15:
            self.last_drag_request = now
            self._request_initial_plan()

    def _drag_end(self, _event: tk.Event) -> None:
        if self.dragging:
            self.dragging = False
            self._request_initial_plan()

    def _rotate_motion(self, event: tk.Event) -> None:
        if self.mode_var.get() != "simulation":
            return
        cx, cy = self._to_canvas(self.vehicle_x, self.vehicle_y)
        self.vehicle_yaw = normalize_angle(-math.atan2(event.y - cy, event.x - cx))
        self.draw()
        now = time.monotonic()
        if now - self.last_drag_request >= 0.15:
            self.last_drag_request = now
            self._request_initial_plan()

    def _wheel_rotate(self, event: tk.Event) -> None:
        if self.mode_var.get() != "simulation":
            return
        self.vehicle_yaw = normalize_angle(self.vehicle_yaw + math.copysign(math.radians(5.0), event.delta))
        self.draw()
        self._request_initial_plan()

    def reset_vehicle(self) -> None:
        if self.mission_running:
            self.status_var.set("自动任务尚未完成，不能重置任务状态")
            return
        self.odom_trace.clear()
        if self.mode_var.get() == "vehicle" and self._odometry_is_ready():
            self._apply_current_odometry()
        else:
            self.vehicle_x, self.vehicle_y, self.vehicle_yaw = self.default_pose
        self.playing = False
        self.view.play_button.configure(text="播放")
        self.remaining_labels = [str(label) for label, _position in self.tasks]
        self.planned_deferred_labels = []
        self.retry_pending = False
        self.retry_used = False
        self.covered_edges = []
        self.layer_routes.clear()
        self.layer_display_segments.clear()
        self.layer_mode = 1
        self.visible_layer = 1
        self.layer_start = self.default_pose
        self.view.next_layer_button.configure(text="进入第二阶段", state=tk.DISABLED)
        self._request_initial_plan()
        self.draw()

    def _add_obstacle(self, event: tk.Event) -> None:
        if self.mission_running or self.mode_var.get() != "simulation":
            return
        x, y = self._to_world(event.x, event.y)
        half_width = 0.11
        self.dynamic_obstacles.append((
            max(0.0, x - half_width), max(0.0, y - half_width),
            min(self.width_m, x + half_width), min(self.height_m, y + half_width),
        ))
        self._request_initial_plan()
        self.draw()

    def _remove_obstacle(self, event: tk.Event) -> None:
        if (
            self.mission_running or self.mode_var.get() != "simulation"
            or not self.dynamic_obstacles
        ):
            return
        x, y = self._to_world(event.x, event.y)
        selected = min(
            range(len(self.dynamic_obstacles)),
            key=lambda index: math.hypot(
                x - (self.dynamic_obstacles[index][0] + self.dynamic_obstacles[index][2]) * 0.5,
                y - (self.dynamic_obstacles[index][1] + self.dynamic_obstacles[index][3]) * 0.5,
            ),
        )
        self.dynamic_obstacles.pop(selected)
        self._request_initial_plan()
        self.draw()

    def clear_obstacles(self) -> None:
        if self.mission_running:
            self.status_var.set("自动任务尚未完成，不能改写当前规划环境")
            return
        self.dynamic_obstacles.clear()
        self._request_initial_plan()
        self.draw()

    def toggle_playback(self) -> None:
        if self.mode_var.get() != "simulation" or len(self.route) < 2:
            return
        self.playing = not self.playing
        self.play_last_time = time.monotonic()
        self.view.play_button.configure(text="暂停" if self.playing else "播放")

    def _advance_playback(self) -> None:
        if len(self.route) < 2:
            self.playing = False
            return
        now = time.monotonic()
        self.play_distance += min(0.1, now - self.play_last_time) * 0.45
        self.play_last_time = now
        remaining = self.play_distance
        for left, right in zip(self.route, self.route[1:]):
            segment = math.hypot(right[0] - left[0], right[1] - left[1])
            if remaining <= segment:
                ratio = remaining / segment if segment > 1.0e-9 else 1.0
                self.vehicle_x = left[0] + ratio * (right[0] - left[0])
                self.vehicle_y = left[1] + ratio * (right[1] - left[1])
                self.vehicle_yaw = math.atan2(right[1] - left[1], right[0] - left[0])
                self.draw()
                return
            remaining -= segment
        self.vehicle_x, self.vehicle_y, self.vehicle_yaw = self.route[-1]
        self.playing = False
        self.view.play_button.configure(text="播放")
        self.remaining_labels = list(self.planned_deferred_labels)
        self.retry_pending = bool(self.remaining_labels) and not self.retry_used
        self.draw()
        if self.retry_pending and not self.retry_used:
            self.retry_pending = False
            self.retry_used = True
            self._request_initial_plan()

    def close(self) -> None:
        if self.closing:
            return
        self.closing = True
        self.playing = False
        self.node.cancel_requests()
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def load_config() -> dict[str, Any]:
    configured_path = os.environ.get("ARENA_MAP")
    path = (
        Path(configured_path).expanduser().resolve()
        if configured_path
        else Path(get_package_share_directory("arena_path_planner")) / "config" / "arena_map.yaml"
    )
    if not path.is_file():
        raise FileNotFoundError(f"Arena map does not exist: {path}")
    print(f"[arena-frontend] Loading map: {path}", flush=True)
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def main(_arguments: Optional[Sequence[str]] = None) -> int:
    root = tk.Tk()
    ArenaFrontend(root, load_config())
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
