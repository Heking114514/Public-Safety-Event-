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
from tkinter import ttk
from typing import Any, Optional, Sequence

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from arena_path_planner.srv import PlanArenaPath
from geometry_msgs.msg import Point, Point32, Polygon, Pose
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


SERVICE_NAME = "/arena_path_planner/plan"
ODOMETRY_TOPIC = "/odometry/fused"
ODOMETRY_TIMEOUT_S = 0.5
BACKGROUND = "#f7f8fa"
PANEL = "#ffffff"
ROAD = "#f7f8fa"
OBSTACLE = "#252b31"
GRID = "#98a2b3"
ROUTE = "#d92d20"
ROUTE_GLOW = "#fda29b"
CAR = "#dc2626"
TARGET = "#f59e0b"
GRID_METERS = 0.6


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
    ) -> None:
        super().__init__("arena_route_frontend")
        self.client = self.create_client(PlanArenaPath, SERVICE_NAME)
        self.responses = responses
        self.odometry_updates = odometry_updates
        self.lock = threading.Lock()
        self.in_flight = False
        self.pending: Optional[tuple[Any, ...]] = None
        self.odom_subscription = self.create_subscription(
            Odometry, ODOMETRY_TOPIC, self._handle_odometry, qos_profile_sensor_data
        )

    def _handle_odometry(self, message: Odometry) -> None:
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        yaw = quaternion_to_yaw(message.pose.pose.orientation)
        frame = message.header.frame_id
        update = (
            (x, y, yaw, time.monotonic())
            if math.isfinite(x)
            and math.isfinite(y)
            and yaw is not None
            and (not frame or frame == "map")
            else None
        )
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

    def request(
        self, generation: int, start: Pose, mode: str, activate: bool,
        obstacles: list[tuple[float, float, float, float]],
        targets: list[tuple[str, float, float]],
        covered_edges: list[str],
    ) -> None:
        item = (generation, start, mode, activate, obstacles, targets, covered_edges)
        if not self.client.service_is_ready():
            self.responses.put((generation, RuntimeError("规划后端未启动")))
            return
        with self.lock:
            if self.in_flight:
                self.pending = item
                return
            self.in_flight = True
        self._send(item)

    def _send(self, item: tuple[Any, ...]) -> None:
        generation, start, mode, activate, obstacles, targets, covered_edges = item
        request = PlanArenaPath.Request()
        request.start = start
        request.mode = mode
        request.activate_navigation = activate
        request.covered_edges = covered_edges
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
            self.responses.put((generation, exception))
            with self.lock:
                next_item = self.pending
                self.pending = None
                if next_item is None:
                    self.in_flight = False
            if next_item is not None:
                self._send(next_item)
            return
        future.add_done_callback(lambda completed: self._complete(generation, completed))

    def _complete(self, generation: int, future: Any) -> None:
        try:
            self.responses.put((generation, future.result()))
        except Exception as exception:  # rclpy propagates transport failures here.
            self.responses.put((generation, exception))
        with self.lock:
            next_item = self.pending
            self.pending = None
            if next_item is None:
                self.in_flight = False
        if next_item is not None:
            self._send(next_item)


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
        self.covered_edges: list[str] = []
        self.generation = 0
        self.applied_generation = -1
        self.request_count = 0
        self.activation_requests: set[int] = set()
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

        rclpy.init(args=[])
        self.node = PlannerClient(self.responses, self.odometry_updates)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.mode_var = tk.StringVar(value="simulation")
        self.order_var = tk.StringVar(value="shortest")
        self.status_var = tk.StringVar(value="等待规划后端")
        self.metrics_var = tk.StringVar(value="路径 -- m    规划 -- ms    重规划 0")
        self._build_ui()
        self.root.after(50, self._poll)
        self.root.after(200, self._request_initial_plan)

    def _build_ui(self) -> None:
        self.root.title("赛场闭环路径规划器")
        self.root.geometry("1040x790")
        self.root.minsize(900, 700)
        self.root.configure(bg=BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Simulation.Status.TLabel", foreground="#166534")
        style.configure("NoOdom.Status.TLabel", foreground="#991b1b")
        style.configure("OdomReady.Status.TLabel", foreground="#15803d")
        style.configure(
            "Navigation.TButton",
            background="#15803d",
            foreground="#ffffff",
            font=("Sans", 10, "bold"),
            padding=(12, 6),
        )
        style.map(
            "Navigation.TButton",
            background=[("active", "#166534"), ("disabled", "#9ca3af")],
            foreground=[("disabled", "#f3f4f6")],
        )
        toolbar = ttk.Frame(self.root, padding=(10, 8))
        toolbar.pack(fill=tk.X)
        self.publish_button = ttk.Button(
            toolbar,
            text="开始导航",
            command=self.publish_navigation,
            style="Navigation.TButton",
        )
        self.publish_button.pack(side=tk.RIGHT, padx=(10, 0))
        self.status_label = ttk.Label(
            toolbar, text="SIMULATION", style="Simulation.Status.TLabel"
        )
        self.status_label.pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(toolbar, text="模式").pack(side=tk.LEFT, padx=(0, 4))
        self.mode_box = ttk.Combobox(toolbar, textvariable=self.mode_var,
                                    values=("simulation", "vehicle"), width=11,
                                    state="readonly")
        self.mode_box.pack(side=tk.LEFT)
        self.mode_box.bind("<<ComboboxSelected>>", lambda _event: self._mode_changed())
        ttk.Label(toolbar, text="顺序").pack(side=tk.LEFT, padx=(14, 4))
        self.order_box = ttk.Combobox(toolbar, textvariable=self.order_var,
                                    values=("shortest", "numbered"), width=11,
                                    state="readonly")
        self.order_box.pack(side=tk.LEFT)
        self.order_box.bind("<<ComboboxSelected>>", lambda _event: self._request_initial_plan())
        self.start_button = ttk.Button(toolbar, text="开始规划", command=self._request_initial_plan)
        self.start_button.pack(side=tk.LEFT, padx=(14, 4))
        self.next_layer_button = ttk.Button(
            toolbar, text="进入第二阶段", command=self.advance_layer,
            state=tk.DISABLED,
        )
        self.next_layer_button.pack(side=tk.LEFT, padx=3)
        self.reset_button = ttk.Button(toolbar, text="重置", command=self.reset_vehicle)
        self.reset_button.pack(side=tk.LEFT, padx=3)
        self.play_button = ttk.Button(toolbar, text="播放", command=self.toggle_playback)
        self.play_button.pack(side=tk.LEFT, padx=3)
        self.clear_button = ttk.Button(toolbar, text="清除障碍", command=self.clear_obstacles)
        self.clear_button.pack(side=tk.LEFT, padx=3)

        body = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        body.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(body, bg="#20252b", highlightthickness=1,
                                highlightbackground="#667085", cursor="crosshair")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        panel = ttk.Frame(body, width=270, padding=(14, 8))
        panel.pack(side=tk.RIGHT, fill=tk.Y)
        panel.pack_propagate(False)
        self._panel_row(panel, "规划状态", self.status_var)
        self._panel_row(panel, "路径指标", self.metrics_var)
        self.remaining_var = tk.StringVar(value=f"{len(self.remaining_labels)} 个")
        self._panel_row(panel, "剩余目标", self.remaining_var)
        ttk.Separator(panel).pack(fill=tk.X, pady=12)
        self._panel_row(panel, "模拟车辆", tk.StringVar(value="拖动地图中的红色车辆"))
        self._panel_row(panel, "动态障碍", tk.StringVar(value="Ctrl+左键添加，右键删除"))
        ttk.Separator(panel).pack(fill=tk.X, pady=12)
        ttk.Label(panel, text="shortest：最短闭环\nnumbered：按标签顺序",
                justify=tk.LEFT, wraplength=240).pack(anchor=tk.W)

        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.canvas.bind("<ButtonPress-1>", self._drag_start)
        self.canvas.bind("<B1-Motion>", self._drag_motion)
        self.canvas.bind("<ButtonRelease-1>", self._drag_end)
        self.canvas.bind("<Control-Button-1>", self._add_obstacle)
        self.canvas.bind("<Control-Button-3>", self._remove_obstacle)
        self.canvas.bind("<B3-Motion>", self._rotate_motion)
        self.canvas.bind("<MouseWheel>", self._wheel_rotate)

    def _panel_row(self, parent: Any, title: str, variable: Any) -> None:
        ttk.Label(parent, text=title).pack(anchor=tk.W, pady=(4, 1))
        ttk.Label(parent, textvariable=variable, wraplength=240,
                justify=tk.LEFT).pack(anchor=tk.W)

    def _transform(self) -> tuple[float, float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        padding = 24.0
        scale = min((width - 2 * padding) / self.width_m,
                    (height - 2 * padding) / self.height_m)
        return scale, (width - self.width_m * scale) * 0.5, (height - self.height_m * scale) * 0.5

    def _to_canvas(self, x: float, y: float) -> tuple[float, float]:
        scale, origin_x, origin_y = self._transform()
        return origin_x + x * scale, self.canvas.winfo_height() - origin_y - y * scale

    def _to_world(self, x: float, y: float) -> tuple[float, float]:
        scale, origin_x, origin_y = self._transform()
        return (x - origin_x) / scale, (self.canvas.winfo_height() - origin_y - y) / scale

    def draw(self) -> None:
        self.canvas.delete("all")
        scale, offset_x, offset_y = self._transform()
        for index in range(int(self.width_m / GRID_METERS) + 1):
            x0, y0 = self._to_canvas(index * GRID_METERS, 0.0)
            _, y1 = self._to_canvas(index * GRID_METERS, self.height_m)
            self.canvas.create_line(x0, y0, x0, y1, fill=GRID, dash=(2, 5))
        for index in range(int(self.height_m / GRID_METERS) + 1):
            x0, y0 = self._to_canvas(0.0, index * GRID_METERS)
            x1, _ = self._to_canvas(self.width_m, index * GRID_METERS)
            self.canvas.create_line(x0, y0, x1, y0, fill=GRID, dash=(2, 5))
        for region in self.arena["free_regions"]:
            x0, y0 = self._to_canvas(float(region[0]), float(region[1]))
            x1, y1 = self._to_canvas(float(region[2]), float(region[3]))
            self.canvas.create_rectangle(x0, y0, x1, y1, fill=ROAD, outline=ROAD)
        for obstacle in self.arena["obstacles"]:
            x0, y0 = self._to_canvas(float(obstacle[0]), float(obstacle[1]))
            x1, y1 = self._to_canvas(float(obstacle[2]), float(obstacle[3]))
            self.canvas.create_rectangle(x0, y0, x1, y1, fill=OBSTACLE, outline="#68717d")
        # Tunnels are mandatory inspection bands. Draw their diagonal hatch
        # explicitly so the visual map matches the competition diagram.
        for label, tunnel in self.config.get("tunnels", {}).items():
            if not isinstance(tunnel, dict):
                continue
            entry = tuple(map(float, tunnel["entry"]))
            exit_point = tuple(map(float, tunnel["exit"]))
            ex, ey = self._to_canvas(*entry)
            xx, xy = self._to_canvas(*exit_point)
            self.canvas.create_line(ex, ey, xx, xy, fill="#f5d0a9", width=18)
            self.canvas.create_line(ex, ey, xx, xy, fill="#b45309", width=2)
            for fraction in (0.2, 0.4, 0.6, 0.8):
                cx = ex + fraction * (xx - ex)
                cy = ey + fraction * (xy - ey)
                self.canvas.create_line(cx - 8, cy - 8, cx + 8, cy + 8, fill="#92400e", width=1)
            self.canvas.create_text(
                (ex + xx) * 0.5, (ey + xy) * 0.5 - 12,
                text=label, fill="#78350f", font=("Sans", 8, "bold"),
            )
        for obstacle in self.dynamic_obstacles:
            x0, y0 = self._to_canvas(obstacle[0], obstacle[1])
            x1, y1 = self._to_canvas(obstacle[2], obstacle[3])
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill="#ef4444", outline="#fecaca", width=2,
            )
        grid_step = 0.6
        for index in range(int(self.width_m / grid_step) + 1):
            x = index * grid_step
            x0, y0 = self._to_canvas(x, 0.0)
            _, y1 = self._to_canvas(x, self.height_m)
            self.canvas.create_line(x0, y0, x0, y1, fill=GRID, dash=(2, 5), width=1)
        for index in range(int(self.height_m / grid_step) + 1):
            y = index * grid_step
            x0, y0 = self._to_canvas(0.0, y)
            x1, _ = self._to_canvas(self.width_m, y)
            self.canvas.create_line(x0, y0, x1, y0, fill=GRID, dash=(2, 5), width=1)

        layer_colours = {1: ("#2563eb", "#93c5fd"), 2: ("#16a34a", "#86efac"), 3: ("#9333ea", "#d8b4fe")}
        layer_route = self.layer_routes.get(self.visible_layer, [])
        segments = self.layer_display_segments.get(self.visible_layer)
        if not segments and len(layer_route) >= 2:
            segments = [layer_route]
        # Keep navigation connectors visible in stage 3.  They are required
        # travel legs, but only the newly inspected road portions are purple.
        if self.visible_layer == 3 and len(layer_route) >= 2:
            connector_coordinates = [
                coordinate for point in layer_route
                for coordinate in self._to_canvas(point[0], point[1])]
            self.canvas.create_line(
                *connector_coordinates, fill="#64748b", width=2, dash=(5, 4))
        if segments:
            colour, glow = layer_colours[self.visible_layer]
            for segment in segments:
                if len(segment) < 2:
                    continue
                coordinates = [coordinate for point in segment for coordinate in self._to_canvas(point[0], point[1])]
                self.canvas.create_line(*coordinates, fill=glow, width=7)
                self.canvas.create_line(*coordinates, fill=colour, width=3)
                for index in range(0, len(segment), max(1, len(segment) // 12)):
                    cx, cy = self._to_canvas(segment[index][0], segment[index][1])
                    self.canvas.create_oval(cx - 2, cy - 2, cx + 2, cy + 2, fill=colour, outline="")

        if self.mode_var.get() == "vehicle" and len(self.odom_trace) >= 2:
            trace_coordinates = [
                coordinate
                for point in self.odom_trace
                for coordinate in self._to_canvas(point[0], point[1])
            ]
            self.canvas.create_line(
                *trace_coordinates, fill="#0891b2", width=2, smooth=True
            )

        radius = max(5.0, min(10.0, scale * 0.035))
        for label, position in self.tasks:
            cx, cy = self._to_canvas(float(position[0]), float(position[1]))
            self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                                    fill=TARGET, outline="#7c2d12", width=1)
            self.canvas.create_text(cx, cy, text=str(label), fill="#171717", font=("Sans", 8, "bold"))
        self._draw_vehicle(scale)

    def _draw_vehicle(self, scale: float) -> None:
        cx, cy = self._to_canvas(self.vehicle_x, self.vehicle_y)
        length = self.vehicle_length_m * scale
        width = self.vehicle_width_m * scale
        direction = -self.vehicle_yaw
        forward = (math.cos(direction), math.sin(direction))
        side = (-forward[1], forward[0])
        collision_radius = 0.5 * math.hypot(length, width) + self.safety_margin_m * scale
        self.canvas.create_oval(
            cx - collision_radius, cy - collision_radius,
            cx + collision_radius, cy + collision_radius,
            outline="#fca5a5", dash=(3, 3), width=1,
        )
        points = [
            (cx + forward[0] * length * 0.5 + side[0] * width * 0.5,
            cy + forward[1] * length * 0.5 + side[1] * width * 0.5),
            (cx + forward[0] * length * 0.5 - side[0] * width * 0.5,
            cy + forward[1] * length * 0.5 - side[1] * width * 0.5),
            (cx - forward[0] * length * 0.5 - side[0] * width * 0.5,
            cy - forward[1] * length * 0.5 - side[1] * width * 0.5),
            (cx - forward[0] * length * 0.5 + side[0] * width * 0.5,
            cy - forward[1] * length * 0.5 + side[1] * width * 0.5),
        ]
        flattened = [coordinate for point in points for coordinate in point]
        self.canvas.create_polygon(*flattened, fill=CAR, outline="white", width=2)
        self.canvas.create_line(
            cx + forward[0] * length * 0.5 - side[0] * width * 0.5,
            cy + forward[1] * length * 0.5 - side[1] * width * 0.5,
            cx + forward[0] * length * 0.5 + side[0] * width * 0.5,
            cy + forward[1] * length * 0.5 + side[1] * width * 0.5,
            fill="white", width=3,
        )
        self.canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="white", outline="")

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
                self.status_label.configure(
                    text="ODOM READY" if ready else "NO ODOM",
                    style="OdomReady.Status.TLabel" if ready else "NoOdom.Status.TLabel",
                )
                self.draw()

    def request_plan(self, activate: bool = False) -> None:
        if activate and self.mode_var.get() != "vehicle":
            self.status_var.set("模拟模式禁止发布导航命令")
            return
        self.generation += 1
        self.request_count += 1
        if activate:
            self.activation_requests.add(self.generation)
        self.status_var.set("正在规划..." if self.node.service_ready() else "规划后端未就绪")
        mode = {1: "layer1", 2: "layer2", 3: "layer3"}.get(self.layer_mode, "layer1")
        self.node.request(
            self.generation, self._pose(self.layer_start), mode, activate,
            list(self.dynamic_obstacles),
            [
                (str(label), float(position[0]), float(position[1]))
                for label, position in self.tasks
                if str(label) in self.remaining_labels
            ],
            list(self.covered_edges),
        )

    def _request_initial_plan(self) -> None:
        self.layer_mode = 1
        self.visible_layer = 1
        self.layer_routes.clear()
        self.layer_display_segments.clear()
        self.covered_edges.clear()
        self.layer_start = (self.vehicle_x, self.vehicle_y, self.vehicle_yaw)
        self.next_layer_button.configure(text="进入第二阶段", state=tk.DISABLED)
        self.request_plan()

    def advance_layer(self) -> None:
        """Start the next inspection phase only after operator confirmation."""
        if self.layer_mode >= 3 or self.node.in_flight:
            return
        current_route = self.layer_routes.get(self.layer_mode, [])
        if len(current_route) < 2:
            self.status_var.set("当前阶段尚未规划完成")
            return
        self.layer_start = current_route[-1]
        self.layer_mode += 1
        self.visible_layer = self.layer_mode
        self.next_layer_button.configure(state=tk.DISABLED)
        self.status_var.set(f"正在规划第 {self.layer_mode} 阶段...")
        self.request_plan()

    def publish_navigation(self) -> None:
        if self.mode_var.get() != "vehicle":
            self.status_var.set("当前是模拟模式：请先切换到 vehicle 再开始导航")
            return
        if not self.node.service_ready():
            self.status_var.set("无法开始导航：规划后端未就绪")
            return
        if not self._odometry_is_ready():
            self.status_var.set("无法开始导航：融合里程计未就绪或已超时")
            return
        self.generation += 1
        self.request_count += 1
        self.activation_requests.add(self.generation)
        self.publish_button.configure(state=tk.DISABLED, text="正在启动...")
        self.status_var.set("正在发布三层完整路线...")
        self.node.request(
            self.generation, self._pose(), "layered", True,
            list(self.dynamic_obstacles),
            [(str(label), float(position[0]), float(position[1])) for label, position in self.tasks],
            list(self.covered_edges),
        )

    def _poll(self) -> None:
        if self.closing or not rclpy.ok():
            return
        self._consume_odometry()
        try:
            while True:
                generation, response = self.responses.get_nowait()
                if generation < self.applied_generation or generation < self.generation:
                    if generation in self.activation_requests:
                        self.activation_requests.discard(generation)
                        self.publish_button.configure(state=tk.NORMAL, text="开始导航")
                    continue
                self.applied_generation = generation
                if isinstance(response, Exception):
                    self.activation_requests.discard(generation)
                    self.status_var.set(f"服务调用失败：{response}")
                    self.publish_button.configure(state=tk.NORMAL, text="开始导航")
                    continue
                if not response.success:
                    self.activation_requests.discard(generation)
                    self.route = []
                    self.status_var.set(f"规划失败：{response.message}")
                    self.publish_button.configure(state=tk.NORMAL, text="开始导航")
                    self.draw()
                    continue
                self.route = []
                for pose in response.arena_path.poses:
                    orientation = pose.pose.orientation
                    yaw = math.atan2(
                        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
                    )
                    self.route.append((pose.pose.position.x, pose.pose.position.y, yaw))
                activation_requested = generation in self.activation_requests
                self.activation_requests.discard(generation)
                # The backend deliberately refuses activation for a partial
                # route. Keep that distinction visible instead of reporting
                # that the complete mission started.
                activated = activation_requested and bool(response.all_targets_reached)
                incomplete = bool(response.deferred_targets) or not bool(response.all_targets_reached)
                if activation_requested:
                    self.publish_button.configure(state=tk.NORMAL, text="开始导航")
                if not activated:
                    self.layer_routes[self.layer_mode] = list(self.route)
                    self.layer_display_segments[self.layer_mode] = self._display_segments(
                        self.layer_mode, self.route, list(response.visit_order))
                    self.visible_layer = self.layer_mode
                self.planned_deferred_labels = list(response.deferred_targets)
                for edge in response.covered_edges:
                    if edge not in self.covered_edges:
                        self.covered_edges.append(edge)
                if activated:
                    self.status_var.set("三层完整路线已发布并启动")
                elif activation_requested and incomplete:
                    self.status_var.set("路线不完整，未启动：请先处理延迟目标")
                elif incomplete:
                    self.status_var.set(
                        f"第 {self.layer_mode} 阶段部分完成：请先处理延迟目标")
                else:
                    self.status_var.set(f"第 {self.layer_mode} 阶段规划完成")
                self.remaining_var.set(f"{len(self.planned_deferred_labels)} 个延迟目标")
                self.metrics_var.set(
                    f"路径 {response.length:.2f} m    规划 {response.planning_time_ms:.0f} ms\n"
                    f"点数 {len(self.route)}    重规划 {self.request_count}\n"
                    f"延迟目标 {', '.join(response.deferred_targets) or '无'}"
                )
                self.play_distance = 0.0
                self.draw()
                if not activated:
                    if self.layer_mode < 3:
                        self.next_layer_button.configure(
                            text=f"进入第 {self.layer_mode + 1} 阶段",
                            state=tk.DISABLED if incomplete else tk.NORMAL,
                        )
                    else:
                        self.next_layer_button.configure(text="三阶段已完成", state=tk.DISABLED)
        except queue.Empty:
            pass
        if not self.node.service_ready() and not self.node.in_flight:
            self.status_var.set("规划后端未启动")
        if self.playing:
            self._advance_playback()
        self.root.after(35, self._poll)

    def _mode_changed(self) -> None:
        simulation = self.mode_var.get() == "simulation"
        if simulation:
            self.status_label.configure(text="SIMULATION", style="Simulation.Status.TLabel")
        else:
            self._apply_current_odometry(force_draw=True)
            ready = self._odometry_is_ready()
            self.odom_ready = ready
            self.status_label.configure(
                text="ODOM READY" if ready else "NO ODOM",
                style="OdomReady.Status.TLabel" if ready else "NoOdom.Status.TLabel",
            )
        self.play_button.configure(state=tk.NORMAL if simulation else tk.DISABLED)
        self.publish_button.configure(state=tk.NORMAL, text="开始导航")
        if not simulation:
            self.playing = False
            self.play_button.configure(text="播放")
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
            self.play_button.configure(text="播放")

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
        self.odom_trace.clear()
        if self.mode_var.get() == "vehicle" and self._odometry_is_ready():
            self._apply_current_odometry()
        else:
            self.vehicle_x, self.vehicle_y, self.vehicle_yaw = self.default_pose
        self.playing = False
        self.play_button.configure(text="播放")
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
        self.next_layer_button.configure(text="进入第二阶段", state=tk.DISABLED)
        self._request_initial_plan()
        self.draw()

    def _add_obstacle(self, event: tk.Event) -> None:
        if self.mode_var.get() != "simulation":
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
        if self.mode_var.get() != "simulation" or not self.dynamic_obstacles:
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
        self.dynamic_obstacles.clear()
        self._request_initial_plan()
        self.draw()

    def toggle_playback(self) -> None:
        if self.mode_var.get() != "simulation" or len(self.route) < 2:
            return
        self.playing = not self.playing
        self.play_last_time = time.monotonic()
        self.play_button.configure(text="暂停" if self.playing else "播放")

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
        self.play_button.configure(text="播放")
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
