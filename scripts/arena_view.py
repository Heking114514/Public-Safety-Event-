"""Tk widgets and canvas rendering for the arena planner frontend."""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from typing import Any, Protocol


BACKGROUND = "#f7f8fa"
ROAD = "#f7f8fa"
OBSTACLE = "#252b31"
GRID = "#98a2b3"
CAR = "#dc2626"
TARGET = "#f59e0b"
GRID_METERS = 0.6


class ArenaController(Protocol):
    """State and actions consumed by the view."""

    config: dict[str, Any]
    arena: dict[str, Any]
    tasks: list[tuple[Any, Any]]
    mode_var: tk.StringVar
    order_var: tk.StringVar
    status_var: tk.StringVar
    metrics_var: tk.StringVar
    remaining_labels: list[str]
    dynamic_obstacles: list[tuple[float, float, float, float]]
    layer_routes: dict[int, list[tuple[float, float, float]]]
    layer_display_segments: dict[int, list[list[tuple[float, float, float]]]]
    visible_layer: int
    odom_trace: list[tuple[float, float]]
    vehicle_x: float
    vehicle_y: float
    vehicle_yaw: float
    vehicle_length_m: float
    vehicle_width_m: float
    safety_margin_m: float

    def close(self) -> None: ...
    def publish_navigation(self) -> None: ...
    def advance_layer(self) -> None: ...
    def reset_vehicle(self) -> None: ...
    def toggle_playback(self) -> None: ...
    def clear_obstacles(self) -> None: ...
    def draw(self) -> None: ...
    def _mode_changed(self) -> None: ...
    def _request_initial_plan(self) -> None: ...
    def _drag_start(self, event: tk.Event) -> None: ...
    def _drag_motion(self, event: tk.Event) -> None: ...
    def _drag_end(self, event: tk.Event) -> None: ...
    def _add_obstacle(self, event: tk.Event) -> None: ...
    def _remove_obstacle(self, event: tk.Event) -> None: ...
    def _rotate_motion(self, event: tk.Event) -> None: ...
    def _wheel_rotate(self, event: tk.Event) -> None: ...


class ArenaView:
    """Owns Tk widgets and renders controller state without changing it."""

    def __init__(
        self,
        root: tk.Tk,
        controller: ArenaController,
        width_m: float,
        height_m: float,
    ) -> None:
        self.root = root
        self.controller = controller
        self.width_m = width_m
        self.height_m = height_m
        self._build()

    def _build(self) -> None:
        controller = self.controller
        self.root.title("赛场闭环路径规划器")
        self.root.geometry("1040x790")
        self.root.minsize(900, 700)
        self.root.configure(bg=BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", controller.close)
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
            command=controller.publish_navigation,
            style="Navigation.TButton",
        )
        self.publish_button.pack(side=tk.RIGHT, padx=(10, 0))
        self.status_label = ttk.Label(
            toolbar, text="SIMULATION", style="Simulation.Status.TLabel"
        )
        self.status_label.pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(toolbar, text="模式").pack(side=tk.LEFT, padx=(0, 4))
        self.mode_box = ttk.Combobox(
            toolbar,
            textvariable=controller.mode_var,
            values=("simulation", "vehicle"),
            width=11,
            state="readonly",
        )
        self.mode_box.pack(side=tk.LEFT)
        self.mode_box.bind(
            "<<ComboboxSelected>>", lambda _event: controller._mode_changed()
        )
        ttk.Label(toolbar, text="顺序").pack(side=tk.LEFT, padx=(14, 4))
        self.order_box = ttk.Combobox(
            toolbar,
            textvariable=controller.order_var,
            values=("shortest", "numbered"),
            width=11,
            state="readonly",
        )
        self.order_box.pack(side=tk.LEFT)
        self.order_box.bind(
            "<<ComboboxSelected>>", lambda _event: controller._request_initial_plan()
        )
        self.start_button = ttk.Button(
            toolbar, text="开始规划", command=controller._request_initial_plan
        )
        self.start_button.pack(side=tk.LEFT, padx=(14, 4))
        self.next_layer_button = ttk.Button(
            toolbar,
            text="进入第二阶段",
            command=controller.advance_layer,
            state=tk.DISABLED,
        )
        self.next_layer_button.pack(side=tk.LEFT, padx=3)
        self.reset_button = ttk.Button(
            toolbar, text="重置", command=controller.reset_vehicle
        )
        self.reset_button.pack(side=tk.LEFT, padx=3)
        self.play_button = ttk.Button(
            toolbar, text="播放", command=controller.toggle_playback
        )
        self.play_button.pack(side=tk.LEFT, padx=3)
        self.clear_button = ttk.Button(
            toolbar, text="清除障碍", command=controller.clear_obstacles
        )
        self.clear_button.pack(side=tk.LEFT, padx=3)

        body = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        body.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(
            body,
            bg="#20252b",
            highlightthickness=1,
            highlightbackground="#667085",
            cursor="crosshair",
        )
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        panel = ttk.Frame(body, width=270, padding=(14, 8))
        panel.pack(side=tk.RIGHT, fill=tk.Y)
        panel.pack_propagate(False)
        self._panel_row(panel, "规划状态", controller.status_var)
        self._panel_row(panel, "路径指标", controller.metrics_var)
        self.remaining_var = tk.StringVar(value=f"{len(controller.remaining_labels)} 个")
        self._panel_row(panel, "剩余目标", self.remaining_var)
        ttk.Separator(panel).pack(fill=tk.X, pady=12)
        self._panel_row(panel, "模拟车辆", tk.StringVar(value="拖动地图中的红色车辆"))
        self._panel_row(panel, "动态障碍", tk.StringVar(value="Ctrl+左键添加，右键删除"))
        ttk.Separator(panel).pack(fill=tk.X, pady=12)
        ttk.Label(
            panel,
            text="shortest：最短闭环\nnumbered：按标签顺序",
            justify=tk.LEFT,
            wraplength=240,
        ).pack(anchor=tk.W)

        self.canvas.bind("<Configure>", lambda _event: controller.draw())
        self.canvas.bind("<ButtonPress-1>", controller._drag_start)
        self.canvas.bind("<B1-Motion>", controller._drag_motion)
        self.canvas.bind("<ButtonRelease-1>", controller._drag_end)
        self.canvas.bind("<Control-Button-1>", controller._add_obstacle)
        self.canvas.bind("<Control-Button-3>", controller._remove_obstacle)
        self.canvas.bind("<B3-Motion>", controller._rotate_motion)
        self.canvas.bind("<MouseWheel>", controller._wheel_rotate)

    @staticmethod
    def _panel_row(parent: Any, title: str, variable: Any) -> None:
        ttk.Label(parent, text=title).pack(anchor=tk.W, pady=(4, 1))
        ttk.Label(
            parent, textvariable=variable, wraplength=240, justify=tk.LEFT
        ).pack(anchor=tk.W)

    def _transform(self) -> tuple[float, float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        padding = 24.0
        scale = min(
            (width - 2 * padding) / self.width_m,
            (height - 2 * padding) / self.height_m,
        )
        return (
            scale,
            (width - self.width_m * scale) * 0.5,
            (height - self.height_m * scale) * 0.5,
        )

    def to_canvas(self, x: float, y: float) -> tuple[float, float]:
        scale, origin_x, origin_y = self._transform()
        return origin_x + x * scale, self.canvas.winfo_height() - origin_y - y * scale

    def to_world(self, x: float, y: float) -> tuple[float, float]:
        scale, origin_x, origin_y = self._transform()
        return (
            (x - origin_x) / scale,
            (self.canvas.winfo_height() - origin_y - y) / scale,
        )

    def draw(self) -> None:
        state = self.controller
        self.canvas.delete("all")
        scale, _offset_x, _offset_y = self._transform()
        for index in range(int(self.width_m / GRID_METERS) + 1):
            x0, y0 = self.to_canvas(index * GRID_METERS, 0.0)
            _, y1 = self.to_canvas(index * GRID_METERS, self.height_m)
            self.canvas.create_line(x0, y0, x0, y1, fill=GRID, dash=(2, 5))
        for index in range(int(self.height_m / GRID_METERS) + 1):
            x0, y0 = self.to_canvas(0.0, index * GRID_METERS)
            x1, _ = self.to_canvas(self.width_m, index * GRID_METERS)
            self.canvas.create_line(x0, y0, x1, y0, fill=GRID, dash=(2, 5))
        for region in state.arena["free_regions"]:
            x0, y0 = self.to_canvas(float(region[0]), float(region[1]))
            x1, y1 = self.to_canvas(float(region[2]), float(region[3]))
            self.canvas.create_rectangle(x0, y0, x1, y1, fill=ROAD, outline=ROAD)
        for obstacle in state.arena["obstacles"]:
            x0, y0 = self.to_canvas(float(obstacle[0]), float(obstacle[1]))
            x1, y1 = self.to_canvas(float(obstacle[2]), float(obstacle[3]))
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill=OBSTACLE, outline="#68717d"
            )
        # Tunnels are mandatory inspection bands. Draw their diagonal hatch
        # explicitly so the visual map matches the competition diagram.
        for label, tunnel in state.config.get("tunnels", {}).items():
            if not isinstance(tunnel, dict):
                continue
            entry = tuple(map(float, tunnel["entry"]))
            exit_point = tuple(map(float, tunnel["exit"]))
            ex, ey = self.to_canvas(*entry)
            xx, xy = self.to_canvas(*exit_point)
            self.canvas.create_line(ex, ey, xx, xy, fill="#f5d0a9", width=18)
            self.canvas.create_line(ex, ey, xx, xy, fill="#b45309", width=2)
            for fraction in (0.2, 0.4, 0.6, 0.8):
                cx = ex + fraction * (xx - ex)
                cy = ey + fraction * (xy - ey)
                self.canvas.create_line(
                    cx - 8, cy - 8, cx + 8, cy + 8, fill="#92400e", width=1
                )
            self.canvas.create_text(
                (ex + xx) * 0.5,
                (ey + xy) * 0.5 - 12,
                text=label,
                fill="#78350f",
                font=("Sans", 8, "bold"),
            )
        for obstacle in state.dynamic_obstacles:
            x0, y0 = self.to_canvas(obstacle[0], obstacle[1])
            x1, y1 = self.to_canvas(obstacle[2], obstacle[3])
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill="#ef4444", outline="#fecaca", width=2
            )
        grid_step = 0.6
        for index in range(int(self.width_m / grid_step) + 1):
            x = index * grid_step
            x0, y0 = self.to_canvas(x, 0.0)
            _, y1 = self.to_canvas(x, self.height_m)
            self.canvas.create_line(
                x0, y0, x0, y1, fill=GRID, dash=(2, 5), width=1
            )
        for index in range(int(self.height_m / grid_step) + 1):
            y = index * grid_step
            x0, y0 = self.to_canvas(0.0, y)
            x1, _ = self.to_canvas(self.width_m, y)
            self.canvas.create_line(
                x0, y0, x1, y0, fill=GRID, dash=(2, 5), width=1
            )

        layer_colours = {
            1: ("#2563eb", "#93c5fd"),
            2: ("#16a34a", "#86efac"),
            3: ("#9333ea", "#d8b4fe"),
        }
        layer_route = state.layer_routes.get(state.visible_layer, [])
        segments = state.layer_display_segments.get(state.visible_layer)
        if not segments and len(layer_route) >= 2:
            segments = [layer_route]
        # Keep navigation connectors visible in stage 3. They are required
        # travel legs, but only the newly inspected road portions are purple.
        if state.visible_layer == 3 and len(layer_route) >= 2:
            connector_coordinates = [
                coordinate
                for point in layer_route
                for coordinate in self.to_canvas(point[0], point[1])
            ]
            self.canvas.create_line(
                *connector_coordinates, fill="#64748b", width=2, dash=(5, 4)
            )
        if segments:
            colour, glow = layer_colours[state.visible_layer]
            for segment in segments:
                if len(segment) < 2:
                    continue
                coordinates = [
                    coordinate
                    for point in segment
                    for coordinate in self.to_canvas(point[0], point[1])
                ]
                self.canvas.create_line(*coordinates, fill=glow, width=7)
                self.canvas.create_line(*coordinates, fill=colour, width=3)
                for index in range(0, len(segment), max(1, len(segment) // 12)):
                    cx, cy = self.to_canvas(segment[index][0], segment[index][1])
                    self.canvas.create_oval(
                        cx - 2,
                        cy - 2,
                        cx + 2,
                        cy + 2,
                        fill=colour,
                        outline="",
                    )

        if state.mode_var.get() == "vehicle" and len(state.odom_trace) >= 2:
            trace_coordinates = [
                coordinate
                for point in state.odom_trace
                for coordinate in self.to_canvas(point[0], point[1])
            ]
            self.canvas.create_line(
                *trace_coordinates, fill="#0891b2", width=2, smooth=True
            )

        radius = max(5.0, min(10.0, scale * 0.035))
        for label, position in state.tasks:
            cx, cy = self.to_canvas(float(position[0]), float(position[1]))
            self.canvas.create_oval(
                cx - radius,
                cy - radius,
                cx + radius,
                cy + radius,
                fill=TARGET,
                outline="#7c2d12",
                width=1,
            )
            self.canvas.create_text(
                cx,
                cy,
                text=str(label),
                fill="#171717",
                font=("Sans", 8, "bold"),
            )
        self._draw_vehicle(state, scale)

    def _draw_vehicle(self, state: ArenaController, scale: float) -> None:
        cx, cy = self.to_canvas(state.vehicle_x, state.vehicle_y)
        length = state.vehicle_length_m * scale
        width = state.vehicle_width_m * scale
        direction = -state.vehicle_yaw
        forward = (math.cos(direction), math.sin(direction))
        side = (-forward[1], forward[0])
        collision_radius = (
            0.5 * math.hypot(length, width) + state.safety_margin_m * scale
        )
        self.canvas.create_oval(
            cx - collision_radius,
            cy - collision_radius,
            cx + collision_radius,
            cy + collision_radius,
            outline="#fca5a5",
            dash=(3, 3),
            width=1,
        )
        points = [
            (
                cx + forward[0] * length * 0.5 + side[0] * width * 0.5,
                cy + forward[1] * length * 0.5 + side[1] * width * 0.5,
            ),
            (
                cx + forward[0] * length * 0.5 - side[0] * width * 0.5,
                cy + forward[1] * length * 0.5 - side[1] * width * 0.5,
            ),
            (
                cx - forward[0] * length * 0.5 - side[0] * width * 0.5,
                cy - forward[1] * length * 0.5 - side[1] * width * 0.5,
            ),
            (
                cx - forward[0] * length * 0.5 + side[0] * width * 0.5,
                cy - forward[1] * length * 0.5 + side[1] * width * 0.5,
            ),
        ]
        flattened = [coordinate for point in points for coordinate in point]
        self.canvas.create_polygon(
            *flattened, fill=CAR, outline="white", width=2
        )
        self.canvas.create_line(
            cx + forward[0] * length * 0.5 - side[0] * width * 0.5,
            cy + forward[1] * length * 0.5 - side[1] * width * 0.5,
            cx + forward[0] * length * 0.5 + side[0] * width * 0.5,
            cy + forward[1] * length * 0.5 + side[1] * width * 0.5,
            fill="white",
            width=3,
        )
        self.canvas.create_oval(
            cx - 3, cy - 3, cx + 3, cy + 3, fill="white", outline=""
        )
