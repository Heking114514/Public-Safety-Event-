from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from typing import Any, Protocol

from .model import map_to_arena


BACKGROUND = "#f4f6f8"
CANVAS_BACKGROUND = "#20262d"
ROAD = "#dce3e9"
TUNNEL = "#78a6c8"
OBSTACLE = "#49515a"
NODE = "#8e99a5"
TASK = "#f0b429"
ROUTE = "#18a06a"
TURN = "#ef8354"
VEHICLE = "#e04444"


class Controller(Protocol):
    config: dict[str, Any]
    physical: Any
    mode_var: tk.StringVar
    status_var: tk.StringVar
    odom_var: tk.StringVar
    planner_var: tk.StringVar
    navigator_var: tk.StringVar
    ack_var: tk.StringVar
    stage_var: tk.StringVar
    metric_var: tk.StringVar
    hold_var: tk.StringVar
    obstacle_var: tk.StringVar
    selected_targets: set[str]
    current_pose_arena: tuple[float, float, float] | None
    plan: Any
    dynamic_obstacles: list[list[tuple[float, float]]]

    def preview(self) -> None:
        ...

    def navigate(self) -> None:
        ...

    def toggle_hold(self) -> None:
        ...

    def clear_obstacles(self) -> None:
        ...

    def close(self) -> None:
        ...

    def mode_changed(self) -> None:
        ...

    def target_selection_changed(self) -> None:
        ...


class PlannerView:
    def __init__(self, root: tk.Tk, controller: Controller) -> None:
        self.root = root
        self.controller = controller
        self._build()

    def _build(self) -> None:
        self.root.title("New Map Planner - 实车导航")
        self.root.geometry("1220x820")
        self.root.minsize(980, 680)
        self.root.configure(bg=BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self.controller.close)
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Nav.TButton", padding=(13, 7), font=("Sans", 10, "bold"))
        style.configure(
            "Hold.TButton",
            padding=(13, 7),
            foreground="#ffffff",
            background="#bd2c2c",
            font=("Sans", 10, "bold"),
        )
        style.map("Hold.TButton", background=[("active", "#9f2525")])

        toolbar = ttk.Frame(self.root, padding=(12, 10))
        toolbar.pack(fill=tk.X)
        ttk.Label(toolbar, text="NEW MAP PLANNER", font=("Sans", 13, "bold")).pack(
            side=tk.LEFT, padx=(0, 20)
        )
        self.odom_label = ttk.Label(toolbar, textvariable=self.controller.odom_var)
        self.odom_label.pack(side=tk.LEFT, padx=(0, 14))
        ttk.Label(toolbar, textvariable=self.controller.planner_var).pack(
            side=tk.LEFT, padx=(0, 14)
        )
        ttk.Label(toolbar, textvariable=self.controller.navigator_var).pack(
            side=tk.LEFT, padx=(0, 14)
        )
        ttk.Label(toolbar, textvariable=self.controller.ack_var).pack(side=tk.LEFT)
        self.hold_button = ttk.Button(
            toolbar,
            textvariable=self.controller.hold_var,
            command=self.controller.toggle_hold,
            style="Hold.TButton",
        )
        self.hold_button.pack(side=tk.RIGHT)

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 8))
        map_frame = ttk.Frame(body)
        side = ttk.Frame(body, width=340, padding=(14, 0, 0, 0))
        body.add(map_frame, weight=4)
        body.add(side, weight=1)

        self.canvas = tk.Canvas(
            map_frame,
            background=CANVAS_BACKGROUND,
            highlightthickness=1,
            highlightbackground="#65717c",
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw())

        ttk.Label(side, text="任务阶段", font=("Sans", 11, "bold")).pack(anchor=tk.W)
        self.mode_box = ttk.Combobox(
            side,
            state="readonly",
            textvariable=self.controller.mode_var,
            values=("第一阶段", "第二阶段", "第三阶段"),
        )
        self.mode_box.pack(fill=tk.X, pady=(5, 10))
        self.mode_box.bind(
            "<<ComboboxSelected>>", lambda _event: self.controller.mode_changed()
        )

        ttk.Label(side, text="固定任务点（RFID 编号）").pack(anchor=tk.W)
        target_frame = ttk.Frame(side)
        target_frame.pack(fill=tk.X, pady=(4, 10))
        self.target_list = tk.Listbox(
            target_frame, selectmode=tk.MULTIPLE, exportselection=False, height=8
        )
        target_scroll = ttk.Scrollbar(
            target_frame, orient=tk.VERTICAL, command=self.target_list.yview
        )
        self.target_list.configure(yscrollcommand=target_scroll.set)
        for task_id in self.controller.config["tasks"].keys():
            self.target_list.insert(tk.END, str(task_id))
        self.target_list.selection_set(0, tk.END)
        self.target_list.pack(side=tk.LEFT, fill=tk.X, expand=True)
        target_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.target_list.bind(
            "<<ListboxSelect>>",
            lambda _event: self.controller.target_selection_changed(),
        )

        command_row = ttk.Frame(side)
        command_row.pack(fill=tk.X, pady=(0, 10))
        self.preview_button = ttk.Button(
            command_row, text="规划预览", command=self.controller.preview
        )
        self.preview_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self.navigate_button = ttk.Button(
            command_row,
            text="下发并导航",
            command=self.controller.navigate,
            style="Nav.TButton",
        )
        self.navigate_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))
        self.clear_obstacles_button = ttk.Button(
            side,
            textvariable=self.controller.obstacle_var,
            command=self.controller.clear_obstacles,
            state=tk.DISABLED,
        )
        self.clear_obstacles_button.pack(fill=tk.X, pady=(0, 10))

        ttk.Separator(side).pack(fill=tk.X, pady=6)
        self._status_row(side, "当前状态", self.controller.status_var)
        self._status_row(side, "任务阶段", self.controller.stage_var)
        self._status_row(side, "路线指标", self.controller.metric_var)
        ttk.Separator(side).pack(fill=tk.X, pady=8)
        ttk.Label(side, text="动作序列", font=("Sans", 10, "bold")).pack(anchor=tk.W)
        action_frame = ttk.Frame(side)
        action_frame.pack(fill=tk.BOTH, expand=True, pady=(5, 0))
        self.action_table = ttk.Treeview(
            action_frame,
            columns=("type", "node", "lane"),
            show="headings",
            height=13,
        )
        self.action_table.heading("type", text="动作")
        self.action_table.heading("node", text="位置")
        self.action_table.heading("lane", text="道路")
        self.action_table.column("type", width=58, stretch=False)
        self.action_table.column("node", width=86, stretch=False)
        self.action_table.column("lane", width=150, stretch=True)
        action_scroll = ttk.Scrollbar(
            action_frame, orient=tk.VERTICAL, command=self.action_table.yview
        )
        self.action_table.configure(yscrollcommand=action_scroll.set)
        self.action_table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        action_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        status = ttk.Frame(self.root, padding=(12, 4, 12, 10))
        status.pack(fill=tk.X)
        ttk.Label(status, textvariable=self.controller.status_var).pack(side=tk.LEFT)

    @staticmethod
    def _status_row(parent: Any, title: str, variable: tk.StringVar) -> None:
        ttk.Label(parent, text=title, foreground="#5f6872").pack(
            anchor=tk.W, pady=(3, 0)
        )
        ttk.Label(parent, textvariable=variable, wraplength=320, justify=tk.LEFT).pack(
            anchor=tk.W, pady=(1, 5)
        )

    def selected_task_ids(self) -> list[str]:
        return [
            self.target_list.get(index) for index in self.target_list.curselection()
        ]

    def set_targets_enabled(self, enabled: bool) -> None:
        self.target_list.configure(state=tk.NORMAL if enabled else tk.DISABLED)

    def set_mode_enabled(self, enabled: bool) -> None:
        self.mode_box.configure(state="readonly" if enabled else tk.DISABLED)

    def set_navigate_text(self, text: str) -> None:
        self.navigate_button.configure(text=text)

    def set_commands_enabled(self, preview: bool, navigate: bool) -> None:
        self.preview_button.configure(state=tk.NORMAL if preview else tk.DISABLED)
        self.navigate_button.configure(state=tk.NORMAL if navigate else tk.DISABLED)

    def set_hold_enabled(self, enabled: bool) -> None:
        self.hold_button.configure(state=tk.NORMAL if enabled else tk.DISABLED)

    def set_clear_obstacles_enabled(self, enabled: bool) -> None:
        self.clear_obstacles_button.configure(
            state=tk.NORMAL if enabled else tk.DISABLED
        )

    def update_actions(self, maneuvers: Any, name_for: Any) -> None:
        self.action_table.delete(*self.action_table.get_children())
        for maneuver in maneuvers:
            lane = maneuver.outgoing_lane_id or maneuver.incoming_lane_id
            self.action_table.insert(
                "", tk.END, values=(name_for(maneuver.type), maneuver.node_id, lane)
            )

    def _transform(self) -> tuple[float, float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        arena = self.controller.config["arena"]
        arena_width = float(arena["width_m"])
        arena_height = float(arena["height_m"])
        padding = 28.0
        scale = min(
            (width - 2.0 * padding) / arena_width,
            (height - 2.0 * padding) / arena_height,
        )
        return (
            scale,
            0.5 * (width - arena_width * scale),
            0.5 * (height - arena_height * scale),
        )

    def _point(self, x: float, y: float) -> tuple[float, float]:
        scale, offset_x, offset_y = self._transform()
        return offset_x + x * scale, self.canvas.winfo_height() - offset_y - y * scale

    def draw(self) -> None:
        self.canvas.delete("all")
        config = self.controller.config
        scale, _, _ = self._transform()
        for region in config["arena"]["free_regions"]:
            x1, y1 = self._point(float(region[0]), float(region[1]))
            x2, y2 = self._point(float(region[2]), float(region[3]))
            self.canvas.create_rectangle(x1, y2, x2, y1, fill="#303841", outline="")
        for obstacle in config["arena"]["obstacles"]:
            x1, y1 = self._point(float(obstacle[0]), float(obstacle[1]))
            x2, y2 = self._point(float(obstacle[2]), float(obstacle[3]))
            self.canvas.create_rectangle(
                x1, y2, x2, y1, fill=OBSTACLE, outline="#606b75"
            )

        nodes = config["nodes"]
        for lane in config["lanes"]:
            begin = nodes[lane["from"]]
            end = nodes[lane["to"]]
            x1, y1 = self._point(float(begin[0]), float(begin[1]))
            x2, y2 = self._point(float(end[0]), float(end[1]))
            color = TUNNEL if bool(lane["tunnel"]) else ROAD
            self.canvas.create_line(
                x1,
                y1,
                x2,
                y2,
                fill=color,
                width=max(2.0, float(lane["width_m"]) * scale),
            )
        for position in nodes.values():
            x, y = self._point(float(position[0]), float(position[1]))
            self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=NODE, outline="")

        for task_id, node_id in config["tasks"].items():
            position = nodes[node_id]
            x, y = self._point(float(position[0]), float(position[1]))
            self.canvas.create_oval(
                x - 7, y - 7, x + 7, y + 7, fill=TASK, outline="#242a30"
            )
            self.canvas.create_text(
                x, y, text=str(task_id), fill="#111820", font=("Sans", 8, "bold")
            )

        for polygon in self.controller.dynamic_obstacles:
            arena_points = [
                map_to_arena(point[0], point[1], 0.0, config)[:2] for point in polygon
            ]
            canvas_points = [self._point(*point) for point in arena_points]
            self.canvas.create_polygon(
                *sum(canvas_points, ()), fill="#b42318", outline="#ffb4ab", width=2
            )

        plan = self.controller.plan
        if plan is not None and plan.arena_path.poses:
            points = [
                self._point(float(item.pose.position.x), float(item.pose.position.y))
                for item in plan.arena_path.poses
            ]
            if len(points) >= 2:
                self.canvas.create_line(
                    *sum(points, ()), fill=ROUTE, width=4, arrow=tk.LAST
                )
            for maneuver in plan.maneuvers:
                if int(maneuver.type) not in (1, 2):
                    continue
                # Turn nodes are drawn using their configured arena coordinate.
                if maneuver.node_id in nodes:
                    position = nodes[maneuver.node_id]
                    x, y = self._point(float(position[0]), float(position[1]))
                    self.canvas.create_oval(
                        x - 5, y - 5, x + 5, y + 5, outline=TURN, width=2
                    )

        if self.controller.current_pose_arena is not None:
            x, y, yaw = self.controller.current_pose_arena
            center_x, center_y = self._point(x, y)
            length = float(self.controller.physical.length_m) * scale
            width = float(self.controller.physical.width_m) * scale
            points = []
            for local_x, local_y in (
                (0.55 * length, 0.0),
                (-0.45 * length, 0.5 * width),
                (-0.45 * length, -0.5 * width),
            ):
                world_x = center_x + math.cos(yaw) * local_x + math.sin(yaw) * local_y
                world_y = center_y - math.sin(yaw) * local_x + math.cos(yaw) * local_y
                points.extend((world_x, world_y))
            self.canvas.create_polygon(
                *points, fill=VEHICLE, outline="#ffffff", width=1
            )
