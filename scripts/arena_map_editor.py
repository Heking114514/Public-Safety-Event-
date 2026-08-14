#!/usr/bin/env python3
"""Grid-based arena map editor.

The editor uses a 0.15 m grid.  Draw the outer boundary first, then the inner
obstacle boundary.  Cells between the two polygons become road cells and cells
inside the inner polygon become obstacles.  The resulting YAML is accepted by
arena_path_planner via ``scripts/start_arena_planner.sh --map``.
"""

from __future__ import annotations

import math
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Iterable

import yaml


CELL_M = 0.15
ROAD = "#e5e7eb"
OBSTACLE = "#252b31"
OUTER = "#2563eb"
INNER = "#dc2626"
TASK = "#f59e0b"
START = "#16a34a"
GRID = "#94a3b8"


def inside(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    """Return whether a point is inside a polygon using ray casting."""
    if len(polygon) < 3:
        return False
    x, y = point
    result = False
    previous = polygon[-1]
    for current in polygon:
        x0, y0 = previous
        x1, y1 = current
        crosses = (y0 > y) != (y1 > y)
        if crosses and x < (x1 - x0) * (y - y0) / (y1 - y0) + x0:
            result = not result
        previous = current
    return result


def snap(value: float) -> float:
    return round(value / CELL_M) * CELL_M


class MapEditor:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("15cm 网格地图编辑器")
        self.root.geometry("1120x820")
        self.root.minsize(850, 650)
        self.root.configure(bg="#f8fafc")
        self.canvas = tk.Canvas(root, bg="#f8fafc", highlightthickness=1,
                                highlightbackground="#64748b")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0), pady=10)
        panel = ttk.Frame(root, width=270, padding=12)
        panel.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)
        panel.pack_propagate(False)

        # Default arena is 40 x 40 cells. Each cell represents 0.15 m.
        self.width_var = tk.DoubleVar(value=40 * CELL_M)
        self.height_var = tk.DoubleVar(value=40 * CELL_M)
        self.zoom = 1.0
        self.phase_var = tk.StringVar(value="第 1 步：点击外环点")
        self.info_var = tk.StringVar(value="网格边长 0.15 m")
        self.outer: list[tuple[float, float]] = []
        self.inner: list[tuple[float, float]] = []
        self.inner_rings: list[list[tuple[float, float]]] = []
        self.tasks: list[tuple[float, float]] = []
        self.start: tuple[float, float] | None = None
        self.phase = "outer"
        self.task_mode = False
        self.start_mode = False
        self.closed_outer = False
        self.closed_inner = False

        ttk.Label(panel, text="15cm 网格地图编辑器", font=("Sans", 14, "bold")).pack(anchor=tk.W)
        ttk.Label(panel, textvariable=self.phase_var, wraplength=240).pack(anchor=tk.W, pady=(10, 4))
        ttk.Label(panel, textvariable=self.info_var, wraplength=240).pack(anchor=tk.W, pady=(0, 8))
        self._field(panel, "地图宽度 (m)", self.width_var)
        self._field(panel, "地图高度 (m)", self.height_var)
        ttk.Button(panel, text="应用尺寸", command=self.resize).pack(fill=tk.X, pady=3)
        ttk.Separator(panel).pack(fill=tk.X, pady=8)
        ttk.Button(panel, text="闭合外环 -> 内环", command=self.close_outer).pack(fill=tk.X, pady=3)
        ttk.Button(panel, text="闭合当前内环", command=self.close_inner).pack(fill=tk.X, pady=3)
        ttk.Button(panel, text="完成所有内环 -> 生成道路", command=self.finish_inner).pack(fill=tk.X, pady=3)
        ttk.Button(panel, text="设置起点", command=self.enable_start).pack(fill=tk.X, pady=3)
        ttk.Button(panel, text="添加任务点", command=self.enable_task).pack(fill=tk.X, pady=3)
        ttk.Button(panel, text="撤销最后一点", command=self.undo).pack(fill=tk.X, pady=3)
        ttk.Button(panel, text="清空重画", command=self.clear).pack(fill=tk.X, pady=3)
        ttk.Button(panel, text="导出 YAML", command=self.export).pack(fill=tk.X, pady=(12, 3))
        ttk.Label(panel, text=(
            "操作顺序：\n"
            "1. 点击外环顶点，闭合外环\n"
            "2. 点击内环顶点，闭合当前内环\n"
            "3. 继续点击下一个内环，最后完成所有内环\n"
            "4. 可选设置起点和任务点\n"
            "5. 导出后用 --map 加载\n\n"
            "蓝色=外环，红色=内环，灰色=道路，深色=障碍"
        ), justify=tk.LEFT, wraplength=240).pack(anchor=tk.W, pady=(14, 0))
        self.canvas.bind("<Button-1>", self.click)
        self.canvas.bind("<MouseWheel>", self.zoom_wheel)
        self.canvas.bind("<Button-4>", lambda _event: self.change_zoom(1.15))
        self.canvas.bind("<Button-5>", lambda _event: self.change_zoom(1.0 / 1.15))
        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.resize()

    def _field(self, parent: ttk.Frame, label: str, variable: tk.Variable) -> None:
        ttk.Label(parent, text=label).pack(anchor=tk.W, pady=(3, 1))
        ttk.Entry(parent, textvariable=variable).pack(fill=tk.X)

    def resize(self) -> None:
        self.width_var.set(max(CELL_M, round(float(self.width_var.get()) / CELL_M) * CELL_M))
        self.height_var.set(max(CELL_M, round(float(self.height_var.get()) / CELL_M) * CELL_M))
        self.draw()

    def transform(self) -> tuple[float, float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        wm, hm = self.width_var.get(), self.height_var.get()
        scale = min((width - 32) / wm, (height - 32) / hm) * self.zoom
        return scale, (width - wm * scale) * 0.5, (height - hm * scale) * 0.5

    def change_zoom(self, factor: float) -> None:
        self.zoom = max(0.35, min(5.0, self.zoom * factor))
        self.info_var.set(
            f"网格边长 {CELL_M:.2f} m    缩放 {self.zoom:.2f}x")
        self.draw()

    def zoom_wheel(self, event: tk.Event) -> None:
        # Windows/macOS report a delta; Linux commonly uses Button-4/5 above.
        delta = getattr(event, "delta", 0)
        self.change_zoom(1.15 if delta > 0 else 1.0 / 1.15)

    def to_canvas(self, point: tuple[float, float]) -> tuple[float, float]:
        scale, ox, oy = self.transform()
        return ox + point[0] * scale, self.canvas.winfo_height() - oy - point[1] * scale

    def to_world(self, x: float, y: float) -> tuple[float, float]:
        scale, ox, oy = self.transform()
        return snap((x - ox) / scale), snap((self.canvas.winfo_height() - oy - y) / scale)

    def cells(self) -> Iterable[tuple[int, int, bool]]:
        columns = int(round(self.width_var.get() / CELL_M))
        rows = int(round(self.height_var.get() / CELL_M))
        for column in range(columns):
            for row in range(rows):
                center = ((column + 0.5) * CELL_M, (row + 0.5) * CELL_M)
                if inside(center, self.outer):
                    blocked = any(inside(center, ring) for ring in self.inner_rings)
                    if self.inner and self.closed_inner:
                        blocked = blocked or inside(center, self.inner)
                    yield column, row, blocked

    def draw(self) -> None:
        self.canvas.delete("all")
        scale, _, _ = self.transform()
        columns = int(round(self.width_var.get() / CELL_M))
        rows = int(round(self.height_var.get() / CELL_M))
        for column in range(columns + 1):
            a, b = self.to_canvas((column * CELL_M, 0))
            _, c = self.to_canvas((column * CELL_M, self.height_var.get()))
            self.canvas.create_line(a, b, a, c, fill=GRID, width=1)
            if column % 10 == 0:
                self.canvas.create_line(a, b, a, b + 7, fill="#475569", width=2)
                self.canvas.create_text(
                    a, b + 16, text=f"{column}格 / {column * CELL_M:.2f}m",
                    fill="#334155", font=("Sans", 8), anchor=tk.N,
                )
        for row in range(rows + 1):
            a, b = self.to_canvas((0, row * CELL_M))
            c, _ = self.to_canvas((self.width_var.get(), row * CELL_M))
            self.canvas.create_line(a, b, c, b, fill=GRID, width=1)
            if row % 10 == 0:
                self.canvas.create_line(a, b, a - 7, b, fill="#475569", width=2)
                self.canvas.create_text(
                    a - 12, b, text=f"{row}格 / {row * CELL_M:.2f}m",
                    fill="#334155", font=("Sans", 8), anchor=tk.E,
                )
        if self.closed_inner:
            for column, row, blocked in self.cells():
                x0, y0 = self.to_canvas((column * CELL_M, row * CELL_M))
                x1, y1 = self.to_canvas(((column + 1) * CELL_M, (row + 1) * CELL_M))
                self.canvas.create_rectangle(x0, y0, x1, y1,
                    fill=OBSTACLE if blocked else ROAD, outline="")
        self._polyline(self.outer, OUTER, self.closed_outer)
        for ring in self.inner_rings:
            self._polyline(ring, INNER, True)
        self._polyline(self.inner, INNER, False)
        for index, point in enumerate(self.outer, 1): self._marker(point, str(index), OUTER)
        ring_number = 1
        for ring in self.inner_rings:
            for index, point in enumerate(ring, 1): self._marker(point, f"I{ring_number}.{index}", INNER)
            ring_number += 1
        for index, point in enumerate(self.inner, 1): self._marker(point, f"I{ring_number}.{index}", INNER)
        for index, point in enumerate(self.tasks, 1): self._marker(point, f"T{index}", TASK)
        if self.start is not None: self._marker(self.start, "S", START)

    def _polyline(self, points: list[tuple[float, float]], colour: str, closed: bool) -> None:
        if len(points) < 2: return
        coordinates = [value for point in points for value in self.to_canvas(point)]
        if closed: coordinates += list(self.to_canvas(points[0]))
        self.canvas.create_line(*coordinates, fill=colour, width=3)

    def _marker(self, point: tuple[float, float], label: str, colour: str) -> None:
        x, y = self.to_canvas(point)
        self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=colour, outline="white")
        self.canvas.create_text(x + 9, y - 9, text=label, fill=colour, anchor=tk.W)

    def click(self, event: tk.Event) -> None:
        point = self.to_world(event.x, event.y)
        if self.start_mode:
            self.start = point; self.start_mode = False; self.phase_var.set("已设置起点，可添加任务点"); self.draw(); return
        if self.task_mode:
            self.tasks.append(point); self.draw(); return
        if self.phase == "outer": self.outer.append(point)
        elif self.phase == "inner": self.inner.append(point)
        self.draw()

    def close_outer(self) -> None:
        if len(self.outer) < 3: messagebox.showwarning("点数不足", "外环至少需要 3 个点"); return
        self.closed_outer = True; self.phase = "inner"; self.phase_var.set("第 2 步：点击内环点"); self.draw()

    def close_inner(self) -> None:
        if len(self.inner) < 3: messagebox.showwarning("点数不足", "内环至少需要 3 个点"); return
        self.inner_rings.append(list(self.inner))
        self.inner.clear()
        self.closed_inner = True
        self.phase = "inner"
        self.phase_var.set(f"已闭合 {len(self.inner_rings)} 个内环，可继续点击下一个内环")
        self.draw()

    def finish_inner(self) -> None:
        if self.inner:
            if len(self.inner) < 3:
                messagebox.showwarning("点数不足", "当前内环至少需要 3 个点，或撤销这些点")
                return
            self.inner_rings.append(list(self.inner))
            self.inner.clear()
        if not self.inner_rings:
            messagebox.showwarning("尚未完成", "请至少闭合一个内环")
            return
        self.closed_inner = True
        self.phase = "done"
        self.phase_var.set(f"已完成 {len(self.inner_rings)} 个内环，道路已生成")
        self.draw()

    def enable_start(self) -> None:
        self.start_mode = True; self.task_mode = False; self.phase_var.set("请点击地图设置起点")

    def enable_task(self) -> None:
        if self.phase != "done": messagebox.showwarning("尚未完成", "请先点击“完成所有内环 -> 生成道路”"); return
        self.task_mode = not self.task_mode; self.start_mode = False
        self.phase_var.set("任务点模式：点击添加，再点按钮退出" if self.task_mode else "已退出任务点模式")

    def undo(self) -> None:
        if self.task_mode and self.tasks: self.tasks.pop()
        elif self.phase == "inner" and self.inner: self.inner.pop()
        elif self.phase == "inner" and self.inner_rings:
            self.inner = self.inner_rings.pop()
            self.closed_inner = bool(self.inner_rings)
        elif self.phase == "outer" and self.outer: self.outer.pop()
        self.draw()

    def clear(self) -> None:
        self.outer.clear(); self.inner.clear(); self.inner_rings.clear(); self.tasks.clear(); self.start = None
        self.phase = "outer"; self.closed_outer = False; self.closed_inner = False
        self.task_mode = False; self.start_mode = False; self.phase_var.set("第 1 步：点击外环点"); self.draw()

    def export(self) -> None:
        if not self.closed_inner: messagebox.showwarning("尚未完成", "请先闭合外环和内环"); return
        path = filedialog.asksaveasfilename(defaultextension=".yaml", filetypes=[("YAML", "*.yaml")])
        if not path: return
        width, height = self.width_var.get(), self.height_var.get()
        free_regions, obstacles = [], []
        road_cells: list[tuple[int, int]] = []
        for column, row, blocked in self.cells():
            rect = [column * CELL_M, row * CELL_M, (column + 1) * CELL_M, (row + 1) * CELL_M]
            (obstacles if blocked else free_regions).append([round(value, 4) for value in rect])
            if not blocked:
                road_cells.append((column, row))
        if not road_cells:
            messagebox.showwarning("没有道路", "内环与外环之间没有可用道路格")
            return
        road_cells.sort(key=lambda item: (item[1], item[0]))
        node_index = {cell: index for index, cell in enumerate(road_cells)}
        nodes = [[round((column + 0.5) * CELL_M, 4), round((row + 0.5) * CELL_M, 4)]
                 for column, row in road_cells]
        edges = []
        for column, row in road_cells:
            for dc, dr in ((1, 0), (0, 1)):
                neighbour = (column + dc, row + dr)
                if neighbour in node_index:
                    edges.append([node_index[(column, row)], node_index[neighbour],
                                  f"ROAD_{node_index[(column, row)]}_{node_index[neighbour]}", False])
        start = self.start or tuple(nodes[0])
        tasks = {str(index): [round(x, 4), round(y, 4)] for index, (x, y) in enumerate(self.tasks, 1)}
        if not tasks: tasks = {"1": list(start)}
        document = {
            "arena": {"width_m": width, "height_m": height, "resolution_m": 0.025,
                      "free_regions": free_regions, "obstacles": obstacles,
                      "inflation_radius_m": 0.05, "preferred_clearance_m": 0.15,
                      "clearance_cost_weight": 6.0},
            "start": {"position_m": [round(start[0], 4), round(start[1], 4)], "heading_deg": 0.0},
            "tasks": tasks,
            "inspection_graph": {"nodes": nodes, "edges": edges},
            "planning": {"mode": "shortest", "task_tolerance_m": 0.08,
                         "waypoint_spacing_m": 0.15, "curve_spacing_m": 0.025,
                         "minimum_turning_radius_m": 0.22, "maximum_heading_step_deg": 8.0},
        }
        with Path(path).open("w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False, allow_unicode=False)
        messagebox.showinfo("导出完成", f"已保存地图：\n{path}\n\n启动：bash scripts/start_arena_planner.sh --map {path}")


def main() -> int:
    root = tk.Tk(); MapEditor(root); root.mainloop(); return 0


if __name__ == "__main__":
    raise SystemExit(main())
