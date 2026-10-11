#!/usr/bin/env python3
"""使用正式三阶段道路语义的 Matplotlib 规划演示。"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MAP = ROOT / "src/new_map_planner/config/arena.yaml"


def graph_route(nodes, lanes, start, goal, blocked):
    graph = {}
    for lane in lanes:
        if lane["id"] in blocked:
            continue
        graph.setdefault(lane["from"], []).append((lane["to"], lane["id"]))
        if lane.get("bidirectional", False):
            graph.setdefault(lane["to"], []).append((lane["from"], lane["id"]))
    queue, previous = deque([start]), {start: None}
    while queue:
        node = queue.popleft()
        if node == goal:
            break
        for target, lane_id in graph.get(node, []):
            if target not in previous:
                previous[target] = (node, lane_id)
                queue.append(target)
    if goal not in previous:
        return None
    route, node = [], goal
    while previous[node] is not None:
        source, lane_id = previous[node]
        route.append((source, node, lane_id))
        node = source
    return list(reversed(route))


class Demo:
    def __init__(self, data, log_path):
        self.nodes, self.lanes = data["nodes"], data["lanes"]
        self.tasks = [(data["tasks"][str(i)], str(i)) for i in range(1, 13)]
        inspection = set(data.get("missions", {}).get("default_inspection_lanes", []))
        self.tunnels = [lane["id"] for lane in self.lanes if lane["id"] in inspection and lane.get("tunnel", False)]
        self.ordinary = [lane["id"] for lane in self.lanes if lane["id"] in inspection and not lane.get("tunnel", False)]
        self.stage, self.current, self.target_index = 0, "launch", 0
        self.goal, self.route, self.route_step, self.fraction = "launch", None, 0, 0.0
        self.covered, self.blocked, self.obstacles = set(), set(), []
        self.running, self.finished = False, False
        self.log_path = log_path
        self.figure, self.axis = plt.subplots(figsize=(13, 8))
        self.figure.subplots_adjust(right=0.78, left=0.04, bottom=0.10)
        self.panel = self.figure.add_axes((0.80, 0.10, 0.18, 0.80)); self.panel.axis("off")
        button_axis = self.figure.add_axes((0.80, 0.04, 0.08, 0.045))
        Button(button_axis, "开始", color="#b7e4c7", hovercolor="#95d5b2").on_clicked(self.start)
        self.figure.canvas.mpl_connect("button_press_event", self.click)
        self.animation = FuncAnimation(self.figure, self.update, interval=80, cache_frame_data=False)
        self.log("程序启动：开始后点击道路投放最多三个障碍")
        self.draw()

    def log(self, message):
        line = f"{datetime.now():%H:%M:%S} {message}"
        with self.log_path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")
        print(line)

    def point(self, node):
        return self.nodes[node]

    def lane(self, lane_id):
        return next(lane for lane in self.lanes if lane["id"] == lane_id)

    def start(self, _event=None):
        if self.finished:
            self.stage, self.current, self.target_index = 0, "launch", 0
            self.covered.clear(); self.blocked.clear(); self.obstacles.clear(); self.finished = False
        if not self.running:
            self.running = True
            self.plan()
            self.log("开始三阶段任务")

    def plan(self):
        self.route = None
        if self.stage == 0:
            if self.target_index >= len(self.tasks):
                self.stage, self.target_index = 1, 0
                self.log("layer1 完成，进入 layer2 隧道覆盖")
                return self.plan()
            self.goal, target_id = self.tasks[self.target_index]
            self.route = graph_route(self.nodes, self.lanes, self.current, self.goal, self.blocked)
            label = f"layer1 任务点 {target_id}"
        elif self.stage == 1:
            pending = [x for x in self.tunnels if x not in self.covered]
            if not pending:
                self.stage, self.target_index = 2, 0
                self.log("layer2 完成，进入 layer3 普通道路补漏与回程")
                return self.plan()
            lane = self.lane(pending[0]); self.goal = lane["to"]
            prefix = None if lane["id"] in self.blocked else graph_route(self.nodes, self.lanes, self.current, lane["from"], self.blocked)
            self.route = (prefix or []) + [(lane["from"], lane["to"], lane["id"])] if prefix is not None else None
            label = f"layer2 隧道 {lane['id']}"
        else:
            pending = [x for x in self.ordinary if x not in self.covered]
            if pending:
                lane = self.lane(pending[0]); self.goal = lane["to"]
                prefix = None if lane["id"] in self.blocked else graph_route(self.nodes, self.lanes, self.current, lane["from"], self.blocked)
                self.route = (prefix or []) + [(lane["from"], lane["to"], lane["id"])] if prefix is not None else None
                label = f"layer3 道路 {lane['id']}"
            else:
                self.goal = "launch"
                self.route = graph_route(self.nodes, self.lanes, self.current, self.goal, self.blocked)
                label = "layer3 回到发车点"
        self.route_step, self.fraction = 0, 0.0
        if self.route:
            self.log(label + "：" + " -> ".join([self.route[0][0]] + [x[1] for x in self.route]))
        else:
            self.running = False; self.log(label + "无可行路线")

    def click(self, event):
        if event.inaxes is not self.axis or len(self.obstacles) >= 3:
            return
        lane = self.nearest_lane(event.xdata, event.ydata)
        if lane is None or lane["id"] in self.blocked:
            return
        self.blocked.add(lane["id"]); self.obstacles.append((event.xdata, event.ydata))
        self.log(f"动态障碍 {len(self.obstacles)}/3：封闭 {lane['id']}")
        if self.running:
            self.plan()
        self.draw()

    def nearest_lane(self, x, y):
        if x is None or y is None: return None
        best = None
        for lane in self.lanes:
            x1, y1 = self.point(lane["from"]); x2, y2 = self.point(lane["to"])
            dx, dy = x2-x1, y2-y1
            t = max(0, min(1, ((x-x1)*dx+(y-y1)*dy)/max(dx*dx+dy*dy, 1e-9)))
            distance = (x-x1-t*dx)**2+(y-y1-t*dy)**2
            if best is None or distance < best[0]: best = distance, lane
        return best[1] if best and best[0] <= 0.05**2 else None

    def update(self, _frame):
        if self.running and self.route:
            if self.route_step >= len(self.route):
                self.current = self.goal
                for _, _, lane_id in self.route:
                    if lane_id in self.tunnels or lane_id in self.ordinary: self.covered.add(lane_id)
                if self.stage == 0:
                    self.target_index += 1
                elif self.stage == 2 and self.goal == "launch":
                    self.running, self.finished = False, True; self.log("三阶段任务完成")
                self.plan()
            else:
                self.fraction += 0.035
                if self.fraction >= 1:
                    self.current = self.route[self.route_step][1]; self.route_step += 1; self.fraction = 0
        self.draw(); return ()

    def draw(self):
        self.axis.clear(); route_ids = {x[2] for x in self.route or []}
        for lane in self.lanes:
            x1,y1=self.point(lane["from"]); x2,y2=self.point(lane["to"]); lid=lane["id"]
            color="#dc2626" if lid in self.blocked else "#16a34a" if lid in route_ids else "#a8b0ba"
            self.axis.plot([x1,x2],[y1,y2],color=color,linewidth=4 if lid in route_ids else 1.7)
            self.axis.text((x1+x2)/2,(y1+y2)/2,lid,fontsize=6,color=color)
        for node,(x,y) in self.nodes.items():
            color="#06b6d4" if node==self.current else "#f59e0b" if node==self.goal else "white"
            self.axis.scatter(x,y,s=75,c=color,edgecolors="#263238",zorder=3); self.axis.text(x+.04,y+.04,node,fontsize=7)
        if self.running and self.route and self.route_step<len(self.route):
            a,b,_=self.route[self.route_step]; x1,y1=self.point(a); x2,y2=self.point(b)
            self.axis.scatter(x1+(x2-x1)*self.fraction,y1+(y2-y1)*self.fraction,s=150,c="#00a6d6",edgecolors="#075985",zorder=5)
        for x,y in self.obstacles: self.axis.scatter(x,y,marker="X",s=130,c="#7f1d1d",zorder=6)
        self.axis.set_aspect("equal"); self.axis.set_xlim(-.15,3.3); self.axis.set_ylim(-.15,4.5); self.axis.grid(alpha=.2)
        self.axis.set_title("正式三阶段道路语义动态规划演示")
        state="运行中" if self.running else "已完成" if self.finished else "待机"
        names=["layer1 任务点","layer2 隧道","layer3 补漏回程"]
        self.panel.clear(); self.panel.axis("off"); self.panel.text(0,.95,f"状态：{state}\n阶段：{names[min(self.stage,2)]}\n当前位置：{self.current}\n目标：{self.goal}\n已覆盖道路：{len(self.covered)}\n障碍：{len(self.obstacles)}/3\n\n运行中点击道路投放障碍\n日志：{self.log_path}",va="top",fontsize=10)


def main():
    parser=argparse.ArgumentParser(description="正式三阶段 Matplotlib 避障演示")
    parser.add_argument("--map",type=Path,default=DEFAULT_MAP); parser.add_argument("--log",type=Path,default=ROOT/"three_stage_obstacle_demo.log")
    args=parser.parse_args(); data=yaml.safe_load(args.map.read_text(encoding="utf-8")); Demo(data,args.log); plt.show()


if __name__ == "__main__": main()
