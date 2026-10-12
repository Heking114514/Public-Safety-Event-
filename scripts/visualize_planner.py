#!/usr/bin/env python3
"""Offline matplotlib view of the new_map_planner road and collision logic.

The C++ planner is deliberately kept ROS independent here.  This tool reads
the same arena.yaml, builds its directed road graph, and reports two checks:

* ``centerline``: a point robot can traverse an edge;
* ``footprint``: the configured vehicle (axis-aligned rectangular sweep on
  cardinal roads) can traverse the edge while respecting static/dynamic
  obstacles and the free regions.

Edges that pass the first check but fail the second are highlighted in orange;
these are useful for finding obstacle-avoidance bugs.  The plot can be saved
for CI/headless use or shown interactively.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path
from typing import Iterable, Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from matplotlib.widgets import Button, TextBox
import yaml


Point = tuple[float, float]
Rect = tuple[float, float, float, float]


def load_planner_metadata(path: Path) -> dict:
    """Load optional mission metadata without depending on ROS messages."""
    with path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    tasks = {str(k): str(v) for k, v in (raw.get("tasks") or {}).items()}
    supernodes = {str(node): str(task) for node, task in (raw.get("rfid_supernodes") or {}).items()}
    task_to_supernode = {task: node for node, task in supernodes.items()}
    missions = raw.get("missions") or {}
    return {
        "tasks": tasks,
        "supernodes": supernodes,
        "task_to_supernode": task_to_supernode,
        "default_targets": [str(v) for v in missions.get("default_targets", [])],
        "default_inspection_lanes": [str(v) for v in missions.get("default_inspection_lanes", [])],
    }


def lane_sequence(route: Sequence[str], lanes: Sequence[tuple]) -> list[dict]:
    """Resolve each route hop to a lane and preserve its travel direction."""
    result = []
    for source, target in zip(route, route[1:]):
        match = next((lane for lane in lanes
                      if ((lane[1] == source and lane[2] == target) or
                          (lane[3] and lane[1] == target and lane[2] == source))), None)
        if match:
            result.append({"lane_id": match[0], "from": source, "to": target,
                           "direction": f"{source}->{target}"})
    return result


def recovery_candidates(nodes: dict[str, Point], lanes: Sequence[tuple],
                        blocked_lane_ids: set[str], current: str,
                        heading: str = "") -> list[dict]:
    """Return inspectable, conservative recovery exits from a graph node.

    This is intentionally an assessment-only view. It does not generate a
    reverse path or claim that an unimplemented controller action is available.
    """
    candidates = []
    for lane_id, src, dst, bidirectional in lanes:
        if lane_id in blocked_lane_ids:
            continue
        if src == current:
            resume = dst
        elif bidirectional and dst == current:
            resume = src
        else:
            continue
        candidates.append({"candidate_id": f"{lane_id}:{resume}",
                           "resume_node": resume, "resume_lane": lane_id,
                           "resume_heading": heading or f"{current}->{resume}",
                           "recovery_template": "T_JUNCTION" if len(nodes) else "",
                           "recovery_edges": [lane_id]})
    return candidates


def build_trace(path: Path, route: Sequence[str], lanes: Sequence[tuple],
                blocked_lane_ids: Iterable[str] = (), obstacle_revision: int = 0,
                ordinary_count: int = 0, start: str = "", goal: str = "",
                graph_nodes: dict[str, Point] | None = None,
                recovery_input: Sequence[dict] | None = None) -> dict:
    metadata = load_planner_metadata(path)
    blocked = sorted(set(blocked_lane_ids))
    supernode_by_node = {metadata["tasks"].get(task, ""): supernode
                          for supernode, task in metadata["supernodes"].items()}
    count = int(ordinary_count)
    nodes_trace = []
    for node in route:
        supernode = supernode_by_node.get(node)
        if supernode:
            count = 0
        else:
            count = min(count + 1, 2)
        nodes_trace.append({"node": node, "supernode_id": supernode,
                            "node_type": "supernode" if supernode else "ordinary",
                            "ordinary_count": count})
    lane_trace = lane_sequence(route, lanes)
    return {
        "start": start or (route[0] if route else ""),
        "goal": goal or (route[-1] if route else ""),
        "route_nodes": list(route), "route_lanes": lane_trace,
        "nodes": nodes_trace, "supernodes": metadata["supernodes"],
        "ordinary_count": count, "blocked_lane_ids": blocked,
        "obstacle_revision": int(obstacle_revision),
        "recovery_candidates": [candidate for candidate in (
            list(recovery_input) if recovery_input is not None else recovery_candidates(
                graph_nodes or {node: (0.0, 0.0) for node in metadata["tasks"].values()}, lanes,
                set(blocked), route[-1] if route else ""))
            if not set(candidate.get("recovery_edges", [])) & set(blocked)],
    }


    x, y = (float(v) for v in value.split(","))
    return x, y


def parse_rect(value: str) -> Rect:
    x1, y1, x2, y2 = (float(v) for v in value.split(","))
    return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def segment_intersects_rect(a: Point, b: Point, rect: Rect) -> bool:
    """Liang-Barsky segment/AABB test, including touching the boundary."""
    x0, y0 = a
    dx, dy = b[0] - x0, b[1] - y0
    xmin, ymin, xmax, ymax = rect
    p = (-dx, dx, -dy, dy)
    q = (x0 - xmin, xmax - x0, y0 - ymin, ymax - y0)
    t0, t1 = 0.0, 1.0
    for pi, qi in zip(p, q):
        if abs(pi) < 1e-12:
            if qi < 0:
                return False
            continue
        t = qi / pi
        if pi < 0:
            if t > t1:
                return False
            t0 = max(t0, t)
        else:
            if t < t0:
                return False
            t1 = min(t1, t)
    return True


def point_in_rect(p: Point, rect: Rect) -> bool:
    return rect[0] <= p[0] <= rect[2] and rect[1] <= p[1] <= rect[3]


def expand(rect: Rect, half_x: float, half_y: float) -> Rect:
    return rect[0] - half_x, rect[1] - half_y, rect[2] + half_x, rect[3] + half_y


def inside_free_segment(a: Point, b: Point, free_regions: Sequence[Rect], half_x: float = 0.0, half_y: float = 0.0, samples: int = 40) -> bool:
    # The C++ checker samples a swept footprint.  Sampling the centerline here
    # catches passages that leave the configured free regions.
    if not free_regions:
        return True
    for i in range(samples + 1):
        t = i / samples
        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        # Test the footprint corners against the *union* of free regions.
        # Requiring one rectangle to contain the whole footprint would create
        # a false gap where two configured free rectangles meet (the launch
        # neck is exactly such a junction).
        corners = ((p[0] - half_x, p[1] - half_y),
                   (p[0] - half_x, p[1] + half_y),
                   (p[0] + half_x, p[1] - half_y),
                   (p[0] + half_x, p[1] + half_y))
        if not all(any(point_in_rect(corner, region) for region in free_regions)
                   for corner in corners):
            return False
    return True


def load_config(path: Path):
    with path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    arena = raw["arena"]
    nodes = {name: tuple(float(v) for v in xy) for name, xy in raw["nodes"].items()}
    lanes = []
    for lane in raw["lanes"]:
        lanes.append((lane["id"], lane["from"], lane["to"], bool(lane.get("bidirectional", True))))
    free = [tuple(float(v) for v in r) for r in arena.get("free_regions", [])]
    static = [tuple(float(v) for v in r) for r in arena.get("obstacles", [])]
    vehicle = raw.get("vehicle", {})
    # The C++ loader takes dimensions from vehicle_config.  Read that file when
    # available so this view cannot silently drift from the real planner.
    physical_path = path.parent.parent.parent / "vehicle_config/config/vehicle_physical.yaml"
    if physical_path.exists():
        with physical_path.open(encoding="utf-8") as stream:
            vehicle = yaml.safe_load(stream).get("chassis", {})
    length = float(vehicle.get("length_m", raw.get("planning", {}).get("vehicle_length_m", 0.26)))
    width = float(vehicle.get("width_m", raw.get("planning", {}).get("vehicle_width_m", 0.20)))
    planning = raw.get("planning", {})
    margin = float(planning.get("safety_margin_m", 0.015))
    return arena, nodes, lanes, free, static, length, width, margin


def edge_status(a: Point, b: Point, free: Sequence[Rect], obstacles: Sequence[Rect], length: float, width: float, margin: float):
    dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
    horizontal = dx >= dy
    hx = (length / 2.0 if horizontal else width / 2.0) + margin
    hy = (width / 2.0 if horizontal else length / 2.0) + margin
    center_ok = inside_free_segment(a, b, free) and not any(segment_intersects_rect(a, b, r) for r in obstacles)
    swept_obstacles = [expand(r, hx, hy) for r in obstacles]
    footprint_ok = inside_free_segment(a, b, free, hx, hy) and not any(segment_intersects_rect(a, b, r) for r in swept_obstacles)
    return center_ok, footprint_ok


def dijkstra(nodes, lanes, free, obstacles, length, width, margin, start, goal,
             blocked_lane_ids: Iterable[str] = ()):
    blocked = set(blocked_lane_ids)
    outgoing = {name: [] for name in nodes}
    statuses = {}
    for lane_id, src, dst, bidirectional in lanes:
        if lane_id in blocked:
            continue
        for u, v in ((src, dst), (dst, src)) if bidirectional else ((src, dst),):
            center_ok, footprint_ok = edge_status(nodes[u], nodes[v], free, obstacles, length, width, margin)
            statuses[(u, v)] = (lane_id, center_ok, footprint_ok)
            if footprint_ok:
                outgoing[u].append((v, distance(nodes[u], nodes[v]), lane_id))
    queue = [(0.0, start)]
    cost, previous = {start: 0.0}, {}
    while queue:
        current_cost, u = heapq.heappop(queue)
        if current_cost != cost.get(u):
            continue
        if u == goal:
            break
        for v, weight, lane_id in outgoing[u]:
            candidate = current_cost + weight
            if candidate < cost.get(v, math.inf):
                cost[v] = candidate
                previous[v] = (u, lane_id)
                heapq.heappush(queue, (candidate, v))
    if goal not in cost:
        return [], statuses
    path, u = [goal], goal
    while u != start:
        u = previous[u][0]
        path.append(u)
    return list(reversed(path)), statuses


def nearest_node(nodes, p: Point) -> str:
    return min(nodes, key=lambda name: distance(nodes[name], p))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("src/new_map_planner/config/arena.yaml"))
    parser.add_argument("--start", default="launch", help="node id or x,y")
    parser.add_argument("--goal", default="x1_y4", help="node id or x,y")
    parser.add_argument("--obstacle", action="append", default=[], metavar="x1,y1,x2,y2", help="dynamic rectangle; repeatable")
    parser.add_argument("--save", type=Path, help="save PNG instead of opening a window")
    parser.add_argument("--gui", action="store_true", help="open interactive controls (default when --save is omitted)")
    parser.add_argument("--mode", choices=("map", "mission"), default="map", help="map view or mission metadata view")
    parser.add_argument("--show-supernodes", action="store_true", help="show supernode metadata in mission trace")
    parser.add_argument("--show-ordinary-count", action="store_true", help="include ordinary_count in mission trace")
    parser.add_argument("--show-blocked-edges", action="store_true", help="include blocked lane ids in mission trace")
    parser.add_argument("--blocked-lane", action="append", default=[], help="temporarily blocked lane id; repeatable")
    parser.add_argument("--obstacle-revision", type=int, default=0, help="obstacle snapshot revision for trace output")
    parser.add_argument("--ordinary-count", type=int, default=0, help="ordinary nodes already counted at the start")
    parser.add_argument("--trace", type=Path, help="write a JSON route/state trace")
    args = parser.parse_args()

    _, nodes, lanes, free, static, length, width, margin = load_config(args.config)
    start = nodes.get(args.start, parse_point(args.start) if "," in args.start else None)
    goal = nodes.get(args.goal, parse_point(args.goal) if "," in args.goal else None)
    if start is None or goal is None:
        parser.error("start/goal must be a node id or x,y")
    dynamic = [parse_rect(v) for v in args.obstacle]
    start_name = args.start if args.start in nodes else nearest_node(nodes, start)
    goal_name = args.goal if args.goal in nodes else nearest_node(nodes, goal)
    initial_path, _ = dijkstra(nodes, lanes, free, static + dynamic, length, width, margin,
                                start_name, goal_name, args.blocked_lane)
    if args.trace:
        trace = build_trace(args.config, initial_path, lanes, args.blocked_lane,
                            args.obstacle_revision, args.ordinary_count,
                            start_name, goal_name, nodes)
        # Keep the default map trace useful while allowing --mode mission to
        # request all metadata explicitly.
        if args.mode == "map" and not args.show_supernodes:
            trace.pop("supernodes", None)
        if args.mode == "map" and not args.show_ordinary_count:
            for item in trace["nodes"]:
                item.pop("ordinary_count", None)
            trace.pop("ordinary_count", None)
        if args.mode == "map" and not args.show_blocked_edges:
            trace.pop("blocked_lane_ids", None)
        args.trace.parent.mkdir(parents=True, exist_ok=True)
        args.trace.write_text(json.dumps(trace, ensure_ascii=False, indent=2), encoding="utf-8")
    radius = math.hypot(length, width) / 2.0 + margin
    fig, ax = plt.subplots(figsize=(10, 10))
    plt.subplots_adjust(bottom=.18, left=.08, right=.98)
    route_state = {"points": [], "car": None, "animation": None}
    def redraw(_event=None):
        nonlocal start, goal, dynamic
        try:
            start_value = start_box.text.strip(); goal_value = goal_box.text.strip()
            start = nodes.get(start_value, parse_point(start_value) if "," in start_value else None)
            goal = nodes.get(goal_value, parse_point(goal_value) if "," in goal_value else None)
            if start is None or goal is None: raise ValueError("start/goal must be node id or x,y")
            path, statuses = dijkstra(nodes, lanes, free, static + dynamic, length, width, margin,
                                      nearest_node(nodes, start), nearest_node(nodes, goal),
                                      args.blocked_lane)
        except (ValueError, TypeError) as exc:
            status_box.set_val(str(exc)); return
        ax.clear()
        for x1, y1, x2, y2 in free: ax.add_patch(Rectangle((x1,y1),x2-x1,y2-y1,facecolor="#eef7ee",edgecolor="#9bbb9b",alpha=.6,zorder=0))
        for rect in static: ax.add_patch(Rectangle((rect[0],rect[1]),rect[2]-rect[0],rect[3]-rect[1],color="#555",alpha=.9,zorder=2))
        for rect in dynamic: ax.add_patch(Rectangle((rect[0],rect[1]),rect[2]-rect[0],rect[3]-rect[1],color="#d62728",alpha=.75,zorder=3))
        for (u,v), (_,center_ok,footprint_ok) in statuses.items():
            color="#bdbdbd" if footprint_ok else ("#f39c12" if center_ok else "#e74c3c")
            ax.plot([nodes[u][0],nodes[v][0]],[nodes[u][1],nodes[v][1]],color=color,lw=2.4,zorder=1)
            if center_ok and not footprint_ok: ax.plot([nodes[u][0],nodes[v][0]],[nodes[u][1],nodes[v][1]],color="#f39c12",lw=5.5,alpha=.55,zorder=1)
        xs,ys=zip(*nodes.values()); ax.scatter(xs,ys,s=18,color="#125da8",zorder=4)
        for name,p in nodes.items(): ax.text(p[0]+.025,p[1]+.025,name,fontsize=7,color="#164a7b",zorder=5)
        if path: ax.plot([nodes[n][0] for n in path],[nodes[n][1] for n in path],color="#00a878",lw=4.5,zorder=6,label="planned route")
        ax.scatter(*start,s=90,marker="o",color="#00a878",edgecolor="black",zorder=7,label="start"); ax.scatter(*goal,s=110,marker="*",color="#7b2cbf",edgecolor="black",zorder=7,label="goal")
        route_state["points"] = [nodes[n] for n in path]
        route_state["car"] = ax.scatter(*start, s=75, marker="o", color="#00d4ff", edgecolor="black", zorder=8, label="vehicle")
        ax.set_aspect("equal"); ax.set_xlim(-.1,3.3); ax.set_ylim(-.1,4.5); ax.set_xlabel("arena x [m]"); ax.set_ylabel("arena y [m]"); ax.set_title(f"collision view | footprint radius={radius:.3f} m")
        ax.plot([],[],color="#f39c12",lw=5,label="centerline clear / footprint blocked"); ax.plot([],[],color="#e74c3c",lw=3,label="blocked"); ax.legend(loc="upper right",fontsize=8); status_box.set_val("route: " + (" -> ".join(path) if path else "NO FEASIBLE ROUTE")); fig.canvas.draw_idle()
    start_box = TextBox(plt.axes([.08,.10,.20,.035]), "start ", initial=args.start); goal_box = TextBox(plt.axes([.31,.10,.20,.035]), "goal ", initial=args.goal)
    obstacle_box = TextBox(plt.axes([.54,.10,.25,.035]), "obstacle ", initial="x1,y1,x2,y2")
    status_box = TextBox(plt.axes([.08,.04,.43,.035]), "status ", initial="ready"); add_button=Button(plt.axes([.81,.10,.08,.035]), "Add"); clear_button=Button(plt.axes([.90,.10,.08,.035]), "Clear"); update_button=Button(plt.axes([.54,.04,.12,.035]), "Replan"); play_button=Button(plt.axes([.68,.04,.12,.035]), "Play"); reset_button=Button(plt.axes([.81,.04,.12,.035]), "Reset")
    def add_obstacle(_):
        try: dynamic.append(parse_rect(obstacle_box.text)); redraw()
        except ValueError: status_box.set_val("obstacle format: x1,y1,x2,y2")
    def clear_obstacles(_): dynamic.clear(); redraw()
    def play(_):
        points = route_state["points"]
        if len(points) < 2:
            status_box.set_val("no route to animate"); return
        samples = []
        for a, b in zip(points, points[1:]):
            steps = max(2, int(distance(a, b) / .02))
            samples.extend([(a[0] + (b[0]-a[0])*i/steps, a[1] + (b[1]-a[1])*i/steps) for i in range(steps)])
        samples.append(points[-1])
        def move(frame):
            route_state["car"].set_offsets([samples[frame]])
            status_box.set_val(f"vehicle {frame+1}/{len(samples)}")
            return (route_state["car"],)
        route_state["animation"] = FuncAnimation(fig, move, frames=len(samples), interval=35, repeat=False, blit=False)
    def reset(_):
        if route_state["animation"] is not None: route_state["animation"]=None
        if route_state["car"] is not None: route_state["car"].set_offsets([start]); fig.canvas.draw_idle()
        status_box.set_val("vehicle reset")
    add_button.on_clicked(add_obstacle); clear_button.on_clicked(clear_obstacles); update_button.on_clicked(redraw); play_button.on_clicked(play); reset_button.on_clicked(reset); redraw()
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=160, bbox_inches="tight")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
