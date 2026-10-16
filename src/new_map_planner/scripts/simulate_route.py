#!/usr/bin/env python3
"""只前进的独立巡航规划仿真；RFID 优先于隧道，降级后仍尝试返航。"""

from __future__ import annotations

import argparse
from collections import Counter, deque
from functools import lru_cache
import heapq
import itertools
import json
import math
from pathlib import Path
import re
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml

from route_render import render_setup, write_view

PACKAGE = Path(__file__).resolve().parents[1]
PROTECTED = {"LAUNCH", "ROAD_TOP_LEFT", "ROAD_TOP_RIGHT"}
HEADINGS = ("东", "北", "西", "南")
TURN_ENTRY_M = 0.07  # 用户确认的固定入口距离；本工具仅标记，不替代车体扫掠验收。
CORNER_OBSTACLE_RFID = {
    frozenset((1, 2)): "x0_y3", frozenset((1, 5)): "x0_y3",
    frozenset((3, 4)): "x3_y3", frozenset((4, 8)): "x3_y3",
    frozenset((12, 16)): "x3_y0", frozenset((15, 16)): "x3_y0",
    frozenset((13, 14)): "x0_y0", frozenset((9, 13)): "x0_y0",
}


class Arena:
    def __init__(self, data, pairs=()):
        self.data = data
        self.nodes = {name: tuple(point) for name, point in data["nodes"].items()}
        self.home = data["home_node"]
        self.lanes = {lane["id"]: lane for lane in data["lanes"]}
        self.rfid = {node: str(task) for task, node in data["tasks"].items()}
        self.tunnels = [name for name, lane in self.lanes.items() if lane.get("tunnel")]
        self.green = {4 * row + col + 1: f"x{col}_y{3-row}"
                      for row in range(4) for col in range(4)}
        self.corner_obstacle_pairs = set()
        self.blocked = self.resolve_blocks(pairs)
        self.adj = {node: [] for node in self.nodes}
        for lid, lane in self.lanes.items():
            if lid in self.blocked:
                continue
            a, b = lane["from"], lane["to"]
            distance = math.dist(self.nodes[a], self.nodes[b])
            self.adj[a].append((b, lid, self.heading(a, b), distance))
            if lane["bidirectional"]:
                self.adj[b].append((a, lid, self.heading(b, a), distance))
        self.start_heading = round(data["frame"]["map_x_heading_deg"] / 90) % 4
        self.lane_bits = {lid: 1 << i for i, lid in enumerate(self.lanes)}
        self.available_lane_mask = sum(self.lane_bits[lid] for lid in self.lanes
                                       if lid not in self.blocked)
        self.corridors = self.build_rfid_corridors()
        self.skipped_targets = []
        self.configure_reachable_targets()
        self.assign_goal_bits()

    def heading(self, a, b):
        x, y = self.nodes[a]
        dx, dy = self.nodes[b][0] - x, self.nodes[b][1] - y
        if abs(dx) > 1e-8 and abs(dy) > 1e-8:
            raise ValueError(f"道路 {a}—{b} 不是当前模型支持的水平/竖直道路")
        return round(math.atan2(dy, dx) / (math.pi / 2)) % 4

    def build_rfid_corridors(self):
        corridors = {}
        for start in self.rfid:
            for direction in range(4):
                node, path, ordinary = start, [], 0
                while True:
                    outgoing = [move for move in self.adj[node] if move[2] == direction]
                    if len(outgoing) != 1:
                        break
                    target, lid, _, length = outgoing[0]
                    path.append((node, target, lid, length))
                    if target in self.rfid:
                        if ordinary > 2:
                            corridors[(start, direction)] = tuple(path)
                        break
                    ordinary += 1
                    node = target
        return corridors

    def resolve_blocks(self, pairs):
        by_ends = {frozenset((lane["from"], lane["to"])): lid
                   for lid, lane in self.lanes.items()}
        blocked = set()
        for pair in pairs:
            value = pair.strip()
            if value.startswith("(") and value.endswith(")"):
                value = value[1:-1]
            match = re.fullmatch(r"\s*(\d+)\s*,\s*(\d+)\s*", value)
            if not match:
                raise ValueError(f"封边 {pair!r} 格式错误，请使用 1,2 或 '(1,2)'")
            a, b = map(int, match.groups())
            if a not in self.green or b not in self.green:
                raise ValueError(f"封边 {pair} 的绿色编号必须在 1～16 内")
            lid = by_ends.get(frozenset((self.green[a], self.green[b])))
            if lid is None:
                raise ValueError(f"绿色节点 {a}、{b} 没有直接相连的道路")
            if lid in PROTECTED or self.lanes[lid].get("tunnel"):
                raise ValueError(f"道路 {a}—{b}（{lid}）属于不会放障的道路，不能封闭")
            blocked.add(lid)
            pair_key = frozenset((a, b))
            if pair_key in CORNER_OBSTACLE_RFID:
                self.corner_obstacle_pairs.add(pair_key)
        if len(blocked) > 3:
            raise ValueError("最多允许三条去重后的封边；反向节点对按同一条边处理")
        return blocked

    def green_pair(self, lane_id):
        lane = self.lanes[lane_id]
        numbers = {node: number for number, node in self.green.items()}
        a, b = lane["from"], lane["to"]
        if a not in numbers or b not in numbers:
            return None
        return f"{numbers[a]},{numbers[b]}"

    def allowed_block_lanes(self):
        return {lid for lid in self.lanes
                if lid not in PROTECTED and not self.lanes[lid].get("tunnel")
                and self.green_pair(lid) is not None}

    def moves(self, node, heading, ordinary):
        for target, lid, outgoing, length in self.adj[node]:
            count = 0 if target in self.rfid else ordinary + 1
            turn = (outgoing - heading) % 4
            if count <= 2 and turn != 2:
                yield (target, outgoing, count), lid, length

    def transitions(self, node, heading, ordinary):
        for state, lid, length in self.moves(node, heading, ordinary):
            yield state, ((state, lid, None),), length
            if ordinary != 0 or node not in self.rfid:
                continue
            corridor = self.corridors.get((node, self.heading(node, state[0])))
            if not corridor:
                continue
            end, outgoing = corridor[-1][1], state[1]
            corridor_id = f"{node}:{end}"
            expanded, count = [], 0
            for _, target, edge, _ in corridor:
                count = 0 if target in self.rfid else count + 1
                expanded.append(((target, outgoing, count), edge, corridor_id))
            yield (end, outgoing, 0), tuple(expanded), sum(edge[3] for edge in corridor)

    def configure_reachable_targets(self):
        skipped_corners = {CORNER_OBSTACLE_RFID[pair]
                           for pair in self.corner_obstacle_pairs}
        for node in sorted(skipped_corners):
            green = next(number for number, value in self.green.items() if value == node)
            self.skipped_targets.append({"kind": "RFID", "target": self.rfid[node],
                                         "message": f"绿节点 {green} 角点相邻指定边有障碍，按规则跳过 RFID {self.rfid[node]}"})

        start = (self.home, self.start_heading, 0)
        reachable, reverse, traversed = {start}, {}, []
        queue = deque([start])
        while queue:
            source = queue.popleft()
            for target, path, _ in self.transitions(*source):
                reverse.setdefault(target, []).append(source)
                traversed.extend((source, target, lid) for _, lid, _ in path)
                if target not in reachable:
                    reachable.add(target)
                    queue.append(target)
        returning = {state for state in reachable if state[0] == self.home}
        queue = deque(returning)
        while queue:
            for previous in reverse.get(queue.popleft(), []):
                if previous not in returning:
                    returning.add(previous)
                    queue.append(previous)

        self.required_rfid = {}
        for node, task in self.rfid.items():
            if node in skipped_corners:
                continue
            if any(state[0] == node for state in reachable.intersection(returning)):
                self.required_rfid[node] = task
            else:
                self.skipped_targets.append({"kind": "RFID", "target": task,
                                             "message": f"RFID {task}：当前封边和前进/定位规则下无法到达并返航"})

        self.required_tunnels = set()
        for lid in self.tunnels:
            if any(edge == lid and source in reachable and target in returning
                   for source, target, edge in traversed):
                self.required_tunnels.add(lid)
            else:
                self.skipped_targets.append({"kind": "TUNNEL", "target": lid,
                                             "message": f"隧道 {lid}：当前封边和前进/定位规则下无法合法通过并返航"})

    def assign_goal_bits(self):
        self.rfid_bits = {node: 1 << i for i, node in enumerate(self.required_rfid)}
        offset = len(self.required_rfid)
        self.tunnel_bits = {lid: 1 << (offset + i)
                            for i, lid in enumerate(sorted(self.required_tunnels))}
        self.full_mask = (1 << (offset + len(self.required_tunnels))) - 1

    def heuristic(self):
        names = list(self.nodes) + [f"@{lid}" for lid in self.required_tunnels]
        index = {name: i for i, name in enumerate(names)}
        distances = [[math.inf] * len(names) for _ in names]
        for i in range(len(names)):
            distances[i][i] = 0.0
        for lid, lane in self.lanes.items():
            if lid in self.blocked:
                continue
            a, b = lane["from"], lane["to"]
            chain = [a, f"@{lid}", b] if lane.get("tunnel") and lid in self.required_tunnels else [a, b]
            length = math.dist(self.nodes[a], self.nodes[b]) / (len(chain) - 1)
            for source, target in zip(chain, chain[1:]):
                i, j = index[source], index[target]
                distances[i][j] = length
                if lane["bidirectional"]:
                    distances[j][i] = length
        for k in range(len(names)):
            for i in range(len(names)):
                for j in range(len(names)):
                    distances[i][j] = min(distances[i][j], distances[i][k] + distances[k][j])
        goals = [(node, self.rfid_bits[node]) for node in self.required_rfid]
        goals += [(f"@{lid}", self.tunnel_bits[lid]) for lid in sorted(self.required_tunnels)]
        home = index[self.home]

        @lru_cache(maxsize=65536)
        def remaining_tree(mask):
            pending = [index[name] for name, bit in goals if not mask & bit]
            costs, total = {i: distances[home][i] for i in pending}, 0.0
            while costs:
                node = min(costs, key=costs.get)
                total += costs.pop(node)
                for other in costs:
                    costs[other] = min(costs[other], distances[node][other])
            return total, tuple(pending) + (home,)

        def estimate(node, mask):
            tree, pending = remaining_tree(mask)
            if not pending:
                return distances[index[node]][home]
            return tree + min(distances[index[node]][target] for target in pending)
        return estimate


def solve(arena, timeout=5.0, max_states=200000):
    started = time.perf_counter()
    deadline = started + timeout
    estimate = arena.heuristic()
    start = (arena.home, arena.start_heading, 0,
             arena.rfid_bits.get(arena.home, 0), 0)
    best, parent = {start: (0, 0.0)}, {start: None}
    serial = itertools.count()
    queue = [(2.0 * estimate(start[0], start[3]), 0, 0, 0.0,
              next(serial), start)]
    expanded, best_mask, goal = 0, start[3], None
    goal_score, first_score, first_solution_s = None, None, None
    stop_reason = "exhausted"
    while queue:
        if time.perf_counter() >= deadline:
            stop_reason = "timeout"
            break
        _, _, passes, cost, _, state = heapq.heappop(queue)
        if best[state] != (passes, cost):
            continue
        node, heading, ordinary, mask, lane_mask = state
        expanded += 1
        if mask.bit_count() > best_mask.bit_count():
            best_mask = mask
        if node == arena.home and passes > 0 and mask:
            rfid_count = sum(bool(mask & bit) for bit in arena.rfid_bits.values())
            tunnel_count = sum(bool(mask & bit) for bit in arena.tunnel_bits.values())
            unique = lane_mask.bit_count()
            score = (-rfid_count, -tunnel_count, -unique, passes - unique, cost)
            if first_score is None:
                first_score, first_solution_s = score, time.perf_counter() - started
            if goal_score is None or score < goal_score:
                goal, goal_score = state, score
        for (target, outgoing, count), path, length in arena.transitions(node, heading, ordinary):
            visited, covered, new_passes, turn_cost = mask, lane_mask, passes, 0.0
            expanded_path, previous_base = [], (node, heading, ordinary)
            for edge_state, lid, corridor_id in path:
                edge_node, edge_heading, edge_ordinary = edge_state
                if edge_heading != previous_base[1]:
                    turn_cost += arena.data["planning"]["turn_cost_m"]
                visited |= arena.rfid_bits.get(edge_node, 0) | arena.tunnel_bits.get(lid, 0)
                covered |= arena.lane_bits[lid]
                new_passes += 1
                expanded_path.append((previous_base,
                                      (edge_node, edge_heading, edge_ordinary), lid, corridor_id))
                previous_base = (edge_node, edge_heading, edge_ordinary)
            successor = (target, outgoing, count, visited, covered)
            new_cost = cost + length + turn_cost
            old = best.get(successor)
            if old is not None and (new_passes, new_cost) >= old:
                continue
            if successor not in best and len(best) >= max_states:
                stop_reason = "state_limit"
                queue.clear()
                break
            best[successor] = (new_passes, new_cost)
            parent[successor] = (state, tuple(expanded_path))
            priority = new_cost + 2.0 * estimate(target, visited) - 0.05 * covered.bit_count()
            heapq.heappush(queue, (priority, -covered.bit_count(), new_passes,
                                   new_cost, next(serial), successor))

    selected_goal = goal
    route_goal = selected_goal
    route = []
    while route_goal is not None and parent[route_goal] is not None:
        previous, path = parent[route_goal]
        route[0:0] = path
        route_goal = previous
    steps, visits, spoken = [], Counter(), set()
    for number, (before, after, lid, corridor_id) in enumerate(route, 1):
        a, b = before[0], after[0]
        visits[lid] += 1
        rfid = arena.rfid.get(b)
        turn = (after[1] - before[1]) % 4
        entry = None
        if turn:
            yaw = before[1] * math.pi / 2
            entry = [arena.nodes[a][0] - TURN_ENTRY_M * math.cos(yaw),
                     arena.nodes[a][1] - TURN_ENTRY_M * math.sin(yaw)]
        steps.append({"step": number, "from": a, "to": b, "lane_id": lid,
                      "distance_m": math.dist(arena.nodes[a], arena.nodes[b]),
                      "heading": HEADINGS[after[1]], "heading_index": after[1],
                      "action": {0: "直行", 1: "左转90°", 3: "右转90°"}[turn],
                      "motion": "forward", "turn_entry": entry,
                      "ordinary_after": after[2], "rfid_corridor": corridor_id,
                      "rfid": rfid, "rfid_stop_s": 1 if rfid else 0,
                      "announce": bool(rfid and rfid not in spoken),
                      "tunnel": lid if lid in arena.tunnels else None,
                      "pass_number": visits[lid]})
        if rfid:
            spoken.add(rfid)

    route_mask = selected_goal[3] if selected_goal else 0
    deferred = [item for item in arena.skipped_targets
                if not (item["kind"] == "RFID" and any(
                    arena.rfid.get(node) == item["target"] and route_mask & bit
                    for node, bit in arena.rfid_bits.items()))]
    for node, task in arena.required_rfid.items():
        if not route_mask & arena.rfid_bits[node]:
            deferred.append({"kind": "RFID", "target": task,
                             "message": f"RFID {task}：与返航组合后，本路线无法合法完成"})
    for lid in sorted(arena.required_tunnels):
        if not route_mask & arena.tunnel_bits[lid]:
            deferred.append({"kind": "TUNNEL", "target": lid,
                             "message": f"隧道 {lid}：与优先 RFID 任务及返航组合后，本路线无法合法完成"})
    deferred.sort(key=lambda item: (0 if item["kind"] == "TUNNEL" else 1,
                                    str(item["target"])))
    rfid_total = sum(bool(route_mask & bit) for bit in arena.rfid_bits.values())
    tunnel_total = sum(bool(route_mask & bit) for bit in arena.tunnel_bits.values())
    all_original = rfid_total == len(arena.rfid) and tunnel_total == len(arena.tunnels)
    returned = bool(route and route[-1][1][0] == arena.home)
    if not returned or (rfid_total + tunnel_total == 0):
        status = "NO_FEASIBLE_ROUTE"
    elif all_original:
        status = "SUCCESS"
    else:
        status = "PARTIAL_SUCCESS"
    objective_score = None if goal_score is None else [
        -goal_score[0], -goal_score[1], -goal_score[2], goal_score[3], goal_score[4]]
    return {"status": status, "planning_time_s": time.perf_counter() - started,
            "issues": deferred, "deferred_targets": deferred,
            "best_tasks_seen": best_mask.bit_count(), "expanded_states": expanded,
            "generated_states": len(best), "steps": steps,
            "distance_m": sum(step["distance_m"] for step in steps),
            "rfid_completed": rfid_total, "rfid_required": len(arena.rfid),
            "rfid_planned": rfid_total, "rfid_feasible": len(arena.required_rfid),
            "tunnels_completed": tunnel_total, "tunnels_required": len(arena.tunnels),
            "tunnels_planned": tunnel_total, "tunnels_feasible": len(arena.required_tunnels),
            "lane_counts": dict(visits), "unique_lanes": len(visits),
            "unique_lanes_available": arena.available_lane_mask.bit_count(),
            "repeated_passes": len(steps) - len(visits),
            "first_solution_s": first_solution_s,
            "initial_unique_lanes": -first_score[2] if first_score else 0,
            "initial_repeated_passes": first_score[3] if first_score else 0,
            "optimization_complete": stop_reason == "exhausted",
            "optimization_stop_reason": stop_reason,
            "optimization_score": objective_score,
            "rfid_stop_s": sum(s["rfid_stop_s"] for s in steps),
            "max_ordinary": max((s["ordinary_after"] for s in steps), default=0),
            "returned_home": returned,
            "model": "拓扑仿真：只前进直行和前向90°特定转弯；不进行真实车体扫掠验证",
            "search": "加权搜索优先完成 RFID，再完成隧道，之后优化道路覆盖；返回合法返航候选，限时或状态上限不代表无解"}

def result_metadata(result, data, arena, map_path):
    result.update({"map_path": str(map_path.resolve()), "map_id": data["map_id"],
                   "blocked_lane_ids": sorted(arena.blocked), "nodes": arena.nodes,
                   "lanes": list(arena.lanes.values()), "rfid_nodes": arena.rfid,
                   "green_numbers": {node: number for number, node in arena.green.items()},
                   "home": arena.home, "start_heading": arena.start_heading,
                   "turn_entry_m": TURN_ENTRY_M,
                   "corner_obstacle_pairs": [sorted(pair) for pair in arena.corner_obstacle_pairs],
                   "skipped_before_search": arena.skipped_targets})
    return result


def start_interactive(data, map_path, initial_arena, output, timeout, max_states,
                      host, port, open_browser):
    output.parent.mkdir(parents=True, exist_ok=True)
    setup_page = render_setup(Arena(data), sorted(initial_arena.blocked))
    session = {"phase": "setup", "result": None}
    session_lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send_text(self, code, body, content_type="text/plain; charset=utf-8"):
            payload = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if self.path == "/":
                if session["phase"] == "started":
                    self.send_text(200, output.read_text(encoding="utf-8"),
                                   "text/html; charset=utf-8")
                elif session["phase"] == "planning":
                    self.send_text(200, "<!doctype html><meta charset='utf-8'><meta http-equiv='refresh' content='1'><p>正在规划，封边配置暂时锁定。</p>",
                                   "text/html; charset=utf-8")
                else:
                    self.send_text(200, setup_page, "text/html; charset=utf-8")
            elif self.path == "/result" and session["phase"] == "started":
                self.send_text(200, output.read_text(encoding="utf-8"),
                               "text/html; charset=utf-8")
            elif session["phase"] == "started" and self.path == "/" + output.with_suffix(".svg").name:
                self.send_text(200, output.with_suffix(".svg").read_text(encoding="utf-8"),
                               "image/svg+xml; charset=utf-8")
            elif session["phase"] == "started" and self.path == "/" + output.with_suffix(".json").name:
                self.send_text(200, output.with_suffix(".json").read_text(encoding="utf-8"),
                               "application/json; charset=utf-8")
            elif self.path == "/state":
                self.send_text(200, json.dumps({"locked": session["phase"] == "started",
                                                "phase": session["phase"]}),
                               "application/json")
            else:
                self.send_text(404, "页面不存在")

        def do_POST(self):
            if self.path != "/start":
                self.send_text(404, "接口不存在")
                return
            if session["phase"] != "setup":
                self.send_text(409, "本轮导航已开始，障碍位置已锁定")
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size > 4096:
                    raise ValueError("封边配置数据过大")
                body = json.loads(self.rfile.read(size))
                lane_ids = body["blocked_lane_ids"]
                if not isinstance(lane_ids, list) or len(lane_ids) != 3 or len(set(lane_ids)) != 3:
                    raise ValueError("开始导航必须选择三条不同道路")
                allowed = initial_arena.allowed_block_lanes()
                if not set(lane_ids) <= allowed:
                    raise ValueError("封边只允许选择地图中标出的绿色普通道路")
                pairs = [initial_arena.green_pair(lid) for lid in lane_ids]
                arena = Arena(data, pairs)
            except (ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
                self.send_text(400, str(error))
                return
            with session_lock:
                if session["phase"] != "setup":
                    self.send_text(409, "本轮导航已开始，障碍位置已锁定")
                    return
                session["phase"] = "planning"
            print(f"开始规划：{', '.join(sorted(lane_ids))}", flush=True)
            result = result_metadata(solve(arena, timeout, max_states), data, arena, map_path)
            if result["status"] not in {"SUCCESS", "PARTIAL_SUCCESS"}:
                with session_lock:
                    session["phase"] = "setup"
                detail = "本次封边没有找到可返航的有效巡航路线；导航未开始，可以调整后重试。"
                print(f"{result['status']} | 规划 {result['planning_time_s']:.3f}s | {detail}", flush=True)
                self.send_text(422, json.dumps({"status": result["status"], "message": detail},
                                                ensure_ascii=False), "application/json; charset=utf-8")
                return
            output.with_suffix(".json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            write_view(result, output, mode="started")
            session["result"] = result
            session["phase"] = "started"
            print(f"{result['status']} | 规划 {result['planning_time_s']:.3f}s | "
                  f"RFID {result['rfid_completed']}/{result['rfid_required']} | "
                  f"隧道 {result['tunnels_completed']}/{result['tunnels_required']} | "
                  f"不同道路 {result['unique_lanes']} | 重复 {result['repeated_passes']} | "
                  f"结果：{output}", flush=True)
            self.send_text(200, json.dumps({"redirect": "/result"}),
                           "application/json; charset=utf-8")

    server = ThreadingHTTPServer((host, port), Handler)
    address, actual_port = server.server_address
    url = f"http://{address}:{actual_port}/"
    print(f"交互布障页面：{url}", flush=True)
    print("选择三条合法道路后开始；导航开始后服务端锁定。按 Ctrl+C 退出。", flush=True)
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("交互仿真已停止。", flush=True)
    finally:
        server.server_close()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=PACKAGE / "config/arena.yaml")
    parser.add_argument("--blocked", nargs="*", default=[], metavar="A,B", help="绿色节点对，最多三条；如 1,2 5,6 9,10")
    parser.add_argument("--timeout", type=float, default=5.0, help="规划时限，默认5秒")
    parser.add_argument("--max-states", type=int, default=200000, help="搜索状态上限，限制内存占用")
    parser.add_argument("--output", type=Path, default=Path.home() / "forward_route_sim/route.html")
    parser.add_argument("--static", action="store_true", help="只生成命令行指定障碍的静态路线，不启动交互页面")
    parser.add_argument("--open", action=argparse.BooleanOptionalAction, default=True,
                        help="自动打开交互页面（可用 --no-open 关闭）")
    parser.add_argument("--port", type=int, default=0, help="本机服务端口；0 表示自动选择")
    args = parser.parse_args(argv)
    if args.timeout <= 0 or args.max_states <= 0 or not 0 <= args.port <= 65535:
        parser.error("规划时限和状态上限必须大于零")
    try:
        data = yaml.safe_load(args.map.read_text(encoding="utf-8"))
        arena = Arena(data, args.blocked)
    except (ValueError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))
    output = args.output.expanduser().resolve().with_suffix(".html")
    if not args.static:
        return start_interactive(data, args.map, arena, output, args.timeout,
                                 args.max_states, "127.0.0.1", args.port, args.open)
    result = solve(arena, args.timeout, args.max_states)
    result_metadata(result, data, arena, args.map)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_view(result, output)
    print(f"{result['status']} | 规划 {result['planning_time_s']:.3f}s | 扩展 {result['expanded_states']} 个状态")
    print(f"RFID {result['rfid_completed']}/{result['rfid_required']} | 隧道 {result['tunnels_completed']}/{result['tunnels_required']} | 返航 {result['returned_home']}")
    print(f"路线 {len(result['steps'])} 步 / {result['distance_m']:.2f}m | RFID停车共 {result['rfid_stop_s']}s")
    print(f"不同物理边 {result['unique_lanes']}/{result['unique_lanes_available']} | 重复通行 {result['repeated_passes']} 次 | 覆盖搜索停止：{result['optimization_stop_reason']}")
    if result["deferred_targets"]:
        print("降级目标（RFID 优先，先舍弃隧道再舍弃 RFID）：")
        for issue in result["deferred_targets"]:
            print(f"  {issue['kind']} {issue['target']}：{issue['message']}")
    for lid, count in sorted(result["lane_counts"].items(), key=lambda item: (-item[1], item[0])):
        if count > 1:
            print(f"  重复 ×{count}: {lid}")
    print(f"可视化：{output}\n路线数据：{output.with_suffix('.json')}")
    if args.open:
        webbrowser.open(output.as_uri())
    return 0 if result["status"] in {"SUCCESS", "PARTIAL_SUCCESS"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
