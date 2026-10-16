"""输出离线可播放的路线图；每次经过同一道路都单独绘制。"""

from collections import defaultdict
from html import escape
import json
import math
from pathlib import Path

COLORS = ("#2563eb", "#ea580c", "#9333ea", "#0891b2", "#db2777", "#4d7c0f")
SCALE = 190


def point(position):
    x, y = position
    return 60 + x * SCALE, 60 + (4.4 - y) * SCALE


def label_node(result, node):
    green = result["green_numbers"].get(node)
    if green:
        return f"绿{green}"
    return {"launch": "起点", "launch_junction": "出发路口",
            "x1_y4": "顶部左", "x2_y4": "顶部右"}.get(node, node)


def route_svg(result):
    parts = ['<svg id="route-map" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 740 950" role="img" aria-label="前进路线、车头方向及各道路重复通行次数">', '<defs>']
    for i, color in enumerate(COLORS):
        parts.append(f'<marker id="arrow{i}" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0 0 L10 5 L0 10 Z" fill="{color}"/></marker>')
    parts.extend(['</defs>', '<rect width="740" height="950" fill="#f8fafc" rx="18"/>'])
    for lane in result["lanes"]:
        a, b = [point(result["nodes"][lane[end]]) for end in ("from", "to")]
        lid = escape(lane["id"])
        parts.append(f'<line data-focus="{lid}" x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#e2e8f0" stroke-width="{lane["width_m"]*SCALE}" stroke-linecap="round"><title>{lid}</title></line>')
        if lane.get("tunnel"):
            parts.append(f'<line pointer-events="none" x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#a78bfa" stroke-width="28" stroke-dasharray="3 6" opacity="0.5"/>')
            parts.append(f'<text pointer-events="none" x="{(a[0]+b[0])/2}" y="{(a[1]+b[1])/2+36}" text-anchor="middle" font-size="11" fill="#6d28d9">{lid}</text>')
    by_lane = defaultdict(list)
    for step in result["steps"]:
        by_lane[step["lane_id"]].append(step)
    lanes = {lane["id"]: lane for lane in result["lanes"]}
    for lid, steps in by_lane.items():
        lane = lanes[lid]
        a, b = [point(result["nodes"][lane[end]]) for end in ("from", "to")]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        nx, ny = -dy / length, dx / length
        count = len(steps)
        gap = 12
        if count > 1:
            parts.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#fbbf24" stroke-width="{max(32, count*gap+12)}" opacity="0.25" stroke-linecap="round"/>')
        for i, step in enumerate(steps):
            offset = (i - (count - 1) / 2) * gap
            p = (a[0] + nx * offset, a[1] + ny * offset)
            q = (b[0] + nx * offset, b[1] + ny * offset)
            if step["from"] != lane["from"]:
                p, q = q, p
            color_index = i % len(COLORS)
            color = COLORS[color_index]
            x1, y1 = p[0] + (q[0]-p[0])*.10, p[1] + (q[1]-p[1])*.10
            x2, y2 = p[0] + (q[0]-p[0])*.88, p[1] + (q[1]-p[1])*.88
            sid = step["step"]
            dash = ''
            parts.append(f'<g class="pass" data-lane="{escape(lid)}" data-step="{sid}"><title>第{sid}步 · 本路第{i+1}次 · {escape(step["action"])} · 车头朝{step["heading"]} · {escape(lid)}</title>')
            parts.append(f'<line class="trace" x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="3"{dash} marker-end="url(#arrow{color_index})"/>')
            tx, ty = p[0]*.61 + q[0]*.39, p[1]*.61 + q[1]*.39
            angle = -90 if abs(dy) > abs(dx) else 0
            if length >= 100:
                parts.append(f'<text x="{tx}" y="{ty-3}" transform="rotate({angle} {tx} {ty})" text-anchor="middle" font-size="9" fill="{color}" paint-order="stroke" stroke="#f8fafc" stroke-width="2">#{sid}·第{i+1}次</text>')
            parts.append('</g>')
        if count > 1:
            offset = count * gap / 2 + 18
            if lane.get("tunnel"):
                offset = -offset
            cx, cy = (a[0]+b[0])/2 + nx*offset, (a[1]+b[1])/2 + ny*offset
            if lid == "LAUNCH":
                cx, cy = (a[0]+b[0])/2 + 60, (a[1]+b[1])/2
            parts.append(f'<g data-focus="{escape(lid)}"><rect x="{cx-20}" y="{cy-13}" width="40" height="26" rx="13" fill="#f59e0b" stroke="#fff" stroke-width="2"/><text x="{cx}" y="{cy+5}" text-anchor="middle" fill="#422006" font-size="16" font-weight="800">×{count}</text></g>')
    # 同一点的多次转弯只画一个入口记号，播放时按步骤更新细节。
    entries = {tuple(s["turn_entry"]) for s in result["steps"] if s["turn_entry"]}
    for entry in entries:
        x, y = point(entry)
        parts.append(f'<rect x="{x-2}" y="{y-2}" width="4" height="4" fill="#64748b"><title>中心前7cm转弯入口（示意）</title></rect>')
    bad_nodes = {issue["node"] for issue in result["issues"] if "node" in issue}
    for node, position in result["nodes"].items():
        x, y = point(position)
        task = result["rfid_nodes"].get(node)
        fill = "#fef3c7" if task else "#dcfce7"
        if node == result["home"]:
            fill = "#dbeafe"
        if node in bad_nodes:
            parts.append(f'<circle cx="{x}" cy="{y}" r="23" fill="#fee2e2" stroke="#dc2626" stroke-width="3"/>')
        parts.append(f'<circle cx="{x}" cy="{y}" r="10" fill="{fill}" stroke="#334155" stroke-width="2"><title>{escape(node)}' + (f' · RFID {task}' if task else '') + '</title></circle>')
        title = label_node(result, node) + (f" / R{task}" if task else "")
        parts.append(f'<text x="{x+15}" y="{y-13}" font-size="12" font-weight="600" fill="#0f172a" paint-order="stroke" stroke="#f8fafc" stroke-width="4">{escape(title)}</text>')
    for lid in result["blocked_lane_ids"]:
        lane = lanes[lid]
        a, b = [point(result["nodes"][lane[end]]) for end in ("from", "to")]
        x, y = (a[0]+b[0])/2, (a[1]+b[1])/2
        parts.append(f'<g class="blocked-mark" data-blocked-lane="{escape(lid)}"><title>封闭 {escape(lid)}，两端节点保留</title><circle cx="{x}" cy="{y}" r="17" fill="#fff1f2" stroke="#dc2626" stroke-width="3"/><path d="M{x-7} {y-7} L{x+7} {y+7} M{x-7} {y+7} L{x+7} {y-7}" stroke="#dc2626" stroke-width="4"/></g>')
    x, y = point(result["nodes"][result["home"]])
    parts.append(f'<circle id="progress-dot" cx="{x}" cy="{y}" r="7" fill="#0f172a" stroke="white" stroke-width="3"/>')
    # The arrow points along the chassis heading, which is also the forward travel direction.
    rotation = (1 - result["start_heading"]) * 90
    parts.append(f'<g id="heading-arrow" transform="translate({x} {y}) rotate({rotation})" pointer-events="none" aria-label="车头朝向"><title>车头朝向</title><path d="M0 7 L0 -17" fill="none" stroke="white" stroke-width="8" stroke-linecap="round"/><path d="M0 7 L0 -17" fill="none" stroke="#0f172a" stroke-width="4" stroke-linecap="round"/><path d="M-7 -11 L0 -21 L7 -11 Z" fill="#0f172a" stroke="white" stroke-width="2" stroke-linejoin="round"/></g>')
    parts.append('</svg>')
    return '\n'.join(parts)


def render_view(result, output, mode="locked"):
    svg = route_svg(result)
    rows = []
    for step in result["steps"]:
        color = COLORS[(step["pass_number"] - 1) % len(COLORS)]
        event = f'RFID {step["rfid"]} · 停1s' if step["rfid"] else f'普通节点 {step["ordinary_after"]}/2'
        if step.get("rfid_corridor"):
            event += ' · RFID端点共线直行例外'
        if step["announce"]:
            event += ' · 首次播报'
        if step["tunnel"]:
            event += f' · {step["tunnel"]}'
        rows.append(f'<tr data-step="{step["step"]}" data-lane="{escape(step["lane_id"])}"><td>#{step["step"]}</td><td>{escape(label_node(result, step["from"]))} → {escape(label_node(result, step["to"]))}<small>{escape(step["action"])} · 朝{step["heading"]} · {escape(event)}</small></td><td style="color:{color}">第{step["pass_number"]}次</td></tr>')
    repeats = []
    lanes = {lane["id"]: lane for lane in result["lanes"]}
    for lid, count in sorted(result["lane_counts"].items(), key=lambda item: (-item[1], item[0])):
        if count > 1:
            lane = lanes[lid]
            name = f'{label_node(result, lane["from"])}—{label_node(result, lane["to"])}'
            repeats.append(f'<button class="repeat" data-focus="{escape(lid)}"><b>×{count}</b> {escape(name)}</button>')
    template = Path(__file__).with_name("route_view.html").read_text(encoding="utf-8")
    replacements = {"__MAP__": svg, "__ROWS__": "\n".join(rows),
                    "__REPEATS__": "\n".join(repeats) or '<span>没有重复道路</span>',
                    "__JSON__": json.dumps(result, ensure_ascii=False).replace('<', '\\u003c'),
                    "__MODE__": mode,
                    "__SVG_NAME__": escape(output.with_suffix('.svg').name, quote=True),
                    "__JSON_NAME__": escape(output.with_suffix('.json').name, quote=True)}
    for token, value in replacements.items():
        template = template.replace(token, value)
    return template


def write_view(result, output, mode="locked"):
    svg = route_svg(result)
    output.with_suffix(".svg").write_text(svg + "\n", encoding="utf-8")
    output.write_text(render_view(result, output, mode), encoding="utf-8")


def render_setup(arena, initial_lane_ids):
    result = {"status": "SETUP", "planning_time_s": 0.0, "steps": [],
              "distance_m": 0.0,
              "rfid_stop_s": 0, "max_ordinary": 0, "rfid_completed": 0,
              "rfid_required": len(arena.rfid), "tunnels_completed": 0,
              "tunnels_required": len(arena.tunnels), "returned_home": False,
              "issues": [], "deferred_targets": [], "blocked_lane_ids": initial_lane_ids,
              "nodes": arena.nodes, "lanes": list(arena.lanes.values()),
              "rfid_nodes": arena.rfid,
              "green_numbers": {node: number for number, node in arena.green.items()},
              "home": arena.home, "start_heading": arena.start_heading,
              "lane_counts": {},
              "allowed_block_lane_ids": sorted(arena.allowed_block_lanes())}
    return render_view(result, Path("route.html"), mode="setup")
