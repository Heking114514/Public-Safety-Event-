"""恢复验收的独立道路几何；读取正式地图，不调用控制器内部误差函数。"""
from __future__ import annotations

import math
from pathlib import Path


def angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class RoadMap:
    def __init__(self, path):
        import yaml
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        origin = data["frame"]["map_origin_m"]
        yaw = math.radians(data["frame"]["map_x_heading_deg"])

        def transform(point):
            dx, dy = point[0] - origin[0], point[1] - origin[1]
            return (math.cos(yaw) * dx + math.sin(yaw) * dy,
                    -math.sin(yaw) * dx + math.cos(yaw) * dy)

        self.nodes = {name: transform(point) for name, point in data["nodes"].items()}
        self.lanes = {lane["id"]: lane for lane in data["lanes"]}
        self.map_id = data.get("map_id", "fixture")

    def incident(self, node):
        return [lane for lane in self.lanes.values()
                if node in (lane["from"], lane["to"])]

    def projection(self, point, lane_id, from_node=None):
        lane = self.lanes[lane_id]
        first, last = lane["from"], lane["to"]
        if from_node == last:
            first, last = last, first
        elif from_node is not None and from_node != first:
            raise ValueError("出口节点不在候选道路上")
        a, b = self.nodes[first], self.nodes[last]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        rx, ry = point[0] - a[0], point[1] - a[1]
        return dict(along=(rx * dx + ry * dy) / length,
                    cross=(rx * dy - ry * dx) / length,
                    length=length, heading=math.atan2(dy, dx))

    def turn_entry(self, point, yaw, junction, entry_x, entry_y, outgoing_heading=None,
                   incoming_heading=None):
        """分别给出入口点距离、道路横误差和航向误差，三者不可互相替代。"""
        anchor = self.nodes[junction]
        possibilities = []
        for lane in self.incident(junction):
            projection = self.projection(point, lane["id"], junction)
            for heading in (projection["heading"], projection["heading"] + math.pi):
                if incoming_heading is not None and angle_error(heading, incoming_heading) > .01:
                    continue
                direction = 1.0 if outgoing_heading is None or math.sin(outgoing_heading-heading) >= 0.0 else -1.0
                signed_y = direction * entry_y
                target = (anchor[0] + entry_x * math.cos(heading) - signed_y * math.sin(heading),
                          anchor[1] + entry_x * math.sin(heading) + signed_y * math.cos(heading))
                possibilities.append(dict(
                    lane_id=lane["id"],
                    target_x=target[0], target_y=target[1],
                    longitudinal_error=abs((point[0]-target[0])*math.cos(heading) +
                                           (point[1]-target[1])*math.sin(heading)),
                    lateral_error=abs(-(point[0]-target[0])*math.sin(heading) +
                                      (point[1]-target[1])*math.cos(heading)),
                    position_error=math.hypot(point[0] - target[0], point[1] - target[1]),
                    road_center_error=abs(projection["cross"]),
                    heading_error=angle_error(yaw, heading)))
        return min(possibilities, key=lambda row: row["position_error"] + row["heading_error"])

    def exit_error(self, point, yaw, candidate):
        lane_id, junction = candidate["resume_lane"], candidate["resume_node"]
        if lane_id not in self.lanes or junction not in self.nodes:
            return "恢复出口不在正式地图中"
        try:
            projection = self.projection(point, lane_id, junction)
        except ValueError as error:
            return str(error)
        if abs(projection["cross"]) > .025:
            return "恢复完成时偏离目标道路中心"
        if not .05 <= projection["along"] <= projection["length"] + .02:
            return "恢复完成位置不在路口出口侧道路上"
        if angle_error(yaw, projection["heading"]) > .15:
            return "恢复完成朝向不沿目标道路出口方向"
        return ""


def fixed_entry_stop_error(rows, entry, heading, position_tolerance, lateral_tolerance,
                           yaw_tolerance, dwell):
    """以最终入口的连续实测停车证明到位；阶段名和转动后的近似位置不能替代。"""
    stopped = []
    for row in rows:
        if not 0 < entry["t"] - row["t"] <= 1.0:
            continue
        dx, dy = row["x"] - entry["target_x"], row["y"] - entry["target_y"]
        at_entry = (abs(dx*math.cos(heading)+dy*math.sin(heading)) <= position_tolerance and
                    abs(-dx*math.sin(heading)+dy*math.cos(heading)) <= lateral_tolerance and
                    angle_error(row["yaw"], heading) <= yaw_tolerance)
        if at_entry and abs(row["linear"]) < .002 and abs(row["angular"]) < .03:
            if stopped and not 0 < row["stamp"]-stopped[-1]["stamp"] <= .15:
                stopped = []
            stopped.append(row)
        else:
            stopped = []
    if (len(stopped) >= 3 and stopped[-1]["stamp"]-stopped[0]["stamp"] >= dwell and
        entry["t"]-stopped[-1]["t"] <= .15):
        return ""
    return "特定转弯前没有在唯一入口连续实测停稳"


def measured_direction_stop_error(rows):
    """检查整个恢复中的每次换向；零速必须是连续、时间新鲜的里程计样本。"""
    previous_sign = 0
    stopped = []
    for row in rows:
        if not row["status"].startswith("POSE_RECOVERY_"):
            continue
        sign = 1 if row["linear"] > .005 else -1 if row["linear"] < -.005 else 0
        if sign:
            if previous_sign and sign != previous_sign:
                if len(stopped) < 3 or stopped[-1]["stamp"] - stopped[0]["stamp"] < .10:
                    return "恢复换向缺少连续实测停车"
            previous_sign, stopped = sign, []
        elif abs(row["linear"]) < .002 and abs(row["angular"]) < .03:
            if stopped and not 0 < row["stamp"] - stopped[-1]["stamp"] <= .15:
                stopped = []
            stopped.append(row)
        else:
            stopped = []
    return ""
