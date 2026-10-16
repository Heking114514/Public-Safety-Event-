#!/usr/bin/env python3
"""真实 ROS2 局部避障恢复场景；只通过 initialpose / manager control / 障碍输入驱动。

Run from the workspace (no build-rule change needed)::
  python3 src/vehicle_loopback_sim/scripts/run_pose_recovery_scenarios.py --list
  python3 src/vehicle_loopback_sim/scripts/run_pose_recovery_scenarios.py \
      --base-domain 170 --output /tmp/pose_recovery_scenarios --timeout 150

Source /opt/ros/humble/setup.bash and workspace/install/setup.bash in each worker.
Every scenario gets a fresh domain and launch; only owned process groups are stopped.
JSON contains raw ROS messages; CSV includes odom, cmd, phases and all event payloads.
PASS requires actual detection, candidates -> motion -> complete with matching ID,
then >= .20 m continuous measured forward FOLLOWING with |cmd/odom angular| < .1.
The recovery protocol additionally requires live reverse evidence, a stop before
any direction change, RETREAT_TO_TURN_POINT -> ALIGN_FOR_TURN -> TURN_INTO_EXIT
ordering, real TURN_60 -> TURN_30 evidence for each fixed turn, and independent
map projections for turn-entry and exit-side checks. FAULT, failed phase, static guard intervention,
missing evidence and timeout fail.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
from recovery_evidence import RoadMap, angle_error, measured_direction_stop_error, fixed_entry_stop_error
import re
import signal
import subprocess
import sys
import time
import traceback

ROOT = "/comparison/final_navigation"
JUNCTION = (1.2, -0.5)  # official x1_y3 in map frame: x=4.3-arena_y, y=arena_x-1.6
BAD_LOG = re.compile(
    r"\bNavigation state:\s*FAULT(?:\b|_)|"
    r"\bmission state:[^\r\n;]*->\s*FAULT(?:\b|_)|"
    r"\[ERROR\][^\r\n]*\bFAULT(?:\b|_)|"
    r"(?:static )?collision guard blocked", re.I)
REQUIRED_NODES = {"/new_map_planner", "/navigation_mission_manager",
                  ROOT + "/vehicle_loopback_sim", ROOT + "/final_navigation"}


def scenarios():
    """All obstacle centres are > .25 m from x1_y3, on adjacent straight roads."""
    cases = []
    for name, direction, yaw_offset, gap in [
        ("south_nominal", -1, 0, .36), ("north_mirror", 1, 0, .36),
        ("south_yaw_plus6", -1, 6, .36), ("south_yaw_minus6", -1, -6, .36),
        ("south_less_space", -1, 0, .31),
    ]:
        y = JUNCTION[1] + direction * .16
        cases.append(dict(name=name, initial=[1.2, y, direction * math.pi / 2 +
                                             math.radians(yaw_offset)],
                          obstacle=[1.2, y + direction * gap],
                          targets=["2" if direction < 0 else "11"], after_turn=False))
    cases.append(dict(name="after_normal_90deg_turn", initial=[1.5, -.5, math.pi],
                      obstacle=[1.2, -1.05], targets=["2"], after_turn=True))
    for case in cases:
        case.update(junction="x1_y3", expected_turns=None)
    cases.extend([
        dict(name="t_90", initial=[2.36, -1.5, 0.0], obstacle=[2.72, -1.5],
             targets=["4"], after_turn=False, junction="x0_y2",
             expected_template="t_junction_left", expected_turns=1),
        dict(name="t_u", initial=[2.36, -1.5, 0.0], obstacle=[2.72, -1.5],
             targets=["4"], after_turn=False, junction="x0_y2",
             expected_template="t_junction_reverse", expected_turns=2),
        dict(name="x_left", initial=[1.2, -.66, -math.pi/2], obstacle=[1.2, -1.02],
             targets=["2"], after_turn=False, junction="x1_y3",
             expected_template="x_junction_left", expected_turns=1),
        dict(name="x_right", initial=[1.2, -.66, -math.pi/2], obstacle=[1.2, -1.02],
             targets=["2"], after_turn=False, junction="x1_y3",
             expected_template="x_junction_right", expected_turns=1),
        dict(name="x_reverse", initial=[1.2, -.66, -math.pi/2], obstacle=[1.2, -1.02],
             targets=["2"], after_turn=False, junction="x1_y3",
             expected_template="x_junction_reverse", expected_turns=2),
        dict(name="long_retreat", initial=[1.2, -1.36, -math.pi/2],
             obstacle=[1.2, -1.48], targets=["2"], after_turn=False,
             junction="x1_y3", expected_turns=1, minimum_reverse_m=.85,
             obstacle_half_size=[.09, .01]),
        dict(name="during_normal_90deg_turn", initial=[1.5, -.5, math.pi],
             obstacle=[1.2, -.90], targets=["2"], after_turn=False,
             junction="x1_y3", during_turn=True, expected_turns=None,
             original_lane="ROAD_X1_Y2"),
        # 用户回环第二次遇障：朝北进入十字路口、转东遇障后退回原入口。
        dict(name="cross_return_other_exit", initial=[2.2, -.80, math.pi/2],
             obstacle=[2.524, -.5], targets=["4"], after_turn=False,
             junction="x1_y2", during_turn=True, expected_turns=1,
             reuse_returned_entry=True,
             expected_template="x_junction_left", original_lane="ROAD_Y2_LEFT",
             obstacle_half_size=[.06, .10]),
        dict(name="south_offcenter", initial=[1.215, -.66, -math.pi/2],
             obstacle=[1.2, -1.02], targets=["2"], after_turn=False,
             junction="x1_y3", expected_turns=1),
        dict(name="l_during_turn", initial=[.2, -.30, -math.pi/2],
             obstacle=[.45, -.5], targets=["2"], after_turn=False,
             junction="launch_junction", during_turn=True, expected_turns=None,
             original_lane="ROAD_TOP_LEFT", recovery_heading=-math.pi/2,
             obstacle_half_size=[.04, .09]),
        dict(name="l_after_turn", initial=[.2, -.30, -math.pi/2],
             obstacle=[.892, -.5], targets=["2"], after_turn=True,
             junction="launch_junction", expected_turns=1,
             original_lane="ROAD_TOP_LEFT", recovery_heading=-math.pi/2,
             completed_l=True, normal_heading=0.0, normal_x_min=.33,
             obstacle_half_size=[.06, .10]),
        dict(name="l_bottom_after", initial=[3.98, -1.5, 0.0],
             obstacle=[4.2, -.92], targets=["6"], after_turn=True,
             junction="x0_y1", expected_turns=1, completed_l=True,
             original_lane="ROAD_X0_Y0", recovery_heading=0.0,
             normal_heading=math.pi/2, normal_lane="ROAD_BOTTOM_LEFT",
             normal_corner="x0_y0", obstacle_half_size=[.09, .04]),
        dict(name="l_bottom_during", initial=[3.98, -1.5, 0.0],
             obstacle=[4.2, -1.23], targets=["6"], after_turn=False,
             junction="x0_y1", during_turn=True, expected_turns=1,
             original_lane="ROAD_X0_Y0", recovery_heading=0.0,
             obstacle_half_size=[.09, .04]),
        # 2026-10-15 用户卡住现场的路线起点和第三个障碍位置。
        dict(name="l_user_bottom_after",
             initial=[4.161918579621634, -1.5000356170181426, 6.163065084185959e-05],
             obstacle=[4.2, -.7370374356818018], targets=["6"], after_turn=True,
             junction="x0_y1", expected_turns=1, completed_l=True,
             original_lane="ROAD_X0_Y0", recovery_heading=0.0,
             normal_heading=math.pi/2, normal_lane="ROAD_BOTTOM_LEFT",
             normal_corner="x0_y0", obstacle_half_size=[.10, .05],
             extra_obstacles=[[1.2, -.9617478514809226, .10, .05],
                              [1.2, 1.1713241552092384, .10, .05]]),
    ])
    return cases


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False),
                    encoding="utf-8")


def clean_json(value):
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)  # Keep inf/nan observations without invalid JSON constants.
    return value


def stop_owned(process, first_timeout=8):
    """Only a process started with start_new_session=True by this runner."""
    if process is None or process.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT, first_timeout), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def plot_trace(directory, case, odom, phases, polygon, road_map):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    if odom:
        ax.plot([p["x"] for p in odom], [p["y"] for p in odom], color=".7", lw=1)
        forward = [p for p in odom if p["linear"] > .001]
        reverse = [p for p in odom if p["linear"] < -.001]
        for points, label, color in [(forward, "forward", "tab:blue"),
                                      (reverse, "reverse", "tab:orange")]:
            ax.scatter([p["x"] for p in points], [p["y"] for p in points],
                       s=4, label=label, color=color)
    if polygon:
        ax.fill([p[0] for p in polygon], [p[1] for p in polygon],
                color="tab:red", alpha=.4, label="manual obstacle")
    seen = set()
    for phase in phases:
        pose = phase["pose"]
        if pose and phase["phase"] not in seen:
            ax.annotate(phase["phase"], (pose[0], pose[1]))
            seen.add(phase["phase"])
    ax.scatter(*road_map.nodes[case["junction"]], marker="+", color="black", label="junction")
    ax.scatter(*case["initial"][:2], marker="o", color="green", label="initial")
    ax.set(xlabel="map x [m]", ylabel="map y [m]", title=case["name"], aspect="equal")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(directory / "trajectory.png", dpi=150)
    plt.close(fig)


class Evidence:
    """Evidence state and continuous straight-run measurement; never commands motion."""
    def __init__(self, case, road_map=None, entry_config=None):
        self.case = case
        self.road_map = road_map
        self.entry_config = entry_config or (-.070, 0.0)
        self.entry_limits = (.003, .010, .080, .22)
        self.candidates = {}
        self.candidate_by_id = {}
        self.selected_candidate_id = None
        self.turn_phase_history = []
        self.recovery_active = False
        self.injected_during_turn = False
        self.detected_status = ""
        self.events, self.odom, self.cmd, self.phases = [], [], [], []
        self.status = ""
        self.manager_ready = False
        self.last_cmd = None
        self.injected_at = self.detected_at = self.dynamic_at = None
        self.complete_at = None
        self.recovery_id = None
        self.complete_lane = ""
        self.complete_pose = None
        self.phase_by_id = {}
        self.candidate_headings = {}
        self.recovery_status_history = []
        self.turn_entries = []
        self.protocol_error = ""
        self.turn_seen = False
        self.normal_turn_confirmed = False
        self.segment_start = self.previous = None
        self.distance = self.max_distance = 0.0
        self.failure = ""
        self.success = False
        self.start = time.monotonic()

    def now(self):
        return time.monotonic() - self.start

    def add(self, kind, payload):
        self.events.append(dict(t=self.now(), kind=kind, data=clean_json(payload)))

    def observe_status(self, value):
        self.status = value
        if value.startswith("POSE_RECOVERY_"):
            if not self.recovery_status_history or self.recovery_status_history[-1] != value:
                self.recovery_status_history.append(value)
        self.turn_seen |= "TURN_60" in value or "TURN_30" in value
        if value.startswith("POSE_RECOVERY_"):
            self.recovery_active = True
        if value in ("POSE_RECOVERY_TURN_60", "POSE_RECOVERY_TURN_30"):
            if not self.turn_phase_history or self.turn_phase_history[-1] != value:
                self.turn_phase_history.append(value)
        if "FAULT" in value:
            self.failure = "controller/manager status: " + value

    def observe_mission_status(self, value):
        if "FAULT" in value:
            self.failure = value
        match = re.search(r"pose_recovery_select:candidate=(\d+)", value)
        if match:
            self.selected_candidate_id = int(match.group(1))

    def observe_recovery(self, msg):
        phase = str(msg.phase).lower()
        rid = int(msg.recovery_id)
        pose = self.odom[-1] if self.odom else None
        record = dict(t=self.now(), phase=phase, recovery_id=rid,
                      obstacle_revision=int(msg.obstacle_revision),
                      resume_lane=str(msg.resume_lane),
                      pose=[pose["x"], pose["y"], pose["yaw"]] if pose else None,
                      cmd_direction=("forward" if self.last_cmd and self.last_cmd[1] > .001
                                     else "reverse" if self.last_cmd and self.last_cmd[1] < -.001
                                     else "stopped"))
        self.phases.append(record)
        for candidate in getattr(msg, "candidates", []):
            self.candidate_headings[str(candidate.resume_lane)] = float(candidate.resume_heading)
            record = dict(template_id=str(candidate.template_id), resume_lane=str(candidate.resume_lane),
                          resume_node=str(candidate.resume_node), resume_heading=float(candidate.resume_heading),
                          borrowed_lane_ids=list(candidate.borrowed_lane_ids))
            self.candidate_by_id[int(candidate.candidate_id)] = record
            self.candidates.setdefault(str(candidate.resume_lane), record)
        if phase == "failed":
            self.failure = "recovery phase failed: " + msg.reason
        if self.injected_at is None or rid <= 0:
            return
        history = self.phase_by_id.setdefault(rid, [])
        if phase == "complete" and self.complete_at is None:
            if not ("candidates" in history and "motion" in history and
                    history.index("candidates") < history.index("motion")):
                self.failure = "complete without matching candidates -> motion evidence"
            elif self.detected_at is None or self.dynamic_at is None:
                self.failure = "complete without real Bool detection and dynamic polygon"
            elif not msg.resume_lane:
                self.failure = "complete has empty resume_lane"
            else:
                self.complete_at, self.recovery_id = self.now(), rid
                self.complete_lane = msg.resume_lane
                self.complete_pose = pose
                self.protocol_error = self.validate_recovery_protocol()
                if self.protocol_error:
                    self.failure = self.protocol_error
                self.segment_start = self.previous = None
        elif self.complete_at is not None and phase in ("candidates", "motion"):
            self.failure = "recovery restarted after complete before continuation"
        history.append(phase)

    def validate_recovery_protocol(self):
        history = self.recovery_status_history
        required = ["POSE_RECOVERY_RETREAT_TO_TURN_POINT",
                    "POSE_RECOVERY_TURN_APPROACH_ENTRY",
                    "POSE_RECOVERY_TURN_60",
                    "POSE_RECOVERY_TURN_30"]
        if self.case.get("reuse_returned_entry"):
            # 已倒转回同一路口入口；后续仍独立检查 7 cm 到位及连续停车。
            required[0] = "POSE_RECOVERY_ORIGINAL_LANE_STOP"
        cursor = -1
        for name in required:
            try:
                cursor = history.index(name, cursor + 1)
            except ValueError:
                return "missing recovery phase order: " + name
        reverse = [row for row in self.odom
                   if row["status"] == "POSE_RECOVERY_RETREAT_TO_TURN_POINT" and row["linear"] < -.005]
        if not reverse and not self.case.get("reuse_returned_entry"):
            return "no measured reverse motion to legal turn point"
        stopped = [row for row in self.odom
                   if row["status"] == "POSE_RECOVERY_DIRECTION_STOP" and
                   abs(row["linear"]) < .002 and abs(row["angular"]) < .03]
        if not stopped:
            return "no measured stop before recovery direction change"
        stop_error = measured_direction_stop_error(self.odom)
        if stop_error:
            return stop_error
        rotating = [row for row in self.odom if row["status"] in
                    ("POSE_RECOVERY_TURN_60", "POSE_RECOVERY_TURN_30") and
                    abs(row["angular"]) > .15]
        if not rotating or any(row["linear"] < .005 for row in rotating):
            return "特定转弯缺少实测前进分量，不能用原地旋转替代"
        turns60 = sum("TURN_60" in value for value in self.turn_phase_history)
        turns30 = sum("TURN_30" in value for value in self.turn_phase_history)
        expected = self.case.get("expected_turns")
        if turns60 == 0 or turns60 != turns30 or (expected is not None and turns60 != expected):
            return f"fixed-turn evidence incomplete: TURN_60={turns60}, TURN_30={turns30}, expected={expected}"
        if len(self.turn_entries) != turns60:
            return "no independent measured entry pose for each fixed turn"
        selected = self.candidate_by_id.get(self.selected_candidate_id) or self.candidates.get(self.complete_lane)
        if selected is None:
            return "selected recovery lane has no published candidate"
        if selected["resume_lane"] != self.complete_lane:
            return "reported actual exit lane differs from selected candidate"
        if self.case.get("reuse_returned_entry") and (
                selected["resume_node"] != self.case["junction"] or
                not self.case.get("during_turn")):
            return "返回入口复用必须仍是原先发生转弯的路口"
        if self.road_map is None or self.complete_pose is None:
            return "missing independent map/exit pose evidence"
        for index, entry in enumerate(self.turn_entries):
            lane = selected["borrowed_lane_ids"][0] if index == 0 and "reverse" in selected["template_id"] else selected["resume_lane"]
            outgoing = self.road_map.projection((entry["x"],entry["y"]), lane, selected["resume_node"])["heading"]
            incoming = (self.case.get("recovery_heading", self.case["initial"][2])
                        if index == 0 else first_outgoing)
            # 起始扰动只影响车辆实测姿态，道路入口仍采用地图的正交航向。
            incoming = round(incoming/(math.pi/2))*(math.pi/2)
            if self.case["after_turn"] and index == 0 and not self.case.get("completed_l"):
                incoming = -math.pi/2
            checked = self.road_map.turn_entry((entry["x"],entry["y"]), entry["yaw"],
                selected["resume_node"], *self.entry_config, outgoing_heading=outgoing,
                incoming_heading=incoming)
            entry.update(checked)
            error = fixed_entry_stop_error(self.odom, entry, incoming, *self.entry_limits)
            if error:
                return error + ": " + repr(entry)
            first_outgoing = outgoing
        if self.case.get("expected_template") and selected["template_id"] != self.case["expected_template"]:
            return "selected template does not exercise requested direction: " + selected["template_id"]
        error = self.road_map.exit_error(
            (self.complete_pose["x"], self.complete_pose["y"]), self.complete_pose["yaw"], selected)
        if error:
            return error
        measured_reverse = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"])
            for a,b in zip(reverse,reverse[1:]) if 0 < b["stamp"]-a["stamp"] <= .15)
        if measured_reverse < self.case.get("minimum_reverse_m", 0.0):
            return f"long retreat not observed: {measured_reverse:.3f} m"
        if self.case.get("during_turn") or self.case.get("completed_l"):
            if self.case.get("during_turn") and not self.injected_during_turn:
                return "obstacle was not injected during real normal TURN_60/TURN_30"
            if self.case.get("during_turn") and self.detected_status not in ("TURN_60", "TURN_DIAGONAL_CREEP", "TURN_30",
                                            "POSE_RECOVERY_FINISH_CURRENT_TURN"):
                return "obstacle was detected outside active normal turn: " + self.detected_status
            cursor = -1
            phases = [item["phase"] for item in self.phases]
            prelude = ([] if self.case.get("completed_l") else ["finish_turn"])
            for phase in prelude + ["turn_stop", "post_turn_point", "post_turn_stop",
                                    "reverse_turn", "return_stop", "braking"]:
                try:
                    cursor = phases.index(phase, cursor + 1)
                except ValueError:
                    return "missing interrupted-turn recovery phase: " + phase
            returning = [row for row in self.odom if row["status"].startswith("POSE_RECOVERY_REVERSE_RETURN_TURN")
                         and row["linear"] < -.005]
            if not returning:
                return "no measured reverse fixed turn back to original lane"
            original = [row for row in self.odom if row["status"] == "POSE_RECOVERY_RETURN_TURN_STOP"]
            if not original:
                return "no measured original-lane return pose"
            row = original[-1]
            projection = self.road_map.projection((row["x"],row["y"]), self.case["original_lane"])
            if abs(projection["cross"]) > .025 or angle_error(row["yaw"],self.case["initial"][2]) > .15:
                return "interrupted-turn recovery did not restore original lane and heading"
            outgoing_stops = [p for p in self.odom
                if p["status"] == "POSE_RECOVERY_POST_TURN_POINT_STOP" and
                abs(p["linear"]) < .002 and abs(p["angular"]) < .03]
            if len(outgoing_stops) < 3 or outgoing_stops[-1]["stamp"]-outgoing_stops[0]["stamp"] < .22:
                return "L 返回前没有在出口固定点连续实测停稳"
            if self.case.get("completed_l") or self.case["name"].startswith("l_"):
                road = self.road_map.lanes[self.case["original_lane"]]
                corner = road["to"] if road["from"] == selected["resume_node"] else road["from"]
                anchor = self.road_map.nodes[corner]
                heading = self.case["initial"][2]
                dx,dy = row["x"]-anchor[0],row["y"]-anchor[1]
                if (abs(dx*math.cos(heading)+dy*math.sin(heading)-self.entry_config[0]) >
                    self.entry_limits[0] or abs(-dx*math.sin(heading)+dy*math.cos(heading)) >
                    self.entry_limits[1]):
                    return "L 倒转未回到原道路的 7 cm 固定入口"
        heading = self.candidate_headings.get(self.complete_lane)
        if heading is None:
            return "selected recovery lane has no published candidate heading"
        if self.complete_pose is None:
            return "recovery complete has no odometry pose"
        heading_error = abs(math.atan2(
            math.sin(self.complete_pose["yaw"] - heading),
            math.cos(self.complete_pose["yaw"] - heading)))
        if heading_error > .15:
            return "strict turn completed with wrong heading: " + f"{heading_error:.3f} rad"
        return ""

    def observe_odom(self, msg):
        p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        row = dict(t=self.now(), stamp=stamp, x=p.x, y=p.y, yaw=yaw,
                   linear=twist.linear.x, angular=twist.angular.z, status=self.status)
        self.odom.append(row)
        turns60 = sum("TURN_60" in value for value in self.turn_phase_history)
        if self.status == "POSE_RECOVERY_TURN_60" and len(self.turn_entries) < turns60:
            if self.road_map is not None:
                entry = self.road_map.turn_entry((p.x, p.y), yaw, self.case["junction"], *self.entry_config)
                entry.update(t=row["t"], x=p.x, y=p.y, yaw=yaw)
                self.turn_entries.append(entry)
        if self.case["after_turn"] and self.turn_seen and self.status == "FOLLOWING":
            normal_heading = self.case.get("normal_heading", -math.pi/2)
            confirmed_position = (p.x >= self.case["normal_x_min"] if "normal_x_min" in self.case
                                  else abs(p.x-1.2) < .06)
            if "normal_lane" in self.case:
                projection = self.road_map.projection((p.x,p.y),self.case["normal_lane"],
                  self.case["normal_corner"])
                confirmed_position = projection["along"] >= .13 and abs(projection["cross"]) <= .03
            self.normal_turn_confirmed |= angle_error(yaw,normal_heading) < .15 and confirmed_position
        if self.complete_at is None or self.failure:
            return
        fresh_cmd = self.last_cmd and 0 <= row["t"] - self.last_cmd[0] <= .25
        eligible = (self.status == "FOLLOWING" and twist.linear.x > .005 and
                    abs(twist.angular.z) < .1 and fresh_cmd and self.last_cmd[1] > .005
                    and abs(self.last_cmd[2]) < .1)
        previous = self.previous
        contiguous = previous and 0 < stamp - previous["stamp"] <= .15
        if not eligible:
            self.segment_start = self.previous = None
            self.distance = 0.0
            return
        if not contiguous:
            self.segment_start, self.distance = row, 0.0
        else:
            step = math.hypot(p.x - previous["x"], p.y - previous["y"])
            if step > .03:
                self.failure = "odom jump during post-complete continuation"
                return
            self.distance += step
        self.previous = row
        displacement = math.hypot(p.x - self.segment_start["x"], p.y - self.segment_start["y"])
        self.max_distance = max(self.max_distance, min(self.distance, displacement))
        self.success = self.max_distance >= .20


def worker(config_path):
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    directory, case = Path(config["directory"]), config["case"]
    map_path = config["planner_config"]
    import yaml
    controller_config = Path(config["controller_config"])
    params = yaml.safe_load(controller_config.read_text(encoding="utf-8"))["final_navigation"]["ros__parameters"]
    entry_config = (params["recovery_turn_entry_target_x"], params["recovery_turn_entry_target_y"])
    evidence, process, node = Evidence(case, RoadMap(map_path), entry_config), None, None
    evidence.entry_limits = (params["recovery_turn_entry_tolerance"],
        params["turn_maneuver_entry_lateral_tolerance"],
        params["turn_maneuver_entry_yaw_tolerance"], params["recovery_switch_stop_s"])
    polygon, result = [], dict(pass_=False, reason="not started")
    interrupted = False

    def interrupt(signum, frame):
        nonlocal interrupted
        interrupted = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from geometry_msgs.msg import PoseWithCovarianceStamped, PolygonStamped, Point32, Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Range
        from std_msgs.msg import Bool, String
        from mission_control_interfaces.msg import NavigationRecoveryState, MissionState
        from mission_control_interfaces.srv import ControlMission
        import matplotlib  # Fail early if required PNG cannot be produced.
        if not hasattr(NavigationRecoveryState(), "phase"):
            raise RuntimeError("installed NavigationRecoveryState lacks phase; rebuild/source workspace")
        rclpy.init()
        node = rclpy.create_node("pose_recovery_scenario_probe",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        occupied_until = time.monotonic() + 1.5
        while time.monotonic() < occupied_until and not interrupted:
            rclpy.spin_once(node, timeout_sec=.05)
        others = [n for n, ns in node.get_node_names_and_namespaces()
                  if n != node.get_name()]
        if others:
            raise RuntimeError("ROS_DOMAIN_ID already has nodes; refusing to interfere: " + repr(others))
        durable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = []

        def subscribe(kind, msg_type, topic, handler=None, qos=qos_profile_sensor_data):
            def callback(msg):
                evidence.add(kind, message_to_ordereddict(msg))
                if handler:
                    handler(msg)
            subscriptions.append(node.create_subscription(msg_type, topic, callback, qos))

        def cmd(msg):
            row = dict(t=evidence.now(), linear=msg.linear.x, angular=msg.angular.z)
            evidence.cmd.append(row)
            evidence.last_cmd = (row["t"], row["linear"], row["angular"])

        def blocked(msg):
            if msg.data and evidence.injected_at is not None and evidence.detected_at is None:
                evidence.detected_at = evidence.now()
                evidence.detected_status = evidence.status

        def dynamic(msg):
            if len(msg.polygon.points) >= 3 and evidence.injected_at is not None:
                evidence.dynamic_at = evidence.now()

        subscribe("odom", Odometry, ROOT + "/odometry/fused", evidence.observe_odom)
        subscribe("cmd", Twist, ROOT + "/cmd_vel", cmd)
        subscribe("status", String, ROOT + "/navigator_status",
                  lambda msg: evidence.observe_status(msg.data), durable)
        subscribe("mission_status", String, "/comparison/mission_status",
                  lambda msg: evidence.observe_mission_status(msg.data))
        def manager_state(msg):
            evidence.manager_ready = msg.state == MissionState.IDLE
            if msg.state == MissionState.FAULT:
                evidence.failure = "manager FAULT: " + msg.message

        subscribe("mission_state", MissionState, "/navigation_mission/state", manager_state)
        subscribe("recovery", NavigationRecoveryState, ROOT + "/navigation_recovery_state",
                  evidence.observe_recovery, durable)
        subscribe("front_blocked", Bool, ROOT + "/obstacle/front_blocked", blocked)
        subscribe("front_range", Range, ROOT + "/obstacle/front_range")
        subscribe("polygons", PolygonStamped, ROOT + "/obstacle/dynamic", dynamic)
        initial_pub = node.create_publisher(PoseWithCovarianceStamped, ROOT + "/initialpose", 10)
        obstacle_pub = node.create_publisher(PolygonStamped, ROOT + "/obstacle/manual", 10)
        client = node.create_client(ControlMission, "/navigation_mission/control")
        launch_args = ["ros2", "launch", "vehicle_loopback_sim",
                       "final_navigation_obstacle_validation.launch.py",
                       "auto_start:=false", "gui:=false", "target_ids:=" + ",".join(case["targets"]),
                       "simulation_time_scale:=" + str(config.get("simulation_time_scale", 1.0))]
        if config.get("planner_config"):
            launch_args.append("planner_config:=" + config["planner_config"])
        dump(directory / "launch_command.json", launch_args)
        with (directory / "launch.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(launch_args, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            def wait_until(predicate, timeout, description):
                deadline = time.monotonic() + timeout
                while not predicate():
                    if interrupted:
                        raise RuntimeError("interrupted")
                    if process.poll() is not None:
                        raise RuntimeError("launch exited before " + description)
                    if evidence.failure:
                        raise RuntimeError(evidence.failure)
                    if time.monotonic() >= deadline:
                        raise RuntimeError("timeout waiting for " + description)
                    rclpy.spin_once(node, timeout_sec=.03)

            def ready():
                names = {ns.rstrip("/") + "/" + n for n, ns in node.get_node_names_and_namespaces()}
                return (REQUIRED_NODES <= names and client.service_is_ready() and evidence.odom
                        and initial_pub.get_subscription_count() > 0
                        and obstacle_pub.get_subscription_count() > 0)
            wait_until(ready, config["startup_timeout"], "four real nodes and manager service")
            actual_nodes = node.get_node_names_and_namespaces()
            evidence.add("nodes", actual_nodes)
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.pose.pose.position.x, pose.pose.pose.position.y, yaw = case["initial"]
            pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            pose.header.stamp = node.get_clock().now().to_msg()
            initial_pub.publish(pose)
            evidence.add("initialpose", message_to_ordereddict(pose))
            wait_until(lambda: evidence.odom and
                       math.hypot(evidence.odom[-1]["x"] - case["initial"][0],
                                  evidence.odom[-1]["y"] - case["initial"][1]) < .01 and
                       abs(math.atan2(math.sin(evidence.odom[-1]["yaw"] - yaw),
                                      math.cos(evidence.odom[-1]["yaw"] - yaw))) < .02,
                       5, "initialpose acknowledged by measured odometry")
            wait_until(lambda: evidence.manager_ready, config["startup_timeout"],
                       "mission manager IDLE readiness")
            request = ControlMission.Request(command=ControlMission.Request.START,
                                             target_ids=case["targets"])
            pending = client.call_async(request)
            wait_until(pending.done, 10, "manager START response")
            response = pending.result()
            evidence.add("manager_start", message_to_ordereddict(response))
            if not response.accepted:
                raise RuntimeError("manager rejected START: " + response.message)
            if case.get("during_turn"):
                wait_until(lambda: evidence.status in ("TURN_60", "TURN_30", "CORNER_SHUNT_TURN_60", "CORNER_SHUNT_TURN_30"),
                           config["timeout"], "real normal fixed-turn phase before injection")
                evidence.injected_during_turn = True
            else:
                wait_until(lambda: evidence.status == "FOLLOWING" and evidence.odom and
                           evidence.odom[-1]["linear"] > .005 and
                           (not case["after_turn"] or evidence.normal_turn_confirmed),
                           config["timeout"], "normal approach / required real 90 degree turn")
            x, y = case["obstacle"]
            # Fixed test fixture, .20 m across road and .12 m along road, same as GUI defaults.
            hx, hy = case.get("obstacle_half_size", [.10, .06])
            polygon = [(x-hx, y-hy), (x+hx, y-hy), (x+hx, y+hy), (x-hx, y+hy)]
            obstacle = PolygonStamped()
            obstacle.header.frame_id = "map"
            obstacle.header.stamp = node.get_clock().now().to_msg()
            obstacle.polygon.points = [Point32(x=float(a), y=float(b), z=0.0) for a, b in polygon]
            evidence.injected_at = evidence.now()
            for ex, ey, ehx, ehy in case.get("extra_obstacles", []):
                extra = PolygonStamped()
                extra.header = obstacle.header
                extra.polygon.points = [Point32(x=float(a), y=float(b), z=0.0) for a, b in
                    [(ex-ehx, ey-ehy), (ex+ehx, ey-ehy),
                     (ex+ehx, ey+ehy), (ex-ehx, ey+ehy)]]
                obstacle_pub.publish(extra)
                evidence.add("injected_extra_polygon", message_to_ordereddict(extra))
            obstacle_pub.publish(obstacle)
            evidence.add("injected_polygon", message_to_ordereddict(obstacle))
            deadline, next_log_check = time.monotonic() + config["timeout"], 0.0
            while not evidence.failure and not evidence.success and not interrupted:
                rclpy.spin_once(node, timeout_sec=.03)
                if process.poll() is not None:
                    evidence.failure = "launch exited during scenario"
                if time.monotonic() > deadline:
                    evidence.failure = "timeout: detection -> complete -> .20m straight sequence incomplete"
                if time.monotonic() >= next_log_check:
                    text = (directory / "launch.log").read_text(encoding="utf-8", errors="replace")
                    match = BAD_LOG.search(text)
                    if match:
                        evidence.failure = "launch log safety/fault evidence: " + match.group(0)
                    next_log_check = time.monotonic() + .3
            if interrupted:
                evidence.failure = "interrupted"
            result = dict(pass_=bool(evidence.success and not evidence.failure),
                          reason=evidence.failure or "detection -> complete -> measured .20m straight confirmed")
    except Exception as error:
        result = dict(pass_=False, reason=str(error), traceback=traceback.format_exc())
    finally:
        stop_owned(process)
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        text = (directory / "launch.log").read_text(encoding="utf-8", errors="replace") if (directory / "launch.log").exists() else ""
        bad_lines = [line for line in text.splitlines() if BAD_LOG.search(line)]
        adjustment = {key: text.count(marker) for key, marker in {
            "blocked": "Pose recovery retreat sweep blocked;",
            "started": "Pose recovery forward adjustment started:",
            "finished": "Pose recovery forward adjustment finished:",
            "retry_stopped": "Pose recovery retreat retry armed after confirmed stop",
        }.items()}
        result["forward_adjustment"] = adjustment
        if config.get("require_forward_adjustment"):
            if any(n != 1 for n in adjustment.values()):
                result.update(pass_=False, reason="missing single forward-adjustment/retry evidence")
            else:
                times = []
                for marker in ("Pose recovery retreat sweep blocked;",
                               "Pose recovery forward adjustment started:",
                               "Pose recovery forward adjustment finished:",
                               "Pose recovery retreat retry armed after confirmed stop"):
                    line = next(line for line in text.splitlines() if marker in line)
                    times.append(float(re.search(r"\[(\d+\.\d+)\]", line).group(1)))
                blocked, started, finished, retry = times
                def stopped_between(begin, end):
                    return sum(begin <= row["stamp"] < end and abs(row["linear"]) < .002
                               and abs(row["angular"]) < .03 for row in evidence.odom)
                forward = [row for row in evidence.odom if started <= row["stamp"] < finished
                           and row["linear"] > .005]
                reverse = [row for row in evidence.odom if row["stamp"] >= retry and
                           row["status"] == "POSE_RECOVERY_RETREAT_TO_TURN_POINT" and row["linear"] < -.005]
                adjustment.update(duration_s=finished-started,
                                  stop_before_samples=stopped_between(blocked, started),
                                  stop_after_samples=stopped_between(finished, retry),
                                  forward_samples=len(forward), reverse_retry_samples=len(reverse),
                                  measured_forward_m=sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"])
                                                         for a, b in zip(forward, forward[1:])))
                if not (blocked < started < finished < retry and finished-started <= .50 and
                        adjustment["stop_before_samples"] and adjustment["stop_after_samples"] and
                        adjustment["measured_forward_m"] > .002 and reverse):
                    result.update(pass_=False, reason="forward adjustment lacks bounded motion/two measured stops/reverse retry")
        if bad_lines:
            result.update(pass_=False, reason="FAULT or static guard intervention in launch log")
        result["pass"] = result.pop("pass_")
        result.update(scenario=case, domain_id=config["domain_id"],
                      detected_at=evidence.detected_at, dynamic_at=evidence.dynamic_at,
                      complete_at=evidence.complete_at, recovery_id=evidence.recovery_id,
                      selected_lane=evidence.complete_lane, phases=evidence.phases,
                      recovery_status_history=evidence.recovery_status_history,
                       turn_entries=evidence.turn_entries,
                       protocol_error=evidence.protocol_error,
                       turn_phase_history=evidence.turn_phase_history,
                       selected_candidate_id=evidence.selected_candidate_id,
                       candidate_by_id=evidence.candidate_by_id,
                       injected_during_turn=evidence.injected_during_turn,
                       detected_status=evidence.detected_status,
                       map_id=evidence.road_map.map_id,
                       max_continuous_straight_m=evidence.max_distance,
                      normal_turn_confirmed=evidence.normal_turn_confirmed,
                      bad_log_lines=bad_lines, nodes_required=sorted(REQUIRED_NODES))
        dump(directory / "events.json", evidence.events)
        for filename, rows, fields in [
            ("odom.csv", evidence.odom, ["t", "stamp", "x", "y", "yaw", "linear", "angular", "status"]),
            ("cmd.csv", evidence.cmd, ["t", "linear", "angular"]),
            ("recovery.csv", evidence.phases, ["t", "phase", "recovery_id", "obstacle_revision", "resume_lane", "pose", "cmd_direction"]),
        ]:
            with (directory / filename).open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
        with (directory / "events.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["t", "kind", "payload_json"])
            for event in evidence.events:
                writer.writerow([event["t"], event["kind"], json.dumps(event["data"], ensure_ascii=False)])
        try:
            plot_trace(directory, case, evidence.odom, evidence.phases, polygon, evidence.road_map)
        except Exception as error:
            result.update({"pass": False, "plot_error": str(error),
                           "reason": result["reason"] + "; PNG export failed: " + str(error)})
        dump(directory / "result.json", result)
    return 0 if result["pass"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    candidates = (Path.cwd(), Path(__file__).resolve().parents[3])
    workspace_default = next((p for p in candidates if (p / "install/setup.bash").is_file()), Path.cwd())
    parser.add_argument("--workspace", type=Path, default=workspace_default)
    preferred = Path("/media/hjh/E/rosbag_recording/navigation_runs")
    fallback = Path("/media/hjh/Data/rosbag_recording/navigation_runs")
    base = next((p for p in (preferred, fallback) if p.is_dir() and os.access(p, os.W_OK)), Path("/tmp"))
    parser.add_argument("--output", type=Path, default=base / ("pose_recovery_" + datetime.now().strftime("%Y%m%d_%H%M%S")))
    parser.add_argument("--base-domain", type=int, default=170, help="one distinct domain per scenario; refuses occupied domains")
    parser.add_argument("--timeout", type=float, default=180, help="maximum seconds per approach and per obstacle recovery")
    parser.add_argument("--startup-timeout", type=float, default=45)
    parser.add_argument("--simulation-time-scale", type=float, default=1.0,
                        help="shared /clock time scale; vehicle speed parameters stay unchanged")
    parser.add_argument("--planner-config", type=Path, help="optional existing arena config; default scenarios require matching map geometry")
    parser.add_argument("--scenario", "--only", action="append", choices=[c["name"] for c in scenarios()], help="select scenarios; direction cases fail unless their expected template actually executes")
    parser.add_argument("--require-forward-adjustment", action="store_true",
                        help="require exactly one blocked retreat, forward adjustment and confirmed-stop retry")
    parser.add_argument("--all", action="store_true", help="run every directional scenario; default is original six plus interrupted turn")
    parser.add_argument("--list", action="store_true", help="list scenarios without importing ROS or starting processes")
    parser.add_argument("--worker", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        return worker(args.worker)
    cases = [c for c in scenarios() if not args.scenario or c["name"] in args.scenario]
    if args.list:
        print(json.dumps(cases, ensure_ascii=False, indent=2))
        return 0
    if not args.scenario and not args.all:
        cases = [case for case in cases if case["name"] in {c["name"] for c in scenarios()[:6]} or case.get("during_turn")]
    if not 0 <= args.base_domain <= 232 - len(cases) + 1:
        parser.error("domain range must lie within 0..232")
    if min(args.timeout, args.startup_timeout) <= 0:
        parser.error("timeouts must be positive")
    if not math.isfinite(args.simulation_time_scale) or not .1 <= args.simulation_time_scale <= 10:
        parser.error("simulation time scale must be within 0.1..10")
    workspace = args.workspace.resolve()
    for setup in [Path("/opt/ros/humble/setup.bash"), workspace / "install/setup.bash"]:
        if not setup.is_file():
            parser.error("missing ROS setup: " + str(setup))
    planner_config = args.planner_config or workspace / "src/new_map_planner/config/arena.yaml"
    if not planner_config.is_file():
        parser.error("missing scenario map: " + str(planner_config))
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)  # Never overwrite earlier evidence.
    results, current = [], None
    try:
        for index, case in enumerate(cases):
            directory = output / case["name"]
            directory.mkdir()
            config = dict(case=case, directory=str(directory), domain_id=args.base_domain + index,
                          timeout=args.timeout, startup_timeout=args.startup_timeout,
                          simulation_time_scale=args.simulation_time_scale,
                          planner_config=str(planner_config.resolve()),
                          controller_config=str(workspace / "src/final_navigation/config/final_navigation.yaml"),
                          require_forward_adjustment=args.require_forward_adjustment)
            dump(directory / "scenario.json", config)
            env = dict(os.environ, ROS_DOMAIN_ID=str(config["domain_id"]),
                       ROS_LOG_DIR=str(directory / "ros_logs"), RCUTILS_COLORIZED_OUTPUT="0")
            # Positional shell arguments preserve paths and never interpolate user shell code.
            command = ["bash", "-c", 'source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && exec /usr/bin/python3 "$2" --worker "$3"',
                       "pose-recovery-worker", str(workspace), str(Path(__file__).resolve()), str(directory / "scenario.json")]
            print(f"RUN {case['name']} ROS_DOMAIN_ID={config['domain_id']}", flush=True)
            with (directory / "worker.log").open("w", encoding="utf-8") as stream:
                current = subprocess.Popen(command, cwd=workspace, env=env, stdout=stream,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                current.wait()
            path = directory / "result.json"
            result = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {
                "pass": False, "reason": "worker failed before evidence export; inspect worker.log",
                "scenario": case, "domain_id": config["domain_id"]}
            if current.returncode != 0:
                result["pass"] = False
            results.append(result)
            print(("PASS " if result["pass"] else "FAIL ") + case["name"] + ": " + result["reason"], flush=True)
            dump(output / "summary.json", results)
            current = None
    except KeyboardInterrupt:
        stop_owned(current, 20)
        print("Interrupted; stopped only runner-owned process group", file=sys.stderr)
        return 130
    finally:
        stop_owned(current, 20)
    with (output / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scenario", "domain_id", "pass", "reason", "straight_m", "resume_lane"])
        for result in results:
            writer.writerow([result["scenario"]["name"], result["domain_id"], result["pass"],
                             result["reason"], result.get("max_continuous_straight_m", 0), result.get("selected_lane", "")])
    print("Evidence: " + str(output), flush=True)
    return 0 if len(results) == len(cases) and all(r["pass"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
