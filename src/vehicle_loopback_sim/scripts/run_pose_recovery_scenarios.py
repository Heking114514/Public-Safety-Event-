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
ordering, and the first turn command to be within the legal junction position,
road-center and heading tolerances. FAULT, failed phase, static guard intervention,
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


def plot_trace(directory, case, odom, phases, polygon):
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
    ax.scatter(*JUNCTION, marker="+", color="black", label="junction")
    ax.scatter(*case["initial"][:2], marker="o", color="green", label="initial")
    ax.set(xlabel="map x [m]", ylabel="map y [m]", title=case["name"], aspect="equal")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(directory / "trajectory.png", dpi=150)
    plt.close(fig)


class Evidence:
    """Evidence state and continuous straight-run measurement; never commands motion."""
    def __init__(self, case):
        self.case = case
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
        self.turn_seen |= "TURN_60" in value or "TURN_30" in value or \
            "POSE_RECOVERY_TURN_INTO_EXIT" in value
        if "FAULT" in value:
            self.failure = "controller/manager status: " + value

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
                    "POSE_RECOVERY_ALIGN_FOR_TURN",
                    "POSE_RECOVERY_TURN_INTO_EXIT"]
        positions = []
        cursor = -1
        for name in required:
            try:
                cursor = history.index(name, cursor + 1)
            except ValueError:
                return "missing recovery phase order: " + name
            positions.append(cursor)
        reverse = [row for row in self.odom
                   if row["status"] == required[0] and row["linear"] < -.005]
        if not reverse:
            return "no measured reverse motion to legal turn point"
        stopped = [row for row in self.odom
                   if row["status"] == "POSE_RECOVERY_DIRECTION_STOP" and
                   abs(row["linear"]) < .002 and abs(row["angular"]) < .03]
        if not stopped:
            return "no measured stop before recovery direction change"
        previous_sign, stopped_since_motion = 0, False
        for row in self.odom:
            if row["status"] in ("POSE_RECOVERY_ALIGN_FOR_TURN", "POSE_RECOVERY_TURN_INTO_EXIT"):
                break
            if not row["status"].startswith("POSE_RECOVERY_"):
                continue
            sign = 1 if row["linear"] > .005 else -1 if row["linear"] < -.005 else 0
            if sign:
                if previous_sign and sign != previous_sign and not stopped_since_motion:
                    return "recovery direction changed without measured stop"
                previous_sign, stopped_since_motion = sign, False
            elif row["status"] == "POSE_RECOVERY_DIRECTION_STOP" and abs(row["angular"]) < .03:
                stopped_since_motion = True
        if history.count("POSE_RECOVERY_ALIGN_FOR_TURN") != 1:
            return "strict recovery turn must begin exactly once"
        if not self.turn_entries:
            return "no measured TURN_INTO_EXIT entry"
        entry = self.turn_entries[0]
        if entry["position_error"] > .04:
            return "turn entered before legal point/road center convergence: " + \
                f'{entry["position_error"]:.3f} m'
        if not self.turn_seen:
            return "strict turn controller never entered TURN_60/TURN_30"
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
        if self.status == "POSE_RECOVERY_TURN_INTO_EXIT" and not self.turn_entries:
            position_error = math.hypot(p.x - JUNCTION[0], p.y - JUNCTION[1])
            self.turn_entries.append(dict(t=row["t"], x=p.x, y=p.y, yaw=yaw,
                                          position_error=position_error,
                                          road_center_error=position_error))
        if self.case["after_turn"] and self.turn_seen and self.status == "FOLLOWING":
            yaw_error = abs(math.atan2(math.sin(yaw + math.pi/2), math.cos(yaw + math.pi/2)))
            self.normal_turn_confirmed |= yaw_error < .15 and abs(p.x - 1.2) < .06
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
    evidence, process, node = Evidence(case), None, None
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
        node = rclpy.create_node("pose_recovery_scenario_probe")
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

        def dynamic(msg):
            if len(msg.polygon.points) >= 3 and evidence.injected_at is not None:
                evidence.dynamic_at = evidence.now()

        subscribe("odom", Odometry, ROOT + "/odometry/fused", evidence.observe_odom)
        subscribe("cmd", Twist, ROOT + "/cmd_vel", cmd)
        subscribe("status", String, ROOT + "/navigator_status",
                  lambda msg: evidence.observe_status(msg.data), durable)
        subscribe("mission_status", String, "/comparison/mission_status",
                  lambda msg: setattr(evidence, "failure", msg.data) if "FAULT" in msg.data else None)
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
                       "auto_start:=false", "gui:=false", "target_ids:=" + ",".join(case["targets"])]
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
            wait_until(lambda: evidence.status == "FOLLOWING" and evidence.odom and
                       evidence.odom[-1]["linear"] > .005 and
                       (not case["after_turn"] or evidence.normal_turn_confirmed),
                       config["timeout"], "normal approach / required real 90 degree turn")
            x, y = case["obstacle"]
            # Fixed test fixture, .20 m across road and .12 m along road, same as GUI defaults.
            polygon = [(x-.10, y-.06), (x+.10, y-.06), (x+.10, y+.06), (x-.10, y+.06)]
            obstacle = PolygonStamped()
            obstacle.header.frame_id = "map"
            obstacle.header.stamp = node.get_clock().now().to_msg()
            obstacle.polygon.points = [Point32(x=float(a), y=float(b), z=0.0) for a, b in polygon]
            evidence.injected_at = evidence.now()
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
            plot_trace(directory, case, evidence.odom, evidence.phases, polygon)
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
    parser.add_argument("--base-domain", type=int, default=170, help="six distinct domains; discovery refuses occupied domain")
    parser.add_argument("--timeout", type=float, default=180, help="maximum seconds per approach and per obstacle recovery")
    parser.add_argument("--startup-timeout", type=float, default=45)
    parser.add_argument("--planner-config", type=Path, help="optional existing arena config; default fixtures assume official arena")
    parser.add_argument("--scenario", "--only", action="append", choices=[c["name"] for c in scenarios()], help="select subset for debugging; default runs all six")
    parser.add_argument("--forward-adjustment-fixture", action="store_true",
                        help="run one corner-sweep fixture using an output-local map with 2 mm planning margin")
    parser.add_argument("--require-forward-adjustment", action="store_true",
                        help="require exactly one blocked retreat, forward adjustment and confirmed-stop retry")
    parser.add_argument("--list", action="store_true", help="list fixtures without importing ROS or starting processes")
    parser.add_argument("--worker", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        return worker(args.worker)
    cases = [c for c in scenarios() if not args.scenario or c["name"] in args.scenario]
    if args.forward_adjustment_fixture:
        if args.scenario or args.planner_config:
            parser.error("--forward-adjustment-fixture uses its own official-arena test map")
        cases = [dict(name="retreat_sweep_fallback",
                      initial=[1.2, -1.1, -math.pi/2-math.radians(6)],
                      obstacle=[1.2, -1.4], targets=["11"], after_turn=False,
                      fixture_obstacle_arena=[.562, 3.030, .563, 3.031],
                      fixture_planning_margin_m=.002)]
        args.require_forward_adjustment = True
    if args.list:
        print(json.dumps(cases, ensure_ascii=False, indent=2))
        return 0
    if not 0 <= args.base_domain <= 232 - len(cases) + 1:
        parser.error("domain range must lie within 0..232")
    if min(args.timeout, args.startup_timeout) <= 0:
        parser.error("timeouts must be positive")
    workspace = args.workspace.resolve()
    for setup in [Path("/opt/ros/humble/setup.bash"), workspace / "install/setup.bash"]:
        if not setup.is_file():
            parser.error("missing ROS setup: " + str(setup))
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)  # Never overwrite earlier evidence.
    results, current = [], None
    try:
        for index, case in enumerate(cases):
            directory = output / case["name"]
            directory.mkdir()
            config = dict(case=case, directory=str(directory), domain_id=args.base_domain + index,
                          timeout=args.timeout, startup_timeout=args.startup_timeout,
                          planner_config=str(args.planner_config.resolve()) if args.planner_config else None,
                          require_forward_adjustment=args.require_forward_adjustment)
            dump(directory / "scenario.json", config)
            if args.forward_adjustment_fixture:
                import yaml
                from ament_index_python.packages import get_package_share_directory
                base_map = Path(get_package_share_directory("new_map_planner")) / "config/arena.yaml"
                arena = yaml.safe_load(base_map.read_text(encoding="utf-8"))
                arena["arena"]["obstacles"].append(case["fixture_obstacle_arena"])
                arena["planning"]["safety_margin_m"] = case["fixture_planning_margin_m"]
                fixture_map = directory / "fixture_arena.yaml"
                fixture_map.write_text(yaml.safe_dump(arena, sort_keys=False), encoding="utf-8")
                config.update(planner_config=str(fixture_map), fixture_base_config=str(base_map))
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
