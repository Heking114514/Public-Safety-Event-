#!/usr/bin/env python3
"""Supervise a recorded DM2325 straight-line test; physical distance is external."""

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time


WORKSPACE = Path(__file__).resolve().parents[1]
EVENT_TOPIC = "/dm2325/test_event"
CMD_TOPIC = "/cmd_vel_nav"
ALLOWED_MODES = {
    "FULL", "DEGRADED_DEAD_RECKONING", "DEGRADED_NO_VISION",
    "DEGRADED_NO_WHEEL", "DEGRADED_NO_IMU", "DEGRADED_VISION_ONLY",
    "DEGRADED_WHEEL_ONLY", "DEGRADED_VISUAL_REALIGNED",
}
TOPICS = [
    "/camera/camera/imu", "/cup_car_serial/bmi088_raw_imu",
    "/cup_car_serial/bmi088_attitude", "/cup_car_serial/encoder_ticks",
    "/cup_car_serial/control_telemetry", "/cup_car_serial/rx",
    "/cup_car_serial/connected", "/cup_car_serial/actuator_healthy",
    "/cup_car_serial/actuator_tracking_status", CMD_TOPIC, "/imu/control",
    "/wheel/odom", "/odometry/local", "/odometry/local_map", "/odometry/fused",
    "/odometry/visual_raw", "/odometry/visual_continuous",
    "/fusion/input/visual_odom", "/fusion/input/wheel_odom", "/tracking_state",
    "/orbslam3/map_change", "/odometry/fusion_status", "/tf", "/tf_static",
    "/diagnostics", "/parameter_events", "/rosout", EVENT_TOPIC,
]
DATA_TOPICS = {
    "connected": "/cup_car_serial/connected",
    "healthy": "/cup_car_serial/actuator_healthy",
    "fusion": "/odometry/fusion_status",
    "fused": "/odometry/fused",
    "local": "/odometry/local",
    "wheel": "/wheel/odom",
    "control_imu": "/imu/control",
    "camera_imu": "/camera/camera/imu",
    "visual": "/odometry/visual_raw",
}
STACK_NODES = {
    "/cmd_vel_serial_node", "/wheel_odometry_node", "/fused_odometry_gate",
    "/fused_ekf", "/map_odom_correction", "/local_map_odometry",
    "/camera/camera", "/orbslam3_stereo_inertial", "/waypoint_navigator",
}


def log(message):
    print(f"[odometry-test] {message}", flush=True)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def runtime_dir():
    key = hashlib.sha256(str(WORKSPACE).encode()).hexdigest()[:12]
    return Path(tempfile.gettempdir()) / f"dm2325_odometry_test_{os.getuid()}_{key}"


def process_identity(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return fields[19] if fields[0] != "Z" else None
    except (OSError, IndexError):
        return None


def atomic_json(path, value):
    # Replace only a known adjacent temporary file, never a user-selected tree.
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    os.replace(temporary, path)


def read_active():
    try:
        value = json.loads((runtime_dir() / "active.json").read_text())
        if value.get("phase") not in ("FINISHED", "FAILED") and process_identity(value["pid"]) == value["process_start"]:
            return value
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def control_action(action):
    active = read_active()
    if active is None:
        log("没有正在运行的里程计测试。")
        return 0
    if action == "status":
        log(f"阶段: {active['phase']}；PID: {active['pid']}")
        log(f"速度: {active['speed']} m/s；距离标记: {active['distance']} m")
        log(f"录包及日志: {active['session']}")
    else:
        try:
            # Recheck the process start token immediately before signalling.
            if process_identity(active["pid"]) != active["process_start"]:
                raise ProcessLookupError
            os.kill(active["pid"], signal.SIGINT)
            log("已请求停车；启动终端会发送零速并完成录包保存。")
        except ProcessLookupError:
            log("测试进程已经退出。")
    return 0


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be finite and nonnegative")
    return number


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="一键录包、串口、融合定位和直行测试；到地上距离标记处按 Ctrl+C。"
    )
    parser.add_argument("action", nargs="?", choices=["start", "stop", "status"], default="start")
    parser.add_argument("--speed", type=positive, default=0.06, help="直行速度 m/s (default: 0.06)")
    parser.add_argument("--distance", type=positive, default=3.0, help="现场标线距离 m (default: 3)")
    parser.add_argument("--duration", type=positive, help="最长运动秒数 (default: 1.2 * distance / speed = 60)")
    parser.add_argument("--warmup", type=nonnegative, default=10.0, help="就绪后静止录制秒数 (default: 10)")
    parser.add_argument("--countdown", type=nonnegative, default=3.0, help="行驶前倒计时秒数 (default: 3)")
    parser.add_argument("--ready-timeout", type=positive, default=90.0, help="等待传感器就绪上限 (default: 90)")
    parser.add_argument("--freshness", type=positive, default=1.0, help="健康和里程计新鲜度秒数 (default: 1)")
    parser.add_argument("--camera-serial", default="038122250473")
    parser.add_argument("--serial-device", default="auto")
    parser.add_argument("--serial-baud-rate", type=int, default=115200)
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "10")))
    parser.add_argument("--output-root", type=Path, help="录包目录；默认外接盘，未挂载时使用 log/odometry_tests")
    parser.add_argument("--setup-only", action="store_true", help="只启动定位、串口和录包，保持零速")
    parser.add_argument("--dry-run", action="store_true", help="打印启动命令，不启动 ROS 或运动")
    args = parser.parse_args(argv)
    if not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be between 0 and 232")
    if args.serial_baud_rate <= 0:
        parser.error("--serial-baud-rate must be positive")
    args.duration = args.duration or 1.2 * args.distance / args.speed
    args.output_root = (args.output_root or default_output_root()).expanduser().resolve()
    return args


def default_output_root():
    external = Path("/media/hjh/E/rosbag_recording")
    if external.is_dir() and os.access(external, os.W_OK):
        return external / "odometry_tests"
    return WORKSPACE / "log" / "odometry_tests"


def launch_commands(args, bag):
    ros2 = os.environ.get("ODOMETRY_TEST_ROS2_COMMAND", "ros2")
    settings = WORKSPACE / "src/orb_slam3/orbslam3_ros2/config/stereo-inertial/RealSense_D455_rk3588.yaml"
    return {
        "recorder": [
            ros2, "bag", "record", "--output", str(bag), "--storage", "sqlite3",
            "--compression-mode", "file", "--compression-format", "zstd",
            "--compression-queue-size", "4", "--compression-threads", "2",
            "--max-bag-size", "1073741824", *TOPICS,
        ],
        "serial": [
            ros2, "launch", "cup_car_serial", "cmd_vel_serial.launch.py",
            f"device:={args.serial_device}", f"baud_rate:={args.serial_baud_rate}",
            f"topic:={CMD_TOPIC}",
        ],
        "localization": [
            ros2, "launch", "fused_odometry", "odometry_bringup.launch.py",
            f"serial_no:=_{args.camera_serial.lstrip('_')}", "use_wheel:=true",
            "use_imu:=true", "use_slam_imu:=false", "equalize:=true",
            "infra_profile:=848x480x15", f"orb_settings_file:={settings}",
        ],
    }


@dataclass
class Sample:
    value: object
    received: float
    header_time: float = None


def health_errors(samples, now, ros_now, freshness):
    errors = []
    for key in ("connected", "healthy", "fusion", "fused", "local"):
        sample = samples.get(key)
        if sample is None:
            errors.append(f"等待 {DATA_TOPICS[key]}")
        elif now - sample.received > freshness:
            errors.append(f"{DATA_TOPICS[key]} 超时")
        elif key in ("connected", "healthy") and sample.value is not True:
            errors.append(f"{DATA_TOPICS[key]} = false（检查下位机导航模式、急停和 CTL）")
        elif key == "fusion" and sample.value not in ALLOWED_MODES:
            errors.append(f"融合状态 {sample.value}")
        elif key in ("fused", "local"):
            if sample.header_time is None or not -0.2 <= ros_now - sample.header_time <= freshness:
                errors.append(f"{DATA_TOPICS[key]} 测量时间过期或异常")
    return errors


class RosMonitor:
    """Use this node's live graph and callbacks, without a ROS CLI daemon cache."""

    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import Twist
        from test_cmd_vel_nav import make_twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Bool, Int32, String

        self.rclpy = rclpy
        self.Twist = Twist
        self.make_twist = make_twist
        self.String = String
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.name = f"odometry_test_{os.getpid()}"
        self.node = rclpy.create_node(self.name)
        self.samples = {}
        self.command_publisher = None
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for key, topic in DATA_TOPICS.items():
            msg_type = Bool if key in ("connected", "healthy") else String if key == "fusion" else Imu if key.endswith("imu") else Odometry
            qos = latched if key in ("connected", "healthy", "fusion") else qos_profile_sensor_data
            self.node.create_subscription(msg_type, topic, lambda msg, k=key: self.receive(k, msg), qos)
        self.node.create_subscription(Int32, "/tracking_state", lambda msg: self.receive("tracking", msg), 10)
        self.event_publisher = self.node.create_publisher(String, EVENT_TOPIC, 10)

    def receive(self, key, msg):
        header_time = None
        if hasattr(msg, "header"):
            stamp = msg.header.stamp
            header_time = stamp.sec + stamp.nanosec * 1e-9
        if key in ("connected", "healthy", "fusion", "tracking"):
            value = msg.data
        elif key in ("fused", "local", "wheel", "visual"):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
                return
            if not 0.9 <= math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) <= 1.1:
                return
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            value = {"x": p.x, "y": p.y, "yaw": yaw}
        else:
            value = {"wx": msg.angular_velocity.x, "wy": msg.angular_velocity.y, "wz": msg.angular_velocity.z}
        self.samples[key] = Sample(value, time.monotonic(), header_time)

    def spin(self, timeout=0.05):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=max(0, deadline-time.monotonic()))

    def health_errors(self, freshness):
        return health_errors(self.samples, time.monotonic(), self.node.get_clock().now().nanoseconds * 1e-9, freshness)

    def preflight(self):
        collisions = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            fqn = namespace.rstrip("/") + "/" + name
            if fqn in STACK_NODES:
                collisions.append(fqn)
        for topic in (CMD_TOPIC, *DATA_TOPICS.values()):
            if self.node.get_publishers_info_by_topic(topic):
                collisions.append(topic)
        if collisions:
            raise RuntimeError("已有导航/定位/串口进程或发布者，请先退出旧程序: " + ", ".join(sorted(set(collisions))))

    def arm(self):
        self.command_publisher = self.node.create_publisher(self.Twist, CMD_TOPIC, 10)

    def graph_errors(self):
        errors = []
        endpoints = self.node.get_publishers_info_by_topic(CMD_TOPIC)
        own = [e for e in endpoints if e.node_name == self.name and e.node_namespace == "/" and e.topic_type == "geometry_msgs/msg/Twist"]
        if len(endpoints) != 1 or len(own) != 1:
            errors.append(f"{CMD_TOPIC} 必须只有本测试的一个速度发布者")
        counts = Counter(ns.rstrip("/") + "/" + name for name, ns in self.node.get_node_names_and_namespaces())
        for name, count in counts.items():
            if count > 1 and (name in STACK_NODES or name == "/" + self.name):
                errors.append(f"重复节点 {name}")
        for key in ("connected", "healthy", "fusion", "fused", "local"):
            if len(self.node.get_publishers_info_by_topic(DATA_TOPICS[key])) > 1:
                errors.append(f"{DATA_TOPICS[key]} 存在多个数据源")
        return errors

    def recorder_ready(self, bag):
        return bool(list(bag.glob("*.db3"))) and any(
            e.node_name.startswith("rosbag2_recorder")
            for e in self.node.get_subscriptions_info_by_topic(EVENT_TOPIC)
        )

    def publish_velocity(self, speed):
        if self.command_publisher is not None:
            self.command_publisher.publish(self.make_twist(speed, 0.0))

    def event(self, value):
        self.event_publisher.publish(self.String(data=json.dumps(value, ensure_ascii=False)))

    def snapshot(self):
        return {key: {"value": sample.value, "header_time": sample.header_time, "age_s": time.monotonic()-sample.received} for key, sample in self.samples.items()}

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def group_members(group_id):
    members = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            if fields[0] != "Z" and int(fields[2]) == group_id:
                members.append(int(entry.name))
        except (OSError, ValueError, IndexError):
            continue
    return members


class Child:
    def __init__(self, name, command, session):
        self.name = name
        self.path = session / f"{name}.log"
        self.stream = self.path.open("w")
        try:
            self.process = subprocess.Popen(
                command, cwd=WORKSPACE, stdin=subprocess.DEVNULL,
                stdout=self.stream, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            self.stream.close()
            raise

    def stop(self, timeout):
        # Each child owns a dedicated process group. Do not signal unrelated ROS.
        try:
            for sig, seconds in ((signal.SIGINT, timeout), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
                self.process.poll()
                if not group_members(self.process.pid):
                    break
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    self.process.poll()
                    if not group_members(self.process.pid):
                        break
                    time.sleep(0.05)
                else:
                    continue
                break
            remaining = group_members(self.process.pid)
            if remaining:
                raise RuntimeError(f"{self.name} 残留进程 {remaining}；查看 {self.path}")
            self.process.wait(timeout=1)
        finally:
            self.stream.close()


class TestRunner:
    def __init__(self, args):
        self.args = args
        self.stopping = False
        self.signal_name = None
        self.phase = "STARTING"
        self.children = {}
        self.monitor = None
        self.session = None
        self.lock = None
        self.state = None
        self.reason = "startup_failed"
        self.previous_handlers = {}

    def signal_stop(self, signum, _frame):
        self.stopping = True
        self.signal_name = signal.Signals(signum).name

    def set_phase(self, phase):
        self.phase = phase
        self.state.update(phase=phase, updated_at=utc_now())
        atomic_json(runtime_dir() / "active.json", self.state)
        atomic_json(self.session / "run.json", self.state)
        log(phase)

    def event(self, kind, **extra):
        value = {"event": kind, "time": utc_now(), "phase": self.phase, **extra}
        if self.monitor:
            value["samples"] = self.monitor.snapshot()
            self.monitor.event(value)
        with (self.session / "events.jsonl").open("a") as stream:
            stream.write(json.dumps(value, ensure_ascii=False) + "\n")

    def prerequisites(self):
        command = os.environ.get("ODOMETRY_TEST_ROS2_COMMAND", "ros2")
        if not shutil.which(command):
            raise RuntimeError("找不到 ros2；请通过 start_odometry_test.sh 启动。")
        if self.args.serial_device == "auto":
            identities = list(Path("/dev/serial/by-id").glob("*"))
            if len(identities) != 1:
                raise RuntimeError(f"串口 auto 需要恰好一个 /dev/serial/by-id 设备，当前 {len(identities)} 个；可用 --serial-device 指定。")
        elif not Path(self.args.serial_device).is_char_device():
            raise RuntimeError(f"串口设备不存在或不是字符设备: {self.args.serial_device}")
        settings = WORKSPACE / "src/orb_slam3/orbslam3_ros2/config/stereo-inertial/RealSense_D455_rk3588.yaml"
        if not settings.is_file():
            raise RuntimeError(f"相机标定不存在: {settings}")

    def acquire(self):
        folder = runtime_dir()
        folder.mkdir(mode=0o700, exist_ok=True)
        self.lock = (folder / "run.lock").open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("本仓库已有测试运行；使用 stop/status。")
        self.args.output_root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.session = self.args.output_root / f"{stamp}_{os.getpid()}"
        self.session.mkdir()
        self.state = {
            "pid": os.getpid(), "process_start": process_identity(os.getpid()),
            "workspace": str(WORKSPACE), "session": str(self.session),
            "speed": self.args.speed, "distance": self.args.distance,
            "max_duration_s": self.args.duration, "domain_id": self.args.domain_id,
            "setup_only": self.args.setup_only, "started_at": utc_now(),
            "distance_reference": "physical floor marks; no odometry-based distance stop",
            "commands": launch_commands(self.args, self.session / "bag"),
        }
        self.set_phase("STARTING")

    def check_children(self):
        for child in self.children.values():
            if child.process.poll() is not None:
                raise RuntimeError(f"{child.name} 已退出（代码 {child.process.returncode}）；查看 {child.path}")

    def tick(self, speed=0.0, require_health=False):
        self.monitor.spin(0.05)
        if self.stopping:
            return
        self.check_children()
        errors = self.monitor.graph_errors()
        if require_health:
            errors += self.monitor.health_errors(self.args.freshness)
        if errors and require_health:
            raise RuntimeError("；".join(errors))
        if errors and speed != 0:
            raise RuntimeError("；".join(errors))
        self.monitor.publish_velocity(speed)

    def wait_ready(self):
        self.set_phase("WAITING_FOR_READY")
        deadline = time.monotonic() + self.args.ready_timeout
        stable_since = None
        next_report = 0
        errors = []
        while not self.stopping and time.monotonic() < deadline:
            self.tick()
            errors = self.monitor.graph_errors() + self.monitor.health_errors(self.args.freshness)
            # These independent diagnostics must appear at least once at startup.
            # During the run visual loss is allowed if the fusion state permits it.
            for key in ("wheel", "control_imu", "camera_imu", "visual"):
                if key not in self.monitor.samples:
                    errors.append(f"专项测试尚未收到 {DATA_TOPICS[key]}")
            if not self.monitor.recorder_ready(self.session / "bag"):
                errors.append("等待录包打开数据库并订阅测试事件")
            if errors:
                stable_since = None
            elif stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= 1.0:
                return
            if time.monotonic() >= next_report:
                log("；".join(errors) if errors else "关键链路已就绪，确认持续健康…")
                next_report = time.monotonic() + 5
        if not self.stopping:
            raise RuntimeError("就绪等待超时：" + "；".join(errors))

    def hold_zero(self, seconds):
        deadline = time.monotonic() + seconds
        while not self.stopping and time.monotonic() < deadline:
            self.tick(require_health=True)

    def run(self):
        result = 0
        os.environ["ROS_DOMAIN_ID"] = str(self.args.domain_id)
        try:
            self.prerequisites()
            self.acquire()
            for sig in (signal.SIGINT, signal.SIGTERM):
                self.previous_handlers[sig] = signal.signal(sig, self.signal_stop)
            log(f"录包及日志: {self.session}")
            log(f"现场标线 {self.args.distance:g} m；速度 {self.args.speed:g} m/s；最长 {self.args.duration:g} s。到线按 Ctrl+C。")
            self.monitor = RosMonitor()
            discovery_deadline = time.monotonic() + 2
            while not self.stopping and time.monotonic() < discovery_deadline:
                self.monitor.spin(0.05)
            if self.stopping:
                self.reason = "operator_stop"
                return 0
            self.monitor.preflight()
            self.monitor.arm()
            for name, command in self.state["commands"].items():
                if self.stopping:
                    break
                log(f"启动 {name}")
                self.children[name] = Child(name, command, self.session)
            self.wait_ready()
            if not self.stopping:
                self.set_phase("STATIONARY_BASELINE")
                self.event("STATIONARY_BEGIN")
                self.hold_zero(self.args.warmup)
            if self.args.setup_only:
                self.set_phase("READY_ZERO_ONLY")
                self.event("SETUP_ONLY_READY")
                log("仅采集模式，保持零速；按 Ctrl+C 结束。")
                while not self.stopping:
                    self.tick(require_health=True)
            elif not self.stopping:
                self.set_phase("COUNTDOWN")
                log(f"{self.args.countdown:g} 秒后开始直行。")
                self.hold_zero(self.args.countdown)
                if not self.stopping:
                    self.set_phase("DRIVING")
                    self.event("STRAIGHT_BEGIN", speed_mps=self.args.speed, target_distance_m=self.args.distance)
                    deadline = time.monotonic() + self.args.duration
                    next_report = time.monotonic() + 5
                    while not self.stopping and time.monotonic() < deadline:
                        self.tick(self.args.speed, require_health=True)
                        if time.monotonic() >= next_report:
                            log(f"直行中，剩余最长 {max(0, deadline-time.monotonic()):.0f} s；到实测标线按 Ctrl+C。")
                            next_report = time.monotonic() + 5
                    self.reason = "operator_stop" if self.stopping else "duration_limit"
            if self.stopping:
                self.reason = "operator_stop"
        except Exception as exc:
            result = 1
            self.reason = str(exc)
            log(f"ERROR: {exc}")
        finally:
            result = self.finish(result)
        return result

    def finish(self, result):
        cleanup_errors = []

        def attempt(label, operation):
            try:
                operation()
            except Exception as exc:
                cleanup_errors.append(f"{label}: {exc}")
                log(f"清理时发生错误，继续关闭其他进程: {label}: {exc}")

        def send_stop():
            # Allow the 20 Hz serial timer real time to forward zero velocity.
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                self.monitor.publish_velocity(0.0)
                self.monitor.spin(0.05)

        if self.monitor is not None:
            attempt("发送零速", send_stop)
        if self.session is not None and self.state is not None:
            attempt("保存停止事件", lambda: self.event(
                "STRAIGHT_END" if self.phase == "DRIVING" else "SESSION_END",
                reason=self.reason, signal=self.signal_name,
            ))
            attempt("更新停止状态", lambda: self.set_phase("STOPPING"))
            if self.monitor is not None:
                attempt("发送录包事件", lambda: self.monitor.spin(0.1))
        for name in ("serial", "localization", "recorder"):
            child = self.children.get(name)
            if child:
                log(f"关闭 {name}，保存数据…")
                attempt(f"关闭 {name}", lambda c=child, n=name: c.stop(30 if n == "recorder" else 8))
        if self.monitor is not None:
            attempt("关闭监视器", self.monitor.close)
        for sig, handler in self.previous_handlers.items():
            attempt("恢复信号处理", lambda s=sig, h=handler: signal.signal(s, h))
        if self.session is not None and self.state is not None:
            metadata = self.session / "bag" / "metadata.yaml"
            if "recorder" in self.children and not metadata.is_file():
                cleanup_errors.append("录包未生成 metadata.yaml；检查 recorder.log")
            self.state.update(
                completed_at=utc_now(), exit_code=1 if cleanup_errors else result,
                stop_reason=self.reason, cleanup_errors=cleanup_errors,
                bag_finalized=metadata.is_file(),
                physical_measurement={"distance_m": None, "lateral_cm": None, "side": None, "yaw_deg": None, "yaw_direction": None},
            )
            attempt("保存最终状态", lambda: self.set_phase("FINISHED" if result == 0 and not cleanup_errors else "FAILED"))
            log(f"录包: {self.session / 'bag'}")
            for error in cleanup_errors:
                log(error)
            log(f"请将实测距离、左右偏移和车头角度填写到 {self.session / 'run.json'} 的 physical_measurement。")
        if self.lock is not None:
            attempt("释放测试锁", self.lock.close)
        return 1 if cleanup_errors else result


def main(argv=None):
    args = parse_arguments(argv)
    if args.action in ("stop", "status"):
        return control_action(args.action)
    if args.dry_run:
        log(f"ROS_DOMAIN_ID={args.domain_id}; v={args.speed:g} m/s; 地面距离={args.distance:g} m; 最长={args.duration:g} s")
        log(f"静止采样={args.warmup:g} s; 倒计时={args.countdown:g} s; setup_only={args.setup_only}")
        for name, command in launch_commands(args, args.output_root / "<session>" / "bag").items():
            log(f"{name}: {shlex.join(command)}")
        return 0
    return TestRunner(args).run()


if __name__ == "__main__":
    sys.exit(main())
