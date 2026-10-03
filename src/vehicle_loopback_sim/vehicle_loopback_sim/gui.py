"""Browser visualization for the loopback navigation stack."""

from __future__ import annotations

import copy
import json
import math
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import socket
import threading
from typing import Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from mission_control_interfaces.msg import ControlTelemetry
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String
from vehicle_config import load_vehicle_physical_config

from .arena_layout import arena_pose_to_map, layout_from_config, transform_from_config


def _yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class GuiState:
    """Thread-safe state store for Server-Sent Events clients."""

    def __init__(self, initial_state: dict[str, Any]) -> None:
        self._condition = threading.Condition()
        self._state = copy.deepcopy(initial_state)
        self._field_seq = {key: 1 for key in initial_state}
        self._seq = 1

    def update(self, updates: dict[str, Any]) -> None:
        if not updates:
            return
        with self._condition:
            self._seq += 1
            for key, value in updates.items():
                self._state[key] = copy.deepcopy(value)
                self._field_seq[key] = self._seq
            self._condition.notify_all()

    def wait_patch(self, last_seq: int, timeout_s: float) -> tuple[int, dict[str, Any]]:
        with self._condition:
            if self._seq <= last_seq:
                self._condition.wait(timeout_s)
            if last_seq <= 0:
                return self._seq, copy.deepcopy(self._state)
            patch = {
                key: copy.deepcopy(value)
                for key, value in self._state.items()
                if self._field_seq.get(key, 0) > last_seq
            }
            return self._seq, patch

    def snapshot(self) -> dict[str, Any]:
        with self._condition:
            return copy.deepcopy(self._state)


class VehicleLoopbackGui(Node):
    """Serve a small live dashboard fed by ROS navigation topics."""

    def __init__(self) -> None:
        super().__init__("vehicle_loopback_gui")
        self._declare_parameters()

        self._vehicle = load_vehicle_physical_config()
        self._arena_transform = None
        self._arena_layout = None
        self._load_arena_config()
        self._state = GuiState(self._initial_state())
        self._pose: dict[str, float] | None = None
        self._twist = {"linear_x_mps": 0.0, "angular_z_rps": 0.0}
        self._cmd = {"linear_x_mps": 0.0, "angular_z_rps": 0.0}
        self._telemetry: dict[str, float | int | bool] = {}
        self._loopback_status = "waiting"
        self._mission_status = "waiting"
        self._navigator_status = "waiting"
        self._current_waypoint = -1
        self._trail: list[dict[str, float]] = []
        self._max_trail_points = max(
            10, int(self.get_parameter("max_trail_points").value)
        )
        self._trail_min_distance_m = max(
            0.0, float(self.get_parameter("trail_min_distance_m").value)
        )

        default_qos = 10
        durable_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._handle_odom,
            default_qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._handle_cmd,
            default_qos,
        )
        self.create_subscription(
            ControlTelemetry,
            str(self.get_parameter("control_telemetry_topic").value),
            self._handle_telemetry,
            default_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("loopback_status_topic").value),
            self._handle_loopback_status,
            default_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_status_topic").value),
            self._handle_mission_status,
            default_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("navigator_status_topic").value),
            self._handle_navigator_status,
            durable_qos,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter("current_waypoint_topic").value),
            self._handle_current_waypoint,
            durable_qos,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter("waypoint_path_topic").value),
            lambda message: self._handle_path("waypoint_path", message),
            durable_qos,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter("navigation_path_topic").value),
            lambda message: self._handle_path("navigation_path", message),
            durable_qos,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter("arena_path_topic").value),
            lambda message: self._handle_path("arena_path", message),
            durable_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._handle_map,
            durable_qos,
        )

        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self._publish_live_state)

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self._server = self._make_server(host, port)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="vehicle_loopback_gui_http",
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(
            f"vehicle loopback GUI: http://{host}:{self._server.server_port}/"
        )

    def destroy_node(self) -> bool:
        if hasattr(self, "_server"):
            self._server.shutdown()
            self._server.server_close()
        return super().destroy_node()

    def _declare_parameters(self) -> None:
        declarations = {
            "host": "127.0.0.1",
            "port": 8765,
            "publish_rate_hz": 12.0,
            "max_trail_points": 1200,
            "trail_min_distance_m": 0.005,
            "odom_topic": "/odometry/fused",
            "cmd_vel_topic": "/cmd_vel_nav",
            "control_telemetry_topic": "/cup_car_serial/control_telemetry",
            "loopback_status_topic": "/vehicle_loopback_sim/status",
            "navigator_status_topic": "/waypoint_navigation/status",
            "current_waypoint_topic": "/waypoint_navigation/current_waypoint",
            "waypoint_path_topic": "/waypoint_path",
            "navigation_path_topic": "/arena_path_planner/navigation_path",
            "arena_path_topic": "/arena_path_planner/arena_path",
            "map_topic": "/arena_path_planner/map",
            "mission_status_topic": "/vehicle_loopback_sim/mission_status",
            "arena_config": self._default_arena_config(),
        }
        self.declare_parameters("", list(declarations.items()))

    def _default_arena_config(self) -> str:
        try:
            return str(
                Path(get_package_share_directory("arena_path_planner"))
                / "config"
                / "arena_map.yaml"
            )
        except Exception:
            return ""

    def _load_arena_config(self) -> None:
        path = str(self.get_parameter("arena_config").value)
        if not path:
            return
        try:
            self._arena_transform = transform_from_config(path)
            self._arena_layout = layout_from_config(path)
        except Exception as error:
            self.get_logger().warn(f"failed to load arena GUI layout: {error}")
            self._arena_transform = None
            self._arena_layout = None

    def _initial_state(self) -> dict[str, Any]:
        return {
            "vehicle": {
                "length_m": self._vehicle.length_m,
                "width_m": self._vehicle.width_m,
                "wheel_track_m": self._vehicle.wheel_track_m,
                "base_link_offset_x_m": self._vehicle.base_link_offset_x_m,
            },
            "pose": None,
            "twist": {"linear_x_mps": 0.0, "angular_z_rps": 0.0},
            "cmd": {"linear_x_mps": 0.0, "angular_z_rps": 0.0},
            "telemetry": {},
            "trail": [],
            "arena_layout": self._arena_layout,
            "loopback_status": "waiting",
            "mission_status": "waiting",
            "navigator_status": "waiting",
            "current_waypoint": -1,
            "waypoint_path": {"frame_id": "", "poses": []},
            "navigation_path": {"frame_id": "", "poses": []},
            "arena_path": {"frame_id": "", "poses": []},
            "map": None,
        }

    def _make_server(self, host: str, port: int) -> ThreadingHTTPServer:
        index_path = (
            Path(get_package_share_directory("vehicle_loopback_sim"))
            / "web"
            / "index.html"
        )
        state = self._state

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path in ("/", "/index.html"):
                    self._serve_file(index_path, "text/html; charset=utf-8")
                    return
                if self.path == "/state":
                    payload = json.dumps(
                        {"state": state.snapshot()},
                        separators=(",", ":"),
                    ).encode("utf-8")
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if self.path == "/events":
                    self._serve_events()
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def log_message(self, fmt: str, *args) -> None:
                return

            def _serve_file(self, path: Path, content_type: str) -> None:
                try:
                    payload = path.read_bytes()
                except OSError:
                    self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def _serve_events(self) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                last_seq = 0
                while True:
                    seq, patch = state.wait_patch(last_seq, 15.0)
                    try:
                        if patch:
                            payload = json.dumps(
                                {"seq": seq, "state": patch},
                                separators=(",", ":"),
                            )
                            self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                            last_seq = seq
                        else:
                            self.wfile.write(b": heartbeat\n\n")
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        return

        class Server(ThreadingHTTPServer):
            allow_reuse_address = True

        server = Server((host, port), Handler)
        server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return server

    def _handle_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self._pose = {
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": _yaw_from_quaternion(pose.orientation),
        }
        twist = message.twist.twist
        self._twist = {
            "linear_x_mps": float(twist.linear.x),
            "angular_z_rps": float(twist.angular.z),
        }
        self._append_trail(self._pose)

    def _handle_cmd(self, message: Twist) -> None:
        self._cmd = {
            "linear_x_mps": float(message.linear.x),
            "angular_z_rps": float(message.angular.z),
        }

    def _handle_telemetry(self, message: ControlTelemetry) -> None:
        self._telemetry = {
            "sequence": int(message.sample_sequence),
            "command_valid": bool(message.command_valid),
            "command_age_ms": int(message.command_age_ms),
            "target_left_mps": float(message.target_left_velocity_mps),
            "target_right_mps": float(message.target_right_velocity_mps),
            "measured_left_mps": float(message.measured_left_velocity_mps),
            "measured_right_mps": float(message.measured_right_velocity_mps),
        }

    def _handle_loopback_status(self, message: String) -> None:
        self._loopback_status = message.data

    def _handle_mission_status(self, message: String) -> None:
        self._mission_status = message.data

    def _handle_navigator_status(self, message: String) -> None:
        self._navigator_status = message.data

    def _handle_current_waypoint(self, message: Int32) -> None:
        self._current_waypoint = int(message.data)

    def _handle_path(self, field: str, message: NavPath) -> None:
        poses = []
        for stamped in message.poses:
            pose = stamped.pose
            x_m = float(pose.position.x)
            y_m = float(pose.position.y)
            yaw_rad = _yaw_from_quaternion(pose.orientation)
            if message.header.frame_id == "arena" and self._arena_transform:
                mapped = arena_pose_to_map(self._arena_transform, x_m, y_m, yaw_rad)
                x_m = mapped["x_m"]
                y_m = mapped["y_m"]
                yaw_rad = mapped["yaw_rad"]
            poses.append(
                {
                    "x_m": x_m,
                    "y_m": y_m,
                    "z_m": float(pose.position.z),
                    "yaw_rad": yaw_rad,
                }
            )
        self._state.update(
            {
                field: {
                    "frame_id": "map" if message.header.frame_id == "arena" else message.header.frame_id,
                    "poses": poses,
                }
            }
        )

    def _handle_map(self, message: OccupancyGrid) -> None:
        origin = message.info.origin.position
        self._state.update(
            {
                "map": {
                    "frame_id": message.header.frame_id,
                    "resolution_m": float(message.info.resolution),
                    "width": int(message.info.width),
                    "height": int(message.info.height),
                    "origin": {
                        "x_m": float(origin.x),
                        "y_m": float(origin.y),
                    },
                    "data": [int(value) for value in message.data],
                }
            }
        )

    def _append_trail(self, pose: dict[str, float]) -> None:
        if self._trail:
            previous = self._trail[-1]
            distance = math.hypot(
                pose["x_m"] - previous["x_m"],
                pose["y_m"] - previous["y_m"],
            )
            if distance < self._trail_min_distance_m:
                return
        self._trail.append(
            {
                "x_m": round(pose["x_m"], 4),
                "y_m": round(pose["y_m"], 4),
            }
        )
        if len(self._trail) > self._max_trail_points:
            del self._trail[: len(self._trail) - self._max_trail_points]

    def _publish_live_state(self) -> None:
        self._state.update(
            {
                "pose": self._pose,
                "twist": self._twist,
                "cmd": self._cmd,
                "telemetry": self._telemetry,
                "trail": self._trail,
                "loopback_status": self._loopback_status,
                "mission_status": self._mission_status,
                "navigator_status": self._navigator_status,
                "current_waypoint": self._current_waypoint,
            }
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleLoopbackGui()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
