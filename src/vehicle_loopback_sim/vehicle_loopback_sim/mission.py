"""Start a real planner request for the loopback stack."""

from __future__ import annotations

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from arena_path_planner.srv import PlanArenaPath
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .arena_layout import load_root, load_start, transform_from_config


def _quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(0.5 * yaw_rad)
    quaternion.w = math.cos(0.5 * yaw_rad)
    return quaternion


def _yaw_from_quaternion(quaternion: Quaternion) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _parse_modes(text: str) -> list[str]:
    modes = [mode.strip() for mode in text.split(",") if mode.strip()]
    if not modes:
        raise ValueError("mission mode list must not be empty")
    allowed = {"layer1", "layer2", "layer3", "shortest", "numbered"}
    invalid = [mode for mode in modes if mode not in allowed]
    if invalid:
        raise ValueError(f"unsupported mission mode(s): {', '.join(invalid)}")
    return modes


class LoopbackMissionStarter(Node):
    """Run planner stages and activate each returned route."""

    def __init__(self) -> None:
        super().__init__("vehicle_loopback_mission")
        self._declare_parameters()
        self._arena_config = str(self.get_parameter("arena_config").value)
        self._root = load_root(self._arena_config)
        self._start = load_start(self._root)
        self._transform = transform_from_config(self._arena_config)
        self._modes = _parse_modes(str(self.get_parameter("mode").value))
        self._client = self.create_client(
            PlanArenaPath,
            str(self.get_parameter("planner_service").value),
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self._navigator_status_subscription = self.create_subscription(
            String,
            str(self.get_parameter("navigator_status_topic").value),
            self._navigator_status_callback,
            10,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            10,
        )
        self._sent = False
        self._stage_index = 0
        self._planning = False
        self._waiting_for_goal = False
        self._completed_stages = 0
        self._latest_odom: Odometry | None = None
        self._covered_edges: list[str] = []
        self._covered_intervals = []
        self._active_planned_intervals = []
        self._active_covered_edges: list[str] = []
        delay_s = max(0.0, float(self.get_parameter("request_delay_s").value))
        self._timer = self.create_timer(delay_s if delay_s > 0.0 else 0.1, self._tick)
        self._publish_status("waiting_for_planner")

    def _declare_parameters(self) -> None:
        default_config = (
            Path(get_package_share_directory("arena_path_planner"))
            / "config"
            / "arena_map.yaml"
        )
        declarations = {
            "arena_config": str(default_config),
            "planner_service": "/arena_path_planner/plan",
            "mode": "layer1,layer2,layer3",
            "activate_navigation": True,
            "request_delay_s": 1.0,
            "status_topic": "/vehicle_loopback_sim/mission_status",
            "navigator_status_topic": "/waypoint_navigation/status",
            "odom_topic": "/odometry/fused",
        }
        self.declare_parameters("", list(declarations.items()))

    def _tick(self) -> None:
        if self._sent:
            self._timer.cancel()
            return
        if not self._client.service_is_ready():
            self._client.wait_for_service(timeout_sec=0.0)
            self._publish_status("waiting_for_planner")
            return
        self._sent = True
        self._timer.cancel()
        self._request_current_stage()

    def _request_current_stage(self) -> None:
        if self._stage_index >= len(self._modes):
            self._publish_status(f"mission_complete:stages={self._completed_stages}")
            self.get_logger().info(
                f"loopback mission complete after {self._completed_stages} stage(s)"
            )
            return
        try:
            request = self._make_request()
        except Exception as error:
            self._publish_status(f"request_error:{error}")
            self.get_logger().error(f"failed to build loopback plan request: {error}")
            return
        self._planning = True
        mode = self._modes[self._stage_index]
        self._publish_status(f"planning:{mode}")
        future = self._client.call_async(request)
        future.add_done_callback(self._handle_response)
        self.get_logger().info(
            f"requested loopback {mode} plan from arena_map.yaml with activation enabled"
        )

    def _make_request(self) -> PlanArenaPath.Request:
        request = PlanArenaPath.Request()
        request.start = Pose()
        if self._stage_index == 0 or self._latest_odom is None:
            request.start.position.x = self._start.x_m
            request.start.position.y = self._start.y_m
            request.start.orientation = _quaternion_from_yaw(self._start.yaw_rad)
        else:
            odom_pose = self._latest_odom.pose.pose
            point = self._transform.point_from_map(
                float(odom_pose.position.x),
                float(odom_pose.position.y),
            )
            request.start.position.x = point["x_m"]
            request.start.position.y = point["y_m"]
            request.start.orientation = _quaternion_from_yaw(
                self._transform.yaw_from_map(
                    _yaw_from_quaternion(odom_pose.orientation)
                )
            )
        request.mode = self._modes[self._stage_index]
        request.covered_edges = list(self._covered_edges)
        request.covered_intervals = list(self._covered_intervals)
        request.activate_navigation = bool(
            self.get_parameter("activate_navigation").value
        )
        return request

    def _handle_response(self, future) -> None:
        self._planning = False
        mode = self._modes[self._stage_index]
        try:
            response = future.result()
        except Exception as error:
            self._publish_status(f"service_error:{error}")
            self.get_logger().error(f"loopback planner request failed: {error}")
            return
        if not response.success:
            self._publish_status(f"plan_failed:{response.message}")
            self.get_logger().error(f"loopback planner failed: {response.message}")
            return
        if not response.navigation_path.poses:
            self._covered_edges = list(response.covered_edges)
            self._covered_intervals = list(response.covered_intervals)
            self._completed_stages += 1
            self._stage_index += 1
            self._publish_status(f"stage_empty:{mode}")
            self._request_current_stage()
            return
        state = "activated" if response.navigation_activated else "planned_only"
        self._publish_status(
            f"{state}:{mode}:poses={len(response.navigation_path.poses)} "
            f"length={response.length:.2f}"
        )
        self.get_logger().info(
            f"loopback {mode} plan {state}: "
            f"{len(response.navigation_path.poses)} poses, "
            f"{response.length:.2f} m, {response.message}"
        )
        if not response.navigation_activated:
            return
        self._active_planned_intervals = list(response.planned_intervals)
        self._active_covered_edges = list(response.covered_edges)
        self._waiting_for_goal = True

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_publisher.publish(message)

    def _odom_callback(self, message: Odometry) -> None:
        self._latest_odom = message

    def _navigator_status_callback(self, message: String) -> None:
        if message.data != "GOAL_REACHED" or not self._waiting_for_goal:
            return
        mode = self._modes[self._stage_index]
        self._covered_edges = list(self._active_covered_edges)
        self._covered_intervals.extend(self._active_planned_intervals)
        self._active_planned_intervals = []
        self._active_covered_edges = []
        self._waiting_for_goal = False
        self._completed_stages += 1
        self._stage_index += 1
        self._publish_status(f"stage_complete:{mode}")
        self.get_logger().info(f"loopback {mode} stage reached GOAL_REACHED")
        if self._planning:
            return
        self._request_current_stage()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoopbackMissionStarter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
