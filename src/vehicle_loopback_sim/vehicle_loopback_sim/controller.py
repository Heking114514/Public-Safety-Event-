"""Simple route controller for loopback planner tests."""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String

from .route_control import (
    Pose2D,
    Projection,
    Waypoint,
    clamp,
    normalize_angle,
    parse_path,
    project,
    target_distance,
    yaw_from_quaternion,
)


class LoopbackRouteController(Node):
    """Execute planner routes with a small, explicit state machine."""

    def __init__(self) -> None:
        super().__init__("loopback_route_controller")
        self._declare_parameters()

        self._pose: Pose2D | None = None
        self._route: list[Waypoint] = []
        self._route_message = Path()
        self._index = 0
        self._active = False
        self._phase = "WAITING_FOR_ROUTE"
        self._last_status = ""

        durable_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._handle_odom,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("route_input_topic").value),
            self._handle_route,
            10,
        )
        self._cmd_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self._path_publisher = self.create_publisher(
            Path,
            str(self.get_parameter("path_topic").value),
            durable_qos,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            durable_qos,
        )
        self._waypoint_publisher = self.create_publisher(
            Int32,
            str(self.get_parameter("current_waypoint_topic").value),
            durable_qos,
        )
        rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._control)
        self._set_status("LOOPBACK_WAITING_FOR_ROUTE")
        self.get_logger().info(
            "loopback route controller ready: "
            f"route={self.get_parameter('route_input_topic').value} "
            f"odom={self.get_parameter('odom_topic').value} "
            f"cmd={self.get_parameter('cmd_vel_topic').value}"
        )

    def _declare_parameters(self) -> None:
        declarations = {
            "route_input_topic": "/waypoint_navigation/route_input",
            "odom_topic": "/odometry/fused",
            "cmd_vel_topic": "/cmd_vel_nav",
            "path_topic": "/waypoint_path",
            "status_topic": "/waypoint_navigation/status",
            "current_waypoint_topic": "/waypoint_navigation/current_waypoint",
            "control_rate_hz": 30.0,
            "cruise_speed_mps": 0.12,
            "minimum_drive_speed_mps": 0.035,
            "slowdown_distance_m": 0.10,
            "stop_tolerance_m": 0.018,
            "pass_tolerance_m": 0.012,
            "lateral_tolerance_m": 0.050,
            "turn_heading_threshold_rad": 0.35,
            "turn_yaw_tolerance_rad": 0.055,
            "heading_gain": 1.7,
            "cross_track_gain": 1.8,
            "stanley_softening_speed_mps": 0.12,
            "turn_gain": 1.8,
            "min_turn_speed_rps": 0.24,
            "max_turn_speed_rps": 0.70,
        }
        self.declare_parameters("", list(declarations.items()))

    def _handle_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self._pose = Pose2D(
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    def _handle_route(self, message: Path) -> None:
        route = parse_path(message)
        if len(route) < 2:
            self._active = False
            self._route = []
            self._publish_stop()
            self._set_status("LOOPBACK_ROUTE_REJECTED")
            self.get_logger().warn("ignoring route with fewer than two valid poses")
            return
        self._route = route
        self._route_message = message
        self._index = 0
        self._active = True
        self._phase = "FOLLOW"
        self._path_publisher.publish(self._route_message)
        self._publish_waypoint()
        self._set_status("LOOPBACK_ROUTE_LOADED")
        self.get_logger().info(f"loaded loopback route with {len(route)} poses")

    def _control(self) -> None:
        if self._pose is None:
            self._publish_stop()
            self._set_status("LOOPBACK_WAITING_FOR_ODOM")
            return
        if not self._active or not self._route:
            self._publish_stop()
            self._set_status("LOOPBACK_WAITING_FOR_ROUTE")
            return

        self._skip_satisfied_start()
        if self._index >= len(self._route):
            self._complete()
            return

        if self._phase == "ALIGN":
            self._run_align()
            return
        self._run_follow()

    def _run_follow(self) -> None:
        assert self._pose is not None
        target = self._route[self._index]
        start = self._segment_start()
        projection = project(start, target, self._pose)
        final = self._index + 1 >= len(self._route)
        stop_required = final or self._heading_change_requires_stop(projection.heading)
        lateral_ok = abs(projection.cross_track) <= self._param("lateral_tolerance_m")

        if self._target_reached(projection, stop_required, lateral_ok):
            if final:
                self._complete()
            elif stop_required:
                self._phase = "ALIGN"
                self._publish_stop()
                self._set_status("LOOPBACK_ALIGNING")
            else:
                self._advance()
            return

        heading_error = normalize_angle(projection.heading - self._pose.yaw)
        cruise = self._param("cruise_speed_mps")
        speed = min(cruise, max(0.0, target_distance(self._pose, target) * 1.2))
        if stop_required:
            speed = min(speed, self._stop_approach_speed(projection.remaining))
        speed = max(0.0, min(cruise, speed))
        if speed > 0.0:
            speed = max(self._param("minimum_drive_speed_mps"), speed)
        correction = math.atan2(
            self._param("cross_track_gain") * projection.cross_track,
            abs(speed) + self._param("stanley_softening_speed_mps"),
        )
        angular = self._param("heading_gain") * normalize_angle(
            heading_error - correction
        )
        command = Twist()
        command.linear.x = speed * max(0.25, math.cos(heading_error))
        command.angular.z = clamp(
            angular,
            -self._param("max_turn_speed_rps"),
            self._param("max_turn_speed_rps"),
        )
        self._cmd_publisher.publish(command)
        self._set_status(
            "LOOPBACK_APPROACHING_TURN" if stop_required else "LOOPBACK_FOLLOWING"
        )

    def _run_align(self) -> None:
        assert self._pose is not None
        desired = self._next_heading()
        if desired is None:
            self._complete()
            return
        error = normalize_angle(desired - self._pose.yaw)
        if abs(error) <= self._param("turn_yaw_tolerance_rad"):
            self._publish_stop()
            self._advance()
            self._phase = "FOLLOW"
            self._set_status("LOOPBACK_PATH_ALIGNED")
            return
        angular = self._param("turn_gain") * error
        angular = clamp(
            angular,
            -self._param("max_turn_speed_rps"),
            self._param("max_turn_speed_rps"),
        )
        if abs(angular) < self._param("min_turn_speed_rps"):
            angular = math.copysign(self._param("min_turn_speed_rps"), error)
        command = Twist()
        command.angular.z = angular
        self._cmd_publisher.publish(command)
        self._set_status("LOOPBACK_ALIGNING")

    def _target_reached(
        self,
        projection: Projection,
        stop_required: bool,
        lateral_ok: bool,
    ) -> bool:
        if not projection.valid:
            return target_distance(self._pose, self._route[self._index]) <= self._param(
                "stop_tolerance_m"
            )
        tolerance = self._param("stop_tolerance_m" if stop_required else "pass_tolerance_m")
        return projection.remaining <= tolerance and lateral_ok

    def _heading_change_requires_stop(self, heading: float) -> bool:
        desired = self._next_heading()
        if desired is None:
            return True
        return abs(normalize_angle(desired - heading)) >= self._param(
            "turn_heading_threshold_rad"
        )

    def _next_heading(self) -> float | None:
        if self._index + 1 >= len(self._route):
            return None
        current = self._route[self._index]
        next_target = self._route[self._index + 1]
        if target_distance(current, next_target) <= 1.0e-6:
            return next_target.yaw
        return math.atan2(next_target.y - current.y, next_target.x - current.x)

    def _segment_start(self) -> Waypoint | Pose2D:
        if self._index <= 0:
            return self._pose if self._pose is not None else self._route[0]
        return self._route[self._index - 1]

    def _stop_approach_speed(self, remaining: float) -> float:
        distance = max(0.0, remaining)
        slowdown = max(1.0e-6, self._param("slowdown_distance_m"))
        ratio = min(1.0, distance / slowdown)
        return self._param("cruise_speed_mps") * ratio

    def _skip_satisfied_start(self) -> None:
        assert self._pose is not None
        while self._index < len(self._route):
            if target_distance(self._pose, self._route[self._index]) > self._param(
                "stop_tolerance_m"
            ):
                return
            self._advance()

    def _advance(self) -> None:
        self._index += 1
        self._publish_waypoint()

    def _complete(self) -> None:
        self._active = False
        self._publish_stop()
        self._set_status("LOOPBACK_GOAL_REACHED")

    def _publish_stop(self) -> None:
        self._cmd_publisher.publish(Twist())

    def _publish_waypoint(self) -> None:
        message = Int32()
        message.data = int(self._index)
        self._waypoint_publisher.publish(message)

    def _set_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        message = String()
        message.data = status
        self._status_publisher.publish(message)
        self.get_logger().info(f"controller state: {status}")

    def _param(self, name: str) -> float:
        return float(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoopbackRouteController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
