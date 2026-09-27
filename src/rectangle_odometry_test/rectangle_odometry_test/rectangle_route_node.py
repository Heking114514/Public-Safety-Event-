"""Publish and monitor a closed clockwise rectangular navigation test."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String, UInt64

from .geometry import (
    Pose2D,
    RectangleWaypoint,
    build_clockwise_rectangle,
    build_clockwise_rounded_rectangle,
    wrap_angle,
)


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0
        - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def quaternion_from_yaw(yaw: float):
    quaternion = Quaternion()
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)
    return quaternion


class RectangleRouteNode(Node):
    def __init__(self) -> None:
        super().__init__("rectangle_odometry_test")

        self.odom_topic = self.declare_parameter(
            "odom_topic", "/odometry/fused"
        ).value
        self.route_input_topic = self.declare_parameter(
            "route_input_topic", "/waypoint_navigation/route_input"
        ).value
        self.route_ack_topic = self.declare_parameter(
            "route_ack_topic", "/waypoint_navigation/route_ack"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/waypoint_navigation/status"
        ).value
        self.route_frame = self.declare_parameter("route_frame", "map").value
        self.child_frame = self.declare_parameter("child_frame", "base_link").value
        self.side_length_m = float(
            self.declare_parameter("side_length_m", 1.8).value
        )
        self.corner_mode = str(
            self.declare_parameter("corner_mode", "pivot").value
        ).strip().lower()
        self.corner_radius_m = float(
            self.declare_parameter("corner_radius_m", 0.25).value
        )
        self.corner_samples = int(
            self.declare_parameter("corner_samples", 16).value
        )
        self.straight_step_m = float(
            self.declare_parameter("straight_step_m", 0.05).value
        )
        self.startup_delay_s = float(
            self.declare_parameter("startup_delay_s", 1.0).value
        )
        self.publish_retry_period_s = float(
            self.declare_parameter("publish_retry_period_s", 0.5).value
        )
        self.anchor_to_current_pose = bool(
            self.declare_parameter("anchor_to_current_pose", True).value
        )
        self.start_x = float(self.declare_parameter("start_x", 0.0).value)
        self.start_y = float(self.declare_parameter("start_y", 0.0).value)
        self.start_yaw = float(self.declare_parameter("start_yaw", 0.0).value)

        if not math.isfinite(self.side_length_m) or self.side_length_m <= 0.0:
            raise ValueError("side_length_m must be finite and positive")
        if self.corner_mode not in {"pivot", "arc"}:
            raise ValueError("corner_mode must be 'pivot' or 'arc'")
        if not math.isfinite(self.corner_radius_m) or self.corner_radius_m <= 0.0:
            raise ValueError("corner_radius_m must be finite and positive")
        if self.corner_radius_m * 2.0 >= self.side_length_m:
            raise ValueError("corner_radius_m must be less than half side_length_m")
        if self.corner_samples < 2:
            raise ValueError("corner_samples must be at least 2")
        if not math.isfinite(self.straight_step_m) or self.straight_step_m <= 0.0:
            raise ValueError("straight_step_m must be finite and positive")
        if self.startup_delay_s < 0.0 or self.publish_retry_period_s <= 0.0:
            raise ValueError("startup_delay_s and publish_retry_period_s are invalid")

        reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient_reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.route_publisher = self.create_publisher(
            Path, self.route_input_topic, reliable
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self._on_odometry, 10
        )
        self.ack_subscription = self.create_subscription(
            UInt64, self.route_ack_topic, self._on_route_ack, transient_reliable
        )
        self.status_subscription = self.create_subscription(
            String, self.status_topic, self._on_status, transient_reliable
        )
        self.timer = self.create_timer(0.05, self._tick)

        self.latest_odom: Optional[Pose2D] = None
        self.origin: Optional[Pose2D] = None
        self.route: Optional[Path] = None
        self.route_id = 0
        self.route_published_at: Optional[float] = None
        self.last_publish_at = -math.inf
        self.route_accepted = False
        self.completed_reported = False
        self.last_status = ""
        self.started_at = self.get_clock().now()

        self.get_logger().warn(
            f"Launching this node will request a real vehicle motion test: "
            f"{self.side_length_m:.2f} m clockwise rectangle "
            f"(corner_mode={self.corner_mode}) using the active waypoint "
            f"navigator speed/tolerance"
        )
        if self.corner_mode == "arc":
            self.get_logger().warn(
                f"Rounded-corner mode: radius={self.corner_radius_m:.2f} m, "
                f"samples_per_corner={self.corner_samples}, "
                f"straight_step={self.straight_step_m:.2f} m; the car will "
                f"keep moving through corners instead of pivoting"
            )
        self.get_logger().info(
            f"Waiting for {self.odom_topic}; route is anchored to current "
            f"pose={'true' if self.anchor_to_current_pose else 'false'}"
        )

    def _on_odometry(self, message: Odometry) -> None:
        if self.route_frame and message.header.frame_id != self.route_frame:
            return
        if self.child_frame and message.child_frame_id != self.child_frame:
            return
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            return
        pose = Pose2D(
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            yaw_from_quaternion(message.pose.pose.orientation),
        )
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            return
        self.latest_odom = pose

    def _on_route_ack(self, message: UInt64) -> None:
        if self.route_id and message.data == self.route_id:
            if not self.route_accepted:
                self.route_accepted = True
                self.get_logger().info(
                    "Rectangle route accepted by waypoint navigator "
                    f"(route_id={self.route_id})"
                )

    def _on_status(self, message: String) -> None:
        if message.data != self.last_status:
            self.last_status = message.data
            self.get_logger().info(f"Navigator status: {message.data}")
        if message.data == "GOAL_REACHED" and not self.completed_reported:
            self.completed_reported = True
            self._report_closure_error()

    def _tick(self) -> None:
        if self.route is None:
            elapsed = (self.get_clock().now() - self.started_at).nanoseconds * 1.0e-9
            if self.latest_odom is None or elapsed < self.startup_delay_s:
                return
            self.origin = self.latest_odom if self.anchor_to_current_pose else Pose2D(
                self.start_x, self.start_y, wrap_angle(self.start_yaw)
            )
            self.route = self._make_route(self.origin)
            self.get_logger().info(
                f"Captured rectangle origin: x={self.origin.x:.3f} "
                f"y={self.origin.y:.3f} yaw={math.degrees(self.origin.yaw):.1f} deg"
            )

        if self.route_accepted:
            return
        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1.0e-9
        if self.route_id == 0:
            self.route.header.stamp = now.to_msg()
            self.route_id = now.nanoseconds
            for pose in self.route.poses:
                pose.header = self.route.header
            self.route_published_at = now_seconds
        if now_seconds - self.last_publish_at >= self.publish_retry_period_s:
            self.route_publisher.publish(self.route)
            self.last_publish_at = now_seconds
            self.get_logger().info(
                f"Published clockwise rectangle route with {len(self.route.poses)} targets"
            )

    def _make_route(self, origin: Pose2D) -> Path:
        if self.corner_mode == "arc":
            waypoints = build_clockwise_rounded_rectangle(
                origin,
                self.side_length_m,
                self.corner_radius_m,
                self.corner_samples,
                self.straight_step_m,
            )
        else:
            waypoints = build_clockwise_rectangle(origin, self.side_length_m)
        path = Path()
        path.header.frame_id = self.route_frame
        path.poses = [
            self._make_pose(waypoint) for waypoint in waypoints
        ]
        return path

    def _make_pose(self, waypoint: RectangleWaypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.pose.position.x = waypoint.pose.x
        pose.pose.position.y = waypoint.pose.y
        pose.pose.position.z = 1.0e-3 if waypoint.turn_junction else 0.0
        pose.pose.orientation = quaternion_from_yaw(waypoint.pose.yaw)
        return pose

    def _report_closure_error(self) -> None:
        if self.origin is None or self.latest_odom is None:
            self.get_logger().warn("Rectangle completed without a final odometry sample")
            return
        dx = self.latest_odom.x - self.origin.x
        dy = self.latest_odom.y - self.origin.y
        dyaw = wrap_angle(self.latest_odom.yaw - self.origin.yaw)
        self.get_logger().info(
            f"Rectangle closure: dx={dx:.3f} m dy={dy:.3f} m "
            f"position_error={math.hypot(dx, dy):.3f} m "
            f"dyaw={math.degrees(dyaw):.2f} deg"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RectangleRouteNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
