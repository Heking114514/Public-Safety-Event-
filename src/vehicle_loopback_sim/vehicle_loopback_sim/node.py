"""ROS 2 loopback simulator for planner/controller testing."""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from mission_control_interfaces.msg import ControlTelemetry
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from vehicle_config import parameters_for

from .motion import Pose2D, Twist2D, integrate_base_link, limited_twist, wheel_speeds


def _yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _quaternion_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion

    quaternion = Quaternion()
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)
    return quaternion


def _finite_parameter(node: Node, name: str) -> float:
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


class VehicleLoopbackSim(Node):
    """Integrate commanded velocity into truth odometry."""

    def __init__(self) -> None:
        super().__init__("vehicle_loopback_sim")
        self._declare_parameters()

        self._map_frame = str(self.get_parameter("map_frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._wheel_track_m = _finite_parameter(self, "wheel_track_m")
        self._command_timeout_s = max(
            0.0, _finite_parameter(self, "command_timeout_s")
        )
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._publish_actuator_health = bool(
            self.get_parameter("publish_actuator_health").value
        )
        self._publish_control_telemetry = bool(
            self.get_parameter("publish_control_telemetry").value
        )

        self._pose = Pose2D(
            _finite_parameter(self, "initial_x_m"),
            _finite_parameter(self, "initial_y_m"),
            _finite_parameter(self, "initial_yaw_rad"),
        )
        self._desired_twist = Twist2D()
        self._applied_twist = Twist2D()
        self._last_command_time = self.get_clock().now()
        self._last_update_time = self.get_clock().now()
        self._last_status_text = ""
        self._telemetry_sequence = 0
        self._started_time = self.get_clock().now()

        self._tf_broadcaster = TransformBroadcaster(self)
        self._cmd_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._cmd_vel_callback,
            10,
        )
        self._initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initial_pose_topic").value),
            self._initial_pose_callback,
            10,
        )
        self._odom_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            10,
        )
        self._imu_publisher = self.create_publisher(
            Imu,
            str(self.get_parameter("imu_topic").value),
            10,
        )
        self._fusion_status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("fusion_status_topic").value),
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._actuator_health_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("actuator_health_topic").value),
            latched_qos,
        )
        self._telemetry_publisher = self.create_publisher(
            ControlTelemetry,
            str(self.get_parameter("control_telemetry_topic").value),
            10,
        )

        update_rate_hz = max(1.0, _finite_parameter(self, "update_rate_hz"))
        status_rate_hz = max(1.0, _finite_parameter(self, "status_rate_hz"))
        self._update_timer = self.create_timer(
            1.0 / update_rate_hz,
            self._update,
        )
        self._status_timer = self.create_timer(
            1.0 / status_rate_hz,
            self._publish_status_bundle,
        )
        self.get_logger().info(
            "vehicle loopback ready: "
            f"cmd={self.get_parameter('cmd_vel_topic').value} "
            f"odom={self.get_parameter('odom_topic').value} "
            f"frame={self._map_frame}->{self._base_frame}"
        )

    def _declare_parameters(self) -> None:
        physical = parameters_for("pure_line_following")
        declarations = {
            "cmd_vel_topic": "/cmd_vel_nav",
            "initial_pose_topic": "/initialpose",
            "odom_topic": "/odometry/fused",
            "imu_topic": "/imu/control",
            "fusion_status_topic": "/odometry/fusion_status",
            "actuator_health_topic": "/cup_car_serial/actuator_healthy",
            "control_telemetry_topic": "/cup_car_serial/control_telemetry",
            "status_topic": "/vehicle_loopback_sim/status",
            "map_frame_id": "map",
            "base_frame_id": "base_link",
            "initial_x_m": 0.0,
            "initial_y_m": 0.0,
            "initial_yaw_rad": 0.0,
            "update_rate_hz": 50.0,
            "status_rate_hz": 10.0,
            "command_timeout_s": 0.50,
            "max_linear_speed_mps": 0.20,
            "max_angular_speed_rps": 0.90,
            "max_linear_accel_mps2": 0.0,
            "max_angular_accel_rps2": 0.0,
            "wheel_track_m": physical["wheel_track_m"],
            "base_offset_x_m": physical["base_offset_x_m"],
            "base_offset_y_m": physical["base_offset_y_m"],
            "publish_tf": True,
            "publish_actuator_health": True,
            "publish_control_telemetry": True,
        }
        self.declare_parameters("", list(declarations.items()))

    def _cmd_vel_callback(self, message: Twist) -> None:
        self._desired_twist = Twist2D(
            float(message.linear.x),
            float(message.angular.z),
        )
        self._last_command_time = self.get_clock().now()

    def _initial_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        self._pose = Pose2D(
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            _yaw_from_quaternion(message.pose.pose.orientation),
        )
        self._applied_twist = Twist2D()
        self._desired_twist = Twist2D()
        self._last_update_time = self.get_clock().now()
        self.get_logger().info(
            "loopback pose reset to "
            f"x={self._pose.x:.3f} y={self._pose.y:.3f} "
            f"yaw={self._pose.yaw:.3f}"
        )

    def _update(self) -> None:
        now = self.get_clock().now()
        elapsed_s = max(0.0, min(0.20, (now - self._last_update_time).nanoseconds * 1.0e-9))
        self._last_update_time = now

        command_age_s = (now - self._last_command_time).nanoseconds * 1.0e-9
        desired = (
            self._desired_twist
            if command_age_s <= self._command_timeout_s
            else Twist2D()
        )
        self._applied_twist = limited_twist(
            desired,
            self._applied_twist,
            elapsed_s,
            _finite_parameter(self, "max_linear_speed_mps"),
            _finite_parameter(self, "max_angular_speed_rps"),
            _finite_parameter(self, "max_linear_accel_mps2"),
            _finite_parameter(self, "max_angular_accel_rps2"),
        )
        self._pose = integrate_base_link(
            self._pose,
            self._applied_twist,
            elapsed_s,
            _finite_parameter(self, "base_offset_x_m"),
            _finite_parameter(self, "base_offset_y_m"),
        )
        self._publish_odometry(now)
        self._publish_imu(now)
        if self._publish_tf:
            self._publish_transform(now)
        if self._publish_control_telemetry:
            self._publish_telemetry(now, command_age_s)

    def _publish_odometry(self, stamp) -> None:
        message = Odometry()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self._map_frame
        message.child_frame_id = self._base_frame
        message.pose.pose.position.x = self._pose.x
        message.pose.pose.position.y = self._pose.y
        message.pose.pose.orientation = _quaternion_from_yaw(self._pose.yaw)
        message.twist.twist.linear.x = self._applied_twist.linear_x
        message.twist.twist.angular.z = self._applied_twist.angular_z
        message.pose.covariance[0] = 1.0e-6
        message.pose.covariance[7] = 1.0e-6
        message.pose.covariance[35] = 1.0e-6
        message.twist.covariance[0] = 1.0e-6
        message.twist.covariance[35] = 1.0e-6
        self._odom_publisher.publish(message)

    def _publish_imu(self, stamp) -> None:
        message = Imu()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self._base_frame
        message.orientation = _quaternion_from_yaw(self._pose.yaw)
        message.angular_velocity.z = self._applied_twist.angular_z
        message.orientation_covariance[8] = 1.0e-6
        message.angular_velocity_covariance[8] = 1.0e-6
        message.linear_acceleration_covariance[0] = -1.0
        self._imu_publisher.publish(message)

    def _publish_transform(self, stamp) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._pose.x
        transform.transform.translation.y = self._pose.y
        transform.transform.rotation = _quaternion_from_yaw(self._pose.yaw)
        self._tf_broadcaster.sendTransform(transform)

    def _publish_telemetry(self, stamp, command_age_s: float) -> None:
        target_left, target_right = wheel_speeds(
            self._desired_twist,
            self._wheel_track_m,
        )
        measured_left, measured_right = wheel_speeds(
            self._applied_twist,
            self._wheel_track_m,
        )
        self._telemetry_sequence = (self._telemetry_sequence + 1) % (2**32)
        message = ControlTelemetry()
        message.header.stamp = stamp.to_msg()
        message.mcu_time_ms = int(
            max(0.0, (stamp - self._started_time).nanoseconds * 1.0e-6)
        ) % (2**32)
        message.sample_sequence = self._telemetry_sequence
        message.mode = ControlTelemetry.MODE_NAVIGATION
        message.emergency_stop = False
        message.command_valid = command_age_s <= self._command_timeout_s
        message.command_age_ms = int(max(0.0, command_age_s) * 1000.0)
        message.received_linear_velocity_mps = self._desired_twist.linear_x
        message.received_angular_velocity_radps = self._desired_twist.angular_z
        message.target_left_velocity_mps = target_left
        message.target_right_velocity_mps = target_right
        message.measured_left_velocity_mps = measured_left
        message.measured_right_velocity_mps = measured_right
        self._telemetry_publisher.publish(message)

    def _publish_status_bundle(self) -> None:
        fusion_status = String()
        fusion_status.data = "FULL"
        self._fusion_status_publisher.publish(fusion_status)

        if self._publish_actuator_health:
            health = Bool()
            health.data = True
            self._actuator_health_publisher.publish(health)

        payload = {
            "pose": {
                "x_m": round(self._pose.x, 4),
                "y_m": round(self._pose.y, 4),
                "yaw_rad": round(self._pose.yaw, 4),
            },
            "twist": {
                "linear_x_mps": round(self._applied_twist.linear_x, 4),
                "angular_z_rps": round(self._applied_twist.angular_z, 4),
            },
            "fusion_status": "FULL",
        }
        text = json.dumps(payload, separators=(",", ":"))
        if text == self._last_status_text:
            return
        self._last_status_text = text
        status = String()
        status.data = text
        self._status_publisher.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleLoopbackSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
