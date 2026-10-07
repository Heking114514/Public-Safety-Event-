"""ROS 2 loopback simulator for planner/controller testing."""

from __future__ import annotations

import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import (
    Point32,
    PolygonStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
)
from mission_control_interfaces.msg import ControlTelemetry
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Empty, String
from tf2_ros import TransformBroadcaster
from vehicle_config import parameters_for

from .arena_layout import layout_from_config
from .motion import Pose2D, Twist2D, integrate_base_link, limited_twist, wheel_speeds
from .obstacle import (
    ObstacleDetection,
    ObstacleDetectionConfig,
    SimulatedObstacleDetector,
    StableObstacleGate,
    layout_obstacles,
)


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
        self._publish_obstacle_detection = bool(
            self.get_parameter("publish_obstacle_detection").value
        )
        self._obstacle_frame_id = str(
            self.get_parameter("obstacle_detection_frame_id").value
        )
        self._obstacle_gate = StableObstacleGate(
            max(0.0, _finite_parameter(self, "obstacle_detection_stable_time_s"))
        )
        self._manual_obstacles: list[tuple[tuple[float, float], ...]] = []
        self._map_obstacles = self._load_map_obstacles()
        self._map_obstacle_detector = self._make_obstacle_detector(
            self._map_obstacles,
            "map",
        )
        self._manual_obstacle_detector = self._make_obstacle_detector(
            self._manual_obstacles,
            "manual",
            activation_floor_m=_finite_parameter(
                self, "obstacle_detection_min_range_m"
            ),
        )
        self._last_obstacle_index: int | None = None

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
        self._manual_obstacle_subscription = self.create_subscription(
            PolygonStamped,
            str(self.get_parameter("manual_obstacle_topic").value),
            self._manual_obstacle_callback,
            10,
        )
        self._clear_manual_obstacles_subscription = self.create_subscription(
            Empty,
            str(self.get_parameter("clear_manual_obstacles_topic").value),
            self._clear_manual_obstacles_callback,
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
        self._front_obstacle_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("front_obstacle_topic").value),
            qos_profile_sensor_data,
        )
        self._front_range_publisher = self.create_publisher(
            Range,
            str(self.get_parameter("front_obstacle_range_topic").value),
            qos_profile_sensor_data,
        )
        self._dynamic_obstacle_publisher = self.create_publisher(
            PolygonStamped,
            str(self.get_parameter("dynamic_obstacle_topic").value),
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
            "front_obstacle_topic": "/obstacle/front_blocked",
            "front_obstacle_range_topic": "/obstacle/front_range",
            "dynamic_obstacle_topic": "/perception/dynamic_obstacle",
            "manual_obstacle_topic": "/vehicle_loopback_sim/manual_obstacle",
            "clear_manual_obstacles_topic": "/vehicle_loopback_sim/clear_obstacles",
            "status_topic": "/vehicle_loopback_sim/status",
            "arena_config": self._default_arena_config(),
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
            "publish_obstacle_detection": True,
            "obstacle_detection_frame_id": "camera_link",
            "obstacle_sensor_x_m": physical["camera_x_m"],
            "obstacle_sensor_y_m": physical["camera_y_m"],
            "obstacle_detection_trigger_range_m": 0.25,
            "obstacle_detection_trigger_jitter_m": 0.0,
            "obstacle_detection_lateral_tolerance_m": 0.08,
            "obstacle_detection_fov_rad": 0.40,
            "obstacle_detection_min_range_m": 0.03,
            "obstacle_detection_stable_time_s": 0.0,
            "obstacle_detection_random_seed": 0,
        }
        self.declare_parameters("", list(declarations.items()))

    def _default_arena_config(self) -> str:
        try:
            return str(
                Path(get_package_share_directory("new_map_planner"))
                / "config"
                / "arena.yaml"
            )
        except Exception:
            return ""

    def _load_map_obstacles(self) -> list[tuple[tuple[float, float], ...]]:
        if not self._publish_obstacle_detection:
            return []
        arena_config = str(self.get_parameter("arena_config").value)
        if not arena_config:
            self.get_logger().warn("simulated obstacle recognition has no arena_config")
            return []
        try:
            return layout_obstacles(layout_from_config(arena_config))
        except Exception as error:
            self.get_logger().warn(
                f"failed to load simulated obstacle map from {arena_config}: {error}"
            )
            return []

    def _make_obstacle_detector(
        self,
        obstacles: list[tuple[tuple[float, float], ...]],
        label: str,
        activation_floor_m: float | None = None,
    ) -> SimulatedObstacleDetector | None:
        if not self._publish_obstacle_detection:
            return None
        if not obstacles:
            if label == "map":
                self.get_logger().warn(
                    "simulated obstacle recognition found no map obstacles"
                )
            return None
        config = self._obstacle_config(activation_floor_m)
        self.get_logger().info(
            "simulated obstacle recognition ready: "
            f"source={label} obstacles={len(obstacles)} "
            f"trigger={config.trigger_range_m:.2f}+/-"
            f"{config.trigger_jitter_m:.2f}m"
        )
        return SimulatedObstacleDetector(obstacles, config)

    def _obstacle_config(
        self,
        activation_floor_m: float | None = None,
    ) -> ObstacleDetectionConfig:
        return ObstacleDetectionConfig(
            sensor_x_m=_finite_parameter(self, "obstacle_sensor_x_m"),
            sensor_y_m=_finite_parameter(self, "obstacle_sensor_y_m"),
            trigger_range_m=max(
                0.01, _finite_parameter(self, "obstacle_detection_trigger_range_m")
            ),
            trigger_jitter_m=max(
                0.0, _finite_parameter(self, "obstacle_detection_trigger_jitter_m")
            ),
            lateral_tolerance_m=max(
                0.0,
                _finite_parameter(self, "obstacle_detection_lateral_tolerance_m"),
            ),
            min_range_m=max(
                0.0, _finite_parameter(self, "obstacle_detection_min_range_m")
            ),
            random_seed=int(self.get_parameter("obstacle_detection_random_seed").value),
            activation_floor_m=activation_floor_m,
        )

    def _detect_obstacle(self) -> ObstacleDetection | None:
        detections = []
        if self._map_obstacle_detector is not None:
            detection = self._map_obstacle_detector.detect(self._pose)
            if detection is not None:
                detections.append(detection)
        if self._manual_obstacle_detector is not None:
            detection = self._manual_obstacle_detector.detect(self._pose)
            if detection is not None:
                detections.append(
                    ObstacleDetection(
                        detection.obstacle_index + len(self._map_obstacles),
                        detection.range_m,
                        detection.trigger_range_m,
                    )
                )
        if not detections:
            return None
        return min(detections, key=lambda detection: detection.range_m)

    def _detected_obstacle_polygon(
        self,
        detection: ObstacleDetection,
    ) -> tuple[tuple[float, float], ...] | None:
        index = detection.obstacle_index
        if index < len(self._map_obstacles):
            return self._map_obstacles[index]
        manual_index = index - len(self._map_obstacles)
        if 0 <= manual_index < len(self._manual_obstacles):
            return self._manual_obstacles[manual_index]
        return None

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
        self._reset_obstacle_reporting()
        self.get_logger().info(
            "loopback pose reset to "
            f"x={self._pose.x:.3f} y={self._pose.y:.3f} "
            f"yaw={self._pose.yaw:.3f}"
        )

    def _manual_obstacle_callback(self, message: PolygonStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self._map_frame:
            self.get_logger().warn(
                "ignoring manual obstacle in frame "
                f"{message.header.frame_id}; expected {self._map_frame}"
            )
            return
        obstacle = tuple(
            (float(point.x), float(point.y)) for point in message.polygon.points
        )
        if len(obstacle) < 3 or not all(
            math.isfinite(value) for point in obstacle for value in point
        ):
            self.get_logger().warn("ignoring invalid manual obstacle polygon")
            return
        self._manual_obstacles.append(obstacle)
        self._manual_obstacle_detector = self._make_obstacle_detector(
            self._manual_obstacles,
            "manual",
            activation_floor_m=_finite_parameter(
                self, "obstacle_detection_min_range_m"
            ),
        )
        self._reset_obstacle_reporting()
        self.get_logger().info(
            f"manual simulated obstacle added: count={len(self._manual_obstacles)}"
        )

    def _clear_manual_obstacles_callback(self, _: Empty) -> None:
        if not self._manual_obstacles:
            return
        self._manual_obstacles.clear()
        self._manual_obstacle_detector = self._make_obstacle_detector(
            self._manual_obstacles,
            "manual",
            activation_floor_m=_finite_parameter(
                self, "obstacle_detection_min_range_m"
            ),
        )
        self._reset_obstacle_reporting()
        self.get_logger().info("manual simulated obstacles cleared")

    def _reset_obstacle_reporting(self) -> None:
        self._obstacle_gate.reset()
        self._last_obstacle_index = None

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

    def _publish_obstacle_report(self, stamp) -> None:
        if not self._publish_obstacle_detection:
            return
        raw_detection = self._detect_obstacle()
        detection = self._obstacle_gate.update(
            raw_detection,
            stamp.nanoseconds * 1.0e-9,
        )
        if detection is not None:
            range_message = Range()
            range_message.header.stamp = stamp.to_msg()
            range_message.header.frame_id = self._obstacle_frame_id
            range_message.radiation_type = Range.INFRARED
            range_message.field_of_view = max(
                0.01, _finite_parameter(self, "obstacle_detection_fov_rad")
            )
            range_message.min_range = _finite_parameter(
                self, "obstacle_detection_min_range_m"
            )
            range_message.max_range = detection.trigger_range_m
            range_message.range = detection.range_m
            self._front_range_publisher.publish(range_message)
            self._publish_dynamic_obstacle(detection, stamp)

        blocked = Bool()
        blocked.data = detection is not None
        self._front_obstacle_publisher.publish(blocked)

        obstacle_index = detection.obstacle_index if detection is not None else None
        if obstacle_index == self._last_obstacle_index:
            return
        self._last_obstacle_index = obstacle_index
        if detection is None:
            self.get_logger().info("simulated front obstacle clear")
        else:
            self.get_logger().warn(
                "simulated front obstacle detected: "
                f"index={detection.obstacle_index} range={detection.range_m:.2f}m "
                f"trigger={detection.trigger_range_m:.2f}m"
            )

    def _publish_dynamic_obstacle(self, detection: ObstacleDetection, stamp) -> None:
        polygon = self._detected_obstacle_polygon(detection)
        if polygon is None:
            return
        message = PolygonStamped()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self._map_frame
        for x_m, y_m in polygon:
            message.polygon.points.append(Point32(x=float(x_m), y=float(y_m), z=0.0))
        self._dynamic_obstacle_publisher.publish(message)

    def _publish_status_bundle(self) -> None:
        fusion_status = String()
        fusion_status.data = "FULL"
        self._fusion_status_publisher.publish(fusion_status)

        if self._publish_actuator_health:
            health = Bool()
            health.data = True
            self._actuator_health_publisher.publish(health)

        now = self.get_clock().now()
        self._publish_obstacle_report(now)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
