"""ROS2 adapter for the rectangular primitive controller."""

from __future__ import annotations

import math
from typing import Optional

from .geometry import Pose2D, wrap_angle
from .primitive_controller import (
    PrimitiveController,
    PrimitivePlanner,
    Twist2D,
    differential_wheel_speeds,
    make_action_primitives,
    pose_is_finite,
    rectangle_goals,
)

try:
    import rclpy
    from geometry_msgs.msg import Twist as RosTwist
    from mission_control_interfaces.msg import MotionHoldState
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    RosTwist = None
    MotionHoldState = None
    Odometry = None
    ExternalShutdownException = KeyboardInterrupt
    Node = object
    Bool = None
    String = None
    DurabilityPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    qos_profile_sensor_data = None


if rclpy is not None:

    class PrimitiveControllerNode(Node):
        """Opt-in ROS2 wrapper around the pure primitive controller."""

        def __init__(self) -> None:
            super().__init__("rectangle_primitive_controller")
            self.enable = bool(self.declare_parameter("enable", False).value)
            self.odom_topic = str(
                self.declare_parameter("odom_topic", "/odometry/fused").value
            )
            self.cmd_vel_topic = str(
                self.declare_parameter(
                    "cmd_vel_topic", "/cmd_vel_primitive_test"
                ).value
            )
            self.route_frame = str(
                self.declare_parameter("route_frame", "map").value
            )
            self.child_frame = str(
                self.declare_parameter("child_frame", "base_link").value
            )
            self.fusion_status_topic = str(
                self.declare_parameter(
                    "fusion_status_topic", "/odometry/fusion_status"
                ).value
            )
            self.require_fusion_status = bool(
                self.declare_parameter("require_fusion_status", True).value
            )
            self.fusion_status_timeout_s = float(
                self.declare_parameter("fusion_status_timeout_s", 0.8).value
            )
            self.actuator_health_topic = str(
                self.declare_parameter(
                    "actuator_health_topic",
                    "/cup_car_serial/actuator_healthy",
                ).value
            )
            self.require_actuator_health = bool(
                self.declare_parameter("require_actuator_health", True).value
            )
            self.actuator_health_timeout_s = float(
                self.declare_parameter(
                    "actuator_health_timeout_s", 0.8
                ).value
            )
            self.obstacle_topic = str(
                self.declare_parameter(
                    "obstacle_topic", "/obstacle/front_blocked"
                ).value
            )
            self.obstacle_timeout_s = float(
                self.declare_parameter("obstacle_timeout_s", 0.5).value
            )
            self.motion_hold_state_topic = str(
                self.declare_parameter(
                    "motion_hold_state_topic",
                    "/waypoint_navigation/motion_hold_state",
                ).value
            )
            self.motion_hold_timeout_s = float(
                self.declare_parameter("motion_hold_timeout_s", 0.8).value
            )
            self.mode = str(
                self.declare_parameter("mode", "rectangle").value
            ).strip().lower()
            self.side_length_m = float(
                self.declare_parameter("side_length_m", 1.8).value
            )
            self.target_x = float(
                self.declare_parameter("target_x", 1.0).value
            )
            self.target_y = float(
                self.declare_parameter("target_y", 0.0).value
            )
            self.target_yaw = float(
                self.declare_parameter("target_yaw", 0.0).value
            )
            self.control_period_s = float(
                self.declare_parameter("control_period_s", 0.05).value
            )
            self.odom_timeout_s = float(
                self.declare_parameter("odom_timeout_s", 0.5).value
            )
            wheel_track_m = float(
                self.declare_parameter("wheel_track_m", 0.1466).value
            )
            differential_wheel_speeds(Twist2D(0.0, 0.0), wheel_track_m)

            action_duration_s = float(
                self.declare_parameter("primitive_duration_s", 0.5).value
            )
            translation_step_m = float(
                self.declare_parameter("translation_step_m", 0.03).value
            )
            planner = PrimitivePlanner(
                actions=make_action_primitives(
                    translation_step_m=translation_step_m,
                    duration_s=action_duration_s,
                ),
                position_resolution_m=float(
                    self.declare_parameter("position_resolution_m", 0.01).value
                ),
                yaw_resolution_rad=float(
                    self.declare_parameter(
                        "yaw_resolution_rad", math.radians(5.0)
                    ).value
                ),
                search_margin_m=float(
                    self.declare_parameter("search_margin_m", 0.6).value
                ),
                max_expansions=int(
                    self.declare_parameter("max_expansions", 50000).value
                ),
                goal_position_tolerance_m=float(
                    self.declare_parameter(
                        "planner_goal_position_tolerance_m", 0.04
                    ).value
                ),
                goal_yaw_tolerance_rad=float(
                    self.declare_parameter(
                        "planner_goal_yaw_tolerance_rad", math.radians(4.0)
                    ).value
                ),
            )
            self.controller = PrimitiveController(
                planner,
                position_tolerance_m=float(
                    self.declare_parameter("position_tolerance_m", 0.05).value
                ),
                yaw_tolerance_rad=float(
                    self.declare_parameter(
                        "yaw_tolerance_rad", math.radians(5.0)
                    ).value
                ),
                replan_position_error_m=float(
                    self.declare_parameter(
                        "replan_position_error_m", 0.10
                    ).value
                ),
                replan_yaw_error_rad=float(
                    self.declare_parameter(
                        "replan_yaw_error_rad", math.radians(12.0)
                    ).value
                ),
                replan_cooldown_s=float(
                    self.declare_parameter("replan_cooldown_s", 0.30).value
                ),
                failed_plan_retry_s=float(
                    self.declare_parameter(
                        "failed_plan_retry_s", 0.50
                    ).value
                ),
                goal_line_tolerance_m=float(
                    self.declare_parameter(
                        "goal_line_tolerance_m", 0.05
                    ).value
                ),
                goal_lateral_tolerance_m=float(
                    self.declare_parameter(
                        "goal_lateral_tolerance_m", 0.10
                    ).value
                ),
                turn_correction_start_rad=float(
                    self.declare_parameter(
                        "turn_correction_start_rad", math.radians(6.0)
                    ).value
                ),
                turn_correction_min_rate_radps=float(
                    self.declare_parameter(
                        "turn_correction_min_rate_radps",
                        math.radians(10.0),
                    ).value
                ),
                turn_correction_max_rate_radps=float(
                    self.declare_parameter(
                        "turn_correction_max_rate_radps",
                        math.radians(25.0),
                    ).value
                ),
                turn_correction_gain=float(
                    self.declare_parameter(
                        "turn_correction_gain", 1.0
                    ).value
                ),
                straight_heading_deadband_rad=float(
                    self.declare_parameter(
                        "straight_heading_deadband_rad",
                        math.radians(1.5),
                    ).value
                ),
                straight_heading_gain=float(
                    self.declare_parameter(
                        "straight_heading_gain", 1.5
                    ).value
                ),
                straight_heading_max_rate_radps=float(
                    self.declare_parameter(
                        "straight_heading_max_rate_radps",
                        math.radians(8.0),
                    ).value
                ),
            )
            if self.mode not in {"rectangle", "goal"}:
                raise ValueError("mode must be 'rectangle' or 'goal'")
            if self.side_length_m <= 0.0:
                raise ValueError("side_length_m must be positive")
            if self.control_period_s <= 0.0 or self.odom_timeout_s <= 0.0:
                raise ValueError("control timing parameters must be positive")
            if (
                self.fusion_status_timeout_s <= 0.0
                or self.actuator_health_timeout_s <= 0.0
                or self.obstacle_timeout_s <= 0.0
                or self.motion_hold_timeout_s <= 0.0
            ):
                raise ValueError("safety timeout parameters must be positive")

            self.cmd_publisher = self.create_publisher(
                RosTwist, self.cmd_vel_topic, 10
            )
            self.odom_subscription = self.create_subscription(
                Odometry, self.odom_topic, self._on_odometry, 10
            )
            reliable_transient = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.fusion_status_subscription = self.create_subscription(
                String,
                self.fusion_status_topic,
                self._on_fusion_status,
                reliable_transient,
            )
            self.actuator_health_subscription = self.create_subscription(
                Bool,
                self.actuator_health_topic,
                self._on_actuator_health,
                reliable_transient,
            )
            self.obstacle_subscription = self.create_subscription(
                Bool,
                self.obstacle_topic,
                self._on_obstacle,
                qos_profile_sensor_data,
            )
            self.motion_hold_subscription = self.create_subscription(
                MotionHoldState,
                self.motion_hold_state_topic,
                self._on_motion_hold,
                reliable_transient,
            )
            self.timer = self.create_timer(
                self.control_period_s, self._on_timer
            )
            self.latest_pose: Optional[Pose2D] = None
            self.latest_received_at = -math.inf
            self.latest_fusion_status = ""
            self.latest_fusion_received_at = -math.inf
            self.latest_actuator_health = False
            self.latest_actuator_received_at = -math.inf
            self.latest_obstacle = False
            self.latest_obstacle_received_at = -math.inf
            self.latest_motion_hold = False
            self.latest_motion_hold_received_at = -math.inf
            self.started = False
            self._reported_complete = False
            self._last_gate_state = ""
            self.get_logger().warn(
                "Primitive controller is disabled by default; "
                f"enable={str(self.enable).lower()} "
                f"cmd_vel_topic={self.cmd_vel_topic}"
            )

        def _on_odometry(self, message: Odometry) -> None:
            if message.header.frame_id != self.route_frame:
                return
            if message.child_frame_id != self.child_frame:
                return
            q = message.pose.pose.orientation
            pose = Pose2D(
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                ),
            )
            if pose_is_finite(pose):
                self.latest_pose = pose
                self.latest_received_at = (
                    self.get_clock().now().nanoseconds * 1.0e-9
                )

        def _on_fusion_status(self, message: String) -> None:
            self.latest_fusion_status = message.data.strip()
            self.latest_fusion_received_at = (
                self.get_clock().now().nanoseconds * 1.0e-9
            )

        def _on_actuator_health(self, message: Bool) -> None:
            self.latest_actuator_health = bool(message.data)
            self.latest_actuator_received_at = (
                self.get_clock().now().nanoseconds * 1.0e-9
            )

        def _on_obstacle(self, message: Bool) -> None:
            self.latest_obstacle = bool(message.data)
            self.latest_obstacle_received_at = (
                self.get_clock().now().nanoseconds * 1.0e-9
            )

        def _on_motion_hold(self, message: MotionHoldState) -> None:
            self.latest_motion_hold = bool(message.held)
            self.latest_motion_hold_received_at = (
                self.get_clock().now().nanoseconds * 1.0e-9
            )

        def _safety_gate_reason(self, now_s: float) -> str:
            allowed_fusion_states = {
                "FULL",
                "DEGRADED_NO_VISION",
                "DEGRADED_NO_IMU",
                "DEGRADED_NO_WHEEL",
                "DEGRADED_VISION_ONLY",
                "DEGRADED_WHEEL_ONLY",
                "DEGRADED_VISUAL_REALIGNED",
            }
            if self.require_fusion_status:
                if now_s - self.latest_fusion_received_at > self.fusion_status_timeout_s:
                    return "fusion_status_stale"
                if self.latest_fusion_status not in allowed_fusion_states:
                    return f"fusion_status_{self.latest_fusion_status or 'missing'}"

            if self.require_actuator_health:
                if (
                    now_s - self.latest_actuator_received_at
                    > self.actuator_health_timeout_s
                ):
                    return "actuator_health_stale"
                if not self.latest_actuator_health:
                    return "actuator_unhealthy"

            if (
                self.latest_obstacle
                and now_s - self.latest_obstacle_received_at
                <= self.obstacle_timeout_s
            ):
                return "front_obstacle"

            if (
                self.latest_motion_hold
                and now_s - self.latest_motion_hold_received_at
                <= self.motion_hold_timeout_s
            ):
                return "motion_hold"
            return ""

        def _publish_gate_state(self, reason: str) -> None:
            if reason == self._last_gate_state:
                return
            self._last_gate_state = reason
            if reason:
                self.get_logger().warn(f"Primitive controller stopped: {reason}")
            else:
                self.get_logger().info("Primitive controller safety gate cleared")

        def _on_timer(self) -> None:
            now_s = self.get_clock().now().nanoseconds * 1.0e-9
            if (
                not self.enable
                or self.latest_pose is None
                or now_s - self.latest_received_at > self.odom_timeout_s
            ):
                self.controller.invalidate_plan("not_enabled_or_stale")
                self._publish(Twist2D(0.0, 0.0))
                return

            gate_reason = self._safety_gate_reason(now_s)
            self._publish_gate_state(gate_reason)
            if gate_reason:
                self.controller.invalidate_plan(gate_reason)
                self._publish(Twist2D(0.0, 0.0))
                return

            if not self.started:
                self.controller.start(
                    self.latest_pose, self._make_goals(self.latest_pose)
                )
                self.started = True
                self.get_logger().info(f"Started {self.mode} primitive test")

            command = self.controller.update(now_s, self.latest_pose)
            self._publish(command)
            if self.controller.is_complete and not self._reported_complete:
                self._reported_complete = True
                self.get_logger().info("Primitive test completed")

        def _make_goals(self, origin: Pose2D) -> tuple[Pose2D, ...]:
            if self.mode == "rectangle":
                return rectangle_goals(origin, self.side_length_m)
            return (
                Pose2D(
                    self.target_x,
                    self.target_y,
                    wrap_angle(self.target_yaw),
                ),
            )

        def _publish(self, command: Twist2D) -> None:
            message = RosTwist()
            message.linear.x = command.linear_x_mps
            message.angular.z = command.angular_z_rps
            self.cmd_publisher.publish(message)

        def destroy_node(self):
            if rclpy.ok():
                self._publish(Twist2D(0.0, 0.0))
            return super().destroy_node()


else:

    class PrimitiveControllerNode:
        def __init__(self, *args, **kwargs) -> None:
            raise RuntimeError(
                "PrimitiveControllerNode requires ROS2 Python packages"
            )


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("PrimitiveControllerNode requires ROS2 Python packages")
    rclpy.init(args=args)
    node = PrimitiveControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["PrimitiveControllerNode", "main"]
