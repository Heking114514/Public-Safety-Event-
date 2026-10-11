import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from mission_control_interfaces.srv import ConfirmRfidCheckpoint, ContinueRetreat, SetMotionHold
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String, UInt64

from .control import arc_command, fillet_path, normalize


class IdealController(Node):
    def __init__(self) -> None:
        super().__init__("ideal_controler")
        self.declare_parameter("planner_service", "/new_map_planner/plan")
        self.declare_parameter("route_input_topic", "/waypoint_navigation/route_input")
        self.declare_parameter("odom_topic", "/odometry/fused")
        self.declare_parameter("obstacle_topic", "/obstacle/front_blocked")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("status_topic", "/waypoint_navigation/status")
        self.declare_parameter("route_ack_topic", "/waypoint_navigation/route_ack")
        self.declare_parameter("current_waypoint_topic", "/waypoint_navigation/current_waypoint")
        self.declare_parameter("waypoint_path_topic", "/waypoint_path")
        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.65)
        self.declare_parameter("arc_radius", 0.10)
        self.declare_parameter("goal_yaw_tolerance", 0.10)
        self.declare_parameter("lookahead_poses", 2)
        self.declare_parameter("waypoint_tolerance", 0.035)
        self.declare_parameter("return_home", False)

        self._path: list = []
        self._track: list[tuple[float, float]] = []
        self._index = 0
        self._pose: Optional[Odometry] = None
        self._obstacle = False
        # The ideal-controller experiment does not replay a reverse trace.
        # After an obstacle-triggered replan, accept the replacement route
        # once and ignore the old forward sensor latch until the sensor has
        # explicitly cleared.  Otherwise a stationary ray sensor would keep
        # the ideal point at the old route's stopping pose forever.
        self._awaiting_obstacle_route = False
        self._ignore_obstacle_until_clear = False
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self._status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, durable)
        self._ack_pub = self.create_publisher(UInt64, self.get_parameter("route_ack_topic").value, durable)
        self._waypoint_pub = self.create_publisher(Int32, self.get_parameter("current_waypoint_topic").value, durable)
        self._path_pub = self.create_publisher(Path, self.get_parameter("waypoint_path_topic").value, durable)
        self.create_service(SetMotionHold, "/waypoint_navigator/set_motion_hold", self._on_hold)
        self.create_service(ContinueRetreat, "/waypoint_navigator/continue_obstacle_retreat", self._on_retreat)
        self.create_service(ContinueRetreat, "/waypoint_navigator/continue_route_retreat", self._on_retreat)
        self.create_service(ConfirmRfidCheckpoint, "/waypoint_navigator/confirm_rfid_checkpoint", self._on_rfid)
        self.create_subscription(Path, self.get_parameter("route_input_topic").value,
                                 self._on_path, rclpy.qos.qos_profile_system_default)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self._on_odom, sensor_qos)
        self.create_subscription(Bool, self.get_parameter("obstacle_topic").value,
                                 self._on_obstacle, sensor_qos)
        self._timer = self.create_timer(0.033, self._control)
        self._set_status("WAITING_FOR_ROUTE")
        self.get_logger().info("理想质点控制器就绪：接管 route_input，圆弧跟踪，不检查车体 footprint")

    def _on_path(self, message: Path) -> None:
        if message.poses:
            replacing_obstacle_route = self._awaiting_obstacle_route
            self._path = message.poses
            raw_points = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
            self._track = fillet_path(raw_points, float(self.get_parameter("arc_radius").value))
            self._index = 1 if len(self._track) > 1 else 0
            if replacing_obstacle_route:
                self._obstacle = False
                self._awaiting_obstacle_route = False
                self._ignore_obstacle_until_clear = True
                self.get_logger().info(
                    "障碍重规划路线已接管：忽略旧障碍前向锁存，等待传感器清除")
            route_id = int(message.header.stamp.sec) * 1000000000 + int(message.header.stamp.nanosec)
            if route_id:
                ack = UInt64()
                ack.data = route_id
                self._ack_pub.publish(ack)
            self._path_pub.publish(message)
            self._set_status("FOLLOWING")
            self.get_logger().info(f"收到规划路径并确认：route_id={route_id} poses={len(self._path)}")

    def _on_odom(self, message: Odometry) -> None:
        self._pose = message

    def _on_obstacle(self, message: Bool) -> None:
        if self._ignore_obstacle_until_clear:
            if not message.data:
                self._ignore_obstacle_until_clear = False
                self._obstacle = False
                self.get_logger().info("障碍传感器已清除，恢复正常障碍触发")
            return
        was_blocked = self._obstacle
        self._obstacle = bool(message.data)
        if self._obstacle and not was_blocked:
            # 质点验证不需要正式控制器的历史轨迹倒退，直接把当前安全停点
            # 交给总控做动态障碍重规划。
            self._awaiting_obstacle_route = True
            self._set_status("OBSTACLE_REPLAN_REQUIRED")

    def _on_hold(self, request, response):
        if request.hold:
            self._obstacle = True
            self._set_status("HELD_FOR_MISSION")
        else:
            self._obstacle = False
            self._set_status("FOLLOWING" if self._path else "WAITING_FOR_ROUTE")
        response.success = True
        response.motion_held = request.hold
        response.active_sources = [request.source] if request.hold else []
        response.message = "ideal controller hold updated"
        return response

    def _on_retreat(self, request, response):
        # ContinueRetreat is deliberately rejected by the ideal controller:
        # this controller has no recorded wheel trace and must never invent a
        # reverse manoeuvre.  Keep the response on the actual service
        # contract so the mission manager can terminate recovery cleanly.
        response.accepted = False
        response.trace_exhausted = True
        response.remaining_breadcrumbs = 0
        response.message = "ideal controller does not execute reverse recovery"
        return response

    def _on_rfid(self, request, response):
        response.accepted = False
        response.message = "RFID confirmation is not used by ideal controller"
        return response

    def _set_status(self, value: str) -> None:
        message = String()
        message.data = value
        self._status_pub.publish(message)

    def _control(self) -> None:
        command = Twist()
        if self._pose is None or not self._track:
            self._cmd_pub.publish(command)
            return
        if self._obstacle:
            self._cmd_pub.publish(command)
            return
        pose = self._pose.pose.pose
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        tolerance = float(self.get_parameter("waypoint_tolerance").value)
        while self._index < len(self._track) - 1:
            target_x, target_y = self._track[self._index]
            if math.hypot(target_x - pose.position.x, target_y - pose.position.y) <= tolerance:
                self._index += 1
            else:
                break
        waypoint = Int32()
        waypoint.data = self._index
        self._waypoint_pub.publish(waypoint)
        if self._index >= len(self._track) - 1:
            target_x, target_y = self._track[-1]
            previous_x, previous_y = self._track[-2]
            final_yaw = math.atan2(target_y - previous_y, target_x - previous_x)
            position_ready = math.hypot(target_x - pose.position.x, target_y - pose.position.y) <= tolerance
            yaw_ready = abs(normalize(final_yaw - yaw)) <= float(
                self.get_parameter("goal_yaw_tolerance").value)
            if position_ready and yaw_ready:
                self._set_status("GOAL_REACHED")
                self._cmd_pub.publish(command)
                return
        self._set_status("FOLLOWING")
        target_x, target_y = self._track[self._index]
        speed = float(self.get_parameter("linear_speed").value)
        max_angular = float(self.get_parameter("max_angular_speed").value)
        command.linear.x, command.angular.z = arc_command(
            pose.position.x, pose.position.y, yaw, target_x, target_y,
            speed, 2.0, max_angular)
        self._cmd_pub.publish(command)


def main() -> None:
    rclpy.init()
    node = IdealController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
