from __future__ import annotations

import queue
import threading
import time
import math
from typing import Any

import rclpy
from geometry_msgs.msg import Point32, Polygon, Pose
from mission_control_interfaces.msg import MissionState, MotionHoldState
from mission_control_interfaces.srv import ControlMission, SetMotionHold
from nav_msgs.msg import Odometry
from new_map_planner.msg import RoutePlan
from new_map_planner.srv import PlanRoute
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String, UInt64

from .model import PlanSelection, VehiclePose, quaternion_yaw


class PlannerRosClient(Node):
    def __init__(self, events: queue.Queue[tuple[str, Any]]) -> None:
        super().__init__("new_map_planner_ui")
        self.events = events
        self.plan_client = self.create_client(PlanRoute, "/new_map_planner/plan")
        self.hold_client = self.create_client(
            SetMotionHold, "/navigation_mission/set_motion_hold"
        )
        self.mission_client = self.create_client(
            ControlMission, "/navigation_mission/control"
        )
        durable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry, "/odometry/fused", self._on_odometry, qos_profile_sensor_data
        )
        self.create_subscription(
            String, "/waypoint_navigation/status", self._on_status, durable
        )
        self.create_subscription(
            UInt64, "/waypoint_navigation/route_ack", self._on_ack, durable
        )
        self.create_subscription(
            MotionHoldState,
            "/waypoint_navigation/motion_hold_state",
            self._on_hold,
            durable,
        )
        self.create_subscription(
            MissionState,
            "/navigation_mission/state",
            self._on_mission_state,
            durable,
        )
        self.create_subscription(
            RoutePlan,
            "/new_map_planner/route_plan",
            self._on_route_plan,
            durable,
        )
        self.create_subscription(
            Range,
            "/obstacle/front_range",
            self._on_front_range,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/obstacle/front_blocked",
            self._on_front_blocked,
            qos_profile_sensor_data,
        )
        self.plan_future: Any = None
        self.hold_future: Any = None
        self.mission_future: Any = None
        self._last_stamp_ns = 0

    def _emit(self, kind: str, value: Any) -> None:
        self.events.put((kind, value))

    def _on_odometry(self, message: Odometry) -> None:
        if message.header.frame_id != "map" or message.child_frame_id != "base_link":
            return
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        current_ns = self.get_clock().now().nanoseconds
        age_ns = current_ns - stamp_ns
        if (
            stamp_ns <= 0
            or stamp_ns <= self._last_stamp_ns
            or age_ns > 600_000_000
            or age_ns < -100_000_000
        ):
            return
        yaw = quaternion_yaw(message.pose.pose.orientation)
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        if yaw is None or not all(math.isfinite(value) for value in (x, y)):
            return
        self._last_stamp_ns = stamp_ns
        self._emit("odometry", VehiclePose(x, y, yaw, time.monotonic()))

    def _on_status(self, message: String) -> None:
        self._emit("navigation_status", str(message.data))

    def _on_ack(self, message: UInt64) -> None:
        self._emit("route_ack", int(message.data))

    def _on_hold(self, message: MotionHoldState) -> None:
        self._emit("hold_state", (bool(message.held), tuple(message.active_sources)))

    def _on_mission_state(self, message: MissionState) -> None:
        self._emit("mission_state", message)

    def _on_route_plan(self, message: RoutePlan) -> None:
        self._emit("planner_plan", message)

    def _on_front_range(self, message: Range) -> None:
        measured_range = float(message.range)
        if math.isfinite(measured_range) and measured_range > 0.0:
            self._emit("front_range", (measured_range, time.monotonic()))

    def _on_front_blocked(self, message: Bool) -> None:
        self._emit("front_blocked", (bool(message.data), time.monotonic()))

    @property
    def planning(self) -> bool:
        return self.plan_future is not None

    def ready(self) -> bool:
        return self.plan_client.service_is_ready()

    def hold_ready(self) -> bool:
        return self.hold_client.service_is_ready()

    def mission_ready(self) -> bool:
        return self.mission_client.service_is_ready()

    def request_plan(
        self,
        pose: VehiclePose,
        spec: PlanSelection,
        dynamic_obstacles: list[list[tuple[float, float]]],
        activate: bool,
    ) -> None:
        if self.plan_future is not None:
            raise RuntimeError("已有规划请求正在处理")
        if not self.ready():
            raise RuntimeError("规划服务未就绪")
        request = PlanRoute.Request()
        request.start = Pose()
        request.start.position.x = pose.x
        request.start.position.y = pose.y
        request.start.orientation.z = math.sin(pose.yaw * 0.5)
        request.start.orientation.w = math.cos(pose.yaw * 0.5)
        request.mode = spec.mode
        request.target_ids = list(spec.target_ids)
        request.required_lane_ids = list(spec.required_lane_ids)
        request.completed_lane_ids = list(spec.completed_lane_ids)
        request.blocked_lane_ids = list(spec.blocked_lane_ids)
        # A manually selected obstacle set is a new immutable snapshot.  The
        # mission manager uses this revision to reject stale route plans.
        request.obstacle_revision = 1 if spec.blocked_lane_ids else 0
        request.dynamic_obstacles = [
            Polygon(points=[Point32(x=float(x), y=float(y)) for x, y in polygon])
            for polygon in dynamic_obstacles
        ]
        request.return_home = spec.return_home
        request.allow_partial = spec.allow_partial
        request.activate = activate
        self.plan_future = self.plan_client.call_async(request)
        self.plan_future.add_done_callback(self._finish_plan)

    def request_mission(
        self, command: int, target_ids: list[str], blocked_lane_ids: list[str] | None = None
    ) -> None:
        if self.mission_future is not None:
            raise RuntimeError("任务命令正在处理")
        if not self.mission_ready():
            raise RuntimeError("自主任务节点未就绪")
        request = ControlMission.Request()
        request.command = int(command)
        request.target_ids = list(target_ids)
        request.blocked_lane_ids = list(blocked_lane_ids or [])
        self.mission_future = self.mission_client.call_async(request)
        self.mission_future.add_done_callback(self._finish_mission)

    def _finish_mission(self, future: Any) -> None:
        try:
            self._emit("mission_response", future.result())
        except Exception as exception:
            self._emit("error", f"任务命令失败：{exception}")
        finally:
            self.mission_future = None

    def _finish_plan(self, future: Any) -> None:
        try:
            self._emit("plan_response", future.result())
        except Exception as exception:
            self._emit("error", f"规划服务调用失败：{exception}")
        finally:
            self.plan_future = None

    def request_hold(self, hold: bool) -> None:
        if self.hold_future is not None:
            raise RuntimeError("驻停请求正在处理")
        if not self.hold_ready():
            raise RuntimeError("导航驻停服务未就绪")
        request = SetMotionHold.Request()
        request.source = "new_map_planner_ui"
        request.hold = bool(hold)
        request.reason = "operator requested hold" if hold else "operator released hold"
        self.hold_future = self.hold_client.call_async(request)
        self.hold_future.add_done_callback(self._finish_hold)

    def _finish_hold(self, future: Any) -> None:
        try:
            response = future.result()
            self._emit(
                "hold_response",
                (
                    bool(response.success),
                    bool(response.motion_held),
                    tuple(response.active_sources),
                    response.message,
                ),
            )
        except Exception as exception:
            self._emit("error", f"驻停服务调用失败：{exception}")
        finally:
            self.hold_future = None


class RosRuntime:
    def __init__(self, events: queue.Queue[tuple[str, Any]]) -> None:
        rclpy.init(args=None)
        self.node = PlannerRosClient(events)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.thread.join(timeout=1.0)
