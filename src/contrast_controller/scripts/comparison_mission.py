#!/usr/bin/env python3
"""A/B route-mode truth checkpoints, not physical RFID localization validation."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import re
from time import monotonic

from mission_control_interfaces.srv import ConfirmRfidCheckpoint, SetMotionHold
from new_map_planner.msg import RoutePlan
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String, UInt64
from vehicle_loopback_sim.mission import LoopbackMissionStarter


HOLD_SOURCE = "contrast_rfid"
CONTROLLER_NODES = {
    "production": "/comparison/production/waypoint_navigator",
    "contrast": "/comparison/contrast/contrast_controller",
}


def parse_checkpoint_status(text: str) -> int | None:
    match = re.fullmatch(r"(?:RFID_SCANNING|WAITING_FOR_RFID):([0-9]+)", text)
    return int(match.group(1)) if match else None


def stamp_ns(stamp) -> int:
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid ROS timestamp")
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class CheckpointPlan:
    def __init__(self, plan) -> None:
        self.plan_id = int(plan.plan_id)
        self.navigation_path = plan.navigation_path
        self.path_stamp_ns = stamp_ns(self.navigation_path.header.stamp)
        self.route_id = self.path_stamp_ns
        self.waypoint_supernode_ids = tuple(plan.waypoint_supernode_ids)
        self.points = tuple(
            (pose.pose.position.x, pose.pose.position.y)
            for pose in self.navigation_path.poses
        )
        if self.route_id == 0 or not self.points:
            raise ValueError("checkpoint route requires a stamped, non-empty Path")
        if len(self.points) != len(self.waypoint_supernode_ids):
            raise ValueError("supernode metadata must match navigation_path indices")
        if not all(math.isfinite(value) for point in self.points for value in point):
            raise ValueError("checkpoint route contains a non-finite position")

    def checkpoint(self, index: int | None) -> tuple | None:
        if index is None or not 0 <= index < len(self.points):
            return None
        supernode_id = self.waypoint_supernode_ids[index]
        if not supernode_id.strip():
            return None
        return self.route_id, index, supernode_id


def motion_is_stationary(twist, linear_epsilon: float, angular_epsilon: float) -> bool:
    linear = math.hypot(twist.linear.x, twist.linear.y, twist.linear.z)
    angular = math.hypot(twist.angular.x, twist.angular.y, twist.angular.z)
    return linear <= linear_epsilon and angular <= angular_epsilon


@dataclass
class StationaryHold:
    since: float | None = None
    last_sample: float | None = None

    def reset(self) -> None:
        self.since = self.last_sample = None

    def observe(self, now: float, stationary: bool, max_gap_s: float) -> None:
        if not stationary or not math.isfinite(now):
            self.reset()
            return
        if self.last_sample is not None and now <= self.last_sample:
            self.reset()
            return
        if self.last_sample is None or now - self.last_sample > max_gap_s:
            self.since = now
        self.last_sample = now

    def ready(self, now: float, hold_s: float, max_gap_s: float) -> bool:
        return (
            self.since is not None
            and self.last_sample is not None
            and 0.0 <= now - self.last_sample <= max_gap_s
            and self.last_sample - self.since >= hold_s
        )


@dataclass
class ControllerCheckpoint:
    name: str
    status_topic: str
    hold_client: object
    confirm_client: object
    ack_route_id: int = 0
    status_route_id: int = 0
    status: str = ""
    saw_activity: bool = False
    truth: object | None = None
    truth_stamp_ns: int = 0
    truth_received_s: float = 0.0
    key: tuple | None = None
    held: bool = False
    releasing: bool = False
    pending: object | None = None
    retry_at: float = 0.0
    completed: set = field(default_factory=set)
    dwell: StationaryHold = field(default_factory=StationaryHold)


class ComparisonMissionStarter(LoopbackMissionStarter):
    def _declare_parameters(self) -> None:
        super()._declare_parameters()
        self.declare_parameters("", [
            ("controllers", "both"),
            ("external_mission_manager", False),
            ("checkpoint_hold_s", 1.0),
            ("checkpoint_radius_m", 0.05),
            ("checkpoint_linear_epsilon", 0.005),
            ("checkpoint_angular_epsilon", 0.02),
            ("checkpoint_pose_timeout_s", 0.40),
        ])

    def __init__(self) -> None:
        super().__init__()
        self._checkpoint_config = {
            name: float(self.get_parameter(name).value)
            for name in (
                "checkpoint_hold_s", "checkpoint_radius_m",
                "checkpoint_linear_epsilon", "checkpoint_angular_epsilon",
                "checkpoint_pose_timeout_s",
            )
        }
        if any(not math.isfinite(value) or value <= 0.0
               for value in self._checkpoint_config.values()):
            raise ValueError("checkpoint parameters must be finite and positive")
        self._external_manager = bool(
            self.get_parameter("external_mission_manager").value)
        if self._external_manager:
            self._timer.cancel()
            self._waiting_for_goal = True
        self._checkpoint_plan: CheckpointPlan | None = None
        self._stopping = False
        for subscription in self._navigator_status_subscriptions:
            self.destroy_subscription(subscription)
        self.destroy_subscription(self._odom_subscription)
        self._navigator_status_subscriptions = []
        self._truth_subscriptions = []
        self._ack_subscriptions = []
        self._controllers = {}
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        selected = str(self.get_parameter("controllers").value)
        if selected not in ("both", "production", "contrast"):
            raise ValueError("controllers must be both, production or contrast")
        self._planning_controller = "contrast" if selected == "contrast" else "production"
        for name, node_name in CONTROLLER_NODES.items():
            if selected != "both" and name != selected:
                continue
            root = f"/comparison/{name}"
            status_topic = f"{root}/navigator_status"
            side = ControllerCheckpoint(
                name, status_topic,
                self.create_client(SetMotionHold, f"{node_name}/set_motion_hold"),
                self.create_client(ConfirmRfidCheckpoint,
                                   f"{node_name}/confirm_rfid_checkpoint"),
            )
            self._controllers[status_topic] = side
            self._navigator_status_subscriptions.append(self.create_subscription(
                String, status_topic,
                lambda message, source=status_topic:
                    self._navigator_status_callback(source, message), durable))
            self._truth_subscriptions.append(self.create_subscription(
                Odometry, f"{root}/odometry/fused",
                lambda message, current=side: self._truth_callback(current, message),
                qos_profile_sensor_data))
            self._ack_subscriptions.append(self.create_subscription(
                UInt64, f"{root}/route_ack",
                lambda message, current=side: self._ack_callback(current, message),
                durable))
        self._navigator_status_topics = list(self._controllers)
        if self._external_manager:
            self._plan_subscription = self.create_subscription(
                RoutePlan, "/new_map_planner/route_plan",
                self._external_plan_callback, durable)
        self._checkpoint_timer = self.create_timer(0.05, self._checkpoint_tick)
        self.get_logger().warning(
            "A/B route-mode truth confirmation only; no physical RFID, pose "
            "correction or announcements. This is not RFID localization validation.")

    def _external_plan_callback(self, plan) -> None:
        if self._stopping or not plan.navigation_path.poses:
            return
        candidate = CheckpointPlan(plan)
        if (self._checkpoint_plan is not None
                and candidate.route_id <= self._checkpoint_plan.route_id):
            return
        self._record_checkpoint_plan(candidate)

    def _handle_response(self, future) -> None:
        if self._stopping:
            return
        try:
            response = future.result()
            candidate = (CheckpointPlan(response.plan)
                         if response.success and response.plan.navigation_path.poses
                         else None)
        except Exception as error:
            self._planning = False
            self._publish_status(f"checkpoint_plan_error:{error}")
            self.get_logger().error(f"cannot record comparison route: {error}")
            return
        if (candidate is not None and self._checkpoint_plan is not None
                and candidate.route_id <= self._checkpoint_plan.route_id):
            self.get_logger().warning("Ignoring stale comparison planner response")
            return
        super()._handle_response(future)
        if not response.success:
            return
        self._record_checkpoint_plan(candidate)

    def _record_checkpoint_plan(self, candidate) -> None:
        self._checkpoint_plan = candidate
        for side in self._controllers.values():
            side.status = ""
            side.status_route_id = 0
            side.saw_activity = False
            side.completed.clear()
            side.dwell.reset()
        if candidate is not None:
            self.get_logger().info(
                f"comparison plan_id={candidate.plan_id} "
                f"route_id(path_stamp_ns)={candidate.route_id}")

    def _ack_callback(self, side, message) -> None:
        if int(message.data) <= side.ack_route_id:
            return
        side.ack_route_id = int(message.data)
        side.status_route_id = 0
        side.status = ""
        side.saw_activity = False
        side.dwell.reset()

    def _navigator_status_callback(self, source: str, message: String) -> None:
        side = self._controllers[source]
        plan = self._checkpoint_plan
        if plan is None or self._planning or side.ack_route_id != plan.route_id:
            return
        side.status = message.data
        side.status_route_id = plan.route_id
        if message.data not in {"GOAL_REACHED", "IDLE", "WAITING_FOR_ROUTE"}:
            side.saw_activity = True
        self._checkpoint_tick()

    def _truth_callback(self, side, message) -> None:
        try:
            timestamp = stamp_ns(message.header.stamp)
        except ValueError:
            side.dwell.reset()
            return
        if timestamp <= side.truth_stamp_ns:
            return
        side.truth_stamp_ns = timestamp
        side.truth = message
        side.truth_received_s = monotonic()
        if side.name == self._planning_controller:
            super()._odom_callback(message)
        if side.held and side.key is not None:
            side.dwell.observe(
                side.truth_received_s,
                self._truth_near(side, side.key[1], stationary=True),
                self._checkpoint_config["checkpoint_pose_timeout_s"])

    def _truth_near(self, side, index: int, stationary: bool = False) -> bool:
        plan = self._checkpoint_plan
        message = side.truth
        timeout = self._checkpoint_config["checkpoint_pose_timeout_s"]
        if (plan is None or message is None or not 0 <= index < len(plan.points)
                or side.ack_route_id != plan.route_id
                or side.truth_stamp_ns < plan.route_id
                or monotonic() - side.truth_received_s > timeout
                or message.header.frame_id != plan.navigation_path.header.frame_id):
            return False
        age_ns = self.get_clock().now().nanoseconds - side.truth_stamp_ns
        if not 0 <= age_ns <= timeout * 1_000_000_000:
            return False
        x, y = plan.points[index]
        position = message.pose.pose.position
        near = math.hypot(position.x - x, position.y - y) <= (
            self._checkpoint_config["checkpoint_radius_m"])
        return near and (not stationary or motion_is_stationary(
            message.twist.twist,
            self._checkpoint_config["checkpoint_linear_epsilon"],
            self._checkpoint_config["checkpoint_angular_epsilon"]))

    def _checkpoint_tick(self) -> None:
        plan = self._checkpoint_plan
        now = monotonic()
        for side in self._controllers.values():
            route_active = (not self._stopping and not self._planning
                            and self._waiting_for_goal and plan is not None
                            and side.ack_route_id == plan.route_id)
            if side.key is not None and (
                    not route_active or side.key[0] != plan.route_id):
                side.releasing = True
                side.dwell.reset()
            if side.pending is not None or now < side.retry_at:
                continue
            if side.key is None and route_active:
                key = plan.checkpoint(parse_checkpoint_status(side.status))
                if (key is not None and key not in side.completed
                        and side.status_route_id == plan.route_id
                        and self._truth_near(side, key[1])):
                    side.key = key
                    side.dwell.reset()
            if side.key is not None:
                if side.releasing:
                    self._call_service(side, "release")
                elif not side.held and self._truth_near(side, side.key[1]):
                    self._call_service(side, "hold")
                elif (side.held and self._truth_near(side, side.key[1], stationary=True)
                      and side.dwell.ready(
                          now, self._checkpoint_config["checkpoint_hold_s"],
                          self._checkpoint_config["checkpoint_pose_timeout_s"])):
                    self._call_service(side, "confirm")
            elif (not self._external_manager and route_active
                  and side.status_route_id == plan.route_id
                  and side.saw_activity and side.status == "GOAL_REACHED"
                  and self._truth_near(side, len(plan.points) - 1, stationary=True)):
                super()._navigator_status_callback(side.status_topic, String(
                    data="GOAL_REACHED"))

    def _call_service(self, side, operation: str) -> None:
        client = side.confirm_client if operation == "confirm" else side.hold_client
        if not client.service_is_ready():
            return
        if operation == "confirm":
            request = ConfirmRfidCheckpoint.Request()
            request.route_id, request.waypoint_index, request.supernode_id = side.key
        else:
            request = SetMotionHold.Request()
            request.source = HOLD_SOURCE
            request.hold = operation == "hold"
            request.reason = "A/B route-mode truth checkpoint (not RFID validation)"
        key = side.key
        side.pending = client.call_async(request)
        side.pending.add_done_callback(
            lambda future: self._service_response(side, key, operation, future))

    def _service_response(self, side, key, operation, future) -> None:
        if side.pending is not future or side.key != key:
            return
        side.pending = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f"{side.name} {operation} failed: {error}")
            side.retry_at = monotonic() + 0.5
            return
        if operation == "hold":
            side.held = (response.success and response.motion_held
                         and HOLD_SOURCE in response.active_sources)
            side.dwell.reset()
            success = side.held
        elif operation == "release":
            success = response.success and HOLD_SOURCE not in response.active_sources
            if success:
                side.key = None
                side.held = side.releasing = False
                side.dwell.reset()
        else:
            success = response.accepted
            if success:
                plan = self._checkpoint_plan
                if (plan is not None and key[0] == plan.route_id
                        and side.ack_route_id == plan.route_id and not self._planning):
                    side.completed.add(key)
                    self.get_logger().info(
                        f"{side.name} truth checkpoint accepted: route={key[0]} "
                        f"waypoint={key[1]} supernode={key[2]}")
                side.releasing = True
        if not success:
            self.get_logger().warning(
                f"{side.name} {operation} rejected: {response.message}")
            side.retry_at = monotonic() + 0.5

    def stop_checkpoints(self) -> None:
        self._stopping = True
        self._timer.cancel()
        self._checkpoint_tick()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ComparisonMissionStarter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_checkpoints()
        deadline = monotonic() + 1.0
        while (rclpy.ok() and monotonic() < deadline
               and any(side.key is not None for side in node._controllers.values())):
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
