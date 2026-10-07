"""Run real new_map_planner requests against the loopback stack."""

from __future__ import annotations

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from new_map_mission.state import MissionLayout, PlanSpec, StagedMission
from new_map_planner.srv import PlanRoute
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml


def _quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(0.5 * yaw_rad)
    quaternion.w = math.cos(0.5 * yaw_rad)
    return quaternion


def _yaw_from_quaternion(quaternion: Quaternion) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def _parse_modes(text: str) -> list[str]:
    modes = [mode.strip() for mode in text.split(",") if mode.strip()]
    if not modes:
        raise ValueError("mission mode list must not be empty")
    allowed = {
        "targets",
        "coverage",
        "mission",
        "layer1",
        "layer2",
        "layer3",
    }
    invalid = [mode for mode in modes if mode not in allowed]
    if invalid:
        raise ValueError(f"unsupported mission mode(s): {', '.join(invalid)}")
    return modes


def _parse_ids(text: str) -> list[str]:
    return list(
        dict.fromkeys(value.strip() for value in text.split(",") if value.strip())
    )


def _parse_status_topics(text: str, fallback: str) -> list[str]:
    topics = [topic.strip() for topic in text.split(",") if topic.strip()]
    if not topics:
        topics = [fallback.strip()]
    if any(not topic for topic in topics):
        raise ValueError("navigator status topics must not be empty")
    return list(dict.fromkeys(topics))


class LoopbackMissionStarter(Node):
    """Run planner stages and activate each returned route."""

    def __init__(self) -> None:
        super().__init__("vehicle_loopback_mission")
        self._declare_parameters()
        self._arena_config = str(self.get_parameter("arena_config").value)
        self._modes = _parse_modes(str(self.get_parameter("mode").value))
        self._target_ids = _parse_ids(str(self.get_parameter("target_ids").value))
        self._required_lane_ids = _parse_ids(
            str(self.get_parameter("required_lane_ids").value)
        )
        with Path(self._arena_config).open(encoding="utf-8") as stream:
            self._staged_mission = StagedMission(
                MissionLayout.from_config(yaml.safe_load(stream))
            )
        self._runs_staged_mission = self._modes == [
            "layer1",
            "layer2",
            "layer3",
        ]
        if self._runs_staged_mission:
            self._staged_mission.start(self._target_ids or None)
        self._client = self.create_client(
            PlanRoute,
            str(self.get_parameter("planner_service").value),
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        primary_status_topic = str(self.get_parameter("navigator_status_topic").value)
        self._navigator_status_topics = _parse_status_topics(
            str(self.get_parameter("navigator_status_topics").value),
            primary_status_topic,
        )
        self._navigator_status_subscriptions = [
            self.create_subscription(
                String,
                topic,
                lambda message, source=topic: self._navigator_status_callback(
                    source, message
                ),
                10,
            )
            for topic in self._navigator_status_topics
        ]
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
        self._reached_navigators: set[str] = set()
        self._completed_stages = 0
        self._latest_odom: Odometry | None = None
        self._completed_lane_ids: list[str] = []
        self._active_covered_lane_ids: list[str] = []
        self._active_visit_order: list[str] = []
        self._active_plan_complete = False
        self._active_spec: PlanSpec | None = None
        delay_s = max(0.0, float(self.get_parameter("request_delay_s").value))
        self._timer = self.create_timer(delay_s if delay_s > 0.0 else 0.1, self._tick)
        self._publish_status("waiting_for_planner")

    def _declare_parameters(self) -> None:
        default_config = (
            Path(get_package_share_directory("new_map_planner"))
            / "config"
            / "arena.yaml"
        )
        declarations = {
            "arena_config": str(default_config),
            "planner_service": "/new_map_planner/plan",
            "mode": "layer1,layer2,layer3",
            "target_ids": "1,2,3,4,5,6,7,8,9,10,11,12",
            "required_lane_ids": "",
            "return_home": True,
            "allow_partial": True,
            "activate_navigation": True,
            "request_delay_s": 1.0,
            "status_topic": "/vehicle_loopback_sim/mission_status",
            "navigator_status_topic": "/waypoint_navigation/status",
            "navigator_status_topics": "",
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
            f"requested loopback {mode} plan from {Path(self._arena_config).name} "
            "with activation enabled"
        )

    def _make_request(self) -> PlanRoute.Request:
        request = PlanRoute.Request()
        request.start = Pose()
        if self._latest_odom is not None:
            odom_pose = self._latest_odom.pose.pose
            request.start.position.x = float(odom_pose.position.x)
            request.start.position.y = float(odom_pose.position.y)
            request.start.orientation = _quaternion_from_yaw(
                _yaw_from_quaternion(odom_pose.orientation)
            )
        else:
            request.start.orientation.w = 1.0
        if self._runs_staged_mission:
            spec = self._staged_mission.plan_spec()
            expected_mode = self._modes[self._stage_index]
            if spec.mode != expected_mode:
                raise RuntimeError(
                    f"staged mission expected {expected_mode}, got {spec.mode}"
                )
        else:
            spec = PlanSpec(
                mode=self._modes[self._stage_index],
                target_ids=tuple(self._target_ids),
                required_lane_ids=tuple(self._required_lane_ids),
                completed_lane_ids=tuple(self._completed_lane_ids),
                return_home=bool(self.get_parameter("return_home").value),
                allow_partial=bool(self.get_parameter("allow_partial").value),
            )
        request.mode = spec.mode
        request.target_ids = list(spec.target_ids)
        request.required_lane_ids = list(spec.required_lane_ids)
        request.completed_lane_ids = list(spec.completed_lane_ids)
        request.return_home = spec.return_home
        request.allow_partial = spec.allow_partial
        request.activate = bool(self.get_parameter("activate_navigation").value)
        self._active_spec = spec
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
        plan = response.plan
        if not plan.navigation_path.poses:
            if (
                self._runs_staged_mission
                and self._active_spec is not None
                and plan.complete
            ):
                self._staged_mission.commit_empty(self._active_spec, True)
                self._completed_stages += 1
                self._stage_index += 1
                self._publish_status(f"stage_empty:{mode}")
                self._request_current_stage()
                return
            self._publish_status(f"plan_failed:{mode}:empty_route")
            self.get_logger().error("new map planner returned an empty route")
            return
        state = "activated" if response.activated else "planned_only"
        self._publish_status(
            f"{state}:{mode}:poses={len(plan.navigation_path.poses)} "
            f"length={plan.length_m:.2f}:complete={str(plan.complete).lower()}"
        )
        self.get_logger().info(
            f"loopback {mode} plan {state}: "
            f"{len(plan.navigation_path.poses)} poses, "
            f"{plan.length_m:.2f} m, {response.message}"
        )
        if not response.activated:
            return
        self._active_covered_lane_ids = list(plan.covered_lane_ids)
        self._active_visit_order = list(plan.visit_order)
        self._active_plan_complete = bool(plan.complete)
        self._reached_navigators.clear()
        self._waiting_for_goal = True

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_publisher.publish(message)

    def _odom_callback(self, message: Odometry) -> None:
        self._latest_odom = message

    def _navigator_status_callback(self, source: str, message: String) -> None:
        if message.data != "GOAL_REACHED" or not self._waiting_for_goal:
            return
        self._reached_navigators.add(source)
        if len(self._reached_navigators) < len(self._navigator_status_topics):
            self._publish_status(
                "waiting_for_controllers:"
                f"reached={len(self._reached_navigators)}/"
                f"{len(self._navigator_status_topics)}"
            )
            return
        mode = self._modes[self._stage_index]
        covered_lane_ids = list(self._active_covered_lane_ids)
        self._completed_lane_ids = list(
            dict.fromkeys(self._completed_lane_ids + covered_lane_ids)
        )
        plan_complete = self._active_plan_complete
        active_spec = self._active_spec
        visit_order = self._active_visit_order
        self._active_covered_lane_ids = []
        self._active_visit_order = []
        self._active_plan_complete = False
        self._active_spec = None
        self._waiting_for_goal = False
        if self._runs_staged_mission:
            if active_spec is None:
                self._publish_status("mission_error:missing_stage_request")
                self.get_logger().error(
                    "staged route completed without an active request"
                )
                return
            self._staged_mission.commit(
                active_spec,
                visit_order,
                covered_lane_ids,
                plan_complete,
            )
            self._completed_lane_ids = sorted(self._staged_mission.confirmed_lanes)
            stage_complete = (
                self._staged_mission.finished
                or self._staged_mission.current_stage() != mode
            )
        else:
            stage_complete = plan_complete
        if stage_complete:
            self._completed_stages += 1
            self._stage_index += 1
            self._publish_status(f"stage_complete:{mode}")
        else:
            self._publish_status(f"partial_complete:{mode}:replanning")
        self.get_logger().info(
            f"loopback {mode} stage reached GOAL_REACHED on "
            f"{len(self._navigator_status_topics)} controller(s)"
        )
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
