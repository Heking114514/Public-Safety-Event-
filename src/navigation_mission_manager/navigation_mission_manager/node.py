from __future__ import annotations

from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PolygonStamped
from mission_control_interfaces.action import ReportEvent
from mission_control_interfaces.msg import EventCandidate, MissionState, MotionHoldState
from mission_control_interfaces.srv import (
    ContinueRetreat,
    ControlMission,
    SetMotionHold,
)
from nav_msgs.msg import Odometry, Path as NavPath
from new_map_planner.srv import PlanRoute
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String, UInt64
from vehicle_config.physical import load_vehicle_physical_config
import yaml

from .events import EventRegistry
from .environment import EnvironmentMixin
from .model import MissionLayout, PlanSpec, StagedMission
from .node_support import NodeSupportMixin
from .obstacles import Polygon
from .recovery import RecoveryMixin
from .ros_support import (
    ActiveRoute,
    VehiclePose,
    polygon_message,
    pose_message,
)


STATE_NAMES = {
    MissionState.IDLE: "IDLE",
    MissionState.WAIT_READY: "WAIT_READY",
    MissionState.PLANNING: "PLANNING",
    MissionState.WAIT_ROUTE_ACK: "WAIT_ROUTE_ACK",
    MissionState.EXECUTING: "EXECUTING",
    MissionState.RECOVERING_OBSTACLE: "RECOVERING_OBSTACLE",
    MissionState.REPLANNING: "REPLANNING",
    MissionState.WAIT_REPORTER: "WAIT_REPORTER",
    MissionState.REPORTING: "REPORTING",
    MissionState.PAUSED: "PAUSED",
    MissionState.FAULT: "FAULT",
    MissionState.COMPLETE: "COMPLETE",
}


def _parse_ids(text: str) -> list[str]:
    return list(dict.fromkeys(item.strip() for item in text.split(",") if item.strip()))


class MissionManager(RecoveryMixin, EnvironmentMixin, NodeSupportMixin, Node):
    """Own the competition mission while planner and controller remain stateless."""

    def __init__(self) -> None:
        super().__init__("navigation_mission_manager")
        self._declare_parameters()
        self._load_configuration()
        self._create_interfaces()

        self._state = MissionState.WAIT_READY
        self._state_names = STATE_NAMES
        self._detail = "等待规划器、控制器和融合里程计"
        self._pose: VehiclePose | None = None
        self._last_odom_stamp_ns = 0
        self._motion_held = False
        self._navigator_status = "UNKNOWN"
        self._active_route: ActiveRoute | None = None
        self._ack_cache: set[int] = set()
        self._ack_deadline = 0.0
        self._planning_future: Any = None
        self._plan_generation = 0
        self._plan_due_at: float | None = None
        self._plan_retries = 0
        self._no_progress_routes = 0
        self._replanning = False
        self._completed_stages = 0
        self._retreat_future: Any = None
        self._retreat_generation = 0
        self._retreat_extensions = 0
        self._awaiting_retreat = False
        self._obstacle_recovery_active = False
        self._completed_route_retreat_available = False
        self._ordinary_nodes_since_task = 1

        self._front_blocked = False
        self._latest_range: tuple[float, float] | None = None
        self._pending_obstacle: tuple[Polygon, float] | None = None
        self._dynamic_obstacles: list[Polygon] = []
        self._dynamic_obstacles_updated_at: float | None = None

        self._report_goal_future: Any = None
        self._report_goal_handle: Any = None
        self._report_result_future: Any = None
        self._report_generation = 0
        self._report_wait_started: float | None = None
        self._report_hold_requested = False

        self._auto_start_pending = bool(self.get_parameter("auto_start").value)
        self._timer = self.create_timer(0.10, self._tick)
        self._publish_state()

    def _declare_parameters(self) -> None:
        planner_share = Path(get_package_share_directory("new_map_planner"))
        physical = load_vehicle_physical_config()
        defaults = {
            "map_file": str(planner_share / "config" / "arena.yaml"),
            "planner_service": "/new_map_planner/plan",
            "odom_topic": "/odometry/fused",
            "navigator_status_topic": "/waypoint_navigation/status",
            "route_ack_topic": "/waypoint_navigation/route_ack",
            "route_input_topic": "/waypoint_navigation/route_input",
            "motion_hold_service": "/waypoint_navigator/set_motion_hold",
            "continue_retreat_service": (
                "/waypoint_navigator/continue_obstacle_retreat"
            ),
            "motion_hold_state_topic": "/waypoint_navigation/motion_hold_state",
            "front_obstacle_topic": "/obstacle/front_blocked",
            "front_range_topic": "/obstacle/front_range",
            "dynamic_obstacle_topic": "/perception/dynamic_obstacle",
            "event_candidate_topic": "/perception/event_candidate",
            "reported_event_topic": "/navigation_mission/reported_event",
            "mission_state_topic": "/navigation_mission/state",
            "status_text_topic": "/navigation_mission/status_text",
            "control_service": "/navigation_mission/control",
            "report_action": "/event_reporter/report",
            "auto_start": False,
            "target_ids": "1,2,3,4,5,6,7,8,9,10,11,12",
            "odom_timeout_s": 0.6,
            "route_ack_timeout_s": 3.0,
            "replan_delay_s": 0.15,
            "plan_retry_delay_s": 1.0,
            "max_plan_retries": 3,
            "retreat_step_m": 0.40,
            "max_retreat_extensions": 4,
            "max_no_progress_routes": 2,
            "obstacle_sensor_forward_offset_m": physical.camera_x_m,
            "obstacle_observation_timeout_s": 5.0,
            "obstacle_range_sync_s": 0.5,
            "obstacle_width_m": 0.24,
            "obstacle_depth_m": 0.12,
            "obstacle_merge_distance_m": 0.18,
            "max_dynamic_obstacles": 20,
            "dynamic_obstacle_retention_s": 120.0,
            "event_confidence_threshold": 0.70,
            "event_merge_distance_m": 0.35,
            "report_requires_hold": True,
            "report_server_timeout_s": 5.0,
            "max_report_retries": 2,
        }
        self.declare_parameters("", list(defaults.items()))

    def _load_configuration(self) -> None:
        map_file = Path(str(self.get_parameter("map_file").value))
        with map_file.open(encoding="utf-8") as stream:
            layout = MissionLayout.from_config(yaml.safe_load(stream))
            self._mission = StagedMission(layout)
        self._default_targets = _parse_ids(str(self.get_parameter("target_ids").value))
        self._events = EventRegistry(
            float(self.get_parameter("event_confidence_threshold").value),
            float(self.get_parameter("event_merge_distance_m").value),
        )

    def _create_interfaces(self) -> None:
        durable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._planner = self.create_client(
            PlanRoute, str(self.get_parameter("planner_service").value)
        )
        self._hold = self.create_client(
            SetMotionHold, str(self.get_parameter("motion_hold_service").value)
        )
        self._retreat_client = self.create_client(
            ContinueRetreat,
            str(self.get_parameter("continue_retreat_service").value),
        )
        self._reporter = ActionClient(
            self, ReportEvent, str(self.get_parameter("report_action").value)
        )
        self._state_publisher = self.create_publisher(
            MissionState, str(self.get_parameter("mission_state_topic").value), durable
        )
        self._text_publisher = self.create_publisher(
            String, str(self.get_parameter("status_text_topic").value), durable
        )
        self._reported_publisher = self.create_publisher(
            EventCandidate,
            str(self.get_parameter("reported_event_topic").value),
            durable,
        )
        self._route_publisher = self.create_publisher(
            NavPath,
            str(self.get_parameter("route_input_topic").value),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("navigator_status_topic").value),
            self._on_navigator_status,
            durable,
        )
        self.create_subscription(
            UInt64,
            str(self.get_parameter("route_ack_topic").value),
            self._on_route_ack,
            durable,
        )
        self.create_subscription(
            MotionHoldState,
            str(self.get_parameter("motion_hold_state_topic").value),
            self._on_hold_state,
            durable,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("front_obstacle_topic").value),
            self._on_front_blocked,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range,
            str(self.get_parameter("front_range_topic").value),
            self._on_front_range,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PolygonStamped,
            str(self.get_parameter("dynamic_obstacle_topic").value),
            self._on_obstacle_polygon,
            10,
        )
        self.create_subscription(
            EventCandidate,
            str(self.get_parameter("event_candidate_topic").value),
            self._on_event_candidate,
            10,
        )
        self.create_service(
            ControlMission,
            str(self.get_parameter("control_service").value),
            self._on_control,
        )

    def _tick(self) -> None:
        now = time.monotonic()
        self._expire_dynamic_obstacles(now)
        if self._auto_start_pending and self._ready():
            self._auto_start_pending = False
            self._start_mission(self._default_targets)
        if self._plan_due_at is not None and now >= self._plan_due_at:
            self._plan_due_at = None
            self._request_plan()
        if (
            self._state == MissionState.WAIT_ROUTE_ACK
            and self._ack_deadline > 0.0
            and now >= self._ack_deadline
        ):
            self._fault("路线 ACK 超时")
        self._service_event_reporting(now)
        if self._state == MissionState.WAIT_READY and self._ready():
            self._set_state(MissionState.IDLE, "系统就绪")
        self._publish_state()

    def _ready(self) -> bool:
        timeout = float(self.get_parameter("odom_timeout_s").value)
        return (
            self._planner.service_is_ready()
            and self._hold.service_is_ready()
            and self._retreat_client.service_is_ready()
            and self._pose is not None
            and self._pose.fresh(timeout)
        )

    def _on_control(
        self, request: ControlMission.Request, response: ControlMission.Response
    ) -> ControlMission.Response:
        try:
            if request.command == ControlMission.Request.START:
                targets = list(request.target_ids) or self._default_targets
                self._start_mission(targets)
                message = "自主任务已启动"
            elif request.command == ControlMission.Request.PAUSE:
                if not self._mission.running:
                    raise RuntimeError("没有正在运行的任务")
                self._mission.pause()
                self._plan_generation += 1
                self._plan_due_at = None
                self._request_hold("operator", True, "operator paused mission")
                self._set_state(MissionState.PAUSED, "任务已暂停")
                message = "暂停请求已接受"
            elif request.command == ControlMission.Request.RESUME:
                if not self._mission.running or not self._mission.paused:
                    raise RuntimeError("任务不在暂停状态")
                self._mission.resume()
                self._request_hold("operator", False, "operator resumed mission")
                self._request_hold("fault", False, "operator resumed mission")
                if self._active_route is not None:
                    self._set_state(MissionState.EXECUTING, "继续当前路线")
                else:
                    self._schedule_plan(False)
                message = "继续请求已接受"
            elif request.command == ControlMission.Request.ABORT:
                self._abort_mission()
                message = "任务已终止并驻停"
            elif request.command == ControlMission.Request.RESET:
                self._reset_mission()
                message = "任务状态已复位"
            else:
                raise RuntimeError("未知任务命令")
            response.accepted = True
        except (RuntimeError, ValueError) as error:
            response.accepted = False
            message = str(error)
        response.message = message
        response.state = self._state_message()
        return response

    def _start_mission(self, targets: list[str]) -> None:
        if self._mission.running or self._planning_future is not None:
            raise RuntimeError("已有任务正在运行")
        reporting = (
            self._report_goal_future is not None
            or self._report_result_future is not None
        )
        if reporting:
            raise RuntimeError("事件播报尚未完成")
        if not self._ready():
            raise RuntimeError("规划器、控制器或融合里程计尚未就绪")
        self._mission.start(targets)
        self._events.reset()
        self._dynamic_obstacles.clear()
        self._dynamic_obstacles_updated_at = None
        self._pending_obstacle = None
        self._front_blocked = False
        self._latest_range = None
        self._clear_active_route()
        self._clear_obstacle_recovery()
        self._completed_route_retreat_available = False
        self._completed_stages = 0
        self._ordinary_nodes_since_task = 1
        self._plan_retries = 0
        self._no_progress_routes = 0
        self._request_hold("operator", False, "new mission started")
        self._request_hold("fault", False, "new mission started")
        self._request_hold("report", False, "new mission started")
        self._schedule_plan(False)

    def _abort_mission(self) -> None:
        self._mission.abort()
        self._cancel_report()
        self._events.reset()
        self._plan_generation += 1
        self._plan_due_at = None
        self._pending_obstacle = None
        self._clear_active_route()
        self._clear_obstacle_recovery()
        self._completed_route_retreat_available = False
        self._request_hold("operator", True, "mission aborted")
        self._set_state(MissionState.IDLE, "任务已终止，车辆保持驻停")

    def _reset_mission(self) -> None:
        self._cancel_report()
        self._mission.reset()
        self._events.reset()
        self._dynamic_obstacles.clear()
        self._dynamic_obstacles_updated_at = None
        self._pending_obstacle = None
        self._plan_generation += 1
        self._plan_due_at = None
        self._clear_active_route()
        self._clear_obstacle_recovery()
        self._completed_route_retreat_available = False
        self._request_hold("operator", True, "mission reset")
        self._set_state(MissionState.IDLE, "任务状态已复位，车辆保持驻停")

    def _schedule_plan(self, replanning: bool) -> None:
        self._replanning = replanning
        delay = float(self.get_parameter("replan_delay_s").value) if replanning else 0.0
        self._plan_due_at = time.monotonic() + max(0.0, delay)
        state = MissionState.REPLANNING if replanning else MissionState.PLANNING
        self._set_state(state, "等待最新位姿重新规划" if replanning else "正在规划")

    def _request_plan(self) -> None:
        if self._planning_future is not None:
            self._plan_due_at = time.monotonic() + 0.10
            return
        if not self._mission.running:
            return
        if self._mission.paused:
            self._set_state(MissionState.PAUSED, "任务已暂停")
            return
        if not self._ready():
            self._plan_due_at = time.monotonic() + 0.25
            self._set_state(MissionState.WAIT_READY, "等待新鲜融合位姿、规划和控制服务")
            return
        try:
            spec = self._mission.plan_spec()
        except RuntimeError as error:
            self._fault(str(error))
            return
        request = PlanRoute.Request()
        request.start = pose_message(self._pose)
        request.mode = spec.mode
        request.target_ids = list(spec.target_ids)
        request.required_lane_ids = list(spec.required_lane_ids)
        request.completed_lane_ids = list(spec.completed_lane_ids)
        request.dynamic_obstacles = [
            polygon_message(polygon) for polygon in self._dynamic_obstacles
        ]
        request.return_home = spec.return_home
        request.allow_partial = spec.allow_partial
        request.activate = False
        request.ordinary_nodes_since_task = self._ordinary_nodes_since_task
        self._plan_generation += 1
        generation = self._plan_generation
        self._planning_future = self._planner.call_async(request)
        self._planning_future.add_done_callback(
            lambda future: self._on_plan_response(generation, spec, future)
        )
        state = MissionState.REPLANNING if self._replanning else MissionState.PLANNING
        self._set_state(state, f"规划 {spec.mode}")

    def _on_plan_response(self, generation: int, spec: PlanSpec, future: Any) -> None:
        if generation != self._plan_generation:
            if future is self._planning_future:
                self._planning_future = None
            return
        self._planning_future = None
        try:
            response = future.result()
        except Exception as error:
            self._plan_failed(f"规划服务调用失败：{error}")
            return
        if not response.success:
            self._handle_plan_rejection(f"规划失败：{response.message}")
            return
        plan = response.plan
        if not plan.navigation_path.poses:
            if plan.complete and spec.mode == "layer3":
                self._mission.commit_empty(spec, True)
                self._completed_stages += 1
                self._finish_mission()
            else:
                self._handle_plan_rejection("规划结果没有可执行路线")
            return
        if not plan.collision_checked or plan.plan_id <= 0:
            self._handle_plan_rejection("规划结果未通过碰撞验证")
            return
        if not spec.has_progress(plan.visit_order, plan.covered_lane_ids):
            self._handle_plan_rejection("规划结果没有新增当前阶段进度")
            return
        self._plan_retries = 0
        self._clear_obstacle_recovery()
        self._completed_route_retreat_available = False
        self._active_route = ActiveRoute(int(plan.plan_id), spec, plan)
        self._ack_deadline = time.monotonic() + float(
            self.get_parameter("route_ack_timeout_s").value
        )
        self._set_state(MissionState.WAIT_ROUTE_ACK, f"等待路线 {plan.plan_id} ACK")
        self._route_publisher.publish(plan.navigation_path)
        if self._active_route.route_id in self._ack_cache:
            self._accept_ack(self._active_route.route_id)

    def _plan_failed(self, message: str) -> None:
        self._plan_retries += 1
        maximum = int(self.get_parameter("max_plan_retries").value)
        if self._plan_retries > maximum:
            self._fault(message)
            return
        delay = float(self.get_parameter("plan_retry_delay_s").value)
        self._plan_due_at = time.monotonic() + max(0.1, delay)
        self._set_state(MissionState.REPLANNING, f"{message}；准备重试")

    def _on_route_ack(self, message: UInt64) -> None:
        route_id = int(message.data)
        self._ack_cache.add(route_id)
        if len(self._ack_cache) > 64:
            self._ack_cache = set(sorted(self._ack_cache)[-32:])
        self._accept_ack(route_id)

    def _accept_ack(self, route_id: int) -> None:
        if self._active_route is None or route_id != self._active_route.route_id:
            return
        self._active_route.acknowledged = True
        self._ack_deadline = 0.0
        self._set_state(MissionState.EXECUTING, f"路线 {route_id} 执行中")

    def _handle_goal_reached(self) -> None:
        route = self._active_route
        if route is None or not route.acknowledged or not route.motion_seen:
            return
        previous_stage = route.spec.mode
        progress = self._mission.commit(
            route.spec,
            route.plan.visit_order,
            route.plan.covered_lane_ids,
            bool(route.plan.complete),
        )
        self._ordinary_nodes_since_task = int(
            route.plan.ordinary_nodes_since_task
        )
        self._clear_active_route()
        self._completed_route_retreat_available = True
        current_stage = self._mission.current_stage()
        if self._mission.finished:
            self._completed_stages += 1
            self._finish_mission()
            return
        if current_stage != previous_stage:
            self._completed_stages += 1
            self._publish_text(f"stage_complete:{previous_stage}")
        if progress == 0:
            self._no_progress_routes += 1
        else:
            self._no_progress_routes = 0
        if self._no_progress_routes > int(
            self.get_parameter("max_no_progress_routes").value
        ):
            self._fault("连续路线没有新增任务进度")
            return
        self._schedule_plan(False)

    def _finish_mission(self) -> None:
        if self._events.pending_count:
            self._set_state(MissionState.WAIT_REPORTER, "导航完成，等待事件播报")
        else:
            self._set_state(MissionState.COMPLETE, "三阶段自主任务完成")
        self._publish_text(f"mission_complete:stages={self._completed_stages}")

    def _clear_active_route(self) -> None:
        self._active_route = None
        self._ack_deadline = 0.0

    def _fault(self, message: str) -> None:
        if self._mission.running:
            self._mission.pause()
        self._plan_generation += 1
        self._plan_due_at = None
        self._clear_active_route()
        self._clear_obstacle_recovery()
        self._completed_route_retreat_available = False
        self._cancel_report()
        self._request_hold("fault", True, message)
        self._set_state(MissionState.FAULT, message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            future = node.request_shutdown_hold()
            if future is not None:
                rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
