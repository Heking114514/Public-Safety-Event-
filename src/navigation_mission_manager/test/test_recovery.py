from concurrent.futures import Future
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from mission_control_interfaces.msg import MissionState
from mission_control_interfaces.srv import ContinueRetreat
from new_map_planner.srv import PlanRoute
from std_msgs.msg import String

from navigation_mission_manager.model import PlanSpec
from navigation_mission_manager.node import MissionManager
from navigation_mission_manager.recovery import RecoveryMixin


class FakeRetreatClient:
    def __init__(self):
        self.ready = True
        self.requests = []
        self.future = Future()

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.future


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeRecoveryManager(RecoveryMixin):
    def __init__(self):
        self._parameters = {
            "max_retreat_extensions": SimpleNamespace(value=4),
            "retreat_step_m": SimpleNamespace(value=0.40),
            "route_ack_timeout_s": SimpleNamespace(value=3.0),
        }
        self._retreat_client = FakeRetreatClient()
        self._retreat_future = None
        self._retreat_generation = 0
        self._retreat_extensions = 0
        self._awaiting_retreat = False
        self._obstacle_recovery_active = True
        self._completed_route_retreat_available = False
        self._navigator_status = "OBSTACLE_REPLAN_REQUIRED"
        self._active_route = None
        self._state = MissionState.REPLANNING
        self._detail = ""
        self.state_updates = []
        self.faults = []
        self.scheduled_plans = []

    def get_parameter(self, name):
        return self._parameters[name]

    def _set_state(self, state, detail):
        self._state = state
        self._detail = detail
        self.state_updates.append((state, detail))

    def _fault(self, message):
        self.faults.append(message)

    def _schedule_plan(self, replanning):
        self.scheduled_plans.append(replanning)


def test_planner_rejection_requests_retreat_then_waits_for_new_anchor():
    manager = FakeRecoveryManager()

    manager._handle_plan_rejection("start pose is not aligned")

    assert len(manager._retreat_client.requests) == 1
    request = manager._retreat_client.requests[0]
    assert request.minimum_distance_m == 0.40
    assert manager.scheduled_plans == []

    response = ContinueRetreat.Response()
    response.accepted = True
    response.remaining_breadcrumbs = 8
    manager._retreat_client.future.set_result(response)

    assert manager._awaiting_retreat
    assert manager._retreat_extensions == 1
    assert manager.scheduled_plans == []

    manager._on_navigator_status(String(data="OBSTACLE_REPLAN_REQUIRED"))

    assert not manager._awaiting_retreat
    assert manager.scheduled_plans == [True]


def test_exhausted_retreat_trace_enters_fault():
    manager = FakeRecoveryManager()

    manager._handle_plan_rejection("no collision-free road")
    response = ContinueRetreat.Response()
    response.accepted = False
    response.trace_exhausted = True
    response.message = "executed breadcrumb trace is exhausted"
    manager._retreat_client.future.set_result(response)

    assert manager.faults == ["绕障规划仍无路且无法继续倒退：已执行的安全轨迹耗尽"]
    assert not manager._awaiting_retreat


def test_controller_fault_during_extended_retreat_enters_mission_fault():
    manager = FakeRecoveryManager()
    manager._active_route = None
    manager._awaiting_retreat = True

    manager._on_navigator_status(String(data="FAULT_OBSTACLE_REVERSE_TIMEOUT"))

    assert manager.faults == ["导航故障：FAULT_OBSTACLE_REVERSE_TIMEOUT"]


def test_route_rejected_before_motion_faults_instead_of_replanning_forever():
    manager = FakeRecoveryManager()
    manager._obstacle_recovery_active = False
    manager._active_route = SimpleNamespace(acknowledged=True, motion_seen=False)

    manager._on_navigator_status(String(data="OBSTACLE_REPLAN_REQUIRED"))

    assert manager.faults == ["控制器拒绝了尚未开始执行的规划路线"]
    assert manager.scheduled_plans == []


def test_successful_replan_clears_recovery_handshake():
    manager = FakeRecoveryManager()
    manager._plan_generation = 7
    manager._planning_future = Future()
    manager._plan_retries = 2
    manager._route_publisher = FakePublisher()
    manager._ack_cache = set()
    manager._ack_deadline = 0.0
    manager._retreat_future = Future()
    manager._retreat_extensions = 2
    manager._awaiting_retreat = True

    response = PlanRoute.Response()
    response.success = True
    response.plan.plan_id = 41
    response.plan.collision_checked = True
    response.plan.navigation_path.poses.append(PoseStamped())
    response.plan.visit_order = ["1"]
    manager._planning_future.set_result(response)

    spec = PlanSpec(mode="layer1", target_ids=("1",))
    MissionManager._on_plan_response(
        manager, manager._plan_generation, spec, manager._planning_future
    )

    assert not manager._obstacle_recovery_active
    assert not manager._awaiting_retreat
    assert manager._retreat_extensions == 0
    assert manager._active_route.route_id == 41
    assert manager._route_publisher.messages == [response.plan.navigation_path]
    assert manager._state == MissionState.WAIT_ROUTE_ACK


def test_stage_handoff_plan_failure_requests_bounded_retreat():
    manager = FakeRecoveryManager()
    manager._obstacle_recovery_active = False
    manager._completed_route_retreat_available = True

    manager._handle_plan_rejection("规划结果没有新增当前阶段进度")

    assert manager._obstacle_recovery_active
    assert not manager._completed_route_retreat_available
    assert len(manager._retreat_client.requests) == 1
