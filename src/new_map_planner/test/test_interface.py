from new_map_planner.msg import RoutePlan
from new_map_planner.srv import PlanRoute


def test_plan_request_exposes_explicit_recovery_contract():
    request = PlanRoute.Request()

    assert request.START_MODE_NORMAL == 0
    assert request.START_MODE_TURN_RECOVERY == 1
    assert request.START_MODE_OBSTACLE_RECOVERY == 2
    assert request.start_mode == request.START_MODE_NORMAL
    assert request.recovery_anchor_node_id == ""
    assert request.incoming_lane_id == ""
    assert request.allow_reverse_departure is False


def test_route_plan_marks_recovery_output_explicitly():
    assert RoutePlan().recovery_only is False
