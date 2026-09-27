import math

from rectangle_odometry_test.geometry import Pose2D
from rectangle_odometry_test.primitive_controller import (
    ActionPrimitive,
    PlannedPrimitive,
    PrimitiveController,
    PrimitivePlanner,
    Twist2D,
    differential_wheel_speeds,
    integrate_pose,
    make_action_primitives,
    rectangle_goals,
)


def test_integrate_pose_uses_metres_and_seconds():
    result = integrate_pose(Pose2D(0.0, 0.0, 0.0), 0.4, 0.0, 0.5)

    assert math.isclose(result.x, 0.2)
    assert math.isclose(result.y, 0.0)
    assert math.isclose(result.yaw, 0.0)


def test_integrate_pose_follows_constant_curvature_arc():
    result = integrate_pose(
        Pose2D(0.0, 0.0, 0.0),
        linear_x_mps=0.2,
        angular_z_rps=0.5,
        duration_s=1.0,
    )
    radius = 0.2 / 0.5

    assert math.isclose(result.x, radius * math.sin(0.5))
    assert math.isclose(result.y, radius * (1.0 - math.cos(0.5)))
    assert math.isclose(result.yaw, 0.5)


def test_wheel_speed_conversion_uses_si_units():
    left, right = differential_wheel_speeds(
        Twist2D(0.2, 0.5), wheel_track_m=0.1466
    )

    assert math.isclose(left, 0.2 - 0.5 * 0.1466 / 2.0)
    assert math.isclose(right, 0.2 + 0.5 * 0.1466 / 2.0)


def test_default_action_set_is_discrete_and_has_explicit_time_base():
    actions = make_action_primitives()

    assert len(actions) == 16
    assert {round(action.duration_s, 3) for action in actions} == {0.5}
    assert {round(abs(action.linear_x_mps) * 0.5, 3) for action in actions} >= {
        0.0,
        0.03,
    }
    translated = [action for action in actions if action.linear_x_mps > 0.0]
    assert sorted(
        round(abs(action.angular_z_rps) * 0.5 * 180.0 / math.pi, 3)
        for action in translated
    ) == [0.0, 3.0, 3.0, 6.0, 6.0, 12.0, 12.0]
    assert any(
        action.linear_x_mps == 0.0 and action.angular_z_rps != 0.0
        for action in actions
    )


def test_planner_returns_a_discrete_plan_for_a_straight_goal():
    actions = (ActionPrimitive(0.1, 0.0, 0.2),)
    planner = PrimitivePlanner(
        actions=actions,
        position_resolution_m=0.02,
        goal_position_tolerance_m=0.011,
        goal_yaw_tolerance_rad=0.01,
        max_expansions=100,
    )

    plan = planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(0.4, 0.0, 0.0))

    assert plan is not None
    assert len(plan) == 20
    assert all(item.action == actions[0] for item in plan)
    assert math.isclose(plan[-1].end.x, 0.4, abs_tol=1.0e-9)


def test_default_planner_can_plan_the_first_rectangle_leg():
    origin = Pose2D(0.0, 0.0, 0.0)
    planner = PrimitivePlanner(max_expansions=10000)

    plan = planner.plan(origin, rectangle_goals(origin, 1.8)[0])

    assert plan is not None
    assert plan
    assert math.isclose(plan[-1].end.y, 0.0, abs_tol=0.05)


def test_default_planner_can_plan_a_right_angle_leg():
    planner = PrimitivePlanner(max_expansions=30000)

    plan = planner.plan(
        Pose2D(1.8, 0.0, 0.0),
        Pose2D(1.8, -1.8, -math.pi / 2.0),
    )

    assert plan is not None
    assert plan
    assert math.isclose(plan[-1].end.x, 1.8, abs_tol=0.08)
    assert plan[-1].end.y < -1.7
    assert abs(plan[-1].end.yaw + math.pi / 2.0) < math.radians(10.0)


def test_planner_checks_collision_during_a_primitive():
    actions = (ActionPrimitive(0.2, 0.0, 1.0),)
    visited = []

    def collision_checker(pose):
        visited.append(pose.x)
        return pose.x > 0.05

    planner = PrimitivePlanner(
        actions=actions,
        collision_checker=collision_checker,
        max_expansions=10,
    )

    assert planner.plan(Pose2D(0.0, 0.0, 0.0), Pose2D(0.2, 0.0, 0.0)) is None
    assert max(visited) < 0.2


def test_controller_replans_after_odometry_deviation():
    action = ActionPrimitive(0.1, 0.0, 1.0)

    class RecordingPlanner:
        def __init__(self):
            self.starts = []

        def plan(self, start, goal):
            self.starts.append(start)
            return [
                PlannedPrimitive(
                    action,
                    start,
                    integrate_pose(
                        start,
                        action.linear_x_mps,
                        action.angular_z_rps,
                        action.duration_s,
                    ),
                )
            ]

    planner = RecordingPlanner()
    controller = PrimitiveController(
        planner,
        position_tolerance_m=0.02,
        yaw_tolerance_rad=0.1,
        replan_position_error_m=0.05,
        replan_yaw_error_rad=0.2,
        replan_cooldown_s=0.1,
    )
    controller.start(Pose2D(0.0, 0.0, 0.0), [Pose2D(0.2, 0.0, 0.0)])

    first = controller.update(0.0, Pose2D(0.0, 0.0, 0.0))
    second = controller.update(0.3, Pose2D(0.0, 0.12, 0.0))

    assert first == Twist2D(0.1, 0.0)
    assert second == Twist2D(0.1, 0.0)
    assert len(planner.starts) == 2
    assert planner.starts[-1] == Pose2D(0.0, 0.12, 0.0)
    assert controller.last_plan_reason == "odometry_deviation"


def test_controller_default_turn_guard_replans_when_yaw_lags():
    action = ActionPrimitive(0.0, math.radians(-40.0), 0.5)

    class RecordingPlanner:
        def __init__(self):
            self.starts = []

        def plan(self, start, goal):
            self.starts.append(start)
            return [PlannedPrimitive(action, start, integrate_pose(
                start,
                action.linear_x_mps,
                action.angular_z_rps,
                action.duration_s,
            ))]

    planner = RecordingPlanner()
    controller = PrimitiveController(planner)
    controller.start(
        Pose2D(0.0, 0.0, 0.0),
        [Pose2D(0.0, 0.0, math.radians(-20.0))],
    )

    controller.update(0.0, Pose2D(0.0, 0.0, 0.0))
    command = controller.update(0.5, Pose2D(0.0, 0.0, math.radians(5.0)))

    assert command == Twist2D(
        action.linear_x_mps, -math.radians(25.0)
    )
    assert len(planner.starts) == 2
    assert controller.last_plan_reason == "odometry_deviation"


def test_controller_keeps_turning_when_planned_action_is_straight():
    action = ActionPrimitive(0.03, 0.0, 0.5)

    class StraightPlanner:
        def plan(self, start, goal):
            return [PlannedPrimitive(action, start, start)]

    controller = PrimitiveController(StraightPlanner())
    controller.start(
        Pose2D(0.0, 0.0, 0.0),
        [
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(0.0, -1.0, -math.pi / 2.0),
        ],
    )

    command = controller.update(0.0, Pose2D(0.0, 0.0, 0.0))

    assert command.linear_x_mps == action.linear_x_mps
    assert command.angular_z_rps < 0.0
    assert math.isclose(
        abs(command.angular_z_rps), math.radians(8.0), abs_tol=1.0e-9
    )


def test_controller_holds_straight_heading_with_small_angle_feedback():
    action = ActionPrimitive(0.06, 0.0, 0.5)

    class StraightPlanner:
        def plan(self, start, goal):
            return [PlannedPrimitive(action, start, start)]

    controller = PrimitiveController(StraightPlanner())
    controller.start(
        Pose2D(0.0, 0.0, 0.0),
        [Pose2D(1.0, 0.0, 0.0)],
    )

    command = controller.update(
        0.0, Pose2D(0.0, 0.0, math.radians(4.0))
    )

    assert math.isclose(command.linear_x_mps, 0.06, abs_tol=1.0e-9)
    assert math.isclose(
        command.angular_z_rps,
        -math.radians(6.0),
        abs_tol=1.0e-9,
    )


def test_controller_does_not_chatter_inside_straight_heading_deadband():
    action = ActionPrimitive(0.06, 0.0, 0.5)

    class StraightPlanner:
        def plan(self, start, goal):
            return [PlannedPrimitive(action, start, start)]

    controller = PrimitiveController(StraightPlanner())
    controller.start(
        Pose2D(0.0, 0.0, 0.0),
        [Pose2D(1.0, 0.0, 0.0)],
    )

    command = controller.update(
        0.0, Pose2D(0.0, 0.0, math.radians(1.0))
    )

    assert command == Twist2D(0.06, 0.0)


def test_controller_stops_when_all_goals_are_reached():
    action = ActionPrimitive(0.1, 0.0, 0.5)

    class OneStepPlanner:
        def plan(self, start, goal):
            return [
                PlannedPrimitive(
                    action,
                    start,
                    integrate_pose(
                        start,
                        action.linear_x_mps,
                        action.angular_z_rps,
                        action.duration_s,
                    ),
                )
            ]

    controller = PrimitiveController(
        OneStepPlanner(),
        position_tolerance_m=0.02,
        yaw_tolerance_rad=0.1,
    )
    controller.start(Pose2D(0.0, 0.0, 0.0), [Pose2D(0.05, 0.0, 0.0)])

    command = controller.update(0.0, Pose2D(0.0, 0.0, 0.0))
    stopped = controller.update(0.5, Pose2D(0.05, 0.0, 0.0))

    assert command == Twist2D(0.1, 0.0)
    assert stopped == Twist2D(0.0, 0.0)
    assert controller.is_complete


def test_controller_accepts_corner_goal_with_small_lateral_odom_bias():
    class NoPlanExpected:
        def plan(self, start, goal):
            raise AssertionError("goal should be accepted before planning")

    controller = PrimitiveController(
        NoPlanExpected(),
        position_tolerance_m=0.05,
        yaw_tolerance_rad=math.radians(5.0),
        goal_line_tolerance_m=0.05,
        goal_lateral_tolerance_m=0.10,
    )
    goal = Pose2D(1.791, -1.810, math.radians(-90.3))
    controller.start(Pose2D(0.0, 0.0, 0.0), [goal])

    command = controller.update(
        72.0,
        Pose2D(1.865, -1.821, math.radians(-87.5)),
    )

    assert command == Twist2D(0.0, 0.0)
    assert controller.is_complete


def test_controller_does_not_advance_corner_goal_too_early_on_approach():
    action = ActionPrimitive(0.06, 0.0, 0.5)

    class OneStepPlanner:
        def plan(self, start, goal):
            return [PlannedPrimitive(action, start, start)]

    controller = PrimitiveController(
        OneStepPlanner(),
        position_tolerance_m=0.05,
        yaw_tolerance_rad=math.radians(5.0),
        goal_line_tolerance_m=0.05,
        goal_lateral_tolerance_m=0.10,
    )
    controller.start(
        Pose2D(0.0, 0.0, 0.0),
        [Pose2D(1.8, 0.0, 0.0)],
    )

    command = controller.update(38.0, Pose2D(1.70, 0.0, 0.0))

    assert command == Twist2D(0.06, 0.0)
    assert not controller.is_complete


def test_rectangle_goals_keep_the_existing_clockwise_geometry():
    goals = rectangle_goals(Pose2D(1.0, 2.0, 0.0), 1.8)

    assert len(goals) == 4
    assert [(round(goal.x, 3), round(goal.y, 3)) for goal in goals] == [
        (2.8, 2.0),
        (2.8, 0.2),
        (1.0, 0.2),
        (1.0, 2.0),
    ]
