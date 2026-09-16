import math
import time
import unittest
from pathlib import Path as FilePath

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from mission_control_interfaces.msg import ControlTelemetry
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String, UInt64
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    route_file = str(
        FilePath(__file__).resolve().parents[1] / "routes" / "example_route.csv"
    )
    navigator = launch_ros.actions.Node(
        package="visual_navigation",
        executable="waypoint_navigator",
        name="waypoint_navigator_test",
        parameters=[{
            "route_file": route_file,
            "route_input_topic": "/test/route_input",
            "route_ack_topic": "/test/route_ack",
            "odom_topic": "/test/odometry",
            "fusion_status_topic": "/test/fusion_status",
            "status_topic": "/test/navigation_status",
            "front_obstacle_topic": "/test/front_blocked",
            "front_obstacle_range_topic": "/test/front_range",
            "control_telemetry_topic": "/test/control_telemetry",
            "front_obstacle_classification_wait": 0.05,
            "pre_turn_minimum_stop_time": 0.05,
            "pre_turn_stop_dwell": 0.05,
            "pre_turn_brake_timeout": 0.20,
            "stop_motion_window": 0.10,
            "require_fusion_status": True,
            "require_tracking_state": False,
            "require_actuator_health": False,
            "autostart": False,
        }],
        output="screen",
    )
    return launch.LaunchDescription([navigator, launch_testing.actions.ReadyToTest()]), {
        "navigator": navigator,
    }


class TestWaypointNavigatorContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("waypoint_navigator_contract_test")
        cls.acks = []
        cls.states = []
        cls.commands = []
        cls.subscription = cls.node.create_subscription(
            UInt64, "/test/route_ack", lambda message: cls.acks.append(message.data), 10
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.status_subscription = cls.node.create_subscription(
            String,
            "/test/navigation_status",
            lambda message: cls.states.append(message.data),
            state_qos,
        )
        cls.command_subscription = cls.node.create_subscription(
            Twist,
            "/cmd_vel_nav",
            lambda message: cls.commands.append(
                (message.linear.x, message.angular.z)
            ),
            10,
        )
        cls.publisher = cls.node.create_publisher(Path, "/test/route_input", 10)
        cls.odom_publisher = cls.node.create_publisher(
            Odometry, "/test/odometry", 10
        )
        cls.fusion_publisher = cls.node.create_publisher(
            String, "/test/fusion_status", 10
        )
        cls.obstacle_publisher = cls.node.create_publisher(
            Bool, "/test/front_blocked", qos_profile_sensor_data
        )
        cls.range_publisher = cls.node.create_publisher(
            Range, "/test/front_range", 10
        )
        cls.telemetry_publisher = cls.node.create_publisher(
            ControlTelemetry, "/test/control_telemetry", 10
        )
        cls.telemetry_sequence = 0
        cls.start_client = cls.node.create_client(
            Trigger, "/waypoint_navigator_test/start"
        )
        cls.stop_client = cls.node.create_client(
            Trigger, "/waypoint_navigator_test/stop"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        for _ in range(3):
            self.publish_fusion("FULL")
            self.spin(0.03)

    def spin(self, seconds=0.1):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def publish_route(self, stamp, points=None):
        route = Path()
        route.header.stamp = stamp
        route.header.frame_id = "map"
        for point in points or [(0.0, 0.0)]:
            x, y = point[:2]
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = point[2] if len(point) > 2 else 0.0
            pose.pose.orientation.w = 1.0
            route.poses.append(pose)
        self.publisher.publish(route)

    def publish_odometry(
        self, child_frame="base_link", x=0.0, y=0.0, yaw=0.0, stamp=None
    ):
        odometry = Odometry()
        odometry.header.stamp = stamp or self.node.get_clock().now().to_msg()
        odometry.header.frame_id = "map"
        odometry.child_frame_id = child_frame
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odometry.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.odom_publisher.publish(odometry)

    def publish_fusion(self, state):
        message = String()
        message.data = state
        self.fusion_publisher.publish(message)

    def call_trigger(self, client):
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        assert future.done()
        return future.result()

    def publish_obstacle(self, blocked, distance=None):
        message = Bool()
        message.data = blocked
        self.obstacle_publisher.publish(message)
        if distance is not None:
            self.spin(0.01)
            self.publish_obstacle_range(distance)

    def publish_obstacle_range(self, distance):
        range_message = Range()
        range_message.header.stamp = self.node.get_clock().now().to_msg()
        range_message.header.frame_id = "camera_link"
        range_message.min_range = 0.05
        range_message.max_range = 3.0
        range_message.range = distance
        self.range_publisher.publish(range_message)
        self.spin(0.05)

    def hold_odometry(self, x, y=0.0, yaw=0.0, cycles=8):
        for _ in range(cycles):
            self.publish_fusion("FULL")
            self.publish_odometry(x=x, y=y, yaw=yaw)
            self.spin(0.04)

    def publish_stopped_wheels(self):
        self.__class__.telemetry_sequence += 1
        sample = ControlTelemetry()
        sample.sample_sequence = self.telemetry_sequence
        sample.mcu_time_ms = self.telemetry_sequence * 50
        self.telemetry_publisher.publish(sample)

    def test_loaded_manual_route_starts_idle(self):
        self.spin(0.5)
        assert self.states
        assert self.states[-1] == "IDLE"

    def test_expected_wall_beyond_turn_does_not_start_reverse(self):
        self.call_trigger(self.stop_client)
        self.publish_obstacle(False)
        self.spin(0.05)
        self.commands.clear()
        self.states.clear()
        self.publish_odometry()
        self.spin(0.05)
        stamp = self.node.get_clock().now()
        self.publish_route(
            stamp.to_msg(), [(0.0, 0.0), (0.30, 0.0, 0.001), (0.30, 1.0)]
        )
        self.hold_odometry(0.0, cycles=3)

        self.publish_obstacle(True, distance=0.30)
        self.spin(0.15)

        assert "OBSTACLE_REVERSING" not in self.states
        assert any(linear > 0.0 for linear, _ in self.commands)
        self.publish_obstacle(False)
        self.call_trigger(self.stop_client)

    def test_unexpected_obstacle_stops_reverses_and_requests_replan(self):
        self.call_trigger(self.stop_client)
        self.publish_obstacle(False)
        self.spin(0.05)
        self.commands.clear()
        self.states.clear()
        self.publish_odometry()
        self.spin(0.05)
        stamp = self.node.get_clock().now()
        self.publish_route(stamp.to_msg(), [(0.0, 0.0), (1.0, 0.0)])
        self.hold_odometry(0.0, cycles=3)
        self.hold_odometry(0.10, cycles=2)
        self.hold_odometry(0.20, cycles=2)

        self.commands.clear()
        self.publish_obstacle(True)
        self.spin(0.02)
        assert self.commands
        assert abs(self.commands[-1][0]) < 1.0e-12
        assert "OBSTACLE_BRAKING" in self.states
        self.publish_obstacle_range(0.12)

        self.hold_odometry(0.20, cycles=8)
        assert "OBSTACLE_REVERSING" in self.states
        assert any(linear < -0.01 for linear, _ in self.commands)

        for x in (0.15, 0.10, 0.05, 0.0):
            self.hold_odometry(x, cycles=2)
        self.spin(0.10)
        assert self.states[-1] == "OBSTACLE_REPLAN_REQUIRED"
        assert abs(self.commands[-1][0]) < 1.0e-12
        self.publish_obstacle(False)

        # A new plan can be geometrically identical. Its new route id must
        # still clear the completed recovery state and receive an ACK.
        replacement_stamp = self.node.get_clock().now()
        self.acks.clear()
        self.publish_route(
            replacement_stamp.to_msg(), [(0.0, 0.0), (1.0, 0.0)]
        )
        self.spin(0.20)
        assert replacement_stamp.nanoseconds in self.acks
        assert self.states[-1] != "OBSTACLE_REPLAN_REQUIRED"
        self.call_trigger(self.stop_client)

    def test_latched_fusion_fault_during_reverse_allows_new_route_ack(self):
        self.call_trigger(self.stop_client)
        self.publish_obstacle(False)
        self.spin(0.05)
        self.commands.clear()
        self.states.clear()
        self.publish_odometry()
        self.spin(0.05)
        stamp = self.node.get_clock().now()
        route_points = [(0.0, 0.0), (1.0, 0.0)]
        self.publish_route(stamp.to_msg(), route_points)
        self.hold_odometry(0.0, cycles=3)
        self.hold_odometry(0.10, cycles=2)
        self.hold_odometry(0.20, cycles=2)

        self.publish_obstacle(True, distance=0.12)
        self.hold_odometry(0.20, cycles=8)
        assert "OBSTACLE_REVERSING" in self.states

        self.publish_fusion("FAULT")
        self.spin(0.10)
        assert self.states[-1] == "FAULT_FUSION_STATUS"
        assert abs(self.commands[-1][0]) < 1.0e-12

        self.publish_fusion("FULL")
        self.publish_odometry(x=0.20)
        self.spin(0.05)
        replacement_stamp = self.node.get_clock().now()
        self.acks.clear()
        replacement_points = [(0.20, 0.0), (1.0, 0.0)]
        self.publish_route(replacement_stamp.to_msg(), replacement_points)
        self.spin(0.20)

        assert replacement_stamp.nanoseconds in self.acks
        assert self.states[-1] != "FAULT_FUSION_STATUS"
        self.commands.clear()
        self.hold_odometry(0.20, cycles=3)
        assert any(linear > 0.0 for linear, _ in self.commands)

        # The detector may continuously report the same blockage without a
        # false edge. Route takeover resets the event generation, so the next
        # true heartbeat must stop the replacement route again. No new Range is
        # sent here: the distance from before the fault must not be reused.
        self.states.clear()
        self.publish_obstacle(True)
        self.spin(0.15)
        assert "OBSTACLE_BRAKING" in self.states
        self.call_trigger(self.stop_client)

    def test_odometry_rejects_bad_frame_or_pose_then_recovers(self):
        # Prime the cache with a valid sample. Isolated malformed samples must
        # be discarded rather than replacing that usable navigation state.
        self.publish_odometry()
        self.spin(0.1)
        self.publish_odometry(child_frame="")
        self.spin(0.05)
        self.publish_odometry(x=math.nan)
        self.spin(0.05)
        invalid_stamp = self.node.get_clock().now().to_msg()
        invalid_stamp.sec = -1
        self.publish_odometry(stamp=invalid_stamp)
        self.spin(0.05)
        invalid_stamp = self.node.get_clock().now().to_msg()
        invalid_stamp.nanosec = 1_000_000_000
        self.publish_odometry(stamp=invalid_stamp)
        self.spin(0.05)
        response = self.call_trigger(self.start_client)
        assert response.success
        self.call_trigger(self.stop_client)

        # Rejected samples did not poison the accepted timestamp baseline.
        self.publish_odometry()
        self.spin(0.1)
        response = self.call_trigger(self.start_client)
        assert response.success
        self.call_trigger(self.stop_client)

    def test_route_ack_requires_nonzero_route_id(self):
        self.acks.clear()
        self.spin(0.5)
        self.publish_route(rclpy.time.Time().to_msg())
        self.spin(0.3)
        assert self.acks == []

        invalid_stamp = self.node.get_clock().now().to_msg()
        invalid_stamp.sec = -1
        self.publish_route(invalid_stamp)
        invalid_stamp = self.node.get_clock().now().to_msg()
        invalid_stamp.nanosec = 1_000_000_000
        self.publish_route(invalid_stamp)
        self.spin(0.3)
        assert self.acks == []

        stamp = self.node.get_clock().now()
        self.publish_route(stamp.to_msg())
        self.spin(0.5)
        assert self.acks == [stamp.nanoseconds]

    def test_turn_alignment_forces_zero_linear_velocity(self):
        self.call_trigger(self.stop_client)
        self.commands.clear()
        stamp = self.node.get_clock().now()
        self.publish_route(
            stamp.to_msg(), [(0.0, 0.0), (0.30, 0.0, 0.001), (0.30, 1.0)]
        )
        self.spin(0.2)

        self.publish_odometry()
        self.spin(0.1)
        response = self.call_trigger(self.start_client)
        assert response.success

        # Complete a planned stop at the corner, with distinct MCU stop samples.
        for x in (0.10, 0.20, 0.30):
            self.hold_odometry(x, cycles=2)
        for _ in range(12):
            self.publish_fusion("FULL")
            self.publish_odometry(x=0.30)
            self.publish_stopped_wheels()
            self.spin(0.04)
        self.commands.clear()
        for _ in range(10):
            self.publish_fusion("FULL")
            self.publish_odometry(x=0.35)
            self.spin(0.04)

        turning_commands = [command for command in self.commands if abs(command[1]) > 0.05]
        assert turning_commands
        assert all(abs(command[0]) < 1.0e-12 for command in turning_commands)
        self.call_trigger(self.stop_client)

    def test_wrong_heading_without_known_junction_does_not_pivot(self):
        self.call_trigger(self.stop_client)
        self.commands.clear()
        self.publish_odometry()
        self.spin(0.05)
        stamp = self.node.get_clock().now()
        self.publish_route(stamp.to_msg(), [(0.0, 0.0), (0.0, 1.0)])
        self.hold_odometry(0.0, cycles=8)

        assert self.states[-1] == "OBSTACLE_REPLAN_REQUIRED"
        assert self.commands
        assert all(abs(linear) < 1e-12 and abs(angular) < 1e-12
                   for linear, angular in self.commands[-5:])
        self.call_trigger(self.stop_client)

    def test_marked_out_and_back_uses_planned_reverse_without_pivot(self):
        self.call_trigger(self.stop_client)
        self.commands.clear()
        self.states.clear()
        self.publish_odometry()
        self.spin(0.05)
        stamp = self.node.get_clock().now()
        self.publish_route(
            stamp.to_msg(), [(0.0, 0.0), (0.30, 0.0, 0.001), (0.0, 0.0)]
        )
        self.hold_odometry(0.0, cycles=5)
        for x in (0.10, 0.20, 0.30):
            self.hold_odometry(x, cycles=2)
        for _ in range(12):
            self.publish_odometry(x=0.30)
            self.publish_stopped_wheels()
            self.spin(0.04)

        self.commands.clear()
        self.hold_odometry(0.28, cycles=6)
        assert "REVERSING_PLANNED_RETREAT" in self.states
        assert "ROTATING_TO_PATH" not in self.states
        assert any(linear < -0.01 for linear, _ in self.commands)
        assert all(abs(angular) < 0.05 for _, angular in self.commands[-5:])
        self.call_trigger(self.stop_client)

    def test_marked_route_start_authorizes_junction_turn(self):
        self.call_trigger(self.stop_client)
        self.commands.clear()
        self.states.clear()
        self.publish_odometry()
        self.spin(0.05)
        stamp = self.node.get_clock().now()
        self.publish_route(stamp.to_msg(), [(0.0, 0.0, 0.001), (0.0, 1.0)])
        self.hold_odometry(0.0, cycles=8)

        assert "OBSTACLE_REPLAN_REQUIRED" not in self.states
        assert "ROTATING_TO_PATH" in self.states
        assert any(angular > 0.05 for _, angular in self.commands)
        self.call_trigger(self.stop_client)

    def test_non_junction_out_and_back_uses_reverse_without_pivot(self):
        self.call_trigger(self.stop_client)
        self.commands.clear()
        self.states.clear()
        self.publish_odometry()
        self.spin(0.05)
        stamp = self.node.get_clock().now()
        self.publish_route(stamp.to_msg(), [(0.0, 0.0), (0.30, 0.0), (0.0, 0.0)])
        self.hold_odometry(0.0, cycles=5)
        for x in (0.10, 0.20, 0.30):
            self.hold_odometry(x, cycles=2)
        for _ in range(12):
            self.publish_odometry(x=0.30)
            self.publish_stopped_wheels()
            self.spin(0.04)

        self.commands.clear()
        self.hold_odometry(0.28, cycles=6)
        assert "REVERSING_PLANNED_RETREAT" in self.states
        assert any(linear < -0.01 for linear, _ in self.commands)
        assert all(abs(angular) < 0.05 for _, angular in self.commands[-5:])
        self.call_trigger(self.stop_client)

    def test_stop_and_turn_corner_does_not_steer_incoming_straight(self):
        self.call_trigger(self.stop_client)
        self.commands.clear()
        self.publish_odometry()
        self.spin(0.1)
        stamp = self.node.get_clock().now()
        self.publish_route(stamp.to_msg(), [(2.0, 0.0), (2.0, 3.0)])

        for _ in range(12):
            self.publish_odometry()
            self.spin(0.04)

        forward_commands = [command for command in self.commands if command[0] > 0.05]
        assert forward_commands
        assert all(abs(command[1]) < 1.0e-12 for command in forward_commands)
        self.call_trigger(self.stop_client)
