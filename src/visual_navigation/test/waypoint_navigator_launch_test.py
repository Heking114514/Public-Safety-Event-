import math
import time
import unittest
from pathlib import Path as FilePath

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt64
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
            "status_topic": "/test/navigation_status",
            "require_fusion_status": False,
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
        cls.publisher = cls.node.create_publisher(Path, "/test/route_input", 10)
        cls.odom_publisher = cls.node.create_publisher(
            Odometry, "/test/odometry", 10
        )
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

    def spin(self, seconds=0.1):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def publish_route(self, stamp):
        route = Path()
        route.header.stamp = stamp
        route.header.frame_id = "map"
        pose = PoseStamped()
        pose.header = route.header
        pose.pose.orientation.w = 1.0
        route.poses.append(pose)
        self.publisher.publish(route)

    def publish_odometry(self, child_frame="base_link", x=0.0, stamp=None):
        odometry = Odometry()
        odometry.header.stamp = stamp or self.node.get_clock().now().to_msg()
        odometry.header.frame_id = "map"
        odometry.child_frame_id = child_frame
        odometry.pose.pose.position.x = x
        odometry.pose.pose.orientation.w = 1.0
        self.odom_publisher.publish(odometry)

    def call_trigger(self, client):
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        assert future.done()
        return future.result()

    def test_loaded_manual_route_starts_idle(self):
        self.spin(0.5)
        assert self.states
        assert self.states[-1] == "IDLE"

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
