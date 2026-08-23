import time

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import UInt64


@pytest.mark.launch_test
def generate_test_description():
    navigator = launch_ros.actions.Node(
        package="visual_navigation",
        executable="waypoint_navigator",
        name="waypoint_navigator_test",
        parameters=[{
            "route_file": "",
            "route_input_topic": "/test/route_input",
            "route_ack_topic": "/test/route_ack",
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


class TestWaypointNavigatorContract:
    @classmethod
    def setup_class(cls):
        rclpy.init()
        cls.node = Node("waypoint_navigator_contract_test")
        cls.acks = []
        cls.subscription = cls.node.create_subscription(
            UInt64, "/test/route_ack", lambda message: cls.acks.append(message.data), 10
        )
        cls.publisher = cls.node.create_publisher(Path, "/test/route_input", 10)

    @classmethod
    def teardown_class(cls):
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

    def test_route_ack_requires_nonzero_route_id(self):
        self.spin(0.5)
        self.publish_route(rclpy.time.Time().to_msg())
        self.spin(0.3)
        assert self.acks == []

        stamp = self.node.get_clock().now()
        self.publish_route(stamp.to_msg())
        self.spin(0.5)
        assert self.acks == [stamp.nanoseconds]
