#!/usr/bin/env python3
"""独立 ROS 域验证真实节点输出；不启动串口，参数和节点日志退出自动清理。"""
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "186"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as Route
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from vehicle_config import parameters_for
import yaml

WORKSPACE = Path(__file__).resolve().parents[3]


class Probe:
    def __init__(self):
        self.node = rclpy.create_node("flexible_probe")
        self.commands, self.states = [], []
        self.odom = self.node.create_publisher(Odometry, "/flexible_test/odom", 10)
        self.imu = self.node.create_publisher(Imu, "/flexible_test/imu", 10)
        self.route = self.node.create_publisher(Route, "/flexible_test/route", 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions = [
            self.node.create_subscription(Twist, "/flexible_test/cmd",
                lambda m: self.commands.append((m.linear.x, m.angular.z)), 10),
            self.node.create_subscription(String, "/flexible_test/state",
                lambda m: self.states.append(m.data), qos),
        ]

    def feed(self, duration, x=0., y=0., yaw=0.):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            stamp = self.node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp, "map"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation.z = math.sin(yaw / 2.)
            odom.pose.pose.orientation.w = math.cos(yaw / 2.)
            odom.twist.twist.linear.x = .05
            self.odom.publish(odom)
            imu = Imu()
            imu.header.stamp = stamp
            self.imu.publish(imu)
            rclpy.spin_once(self.node, timeout_sec=.02)

    def measure(self, index, y, yaw, scan=False, braking=False):
        self.feed(.15)
        self.states.clear()
        endpoint = (.3 if scan else 1.) + .01 * index
        route = Route()
        route.header.stamp = self.node.get_clock().now().to_msg()
        route.header.frame_id = "map"
        pose = PoseStamped()
        pose.header = route.header
        pose.pose.position.x = endpoint
        pose.pose.position.z = .003 if scan else 0.
        pose.pose.orientation.w = 1.
        route.poses.append(pose)
        self.route.publish(route)
        self.feed(.25)
        self.commands.clear()
        x = endpoint if scan else endpoint - .03 if braking else .2
        self.feed(.65, x, y, yaw)
        assert len(self.commands) >= 5, self.states
        expected = "RFID_SCANNING" if scan else "BRAKING_APPROACH" if braking else "FOLLOWING"
        assert self.states and self.states[-1].startswith(expected), self.states
        return dict(v=statistics.median(r[0] for r in self.commands[-5:]),
                    w=statistics.median(r[1] for r in self.commands[-5:]),
                    state=self.states[-1])


def run_variant(package, executable, adaptive):
    source = WORKSPACE / "src"
    params = yaml.safe_load((source / "visual_navigation/config/waypoint_navigation.yaml").read_text())["waypoint_navigator"]["ros__parameters"]
    if package == "contrast_controller":
        params.update(yaml.safe_load((source / "contrast_controller/config/contrast_controller.yaml").read_text())["contrast_controller"]["ros__parameters"])
    params.update(parameters_for("waypoint_navigation"))
    for key in tuple(params):
        if key.endswith("_topic"):
            params[key] = "/flexible_test/" + key
    # 仅关闭没有启动提供方的门控，所有运动参数仍来自实际 YAML。
    params.update(dict(odom_topic="/flexible_test/odom", imu_topic="/flexible_test/imu",
        route_input_topic="/flexible_test/route", cmd_vel_topic="/flexible_test/cmd",
        status_topic="/flexible_test/state", require_fusion_status=False,
        require_actuator_health=False, require_tracking_state=False,
        path_adaptive_correction_enabled=adaptive,
        recovery_map_file=str(source / "new_map_planner/config/arena.yaml")))
    probe = Probe()
    with tempfile.TemporaryDirectory(prefix="flexible_ros_") as directory:
        config = Path(directory) / "params.yaml"
        config.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
        with (Path(directory) / "node.log").open("w+") as log:
            process = subprocess.Popen([str(WORKSPACE / "install" / package / "lib" / package / executable),
                "--ros-args", "-r", "__node:=flexible_navigator", "--params-file", str(config)],
                stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 8.
                while not probe.commands or not probe.route.get_subscription_count():
                    assert process.poll() is None, "节点退出"
                    assert time.monotonic() < deadline, "节点发现超时"
                    probe.feed(.1)
                cases = {}
                for index, (name, y, yaw, scan, brake) in enumerate([
                    ("小横偏", .005, 0., False, False), ("大横偏", .04, 0., False, False),
                    ("左偏", .04, .10, False, False), ("右偏", -.04, -.10, False, False),
                    ("正向扫描", .04, .08, True, False), ("停车区", .04, .20, False, True),
                ]):
                    cases[name] = probe.measure(index, y, yaw, scan, brake)
                assert cases["左偏"]["w"] < 0. < cases["右偏"]["w"]
                assert abs(cases["左偏"]["w"] + cases["右偏"]["w"]) < .003
                assert cases["正向扫描"]["w"] < 0.
                stopped = cases["停车区"]
                assert abs(stopped["w"]) <= params["pre_turn_path_angular_ratio"] * stopped["v"] + 1e-5
                if adaptive:
                    assert abs(cases["大横偏"]["w"]) > 2. * abs(cases["小横偏"]["w"])
                    assert cases["大横偏"]["v"] < cases["小横偏"]["v"]
                return cases
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                probe.node.destroy_node()


def main():
    rclpy.init()
    results = {}
    try:
        for package, executable in (("visual_navigation", "waypoint_navigator"),
                                    ("contrast_controller", "contrast_controller")):
            results[package] = {str(enabled): run_variant(package, executable, enabled)
                                for enabled in (False, True)}
            old, new = results[package]["False"], results[package]["True"]
            assert abs(new["大横偏"]["w"]) > 2. * abs(old["大横偏"]["w"])
        print(json.dumps(results, ensure_ascii=False, indent=2))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
