#!/usr/bin/env python3
"""独立 ROS 域验收出门 11 cm 停车和普通路口；不启动串口或车辆节点。

真实 Contrast 节点输出驱动理想运动学积分，物理参数来自工作空间 YAML。
仅验证首段停车及转弯入口，不以此替代实车、定位或转弯碰撞验收。
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "187"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mission_control_interfaces.msg import ControlTelemetry
from mission_control_interfaces.srv import SetMotionHold
from nav_msgs.msg import Odometry, Path as Route
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from vehicle_config import load_vehicle_physical_config, parameters_for
import yaml

WORKSPACE = Path(__file__).resolve().parents[3]


class Probe:
    def __init__(self, start_x, physical):
        self.node = rclpy.create_node("launch_probe")
        self.x, self.y, self.yaw = start_x, 0., 0.
        self.command = (0., 0.)
        self.state = ""
        self.states, self.rows = [], []
        self.sequence = 0
        self.physical = physical
        self.last_tick = time.monotonic()
        self.odom = self.node.create_publisher(Odometry, "/launch_test/odom", 10)
        self.imu = self.node.create_publisher(Imu, "/launch_test/imu", 10)
        self.telemetry = self.node.create_publisher(ControlTelemetry, "/launch_test/wheels", 10)
        self.route = self.node.create_publisher(Route, "/launch_test/route", 10)
        qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions = [
            self.node.create_subscription(Twist, "/launch_test/cmd", self.on_command, 10),
            self.node.create_subscription(String, "/launch_test/state", self.on_state, qos),
        ]
        self.hold = self.node.create_client(SetMotionHold, "/launch_navigator/set_motion_hold")

    def on_command(self, message):
        self.command = (message.linear.x, message.angular.z)
        self.rows.append((self.x, self.y, self.yaw, *self.command, self.state))

    def on_state(self, message):
        self.state = message.data
        self.states.append((self.state, self.x, self.y, self.yaw))

    def tick(self, integrate=True):
        current = time.monotonic()
        dt = current - self.last_tick
        self.last_tick = current
        v, w = self.command if integrate else (0., 0.)
        # cmd_vel 的线速度定义在轮轴，base_link 位于其前方 3 cm（YAML）。
        bx = self.physical.base_link_offset_x_m
        by = self.physical.base_link_offset_y_m
        vx, vy = v - w * by, w * bx
        middle = self.yaw + w * dt / 2.
        self.x += (vx * math.cos(middle) - vy * math.sin(middle)) * dt
        self.y += (vx * math.sin(middle) + vy * math.cos(middle)) * dt
        self.yaw += w * dt
        stamp = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        self.odom.publish(odom)
        imu = Imu()
        imu.header.stamp, imu.angular_velocity.z = stamp, w
        self.imu.publish(imu)
        self.sequence += 1
        wheels = ControlTelemetry()
        wheels.header.stamp = stamp
        wheels.sample_sequence = self.sequence
        wheels.mcu_time_ms = self.sequence * 10
        wheels.mode, wheels.command_valid = ControlTelemetry.MODE_NAVIGATION, True
        left, right = v - w * self.physical.wheel_track_m / 2., v + w * self.physical.wheel_track_m / 2.
        wheels.measured_left_velocity_mps = wheels.target_left_velocity_mps = left
        wheels.measured_right_velocity_mps = wheels.target_right_velocity_mps = right
        self.telemetry.publish(wheels)
        rclpy.spin_once(self.node, timeout_sec=.01)

    def feed(self, duration, integrate=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.tick(integrate)

    def make_route(self, start_x, direction):
        route = Route()
        route.header.stamp = self.node.get_clock().now().to_msg()
        route.header.frame_id = "map"
        for x, y, z in ((start_x, 0., 0.), (start_x + .2, 0., .001),
                        (start_x + .2, direction * .5, 0.)):
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.w = 1.
            route.poses.append(pose)
        return route

    def set_hold(self, value):
        request = SetMotionHold.Request(source="launch_acceptance", hold=value,
                                        reason="验证出门中途暂停与恢复")
        assert self.hold.service_is_ready(), "暂停服务未发现"
        future = self.hold.call_async(request)
        deadline = time.monotonic() + 2.
        while not future.done():
            assert time.monotonic() < deadline, "暂停服务超时"
            self.tick()
        assert future.result().success, future.result().message


def parameters():
    source = WORKSPACE / "src"
    physical = load_vehicle_physical_config(source / "vehicle_config/config/vehicle_physical.yaml")
    params = yaml.safe_load((source / "visual_navigation/config/waypoint_navigation.yaml").read_text())["waypoint_navigator"]["ros__parameters"]
    params.update(yaml.safe_load((source / "contrast_controller/config/contrast_controller.yaml").read_text())["contrast_controller"]["ros__parameters"])
    params.update(parameters_for("waypoint_navigation", physical))
    for key in tuple(params):
        if key.endswith("_topic"):
            params[key] = "/launch_test/" + key
    # 隔离测试没有融合/硬件健康提供方；运动参数全部保留实际 YAML。
    params.update(odom_topic="/launch_test/odom", imu_topic="/launch_test/imu",
                  control_telemetry_topic="/launch_test/wheels",
                  route_input_topic="/launch_test/route", cmd_vel_topic="/launch_test/cmd",
                  status_topic="/launch_test/state", require_fusion_status=False,
                  require_actuator_health=False, require_tracking_state=False,
                  recovery_map_file=str(source / "new_map_planner/config/arena.yaml"))
    return params, physical


def run_case(name, start_x, direction, pause=False):
    params, physical = parameters()
    probe = Probe(start_x, physical)
    with tempfile.TemporaryDirectory(prefix="launch_ros_") as directory:
        config = Path(directory) / "params.yaml"
        config.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
        with (Path(directory) / "node.log").open("w+") as log:
            binary = WORKSPACE / "install/contrast_controller/lib/contrast_controller/contrast_controller"
            process = subprocess.Popen([str(binary), "--ros-args", "-r",
                "__node:=launch_navigator", "--params-file", str(config)],
                stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 8.
                while not probe.rows or not probe.route.get_subscription_count() or not probe.hold.service_is_ready():
                    assert process.poll() is None, "节点退出"
                    assert time.monotonic() < deadline, "节点发现超时"
                    probe.feed(.1, False)
                route = probe.make_route(start_x, direction)
                probe.states.clear()
                probe.rows.clear()
                probe.route.publish(route)
                deadline = time.monotonic() + 15.
                paused = False
                while not any(state == "TURN_60" for state, *_ in probe.states):
                    assert process.poll() is None, "节点提前退出"
                    assert time.monotonic() < deadline, (name, "入弯超时", probe.states)
                    assert not any(state.startswith("FAULT") or state == "TURN_NEED_REPLAN"
                                   for state, *_ in probe.states), probe.states
                    probe.tick()
                    if pause and not paused and probe.x > start_x + .04:
                        probe.set_hold(True)
                        probe.feed(.15)
                        held_x = probe.x
                        probe.feed(.15)
                        assert abs(probe.x - held_x) < .001, "暂停期间仍然移动"
                        # ACK 丢失重发同一几何路线不能清空已执行进度。
                        route.header.stamp = probe.node.get_clock().now().to_msg()
                        probe.route.publish(route)
                        probe.feed(.1)
                        probe.set_hold(False)
                        paused = True
                stops = [x for state, x, _, _ in probe.states if state == "BRAKING_AT_WAYPOINT"]
                assert stops, probe.states
                stop_x = stops[0]
                turn_x = next(x for state, x, _, _ in probe.states if state == "TURN_60")
                maximum = max(row[0] for row in probe.rows)
                reverse = min(row[3] for row in probe.rows)
                if start_x == 0.:
                    assert abs(stop_x - .11) < .006, ("出门停车位置", stop_x)
                    assert abs(turn_x - .11) < .012, ("出门入弯位置", turn_x)
                    assert maximum < .125, ("先超前再倒退", maximum)
                    assert reverse >= -1e-9, ("首弯前不应倒退", reverse)
                else:
                    assert abs(stop_x - (start_x + .175)) < .006, ("普通路口停车改变", stop_x)
                    assert abs(turn_x - (start_x + .13)) < .012, ("普通路口入口改变", turn_x)
                    assert reverse < -.01, ("普通路口未保持原倒车入口调整", reverse)
                return dict(name=name, first_stop_x=stop_x, turn_60_x=turn_x,
                            maximum_x=maximum, minimum_v=reverse, paused=paused,
                            states=[state for state, *_ in probe.states])
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--case", choices=("all", "right", "left", "ordinary", "pause"), default="all")
    args = parser.parse_args()
    cases = (("right", 0., -1., False), ("left", 0., 1., False),
             ("ordinary", 1., -1., False), ("pause", 0., -1., True))
    rclpy.init()
    results = []
    try:
        for case in cases:
            if args.case in ("all", case[0]):
                results.append(run_case(*case))
        report = json.dumps(results, ensure_ascii=False, indent=2)
        print(report)
        if args.output:
            args.output.write_text(report)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
