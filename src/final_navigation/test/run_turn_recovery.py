#!/usr/bin/env python3
"""真实 Contrast 节点验收转弯无进展后的回放恢复；不连接车辆硬件。

保留正式转弯及恢复超时，冻结运动学制造故障。复用出门测试中的物理
配置、轮速遥测和节点参数，退出时关闭节点并删除临时参数与日志。
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from run_launch_ros import Probe as LaunchProbe, WORKSPACE, parameters

# run_launch_ros 在导入时设置自己的域；本验收使用独立域。
os.environ["ROS_DOMAIN_ID"] = "189"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from mission_control_interfaces.srv import ContinueRetreat
import yaml


class Probe(LaunchProbe):
    def __init__(self, start_x, physical):
        super().__init__(start_x, physical)
        self.retreat = self.node.create_client(
            ContinueRetreat, "/launch_navigator/continue_route_retreat")
        self.events = []

    def on_state(self, message):
        super().on_state(message)
        self.events.append((time.monotonic(), message.data))

    def until(self, predicate, timeout, integrate=True):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, ("状态等待超时", self.states)
            self.tick(integrate)

    def request_retreat(self, distance):
        request = ContinueRetreat.Request(minimum_distance_m=distance)
        future = self.retreat.call_async(request)
        self.until(future.done, 3., False)
        response = future.result()
        assert response is not None, "回放服务没有响应"
        return response


def run_case(name):
    params, physical = parameters()
    mission = yaml.safe_load((WORKSPACE / "src/navigation_mission_manager/config/mission_manager.yaml")
                             .read_text())
    distance = next(iter(mission.values()))["ros__parameters"]["retreat_step_m"]
    # 空轨迹用例直接从出门停车点开始；不伪造任何已行驶位置或轮速。
    probe = Probe(.11 if name == "empty_trace" else 0., physical)
    with tempfile.TemporaryDirectory(prefix="turn_recovery_ros_") as directory:
        config = Path(directory) / "params.yaml"
        config.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
        with (Path(directory) / "node.log").open("w+") as log:
            binary = WORKSPACE / "install/final_navigation/lib/final_navigation/final_navigation"
            process = subprocess.Popen([str(binary), "--ros-args", "-r",
                "__node:=launch_navigator", "--params-file", str(config)],
                stdout=log, stderr=subprocess.STDOUT)
            try:
                probe.until(lambda: probe.rows and probe.route.get_subscription_count()
                            and probe.retreat.service_is_ready(), 8., False)
                assert process.poll() is None, "控制器启动失败"
                probe.states.clear()
                probe.events.clear()
                probe.rows.clear()
                route = probe.make_route(0., -1.)
                if name == "empty_trace":
                    route.poses[0].pose.position.x = .105
                probe.route.publish(route)
                probe.until(lambda: probe.state == "TURN_60", 15., name != "empty_trace")
                turn_started = next(t for t, state in probe.events if state == "TURN_60")
                # 冻结融合位姿和实际轮速，保留真实控制器的命令及进展判定。
                probe.until(lambda: probe.state == "TURN_NEED_REPLAN", 4., False)
                turn_failed = next(t for t, state in probe.events if state == "TURN_NEED_REPLAN")
                elapsed = turn_failed - turn_started
                assert params["turn_maneuver_no_progress_timeout"] - .1 <= elapsed < 2.2, elapsed
                probe.feed(.1, False)
                assert probe.command == (0., 0.), ("失败后仍输出运动命令", probe.command)
                failure_x = probe.x
                response = probe.request_retreat(distance)
                result = dict(case=name, no_progress_seconds=elapsed,
                              accepted=response.accepted, trace_exhausted=response.trace_exhausted,
                              response=response.message, failure_x=failure_x)
                if name == "empty_trace":
                    assert not response.accepted and response.trace_exhausted, result
                    probe.feed(.15, False)
                    assert probe.command == (0., 0.), probe.command
                else:
                    assert response.accepted, result
                    probe.until(lambda: probe.state == "ROUTE_TRANSITION_BRAKING", 1., False)
                    braking_rows = len(probe.rows)
                    probe.until(lambda: probe.state == "ROUTE_TRANSITION_REPLAYING", 4., False)
                    # 停车确认完成前应持续输出零命令；最后一条属于首次回放。
                    assert all(abs(row[3]) < 1e-9 and abs(row[4]) < 1e-9
                               for row in probe.rows[braking_rows:-1]), probe.rows[braking_rows:]
                    replay_started = time.monotonic()
                    if name == "success":
                        probe.until(lambda: probe.state == "ROUTE_TRANSITION_REPLAN_REQUIRED",
                                    12., True)
                        assert failure_x - probe.x > .03, (failure_x, probe.x)
                        assert any(row[3] < -.01 for row in probe.rows), "没有执行倒车回放"
                        assert not any(state.startswith("FAULT") for state, *_ in probe.states)
                    else:
                        # 真实恢复重试与正式 3 秒无进展超时应最终进入明确故障。
                        max_wait = (params["obstacle_reverse_max_recovery_attempts"] + 1) * (
                            params["obstacle_reverse_no_progress_timeout"] + 2.) + 2.
                        probe.until(lambda: probe.state.startswith("FAULT_ROUTE_TRANSITION_"),
                                    max_wait, False)
                        assert probe.state == "FAULT_ROUTE_TRANSITION_NO_PROGRESS", probe.states
                    probe.feed(.1, False)
                    assert probe.command == (0., 0.), ("终态没有停车", probe.command)
                    result.update(final_x=probe.x, terminal_state=probe.state,
                                  replay_seconds=time.monotonic() - replay_started)
                    if name == "success":
                        replacement = probe.make_route(probe.x, -1.)
                        replacement.poses = replacement.poses[:2]
                        for index, pose in enumerate(replacement.poses):
                            pose.pose.position.x = probe.x + .3 * index
                            pose.pose.position.y = probe.y
                            pose.pose.position.z = 0.
                        resumed_x = probe.x
                        probe.route.publish(replacement)
                        probe.until(lambda: probe.state == "FOLLOWING", 3., True)
                        probe.feed(.5)
                        assert probe.x - resumed_x > .005, ("新路线未接管", probe.states)
                        assert probe.command[0] > 0., probe.command
                        result["replacement_route_resumed"] = True
                        result["replacement_advance_m"] = probe.x - resumed_x
                result["states"] = [state for state, *_ in probe.states]
                return result
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
    parser.add_argument("--case", choices=("all", "success", "frozen_replay", "empty_trace"),
                        default="all")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    try:
        results = [run_case(case) for case in ("success", "frozen_replay", "empty_trace")
                   if args.case in ("all", case)]
        report = json.dumps(results, ensure_ascii=False, indent=2)
        print(report)
        if args.output:
            args.output.write_text(report)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
