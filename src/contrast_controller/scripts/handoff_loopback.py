#!/usr/bin/env python3
"""复现两次避障后到任务点再规划，检查激活路线是否仍从真实车辆位置开始。

使用正式 launch、默认起点及参数；通过仿真障碍话题放置用户日志中的两个障碍。
输出位姿、状态、激活路线和完整 launch 日志；只停止本脚本启动的进程组。
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

from run_pose_recovery_scenarios import stop_owned


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=226)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=450)
    parser.add_argument("--scene", choices=["handoff", "cross_return"], default="handoff",
                        help="cross_return 复现用户 GUI 的边界 T、十字转弯连续遇障")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"

    import rclpy
    from geometry_msgs.msg import Point32, PolygonStamped
    from mission_control_interfaces.msg import MissionState, NavigationRecoveryState
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("recovery_handoff_check")
    root = "/comparison/contrast"
    durable = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    events, odom, recoveries = [], [], set()
    pose, status, injected = None, "", 0
    after_goal = False
    result = {}
    subscriptions = []
    process = None
    started = time.monotonic()
    continuation_start = None

    def event(kind, **data):
        row = dict(t=round(time.monotonic()-started, 3), kind=kind, **data)
        events.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)

    def on_odom(msg):
        nonlocal pose, continuation_start
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = [p.x, p.y, yaw]
        odom.append(dict(t=time.monotonic()-started, pose=pose, status=status,
                         linear=msg.twist.twist.linear.x, angular=msg.twist.twist.angular.z))
        if args.scene == "cross_return" and len(recoveries) >= 2:
            if status == "FOLLOWING" and msg.twist.twist.linear.x > .005 and abs(msg.twist.twist.angular.z) < .1:
                if continuation_start is None:
                    continuation_start = pose[:2]
                distance = math.hypot(p.x-continuation_start[0], p.y-continuation_start[1])
                if distance >= .20:
                    result.update(passed=True, reason="默认任务连续两次避障后实测前进超过 20 cm",
                                  continuation_m=distance)
            else:
                continuation_start = None

    def on_status(msg):
        nonlocal status, after_goal
        if msg.data == status:
            return
        status = msg.data
        event("status", status=status, pose=pose)
        if status in {"RETREATING_TO_ROUTE_START", "RETREATING_TO_JUNCTION"}:
            result.update(passed=False, reason="严格模式触发了旧航向倒车")
        if status == "GOAL_REACHED" and len(recoveries) >= 2:
            after_goal = True

    def on_recovery(msg):
        if msg.phase == "complete" and msg.recovery_id not in recoveries:
            recoveries.add(msg.recovery_id)
            event("recovery_complete", recovery_id=msg.recovery_id, pose=pose)
        if msg.phase == "failed":
            result.update(passed=False, reason=msg.reason)

    def on_route(msg):
        if pose is None or not msg.poses:
            return
        p = msg.poses[0].pose.position
        error = math.hypot(p.x-pose[0], p.y-pose[1])
        event("activated_route", first=[p.x, p.y], pose=pose, error_m=error,
              after_two_recoveries_goal=after_goal)
        if error > .08:
            result.update(passed=False, reason="激活路线起点与实测位姿相差超过 8 cm")
        elif after_goal and args.scene == "handoff":
            result.update(passed=True, reason="两次避障后任务点再规划使用实时位置",
                          next_route_error_m=error)

    def on_mission(msg):
        if msg.state == MissionState.FAULT:
            event("mission_fault", message=msg.message, pose=pose)
            result.update(passed=False, reason=msg.message)

    for message_type, topic, callback, qos in [
        (Odometry, root+"/odometry/fused", on_odom, 10),
        (String, root+"/navigator_status", on_status, durable),
        (NavigationRecoveryState, root+"/navigation_recovery_state", on_recovery, durable),
        (RosPath, "/waypoint_navigation/route_input", on_route, 10),
        (MissionState, "/navigation_mission/state", on_mission, 10),
    ]:
        subscriptions.append(node.create_subscription(message_type, topic, callback, qos))
    publisher = node.create_publisher(PolygonStamped, root+"/obstacle/manual", 10)

    def inject(x, y, hx, hy):
        msg = PolygonStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.polygon.points = [Point32(x=a, y=b, z=0.0) for a, b in
                              [(x-hx,y-hy), (x+hx,y-hy), (x+hx,y+hy), (x-hx,y+hy)]]
        publisher.publish(msg)
        event("obstacle", center=[x,y], half_size=[hx,hy], pose=pose)

    try:
        command = ["ros2", "launch", "contrast_controller", "obstacle_validation.launch.py",
                   "gui:=false", "auto_start:=true"]
        (args.output/"launch_command.json").write_text(json.dumps(command, indent=2)+"\n")
        with (args.output/"launch.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            while not result and time.monotonic()-started < args.timeout:
                rclpy.spin_once(node, timeout_sec=.03)
                if process.poll() is not None:
                    raise RuntimeError("launch 提前退出")
                if pose is None or publisher.get_subscription_count() == 0:
                    continue
                x, y, yaw = pose
                if args.scene == "cross_return":
                    if injected == 0 and status == "FOLLOWING" and 2.15 < x < 2.30 and abs(y+1.5) < .03 and abs(yaw) < .1:
                        inject(2.639, -1.5, .06, .10)
                        injected = 1
                    elif injected == 1 and recoveries and status == "FOLLOWING" and abs(x-2.2) < .03 and -1.2 < y < -.8:
                        inject(2.524, -.5, .06, .10)
                        injected = 2
                    continue
                if injected == 0 and status == "FOLLOWING" and .5 < x < .9 and abs(y+.5) < .03:
                    inject(1.2, -1.007, .10, .05)
                    injected = 1
                elif injected == 1 and recoveries and status == "FOLLOWING" and .5 < x < .9 and abs(y-.5) < .03:
                    inject(1.778, 1.5, .05, .10)
                    injected = 2
        if not result:
            result.update(passed=False, reason="超时，尚未完成场景的两次恢复和后续行驶")
    except (Exception, KeyboardInterrupt) as error:
        result.update(passed=False, reason=str(error) or "中断")
    finally:
        stop_owned(process)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result.update(scene=args.scene, recovery_count=len(recoveries), injected=injected,
                      after_two_recoveries_goal=after_goal, elapsed_s=time.monotonic()-started)
        log = (args.output/"launch.log").read_text(errors="replace")
        if "collision guard blocked" in log:
            result.update(passed=False, reason="仿真碰撞保护介入")
        requests = re.findall(r"发送规划请求:.*start=\(([^)]+)\)", log)
        result["manager_request_starts"] = requests
        planner_starts = re.findall(r"plan request:.*start_map=\(([^)]+)\)", log)
        result["planner_request_starts"] = planner_starts
        if after_goal and planner_starts and pose is not None:
            actual_start = [float(value) for value in planner_starts[-1].split(",")[:2]]
            error = math.hypot(actual_start[0]-pose[0], actual_start[1]-pose[1])
            result["post_goal_request_error_m"] = error
            result["post_goal_request_uses_live_position"] = error < .01
        for name, data in [("events",events), ("odom",odom), ("result",result)]:
            (args.output/(name+".json")).write_text(json.dumps(data, ensure_ascii=False, indent=2)+"\n")
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
