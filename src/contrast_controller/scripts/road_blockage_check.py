#!/usr/bin/env python3
"""复现 RFID 层结束处的已知封路恢复；通过公开服务验证局部恢复和隧道续行。

精确起点与两块障碍来自 2026-10-15 用户 GUI 现场。此脚本不模拟此前
12 个任务点的执行，也不代表三层任务验收。仅停止自身启动的进程组。
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

from run_pose_recovery_scenarios import stop_owned


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=229)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=150)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from geometry_msgs.msg import Point32, Polygon, PolygonStamped, Pose, PoseStamped
    from mission_control_interfaces.msg import NavigationRecoveryState
    from mission_control_interfaces.srv import SelectRecovery, StartRoadRecovery
    from nav_msgs.msg import Odometry, Path as RosPath
    from new_map_planner.srv import PlanRoute
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("road_blockage_check")
    root = "/comparison/contrast"
    controller = root + "/contrast_controller"
    durable = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    observations = {"status": "", "pose": None, "recovery": None, "fusion": ""}
    events, odom = [], []
    started = time.monotonic()
    deadline = started + args.timeout
    process = None
    result = {"passed": False}

    def event(kind, **values):
        row = dict(t=time.monotonic()-started, kind=kind, **values)
        events.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)

    def status(msg):
        if observations["status"] != msg.data:
            observations["status"] = msg.data
            event("status", value=msg.data)

    def recovery(msg):
        observations["recovery"] = msg
        event("recovery", phase=msg.phase, reason=msg.reason,
              recovery_id=msg.recovery_id, candidates=len(msg.candidates))

    def position(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        observations["pose"] = msg.pose.pose
        odom.append(dict(t=time.monotonic()-started, x=p.x, y=p.y,
                         yaw=math.atan2(2*q.w*q.z, 1-2*q.z*q.z),
                         linear=msg.twist.twist.linear.x,
                         angular=msg.twist.twist.angular.z,
                         status=observations["status"]))

    def fusion(msg):
        observations["fusion"] = msg.data

    subscriptions = [node.create_subscription(String, root+"/navigator_status", status, durable),
                     node.create_subscription(NavigationRecoveryState, root+"/navigation_recovery_state", recovery, durable),
                     node.create_subscription(Odometry, root+"/odometry/fused", position, 10),
                     node.create_subscription(String, root+"/odometry/fusion_status", fusion, 10)]
    publisher = node.create_publisher(RosPath, "/waypoint_navigation/route_input", 10)
    obstacle_publisher = node.create_publisher(PolygonStamped, root+"/obstacle/manual", 10)
    start_client = node.create_client(StartRoadRecovery, controller+"/start_road_recovery")
    select_client = node.create_client(SelectRecovery, controller+"/select_recovery")
    plan_client = node.create_client(PlanRoute, "/new_map_planner/plan")
    blocked = ["ROAD_X1_Y3", "ROAD_Y3_LEFT"]
    boxes = [(.1, -.55, .3, -.45), (1.1, -1.2263966068711274, 1.3, -1.1263966068711273)]
    obstacles = [Polygon(points=[Point32(x=x, y=y) for x, y in
                                [(a,b), (c,b), (c,d), (a,d)]]) for a,b,c,d in boxes]

    def wait_until(predicate):
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.03)
            if "FAULT" in observations["status"]:
                raise RuntimeError(observations["status"])
            if predicate():
                return
            if process.poll() is not None:
                raise RuntimeError("launch 提前退出")
        raise TimeoutError("未在期限内完成封路恢复及续行")

    def call(client, request):
        future = client.call_async(request)
        wait_until(future.done)
        return future.result()

    def plan(pose):
        request = PlanRoute.Request(start=pose, mode="layer2", required_lane_ids=["TUNNEL_1", "TUNNEL_2", "TUNNEL_3", "TUNNEL_4"],
                                    dynamic_obstacles=obstacles, blocked_lane_ids=blocked, obstacle_revision=4,
                                    validate_geometry=True, online_geometry_validation=True, allow_partial=True,
                                    ordinary_nodes_since_task_exact=True)
        return call(plan_client, request)

    command = ["ros2", "launch", "contrast_controller", "contrast_comparison.launch.py",
               "controllers:=contrast", "gui:=false", "plan_on_start:=false", "obstacle_detection:=true",
               "initial_x:=0.23876371210204264", "initial_y:=0.49981146930033304", "initial_yaw:=-3.140625147480947"]
    try:
        with (args.output/"launch.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            wait_until(lambda: observations["pose"] is not None and publisher.get_subscription_count() > 0 and
                       "FULL" in observations["fusion"] and len(odom)>30 and
                       obstacle_publisher.get_subscription_count()>0 and
                       all(c.service_is_ready() for c in [start_client, select_client, plan_client]))
            for polygon in obstacles:
                message = PolygonStamped(polygon=polygon)
                message.header.frame_id = "map"
                message.header.stamp = node.get_clock().now().to_msg()
                obstacle_publisher.publish(message)
            original = plan(observations["pose"])
            event("original_plan", success=original.success, status=original.plan.status, message=original.message)
            if original.success or original.plan.status != "NO_FEASIBLE_ROUTE":
                raise RuntimeError("未复现现场前进规划无路")
            # 建立真实路线完成令牌；末点距现场位姿 3.88 cm，在正常到点范围内。
            path = RosPath()
            path.header.frame_id = "map"
            path.header.stamp = node.get_clock().now().to_msg()
            first = PoseStamped(pose=observations["pose"])
            last = PoseStamped(pose=Pose())
            last.pose.position.x, last.pose.position.y = .2, .5
            last.pose.orientation.z, last.pose.orientation.w = -1., 0.
            path.poses = [first, last]
            publisher.publish(path)
            wait_until(lambda: observations["status"] == "GOAL_REACHED")
            route_id = path.header.stamp.sec*10**9 + path.header.stamp.nanosec
            response = call(start_client, StartRoadRecovery.Request(route_id=route_id, obstacle_revision=4,
                                                                  obstacles=obstacles, blocked_lane_ids=blocked))
            event("start_road_recovery", accepted=response.accepted, message=response.message)
            if not response.accepted:
                raise RuntimeError(response.message)
            wait_until(lambda: observations["recovery"] is not None and observations["recovery"].phase in {"candidates", "failed"})
            state = observations["recovery"]
            if state.phase == "failed":
                raise RuntimeError(state.reason)
            selected = None
            for candidate in state.candidates:
                if any(lane in blocked for lane in candidate.borrowed_lane_ids+[candidate.resume_lane]):
                    continue
                evaluation = plan(candidate.exit_pose)
                event("candidate_plan", candidate_id=candidate.candidate_id, success=evaluation.success,
                      message=evaluation.message, resume_node=candidate.resume_node, resume_lane=candidate.resume_lane)
                if evaluation.success:
                    selected = candidate
                    break
            if selected is None:
                raise RuntimeError("没有可接续隧道任务的恢复出口")
            response = call(select_client, SelectRecovery.Request(recovery_id=state.recovery_id, obstacle_revision=4,
                                                                 candidate_id=selected.candidate_id, blocked_lane_ids=blocked))
            if not response.success:
                raise RuntimeError(response.message)
            wait_until(lambda: observations["recovery"].phase in {"complete", "failed"})
            state = observations["recovery"]
            if state.phase == "failed":
                raise RuntimeError(state.reason)
            continuation = plan(state.safe_pose.pose)
            event("continuation_plan", success=continuation.success, message=continuation.message)
            if not continuation.success:
                raise RuntimeError(continuation.message)
            publisher.publish(continuation.plan.navigation_path)
            def continued():
                eligible = []
                for row in reversed(odom):
                    if row["status"] != "FOLLOWING" or row["linear"] <= .005 or abs(row["angular"]) >= .1:
                        break
                    eligible.append(row)
                if len(eligible) < 2:
                    return False
                distance = math.hypot(eligible[0]["x"]-eligible[-1]["x"], eligible[0]["y"]-eligible[-1]["y"])
                result["continuation_m"] = distance
                return distance >= .20
            wait_until(continued)
            reverse = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"]) for a,b in zip(odom,odom[1:])
                          if b["status"] == "POSE_RECOVERY_RETREAT_TO_TURN_POINT" and b["linear"] < -.005)
            result.update(passed=reverse>.8, reverse_m=reverse, resume_node=selected.resume_node,
                          resume_lane=selected.resume_lane, reason="封路恢复后隧道路线接续并实测前进 20 cm")
    except (Exception, KeyboardInterrupt) as error:
        result.update(passed=False, reason=str(error) or "中断")
    finally:
        stop_owned(process)
        node.destroy_node()
        rclpy.shutdown()
        if "collision guard blocked" in (args.output/"launch.log").read_text(errors="replace"):
            result.update(passed=False, reason="仿真碰撞保护介入")
        result["elapsed_s"] = time.monotonic()-started
        for name, data in [("result",result), ("events",events), ("odom",odom)]:
            (args.output/(name+".json")).write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n")
        print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
