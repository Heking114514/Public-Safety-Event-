#!/usr/bin/env python3
"""一键启动的配置检查和延迟任务激活，不直接发布底盘速度。"""
from pathlib import Path
import json
import os
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen

import yaml

ROOT = Path(__file__).resolve().parents[1]


def configuration():
    with (ROOT / "config/mission_startup.yaml").open() as stream:
        config = yaml.safe_load(stream)["mission_startup"]
    with (ROOT / "config/navigation_startup.yaml").open() as stream:
        navigation = yaml.safe_load(stream)["navigation_startup"]
    if not isinstance(config["image_topic"], str) or not config["image_topic"].startswith("/"):
        raise ValueError("image_topic 必须是绝对 ROS 话题")
    if any(c in config["image_topic"] for c in "\n\r\t"):
        raise ValueError("image_topic 含非法字符")
    for key in ("activation_delay_s", "startup_timeout_s", "window_interval_s"):
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value < 3600:
            raise ValueError(f"{key} 必须在 0～3600 秒之间")
    port = config["insight_port"]
    if isinstance(port, bool) or not isinstance(port, int) or not 1024 <= port <= 65535:
        raise ValueError("insight_port 必须在 1024～65535 之间")
    return config, navigation


def check():
    insight = ROOT / "src/insight_cup"
    required = [
        ROOT / "build/insight_cup/insight-cup",
        insight / "models/yolo/best.onnx",
        insight / "models/face/models/face_only_r50/det_10g.onnx",
        insight / "models/face/models/face_only_r50/w600k_r50.onnx",
        insight / "models/knife/pplcnetv2_base_knife10/inference.onnx",
        insight / "models/knife/pplcnetv2_base_knife10/labels.txt",
        insight / "data/face_gallery/cpp/trainv5_gallery_r50.icg",
        insight / "outputs/distance/stage1_roi.json",
    ]
    missing = [str(path.relative_to(ROOT)) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError("缺少以下运行文件，本次不会打开窗口或启动小车：\n  " + "\n  ".join(missing))
    print("模型、图库、标定和识别可执行文件检查通过。", flush=True)


def recognition_ready(port):
    try:
        with urlopen(f"http://127.0.0.1:{port}/api/state", timeout=0.5) as response:
            state = json.load(response)
        return state.get("status") == "running" and state.get("processed_frames", 0) > 0
    except (URLError, TimeoutError, OSError, ValueError):
        return False


def activate(config, run_dir):
    import rclpy
    from mission_control_interfaces.msg import MissionState
    from mission_control_interfaces.srv import ControlMission
    from rclpy.qos import QoSProfile, DurabilityPolicy

    rclpy.init()
    node = rclpy.create_node("mission_startup")
    state = None

    def on_state(message):
        nonlocal state
        state = message

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(MissionState, "/navigation_mission/state", on_state, qos)
    client = node.create_client(ControlMission, "/navigation_mission/control")
    deadline = time.monotonic() + config["startup_timeout_s"]
    activate_at = None
    print("等待规划界面节点出现……", flush=True)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if (run_dir / "cancel").exists():
                raise RuntimeError("总启动脚本已取消，本次未发送 START")
            for role in ("insight", "navigation", "planner"):
                pid_file = run_dir / f"{role}.pid"
                if not pid_file.exists():
                    raise RuntimeError(f"{role} 窗口中的进程已退出，取消自动启动")
                os.kill(int(pid_file.read_text()), 0)
            ui_present = "new_map_planner_ui" in node.get_node_names()
            if activate_at is None and ui_present:
                activate_at = time.monotonic() + config["activation_delay_s"]
                print(f"规划界面节点已出现，{config['activation_delay_s']:g} 秒后尝试启动完整巡视。", flush=True)
            if activate_at is None or time.monotonic() < activate_at:
                continue
            if not ui_present:
                raise RuntimeError("规划界面已关闭，取消自动启动")
            if state is not None and state.state not in (MissionState.IDLE, MissionState.WAIT_READY):
                raise RuntimeError(f"任务状态为 {state.state_name}，取消自动启动，避免覆盖人工操作")
            if state is None or state.state != MissionState.IDLE or not client.service_is_ready():
                continue
            if not recognition_ready(config["insight_port"]):
                continue
            request = ControlMission.Request()
            request.command = ControlMission.Request.START
            request.target_ids = []  # 使用任务总控 YAML 中的默认任务点。
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if not future.done():
                raise RuntimeError("START 响应超时，结果未知；不重发，请检查任务状态")
            response = future.result()
            if not response.accepted:
                raise RuntimeError(f"启动被总控拒绝：{response.message}；请检查后手动启动")
            print(f"自动启动已接受：{response.message}；总控将规划、下发并激活导航。", flush=True)
            return
        raise RuntimeError("等待系统就绪超时，本次未发送 START")
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main():
    config, navigation = configuration()
    action = sys.argv[1]
    if action == "settings":
        for value in (navigation["ros_domain_id"], navigation["rmw_implementation"],
                      config["image_topic"], config["insight_port"], config["window_interval_s"]):
            print(value)
    elif action == "check":
        check()
    elif action == "activate":
        activate(config, Path(sys.argv[2]))
    else:
        raise ValueError(f"未知动作：{action}")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError) as error:
        print(f"[一键启动] {error}", file=sys.stderr, flush=True)
        sys.exit(1)
