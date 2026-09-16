#!/usr/bin/env python3
"""Publish deterministic fake fused odometry for isolated navigation tests."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Optional


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "arena_path_planner"
    / "config"
    / "arena_map.yaml"
)


@dataclass
class PlanarPose:
    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def arena_pose_to_map_pose(arena_pose: PlanarPose, start_pose: PlanarPose) -> PlanarPose:
    dx = arena_pose.x - start_pose.x
    dy = arena_pose.y - start_pose.y
    cosine = math.cos(start_pose.yaw)
    sine = math.sin(start_pose.yaw)
    return PlanarPose(
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        normalize_angle(arena_pose.yaw - start_pose.yaw),
    )


def load_start_pose(config_path: Path) -> PlanarPose:
    import yaml

    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    start = config["start"]
    position = start["position_m"]
    return PlanarPose(
        float(position[0]),
        float(position[1]),
        math.radians(float(start["heading_deg"])),
    )


def finite_number(value: Optional[float], fallback: float) -> float:
    if value is None:
        return fallback
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("pose values must be finite")
    return value


def resolve_initial_pose(args: argparse.Namespace) -> PlanarPose:
    map_fields = (args.map_x, args.map_y, args.map_yaw_deg)
    if any(value is not None for value in map_fields):
        return PlanarPose(
            finite_number(args.map_x, 0.0),
            finite_number(args.map_y, 0.0),
            math.radians(finite_number(args.map_yaw_deg, 0.0)),
        )

    start_pose = load_start_pose(args.config)
    arena_pose = PlanarPose(
        finite_number(args.arena_x, start_pose.x),
        finite_number(args.arena_y, start_pose.y),
        math.radians(finite_number(args.arena_yaw_deg, math.degrees(start_pose.yaw))),
    )
    return arena_pose_to_map_pose(arena_pose, start_pose)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Publish fake /odometry/fused samples in map->base_link. By default "
            "the pose is the arena-map start pose, which appears in the GUI at "
            "the configured arena start."
        )
    )
    parser.add_argument("--topic", default="/odometry/fused")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--child-frame-id", default="base_link")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--arena-x", type=float)
    parser.add_argument("--arena-y", type=float)
    parser.add_argument("--arena-yaw-deg", type=float)
    parser.add_argument("--map-x", type=float)
    parser.add_argument("--map-y", type=float)
    parser.add_argument("--map-yaw-deg", type=float)
    parser.add_argument("--velocity-mps", type=float, default=0.0)
    parser.add_argument("--yaw-rate-deg-s", type=float, default=0.0)
    parser.add_argument("--publish-fusion-status", action="store_true")
    parser.add_argument("--fusion-status-topic", default="/odometry/fusion_status")
    parser.add_argument("--fusion-status", default="FULL")
    return parser


def run(args: argparse.Namespace) -> None:
    if args.rate_hz <= 0.0 or not math.isfinite(args.rate_hz):
        raise ValueError("--rate-hz must be finite and positive")
    if args.duration_s < 0.0 or not math.isfinite(args.duration_s):
        raise ValueError("--duration-s must be finite and non-negative")
    if not args.config.exists():
        raise FileNotFoundError(args.config)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    pose = resolve_initial_pose(args)
    velocity = finite_number(args.velocity_mps, 0.0)
    yaw_rate = math.radians(finite_number(args.yaw_rate_deg_s, 0.0))

    rclpy.init()
    node = rclpy.create_node("virtual_odometry_publisher")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    odom_publisher = node.create_publisher(Odometry, args.topic, qos)
    status_publisher = None
    if args.publish_fusion_status:
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_publisher = node.create_publisher(
            String, args.fusion_status_topic, status_qos
        )

    last_steady = time.monotonic()
    started_at = last_steady

    def publish() -> None:
        nonlocal pose, last_steady
        current_steady = time.monotonic()
        dt = max(0.0, min(1.0, current_steady - last_steady))
        last_steady = current_steady
        if dt > 0.0:
            pose = PlanarPose(
                pose.x + velocity * math.cos(pose.yaw) * dt,
                pose.y + velocity * math.sin(pose.yaw) * dt,
                normalize_angle(pose.yaw + yaw_rate * dt),
            )

        message = Odometry()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = args.frame_id
        message.child_frame_id = args.child_frame_id
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_quaternion(pose.yaw)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.pose.covariance[0] = 1.0e-4
        message.pose.covariance[7] = 1.0e-4
        message.pose.covariance[14] = 1.0e6
        message.pose.covariance[21] = 1.0e6
        message.pose.covariance[28] = 1.0e6
        message.pose.covariance[35] = 1.0e-4
        message.twist.twist.linear.x = velocity
        message.twist.twist.angular.z = yaw_rate
        message.twist.covariance[0] = 1.0e-4
        message.twist.covariance[35] = 1.0e-4
        odom_publisher.publish(message)

        if status_publisher is not None:
            status = String()
            status.data = args.fusion_status
            status_publisher.publish(status)

    timer = node.create_timer(1.0 / args.rate_hz, publish)
    node.get_logger().info(
        f"publishing fake odometry on {args.topic} as "
        f"{args.frame_id}->{args.child_frame_id} at {args.rate_hz:.1f} Hz"
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration_s > 0.0 and time.monotonic() - started_at >= args.duration_s:
                break
    finally:
        timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = build_parser()
    run(parser.parse_args())


if __name__ == "__main__":
    main()
