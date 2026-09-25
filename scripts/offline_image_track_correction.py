#!/usr/bin/env python3
"""Offline image-based track/map correction for fused odometry."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sqlite3
import subprocess
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import rosbag2_py

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

VISION_SOURCE = Path(__file__).resolve().parents[1] / "src" / "vision_correction"
if str(VISION_SOURCE) not in sys.path:
    sys.path.insert(0, str(VISION_SOURCE))
from vision_correction import road_detector  # noqa: E402


DEFAULT_BAG = Path("/media/hjh/Data/rosbag_recording/latest_navigation_bag")
DEFAULT_MAP = Path("src/arena_path_planner/config/arena_map.yaml")


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class LineFeature:
    ax: float
    ay: float
    bx: float
    by: float
    label: str
    kind: str


@dataclass(frozen=True)
class GroundSegment:
    ax: float
    ay: float
    bx: float
    by: float
    length: float
    yaw: float


@dataclass(frozen=True)
class ObservationFit:
    residual_x: float
    residual_y: float
    residual_yaw: float
    matches: int
    labels: str
    total_weight: float
    mean_distance: float
    mean_angle_error: float
    score: float


@dataclass
class CameraModel:
    fx: float = 427.893615722656
    fy: float = 427.893615722656
    cx: float = 421.134613037109
    cy: float = 242.897872924805
    x: float = 0.055
    y: float = 0.0
    z: float = 0.121
    pitch_down: float = 0.0
    distortion: np.ndarray | None = None
    color_to_base: np.ndarray | None = None


@dataclass
class CorrectionRow:
    stamp: float
    fused_x: float
    fused_y: float
    fused_yaw: float
    corrected_x: float
    corrected_y: float
    corrected_yaw: float
    offset_x: float
    offset_y: float
    offset_yaw: float
    residual_x: float
    residual_y: float
    residual_yaw: float
    detected_segments: int
    matched_segments: int
    matched_labels: str
    map_feature_error_fused: float
    map_feature_error_corrected: float
    visual_error_fused: float
    visual_error_corrected: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 1.0e-12:
        return 0.0
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def set_yaw_quaternion(q, yaw: float) -> None:
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)


def stamp_seconds(msg, fallback_ns: int) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is not None:
        value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if value > 0.0:
            return value
    return float(fallback_ns) * 1.0e-9


def decompress_bag_files(bag: Path, cache_dir: Path) -> list[Path]:
    if bag.is_file() and bag.suffix == ".db3":
        return [bag]
    if bag.is_dir():
        db3 = sorted(bag.glob("*.db3"))
        if db3:
            return db3
        compressed = sorted(bag.glob("*.db3.zstd"))
        if compressed:
            cache_dir.mkdir(parents=True, exist_ok=True)
            outputs: list[Path] = []
            for source in compressed:
                target = cache_dir / source.name.removesuffix(".zstd")
                if not target.exists() or target.stat().st_size == 0:
                    subprocess.run(
                        ["zstd", "-d", "-f", str(source), "-o", str(target)],
                        check=True,
                    )
                outputs.append(target)
            return outputs
    raise FileNotFoundError(f"no readable rosbag sqlite file found at {bag}")


def topic_table(db_paths: list[Path]) -> dict[str, tuple[str, int]]:
    table: dict[str, tuple[str, int]] = {}
    for db_path in db_paths:
        with sqlite3.connect(db_path) as conn:
            rows = conn.execute(
                """
                select topics.name, topics.type, count(messages.id)
                from topics left join messages on topics.id = messages.topic_id
                group by topics.id
                """
            ).fetchall()
        for name, topic_type, count in rows:
            previous_type, previous_count = table.get(name, (topic_type, 0))
            table[name] = (previous_type, previous_count + int(count))
    return table


def iter_topic(db_paths: list[Path], topic: str):
    for db_path in db_paths:
        with sqlite3.connect(db_path) as conn:
            topic_row = conn.execute(
                "select id from topics where name = ?",
                (topic,),
            ).fetchone()
            if topic_row is None:
                continue
            topic_id = int(topic_row[0])
            cursor = conn.execute(
                "select timestamp, data from messages where topic_id = ? order by timestamp",
                (topic_id,),
            )
            for timestamp_ns, data in cursor:
                yield int(timestamp_ns), data


def load_odometry(db_paths: list[Path], topic: str) -> list[PoseSample]:
    samples: list[PoseSample] = []
    for timestamp_ns, data in iter_topic(db_paths, topic):
        msg = deserialize_message(data, Odometry)
        pose = msg.pose.pose
        samples.append(
            PoseSample(
                stamp_seconds(msg, timestamp_ns),
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
            )
        )
    samples.sort(key=lambda sample: sample.stamp)
    return samples


def quaternion_matrix(q) -> np.ndarray:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 1.0e-12:
        return np.eye(3, dtype=np.float64)
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def load_static_transforms(
    db_paths: list[Path],
    topic: str,
) -> dict[tuple[str, str], tuple[np.ndarray, np.ndarray]]:
    result: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}
    for timestamp_ns, data in iter_topic(db_paths, topic):
        del timestamp_ns
        message = deserialize_message(data, TFMessage)
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if not parent or not child:
                continue
            translation = np.array(
                [
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y),
                    float(transform.transform.translation.z),
                ],
                dtype=np.float64,
            )
            result[(parent, child)] = (
                translation,
                quaternion_matrix(transform.transform.rotation),
            )
    return result


def compose_transform(
    first: tuple[np.ndarray, np.ndarray],
    second: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    first_translation, first_rotation = first
    second_translation, second_rotation = second
    return (
        first_translation + first_rotation @ second_translation,
        first_rotation @ second_rotation,
    )


def lookup_transform(
    transforms: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]],
    parent: str,
    child: str,
) -> tuple[np.ndarray, np.ndarray] | None:
    direct = transforms.get((parent, child))
    if direct is not None:
        return direct
    reverse = transforms.get((child, parent))
    if reverse is None:
        return None
    translation, rotation = reverse
    return -rotation.T @ translation, rotation.T


def load_camera_model(
    db_paths: list[Path],
    topic: str,
    fallback: CameraModel,
    tf_topic: str = "/tf_static",
) -> CameraModel:
    for timestamp_ns, data in iter_topic(db_paths, topic):
        del timestamp_ns
        msg = deserialize_message(data, CameraInfo)
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        if fx > 0.0 and fy > 0.0 and math.isfinite(cx) and math.isfinite(cy):
            fallback.fx = fx
            fallback.fy = fy
            fallback.cx = cx
            fallback.cy = cy
            fallback.distortion = np.asarray(msg.d, dtype=np.float64)
            fallback.color_to_base = road_detector.DEFAULT_OPTICAL_TO_BASE.copy()

            transforms = load_static_transforms(db_paths, tf_topic) if tf_topic else {}
            base_to_link = lookup_transform(
                transforms,
                "base_link",
                "camera_link",
            )
            link_to_color = lookup_transform(
                transforms,
                "camera_link",
                "camera_color_frame",
            )
            color_to_optical = lookup_transform(
                transforms,
                "camera_color_frame",
                "camera_color_optical_frame",
            )
            if base_to_link is not None:
                link_translation, link_rotation = base_to_link
                camera_translation = np.array(
                    [
                        link_translation[0],
                        link_translation[1],
                        fallback.z + link_translation[2],
                    ],
                    dtype=np.float64,
                )
                optical_to_base = link_rotation @ road_detector.DEFAULT_OPTICAL_TO_BASE
                if link_to_color is not None:
                    color_translation, color_rotation = link_to_color
                    camera_translation += link_rotation @ color_translation
                    optical_to_base = (
                        link_rotation
                        @ color_rotation
                        @ road_detector.DEFAULT_OPTICAL_TO_BASE
                    )
                    if color_to_optical is not None:
                        camera_translation += (
                            link_rotation
                            @ color_rotation
                            @ color_to_optical[0]
                        )
                        optical_to_base = (
                            link_rotation
                            @ color_rotation
                            @ color_to_optical[1]
                        )
                fallback.x = float(camera_translation[0])
                fallback.y = float(camera_translation[1])
                fallback.z = float(camera_translation[2])
                fallback.color_to_base = optical_to_base
            return fallback
    return fallback


def camera_geometry(camera: CameraModel) -> road_detector.CameraGeometry:
    return road_detector.CameraGeometry(
        fx=float(camera.fx),
        fy=float(camera.fy),
        cx=float(camera.cx),
        cy=float(camera.cy),
        camera_x=float(camera.x),
        camera_y=float(camera.y),
        camera_z=float(camera.z),
        distortion=camera.distortion,
        color_to_base=camera.color_to_base,
    )


def image_to_bgr(message: Image) -> np.ndarray | None:
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding in ("rgb8", "bgr8"):
        channels = 3
        row_width = message.step // channels
        image = data.reshape((message.height, row_width, channels))[
            :, : message.width
        ]
        if message.encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()
    if message.encoding in ("mono8", "8UC1"):
        image = data.reshape((message.height, message.step))[:, : message.width]
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if message.encoding in ("rgba8", "bgra8"):
        channels = 4
        row_width = message.step // channels
        image = data.reshape((message.height, row_width, channels))[
            :, : message.width
        ]
        code = (
            cv2.COLOR_RGBA2BGR
            if message.encoding == "rgba8"
            else cv2.COLOR_BGRA2BGR
        )
        return cv2.cvtColor(image, code)
    return None


def interpolate_pose(samples: list[PoseSample], stamp: float, tolerance: float) -> PoseSample | None:
    if not samples:
        return None
    lo = 0
    hi = len(samples)
    while lo < hi:
        mid = (lo + hi) // 2
        if samples[mid].stamp < stamp:
            lo = mid + 1
        else:
            hi = mid
    idx = lo
    if idx < len(samples) and abs(samples[idx].stamp - stamp) <= tolerance:
        return samples[idx]
    if idx == 0:
        return samples[0] if abs(samples[0].stamp - stamp) <= tolerance else None
    if idx >= len(samples):
        return samples[-1] if abs(samples[-1].stamp - stamp) <= tolerance else None
    before = samples[idx - 1]
    after = samples[idx]
    if stamp - before.stamp > tolerance or after.stamp - stamp > tolerance:
        return None
    span = after.stamp - before.stamp
    if span <= 0.0:
        return before
    ratio = (stamp - before.stamp) / span
    return PoseSample(
        stamp,
        before.x + (after.x - before.x) * ratio,
        before.y + (after.y - before.y) * ratio,
        wrap_angle(before.yaw + wrap_angle(after.yaw - before.yaw) * ratio),
    )


def load_map_features(map_path: Path) -> list[LineFeature]:
    with map_path.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    start_x, start_y = root["start"]["position_m"]
    start_yaw = math.radians(float(root["start"]["heading_deg"]))
    c = math.cos(start_yaw)
    s = math.sin(start_yaw)

    def arena_to_map(x: float, y: float) -> tuple[float, float]:
        dx = x - float(start_x)
        dy = y - float(start_y)
        return c * dx + s * dy, -s * dx + c * dy

    features: list[LineFeature] = []
    nodes = [tuple(map(float, node)) for node in root["inspection_graph"]["nodes"]]
    for edge in root["inspection_graph"]["edges"]:
        ax, ay = arena_to_map(*nodes[int(edge[0])])
        bx, by = arena_to_map(*nodes[int(edge[1])])
        features.append(LineFeature(ax, ay, bx, by, str(edge[2]), "road_center"))

    for idx, rect in enumerate(root["arena"]["obstacles"]):
        x1, y1, x2, y2 = map(float, rect)
        corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        for side, (a, b) in enumerate(zip(corners, corners[1:] + corners[:1])):
            ax, ay = arena_to_map(*a)
            bx, by = arena_to_map(*b)
            features.append(LineFeature(ax, ay, bx, by, f"OBSTACLE_{idx}_{side}", "obstacle_edge"))

    for idx, rect in enumerate(root["arena"]["free_regions"]):
        x1, y1, x2, y2 = map(float, rect)
        corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        for side, (a, b) in enumerate(zip(corners, corners[1:] + corners[:1])):
            ax, ay = arena_to_map(*a)
            bx, by = arena_to_map(*b)
            features.append(LineFeature(ax, ay, bx, by, f"FREE_{idx}_{side}", "free_edge"))

    for junction in root.get("arena", {}).get("turn_junctions", []):
        ax, ay = arena_to_map(float(start_x), float(start_y))
        bx, by = arena_to_map(float(junction[0]), float(junction[1]))
        features.append(LineFeature(ax, ay, bx, by, "LAUNCH_SPUR", "road_center"))
    return features


def segment_length(feature: LineFeature) -> float:
    return math.hypot(feature.bx - feature.ax, feature.by - feature.ay)


def nearest_on_feature(x: float, y: float, feature: LineFeature) -> tuple[float, float, float]:
    dx = feature.bx - feature.ax
    dy = feature.by - feature.ay
    length2 = dx * dx + dy * dy
    if length2 <= 1.0e-12:
        return feature.ax, feature.ay, math.hypot(x - feature.ax, y - feature.ay)
    ratio = max(0.0, min(1.0, ((x - feature.ax) * dx + (y - feature.ay) * dy) / length2))
    px = feature.ax + ratio * dx
    py = feature.ay + ratio * dy
    return px, py, math.hypot(x - px, y - py)


def undirected_angle_error(target: float, observed: float) -> float:
    error = wrap_angle(target - observed)
    if error > 0.5 * math.pi:
        error -= math.pi
    elif error < -0.5 * math.pi:
        error += math.pi
    return wrap_angle(error)


def base_point_to_map(pose: PoseSample, x: float, y: float) -> tuple[float, float]:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return pose.x + c * x - s * y, pose.y + s * x + c * y


def transform_segment(pose: PoseSample, segment: GroundSegment) -> GroundSegment:
    ax, ay = base_point_to_map(pose, segment.ax, segment.ay)
    bx, by = base_point_to_map(pose, segment.bx, segment.by)
    yaw = math.atan2(by - ay, bx - ax)
    return GroundSegment(ax, ay, bx, by, math.hypot(bx - ax, by - ay), yaw)


def project_pixel(camera: CameraModel, u: float, v: float, limits) -> tuple[float, float] | None:
    x_opt = (u - camera.cx) / camera.fx
    y_opt = (v - camera.cy) / camera.fy
    ray_x = 1.0
    ray_y = -x_opt
    ray_z = -y_opt
    cp = math.cos(camera.pitch_down)
    sp = math.sin(camera.pitch_down)
    ray_x, ray_z = cp * ray_x + sp * ray_z, -sp * ray_x + cp * ray_z
    if ray_z >= -limits["min_ray_down_z"]:
        return None
    scale = -camera.z / ray_z
    if scale <= 0.0 or not math.isfinite(scale):
        return None
    x = camera.x + scale * ray_x
    y = camera.y + scale * ray_y
    if not (limits["min_x"] <= x <= limits["max_x"] and abs(y) <= limits["max_abs_y"]):
        return None
    return x, y


def image_to_gray(msg: Image) -> np.ndarray | None:
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("mono8", "8UC1"):
        return data.reshape((msg.height, msg.step))[:, : msg.width].copy()
    if msg.encoding in ("rgb8", "bgr8"):
        image = data.reshape((msg.height, msg.step // 3, 3))[:, : msg.width]
        code = cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(image, code)
    return None


def detect_ground_segments(
    bgr: np.ndarray,
    camera: CameraModel,
    args,
) -> tuple[list[GroundSegment], np.ndarray]:
    detected, mask, _, _ = road_detector.detect_ground_segments(
        bgr if bgr.ndim == 3 else cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR),
        camera_geometry(camera),
        args,
    )
    segments = [
        GroundSegment(
            item.ax,
            item.ay,
            item.bx,
            item.by,
            item.length,
            item.yaw,
        )
        for item in detected
    ]
    return segments, mask


def match_segments(
    pose: PoseSample,
    ground_segments: list[GroundSegment],
    features: list[LineFeature],
    args,
) -> ObservationFit:
    total_weight = 0.0
    rx = 0.0
    ry = 0.0
    yaw_sin = 0.0
    yaw_cos = 0.0
    distance_error = 0.0
    angle_error = 0.0
    labels: dict[str, int] = {}
    matches = 0
    for segment in ground_segments:
        observed = transform_segment(pose, segment)
        mx = 0.5 * (observed.ax + observed.bx)
        my = 0.5 * (observed.ay + observed.by)
        best = None
        best_score = float("inf")
        for feature in features:
            if args.road_centers_only and feature.kind != "road_center":
                continue
            px, py, dist = nearest_on_feature(mx, my, feature)
            if dist > args.max_match_distance:
                continue
            feature_yaw = math.atan2(feature.by - feature.ay, feature.bx - feature.ax)
            yaw_error = undirected_angle_error(feature_yaw, observed.yaw)
            if abs(yaw_error) > args.max_match_angle:
                continue
            kind_bias = 0.0 if feature.kind == "obstacle_edge" else 0.025
            score = dist + args.angle_weight * abs(yaw_error) + kind_bias
            if score < best_score:
                best_score = score
                best = (feature, px, py, dist, yaw_error)
        if best is None:
            continue
        feature, px, py, dist, yaw_error = best
        distance_weight = max(0.0, 1.0 - dist / args.max_match_distance)
        angle_weight = max(0.0, 1.0 - abs(yaw_error) / args.max_match_angle)
        weight = segment.length * distance_weight * angle_weight
        if weight <= 0.0:
            continue
        total_weight += weight
        rx += (px - mx) * weight
        ry += (py - my) * weight
        yaw_sin += math.sin(yaw_error) * weight
        yaw_cos += math.cos(yaw_error) * weight
        distance_error += dist * weight
        angle_error += abs(yaw_error) * weight
        labels[feature.label] = labels.get(feature.label, 0) + 1
        matches += 1
    if total_weight < args.min_total_weight or matches < args.min_matches:
        return ObservationFit(0.0, 0.0, 0.0, matches, "", total_weight, float("inf"), float("inf"), float("inf"))
    summary = ",".join(f"{label}:{count}" for label, count in sorted(labels.items())[:6])
    mean_distance = distance_error / total_weight
    mean_angle_error = angle_error / total_weight
    score = mean_distance + args.angle_weight * mean_angle_error
    return ObservationFit(
        rx / total_weight,
        ry / total_weight,
        wrap_angle(math.atan2(yaw_sin, yaw_cos)),
        matches,
        summary,
        total_weight,
        mean_distance,
        mean_angle_error,
        score,
    )


def corrected_pose(fused: PoseSample, ox: float, oy: float, oyaw: float) -> PoseSample:
    return PoseSample(fused.stamp, fused.x + ox, fused.y + oy, wrap_angle(fused.yaw + oyaw))


def clamp_vector(x: float, y: float, maximum: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= maximum or norm <= 1.0e-12:
        return x, y
    scale = maximum / norm
    return x * scale, y * scale


def closest_feature_error(pose: PoseSample, features: list[LineFeature]) -> float:
    road_features = [feature for feature in features if feature.kind == "road_center"]
    if not road_features:
        road_features = features
    return min(nearest_on_feature(pose.x, pose.y, feature)[2] for feature in road_features)


def should_accept_candidate(
    current: PoseSample,
    candidate: PoseSample,
    segments: list[GroundSegment],
    features: list[LineFeature],
    current_fit: ObservationFit,
    args,
) -> bool:
    if args.accept_mode == "always":
        return True
    if args.accept_mode == "road_center":
        current_error = closest_feature_error(current, features)
        candidate_error = closest_feature_error(candidate, features)
        return (
            current_error >= args.min_current_error
            and candidate_error <= current_error + args.accept_error_margin
        )

    candidate_fit = match_segments(candidate, segments, features, args)
    if candidate_fit.matches < args.min_matches or not candidate_fit.labels:
        return False
    return candidate_fit.score <= current_fit.score + args.accept_observation_margin


def process_images(db_paths: list[Path], fused: list[PoseSample], features: list[LineFeature], camera: CameraModel, args):
    rows: list[CorrectionRow] = []
    ox = oy = oyaw = 0.0
    last_update: float | None = None
    previous_frame_stamp: float | None = None
    debug_written = 0
    out_debug = args.out / "debug_images"
    out_debug.mkdir(parents=True, exist_ok=True)
    processed = 0

    for timestamp_ns, data in iter_topic(db_paths, args.left_image_topic):
        if processed % args.image_stride != 0:
            processed += 1
            continue
        processed += 1
        msg = deserialize_message(data, Image)
        stamp = stamp_seconds(msg, timestamp_ns)
        fused_pose = interpolate_pose(fused, stamp, args.sync_tolerance)
        if fused_pose is None:
            continue
        frame_dt = 0.0 if previous_frame_stamp is None else max(0.0, stamp - previous_frame_stamp)
        previous_frame_stamp = stamp
        bgr = image_to_bgr(msg)
        if bgr is None:
            continue
        bgr = road_detector.undistort_bgr(bgr, camera_geometry(camera))
        current = corrected_pose(fused_pose, ox, oy, oyaw)
        segments, mask = detect_ground_segments(bgr, camera, args)
        fit = match_segments(current, segments, features, args)
        rx = fit.residual_x
        ry = fit.residual_y
        ryaw = fit.residual_yaw
        matches = fit.matches
        labels = fit.labels
        accepted = False
        if matches >= args.min_matches and labels:
            dt = 0.0 if last_update is None else max(0.0, stamp - last_update)
            alpha = 1.0 if last_update is None else 1.0 - math.exp(-dt / args.correction_tau)
            sx, sy = clamp_vector(rx * alpha * args.translation_gain, ry * alpha * args.translation_gain, args.max_step_m)
            syaw = max(-args.max_step_yaw, min(args.max_step_yaw, ryaw * alpha * args.yaw_gain))
            candidate_ox = ox + sx
            candidate_oy = oy + sy
            candidate_oyaw = wrap_angle(oyaw + syaw)
            candidate_ox, candidate_oy = clamp_vector(candidate_ox, candidate_oy, args.max_total_offset_m)
            candidate_oyaw = max(-args.max_total_yaw, min(args.max_total_yaw, candidate_oyaw))
            candidate = corrected_pose(fused_pose, candidate_ox, candidate_oy, candidate_oyaw)
            if should_accept_candidate(current, candidate, segments, features, fit, args):
                ox = candidate_ox
                oy = candidate_oy
                oyaw = candidate_oyaw
                last_update = stamp
                accepted = True
            else:
                labels = ""
                matches = 0
        if not accepted and args.no_match_decay_tau > 0.0:
            decay = math.exp(-frame_dt / args.no_match_decay_tau)
            ox *= decay
            oy *= decay
            oyaw *= decay

        final = corrected_pose(fused_pose, ox, oy, oyaw)
        fused_fit = match_segments(fused_pose, segments, features, args)
        final_fit = match_segments(final, segments, features, args)
        rows.append(
            CorrectionRow(
                stamp=stamp,
                fused_x=fused_pose.x,
                fused_y=fused_pose.y,
                fused_yaw=fused_pose.yaw,
                corrected_x=final.x,
                corrected_y=final.y,
                corrected_yaw=final.yaw,
                offset_x=ox,
                offset_y=oy,
                offset_yaw=oyaw,
                residual_x=rx,
                residual_y=ry,
                residual_yaw=ryaw,
                detected_segments=len(segments),
                matched_segments=matches,
                matched_labels=labels,
                map_feature_error_fused=closest_feature_error(fused_pose, features),
                map_feature_error_corrected=closest_feature_error(final, features),
                visual_error_fused=fused_fit.score,
                visual_error_corrected=final_fit.score,
            )
        )
        if debug_written < args.debug_images and (matches > 0 or debug_written == 0):
            overlay = bgr.copy()
            overlay[mask > 0] = (0, 0, 255)
            cv2.imwrite(str(out_debug / f"debug_{debug_written:02d}_{matches:02d}.png"), overlay)
            debug_written += 1
    return rows


def finite_values(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def write_outputs(rows: list[CorrectionRow], features: list[LineFeature], args, topics):
    args.out.mkdir(parents=True, exist_ok=True)
    csv_path = args.out / "image_corrected_odom.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(CorrectionRow.__dataclass_fields__))
        writer.writeheader()
        for row in rows:
            writer.writerow(row.__dict__)

    fused_errors = [row.map_feature_error_fused for row in rows if math.isfinite(row.map_feature_error_fused)]
    corrected_errors = [row.map_feature_error_corrected for row in rows if math.isfinite(row.map_feature_error_corrected)]
    visual_fused_errors = finite_values([row.visual_error_fused for row in rows])
    visual_corrected_errors = finite_values([row.visual_error_corrected for row in rows])
    matched = [row for row in rows if row.matched_segments >= args.min_matches]
    image_topics = {name: count for name, (typ, count) in topics.items() if typ == "sensor_msgs/msg/Image"}
    summary = {
        "processed_image_samples": len(rows),
        "matched_image_samples": len(matched),
        "image_topics": image_topics,
        "mean_visual_map_error_fused": sum(visual_fused_errors) / len(visual_fused_errors) if visual_fused_errors else None,
        "mean_visual_map_error_corrected": (
            sum(visual_corrected_errors) / len(visual_corrected_errors) if visual_corrected_errors else None
        ),
        "p95_visual_map_error_fused": percentile(visual_fused_errors, 0.95),
        "p95_visual_map_error_corrected": percentile(visual_corrected_errors, 0.95),
        "reference_mean_centerline_error_fused_m": sum(fused_errors) / len(fused_errors) if fused_errors else None,
        "reference_mean_centerline_error_corrected_m": (
            sum(corrected_errors) / len(corrected_errors) if corrected_errors else None
        ),
        "reference_p95_centerline_error_fused_m": percentile(fused_errors, 0.95),
        "reference_p95_centerline_error_corrected_m": percentile(corrected_errors, 0.95),
        "accept_mode": args.accept_mode,
        "final_offset": {
            "x": rows[-1].offset_x if rows else 0.0,
            "y": rows[-1].offset_y if rows else 0.0,
            "yaw": rows[-1].offset_yaw if rows else 0.0,
        },
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    plot_trajectory(args.out / "image_corrected_trajectory.png", rows, features)
    plot_timeseries(args.out / "image_correction_timeseries.png", rows)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"wrote {args.out}")


def interpolated_offset(rows: list[CorrectionRow], stamp: float) -> tuple[float, float, float]:
    if not rows or stamp < rows[0].stamp:
        return 0.0, 0.0, 0.0
    lo = 0
    hi = len(rows)
    while lo < hi:
        mid = (lo + hi) // 2
        if rows[mid].stamp < stamp:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        row = rows[0]
        return row.offset_x, row.offset_y, row.offset_yaw
    if lo >= len(rows):
        row = rows[-1]
        return row.offset_x, row.offset_y, row.offset_yaw
    before = rows[lo - 1]
    after = rows[lo]
    span = after.stamp - before.stamp
    if span <= 0.0:
        return before.offset_x, before.offset_y, before.offset_yaw
    ratio = (stamp - before.stamp) / span
    return (
        before.offset_x + (after.offset_x - before.offset_x) * ratio,
        before.offset_y + (after.offset_y - before.offset_y) * ratio,
        wrap_angle(before.offset_yaw + wrap_angle(after.offset_yaw - before.offset_yaw) * ratio),
    )


def write_corrected_odom_bag(db_paths: list[Path], rows: list[CorrectionRow], args) -> None:
    if args.output_bag is None:
        return
    output_bag = args.output_bag
    if output_bag.exists():
        if not args.overwrite_output_bag:
            raise FileExistsError(f"{output_bag} already exists; pass --overwrite-output-bag to replace it")
        shutil.rmtree(output_bag)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=args.output_odom_topic,
            type="nav_msgs/msg/Odometry",
            serialization_format="cdr",
        )
    )

    written = 0
    for timestamp_ns, data in iter_topic(db_paths, args.fused_topic):
        msg = deserialize_message(data, Odometry)
        stamp = stamp_seconds(msg, timestamp_ns)
        ox, oy, oyaw = interpolated_offset(rows, stamp)
        pose = msg.pose.pose
        pose.position.x = float(pose.position.x) + ox
        pose.position.y = float(pose.position.y) + oy
        set_yaw_quaternion(pose.orientation, wrap_angle(yaw_from_quaternion(pose.orientation) + oyaw))
        writer.write(args.output_odom_topic, serialize_message(msg), int(timestamp_ns))
        written += 1
    print(f"wrote corrected odometry bag {output_bag} with {written} messages on {args.output_odom_topic}")


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    values = sorted(values)
    return values[int(round((len(values) - 1) * p))]


def plot_trajectory(path: Path, rows: list[CorrectionRow], features: list[LineFeature]):
    fig, ax = plt.subplots(figsize=(8, 8))
    for feature in features:
        if feature.kind == "obstacle_edge":
            ax.plot([feature.ax, feature.bx], [feature.ay, feature.by], color="#f59e0b", linewidth=1.0, alpha=0.8)
        elif feature.kind == "free_edge":
            ax.plot([feature.ax, feature.bx], [feature.ay, feature.by], color="#94a3b8", linewidth=0.8, alpha=0.65)
        if feature.kind == "road_center":
            ax.plot([feature.ax, feature.bx], [feature.ay, feature.by], color="#d1d5db", linewidth=1.1)
    ax.plot([r.fused_x for r in rows], [r.fused_y for r in rows], color="#ef4444", label="fused")
    ax.plot([r.corrected_x for r in rows], [r.corrected_y for r in rows], color="#2563eb", label="image corrected")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.3)
    ax.legend()
    ax.set_title("Image/map corrected odometry - no route snap")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_timeseries(path: Path, rows: list[CorrectionRow]):
    if not rows:
        return
    t0 = rows[0].stamp
    t = [row.stamp - t0 for row in rows]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fused_visual = [r.visual_error_fused if math.isfinite(r.visual_error_fused) else np.nan for r in rows]
    corrected_visual = [r.visual_error_corrected if math.isfinite(r.visual_error_corrected) else np.nan for r in rows]
    axes[0].plot(t, fused_visual, label="fused", color="#ef4444")
    axes[0].plot(t, corrected_visual, label="corrected", color="#2563eb")
    axes[0].set_ylabel("visual-map error")
    axes[0].legend()
    axes[0].grid(True, linewidth=0.3)
    axes[1].plot(t, [r.offset_x for r in rows], label="offset x")
    axes[1].plot(t, [r.offset_y for r in rows], label="offset y")
    axes[1].set_ylabel("offset [m]")
    axes[1].legend()
    axes[1].grid(True, linewidth=0.3)
    axes[2].plot(t, [r.matched_segments for r in rows], label="matched image lines", color="#16a34a")
    axes[2].set_ylabel("matches")
    axes[2].set_xlabel("time [s]")
    axes[2].legend()
    axes[2].grid(True, linewidth=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--out", type=Path, default=Path("analysis_bags/20260921T153423.632162Z_9ead9ca47d9b/image_track_correct"))
    parser.add_argument("--output-bag", type=Path)
    parser.add_argument("--output-odom-topic", default="/odometry/definitely_correct")
    parser.add_argument("--overwrite-output-bag", action="store_true")
    parser.add_argument("--fused-topic", default="/odometry/fused")
    parser.add_argument("--left-image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--left-camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--sync-tolerance", type=float, default=0.12)
    parser.add_argument("--image-stride", type=int, default=1)
    parser.add_argument("--camera-x", type=float, default=0.055)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=0.121)
    parser.add_argument("--camera-pitch-down", type=float, default=0.0)
    parser.add_argument("--roi-top-fraction", type=float, default=0.0)
    parser.add_argument("--roi-bottom-fraction", type=float, default=1.0)
    parser.add_argument("--detector", default="rgb_and_local")
    parser.add_argument("--rgb-dark-percentile", type=float, default=88.0)
    parser.add_argument("--rgb-dark-margin", type=float, default=30.0)
    parser.add_argument("--rgb-dark-min-threshold", type=float, default=18.0)
    parser.add_argument("--rgb-dark-max-threshold", type=float, default=90.0)
    parser.add_argument("--local-dark-delta", type=float, default=16.0)
    parser.add_argument("--local-dark-sigma", type=float, default=11.0)
    parser.add_argument("--max-color-spread", type=float, default=35.0)
    parser.add_argument("--max-dark-value", type=int, default=92)
    parser.add_argument("--use-fixed-dark", action="store_true")
    parser.add_argument("--use-adaptive-dark", action="store_true")
    parser.add_argument("--adaptive-block-size", type=int, default=31)
    parser.add_argument("--adaptive-c", type=float, default=6.0)
    parser.add_argument("--blackhat-size", type=int, default=25)
    parser.add_argument("--blackhat-threshold", type=int, default=10)
    parser.add_argument("--morphology-size", type=int, default=3)
    parser.add_argument("--hough-threshold", type=int, default=14)
    parser.add_argument("--min-line-length-px", type=int, default=12)
    parser.add_argument("--max-line-gap-px", type=int, default=12)
    parser.add_argument("--min-ray-down-z", type=float, default=0.025)
    parser.add_argument("--min-ground-x", type=float, default=0.08)
    parser.add_argument("--max-ground-x", type=float, default=1.60)
    parser.add_argument("--max-abs-ground-y", type=float, default=0.65)
    parser.add_argument("--min-ground-segment-length", type=float, default=0.045)
    parser.add_argument("--max-segments", type=int, default=60)
    parser.add_argument("--max-match-distance", type=float, default=0.08)
    parser.add_argument("--max-match-angle", type=float, default=0.42)
    parser.add_argument("--angle-weight", type=float, default=0.05)
    parser.add_argument("--min-total-weight", type=float, default=0.04)
    parser.add_argument("--min-matches", type=int, default=2)
    parser.add_argument("--correction-tau", type=float, default=2.0)
    parser.add_argument("--translation-gain", type=float, default=0.22)
    parser.add_argument("--yaw-gain", type=float, default=0.0)
    parser.add_argument("--max-step-m", type=float, default=0.0015)
    parser.add_argument("--max-step-yaw", type=float, default=0.0)
    parser.add_argument("--max-total-offset-m", type=float, default=0.10)
    parser.add_argument("--max-total-yaw", type=float, default=0.08)
    parser.add_argument("--accept-mode", choices=("observation", "always", "road_center"), default="observation")
    parser.add_argument("--accept-observation-margin", type=float, default=0.001)
    parser.add_argument("--accept-error-margin", type=float, default=0.0005)
    parser.add_argument("--min-current-error", type=float, default=0.0)
    parser.add_argument("--no-match-decay-tau", type=float, default=0.8)
    parser.add_argument("--debug-images", type=int, default=12)
    parser.add_argument("--road-centers-only", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    cache_dir = args.out / "decompressed"
    db_paths = decompress_bag_files(args.bag, cache_dir)
    topics = topic_table(db_paths)
    fused = load_odometry(db_paths, args.fused_topic)
    if not fused:
        raise RuntimeError(f"no fused odometry on {args.fused_topic}")
    camera = load_camera_model(
        db_paths,
        args.left_camera_info_topic,
        CameraModel(x=args.camera_x, y=args.camera_y, z=args.camera_z, pitch_down=args.camera_pitch_down),
        tf_topic="/tf_static",
    )
    features = load_map_features(args.map)
    rows = process_images(db_paths, fused, features, camera, args)
    write_outputs(rows, features, args, topics)
    write_corrected_odom_bag(db_paths, rows, args)


if __name__ == "__main__":
    main()
