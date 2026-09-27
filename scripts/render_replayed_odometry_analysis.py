#!/usr/bin/env python3
"""Render raw RGB frames with replayed and recorded odometry trajectories.

This is an analysis-only tool.  It deliberately reads the camera stream from
the original recording and the recalculated odometry from a separate replay
bag, matching both sides by the message header timestamp.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import offline_image_track_correction as correction


DEFAULT_RAW_BAG = Path("/media/hjh/E/rosbag_recording/latest_navigation_bag")
DEFAULT_REPLAY_BAG = Path("/tmp/fused_odometry_replay_full_20260926")
DEFAULT_MAP = Path("src/arena_path_planner/config/arena_map.yaml")
DEFAULT_OUT_DIR = Path(
    "/media/hjh/E/rosbag_recording/bag_exports/"
    "20260926T095917.336700Z_a9ed0f6d431e/analysis"
)
DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_SYNC_TOLERANCE = 0.15
MAX_DRAW_STEP_M = 0.45


@dataclass(frozen=True)
class TimedStatus:
    stamp: float
    value: str


@dataclass
class Trajectory:
    name: str
    color: tuple[int, int, int]
    samples: list[correction.PoseSample]
    points: np.ndarray
    stamps: list[float]
    dashed: bool = False
    width: int = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-bag", type=Path, default=DEFAULT_RAW_BAG)
    parser.add_argument("--replay-bag", type=Path, default=DEFAULT_REPLAY_BAG)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--sync-tolerance", type=float, default=DEFAULT_SYNC_TOLERANCE)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--image-stride", type=int, default=1)
    parser.add_argument("--map-width", type=int, default=700)
    return parser.parse_args()


def stamp_for_message(message, storage_timestamp_ns: int) -> float:
    return correction.stamp_seconds(message, storage_timestamp_ns)


def load_status(
    db_paths: list[Path],
    topic: str,
) -> list[TimedStatus]:
    rows: list[TimedStatus] = []
    for timestamp_ns, data in correction.iter_topic(db_paths, topic):
        message = deserialize_message(data, String)
        rows.append(TimedStatus(stamp_for_message(message, timestamp_ns), message.data))
    rows.sort(key=lambda row: row.stamp)
    return rows


def load_map_odom_transforms(
    db_paths: list[Path],
) -> list[correction.PoseSample]:
    """Load recorded map->odom transforms as timestamped planar poses."""
    rows: list[correction.PoseSample] = []
    for timestamp_ns, data in correction.iter_topic(db_paths, "/tf"):
        message = deserialize_message(data, TFMessage)
        for transform in message.transforms:
            if (
                transform.header.frame_id.lstrip("/") != "map"
                or transform.child_frame_id.lstrip("/") != "odom"
            ):
                continue
            rotation = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            rows.append(
                correction.PoseSample(
                    stamp=stamp_for_message(transform, timestamp_ns),
                    x=float(transform.transform.translation.x),
                    y=float(transform.transform.translation.y),
                    yaw=yaw,
                )
            )
    rows.sort(key=lambda row: row.stamp)
    return rows


def transform_odom_to_map(
    samples: list[correction.PoseSample],
    map_odom: list[correction.PoseSample],
) -> list[correction.PoseSample]:
    """Apply a recorded map->odom transform to odom-frame poses."""
    if not samples or not map_odom:
        return samples
    transform_stamps = [sample.stamp for sample in map_odom]
    transformed: list[correction.PoseSample] = []
    for sample in samples:
        index = bisect.bisect_left(transform_stamps, sample.stamp)
        if index <= 0:
            transform = map_odom[0]
        elif index >= len(map_odom):
            transform = map_odom[-1]
        else:
            before = map_odom[index - 1]
            after = map_odom[index]
            span = after.stamp - before.stamp
            if span <= 0.0:
                transform = before
            else:
                ratio = (sample.stamp - before.stamp) / span
                transform = correction.PoseSample(
                    stamp=sample.stamp,
                    x=before.x + ratio * (after.x - before.x),
                    y=before.y + ratio * (after.y - before.y),
                    yaw=correction.wrap_angle(
                        before.yaw
                        + ratio * correction.wrap_angle(after.yaw - before.yaw)
                    ),
                )
        c = math.cos(transform.yaw)
        s = math.sin(transform.yaw)
        transformed.append(
            correction.PoseSample(
                stamp=sample.stamp,
                x=transform.x + c * sample.x - s * sample.y,
                y=transform.y + s * sample.x + c * sample.y,
                yaw=correction.wrap_angle(sample.yaw + transform.yaw),
            )
        )
    return transformed


def status_at(statuses: list[TimedStatus], stamp: float) -> str:
    if not statuses:
        return ""
    index = bisect.bisect_right([row.stamp for row in statuses], stamp) - 1
    if index < 0:
        return ""
    return statuses[index].value


def rotate_into_anchor(
    sample: correction.PoseSample,
    anchor: correction.PoseSample,
) -> correction.PoseSample:
    """Express a pose in the first replayed fused pose's local frame."""
    dx = sample.x - anchor.x
    dy = sample.y - anchor.y
    c = math.cos(anchor.yaw)
    s = math.sin(anchor.yaw)
    return correction.PoseSample(
        stamp=sample.stamp,
        x=c * dx + s * dy,
        y=-s * dx + c * dy,
        yaw=correction.wrap_angle(sample.yaw - anchor.yaw),
    )


def make_trajectory(
    name: str,
    color: tuple[int, int, int],
    samples: list[correction.PoseSample],
    anchor: correction.PoseSample,
    *,
    dashed: bool = False,
    width: int = 2,
) -> Trajectory:
    aligned = [rotate_into_anchor(sample, anchor) for sample in samples]
    points = np.asarray([(sample.x, sample.y) for sample in aligned], dtype=np.float64)
    if points.size == 0:
        points = np.empty((0, 2), dtype=np.float64)
    return Trajectory(
        name=name,
        color=color,
        samples=aligned,
        points=points,
        stamps=[sample.stamp for sample in aligned],
        dashed=dashed,
        width=width,
    )


def image_to_bgr(message: Image) -> np.ndarray | None:
    return correction.image_to_bgr(message)


class MapView:
    def __init__(
        self,
        width: int,
        height: int,
        features: list[correction.LineFeature],
    ) -> None:
        self.width = width
        self.height = height
        self.features = features
        # Coordinates are in the map frame generated by load_map_features:
        # x follows the launch direction and y is lateral to the road.
        self.x_min = -0.25
        self.x_max = 4.55
        self.y_min = -1.85
        self.y_max = 1.85
        self.left = 22
        self.right = width - 22
        self.top = 38
        self.bottom = height - 24
        self.scale = min(
            (self.right - self.left) / (self.x_max - self.x_min),
            (self.bottom - self.top) / (self.y_max - self.y_min),
        )
        self.x_pad = self.left + 0.5 * (
            (self.right - self.left) - self.scale * (self.x_max - self.x_min)
        )
        self.y_pad = self.top + 0.5 * (
            (self.bottom - self.top) - self.scale * (self.y_max - self.y_min)
        )

    def pixel(self, x: float, y: float) -> tuple[int, int]:
        return (
            int(round(self.x_pad + (x - self.x_min) * self.scale)),
            int(round(self.y_pad + (self.y_max - y) * self.scale)),
        )

    def panel(self) -> np.ndarray:
        panel = np.full((self.height, self.width, 3), 246, dtype=np.uint8)
        cv2.rectangle(
            panel,
            (self.left, self.top),
            (self.right, self.bottom),
            (220, 220, 220),
            1,
        )
        for grid_x in np.arange(0.0, 4.51, 0.5):
            p1 = self.pixel(float(grid_x), self.y_min)
            p2 = self.pixel(float(grid_x), self.y_max)
            cv2.line(panel, p1, p2, (232, 232, 232), 1, cv2.LINE_AA)
        for grid_y in np.arange(-1.5, 1.51, 0.5):
            p1 = self.pixel(self.x_min, float(grid_y))
            p2 = self.pixel(self.x_max, float(grid_y))
            cv2.line(panel, p1, p2, (232, 232, 232), 1, cv2.LINE_AA)

        for feature in self.features:
            p1 = self.pixel(feature.ax, feature.ay)
            p2 = self.pixel(feature.bx, feature.by)
            if feature.kind == "road_center":
                color, thickness = (105, 105, 105), 3
            elif feature.kind == "obstacle_edge":
                color, thickness = (55, 105, 180), 2
            else:
                color, thickness = (190, 190, 190), 1
            cv2.line(panel, p1, p2, color, thickness, cv2.LINE_AA)

        start = self.pixel(0.0, 0.0)
        cv2.circle(panel, start, 5, (0, 0, 0), -1, cv2.LINE_AA)
        put_text(panel, "start", start[0] + 8, start[1] - 6, 0.40)
        put_text(panel, "map / replayed odometry", 14, 23, 0.56)
        put_text(panel, "road center", 14, self.height - 10, 0.35)
        return panel


def put_text(
    image: np.ndarray,
    text: str,
    x: int,
    y: int,
    scale: float = 0.45,
    color: tuple[int, int, int] = (255, 255, 255),
) -> None:
    cv2.putText(
        image,
        text,
        (int(x), int(y)),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        text,
        (int(x), int(y)),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        1,
        cv2.LINE_AA,
    )


def overlay_box(
    image: np.ndarray,
    x1: int,
    y1: int,
    x2: int,
    y2: int,
    alpha: float = 0.70,
) -> None:
    overlay = image.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0, dst=image)


def pose_text(
    label: str,
    pose: correction.PoseSample | None,
) -> str:
    if pose is None:
        return f"{label}: n/a"
    return (
        f"{label}: x={pose.x:+.3f} y={pose.y:+.3f} "
        f"yaw={math.degrees(pose.yaw):+.1f}"
    )


def interpolate_aligned(
    trajectory: Trajectory,
    stamp: float,
    tolerance: float,
) -> correction.PoseSample | None:
    return correction.interpolate_pose(trajectory.samples, stamp, tolerance)


def path_indices(trajectory: Trajectory, stamp: float, limit: int = 1800) -> np.ndarray:
    if not trajectory.stamps:
        return np.empty((0,), dtype=np.int64)
    count = bisect.bisect_right(trajectory.stamps, stamp)
    if count <= 0:
        return np.empty((0,), dtype=np.int64)
    if count <= limit:
        return np.arange(count, dtype=np.int64)
    return np.unique(np.linspace(0, count - 1, limit, dtype=np.int64))


def draw_trajectory(
    panel: np.ndarray,
    view: MapView,
    trajectory: Trajectory,
    stamp: float,
    *,
    show_path: bool = True,
) -> correction.PoseSample | None:
    if show_path:
        indices = path_indices(trajectory, stamp)
        if len(indices) >= 2:
            pixels = [
                view.pixel(float(trajectory.points[index, 0]), float(trajectory.points[index, 1]))
                for index in indices
            ]
            distances = [
                float(np.linalg.norm(trajectory.points[indices[index + 1]] - trajectory.points[indices[index]]))
                for index in range(len(indices) - 1)
            ]
            if trajectory.dashed:
                for index in range(0, len(pixels) - 1, 2):
                    if distances[index] > MAX_DRAW_STEP_M:
                        continue
                    cv2.line(
                        panel,
                        pixels[index],
                        pixels[index + 1],
                        trajectory.color,
                        trajectory.width,
                        cv2.LINE_AA,
                    )
            else:
                segment: list[tuple[int, int]] = [pixels[0]]
                for index, pixel in enumerate(pixels[1:]):
                    if distances[index] > MAX_DRAW_STEP_M:
                        if len(segment) >= 2:
                            cv2.polylines(
                                panel,
                                [np.asarray(segment, dtype=np.int32)],
                                False,
                                trajectory.color,
                                trajectory.width,
                                cv2.LINE_AA,
                            )
                        segment = [pixel]
                    else:
                        segment.append(pixel)
                if len(segment) >= 2:
                    cv2.polylines(
                        panel,
                        [np.asarray(segment, dtype=np.int32)],
                        False,
                        trajectory.color,
                        trajectory.width,
                        cv2.LINE_AA,
                    )

    pose = interpolate_aligned(trajectory, stamp, DEFAULT_SYNC_TOLERANCE)
    if pose is None:
        return None
    center = view.pixel(pose.x, pose.y)
    cv2.circle(panel, center, 5, trajectory.color, -1, cv2.LINE_AA)
    tip = view.pixel(
        pose.x + 0.15 * math.cos(pose.yaw),
        pose.y + 0.15 * math.sin(pose.yaw),
    )
    cv2.arrowedLine(panel, center, tip, trajectory.color, 2, cv2.LINE_AA, tipLength=0.30)
    return pose


def draw_legend(
    panel: np.ndarray,
    trajectories: list[Trajectory],
) -> None:
    x = 14
    y = 48
    for trajectory in trajectories:
        cv2.line(panel, (x, y - 4), (x + 22, y - 4), trajectory.color, 3, cv2.LINE_AA)
        put_text(panel, trajectory.name, x + 28, y, 0.36, (35, 35, 35))
        x += 112 if len(trajectory.name) < 14 else 145
        if x > panel.shape[1] - 130:
            x = 14
            y += 20


def render_camera_panel(
    bgr: np.ndarray,
    frame_index: int,
    stamp: float,
    first_stamp: float,
    statuses: list[TimedStatus],
    poses: dict[str, correction.PoseSample | None],
    *,
    width: int,
) -> np.ndarray:
    if bgr.shape[1] != width:
        bgr = cv2.resize(bgr, (width, int(round(bgr.shape[0] * width / bgr.shape[1]))))
    panel = bgr.copy()
    overlay_box(panel, 8, 8, panel.shape[1] - 8, 140, 0.68)
    put_text(
        panel,
        f"RGB frame={frame_index} t={stamp - first_stamp:+.2f}s stamp={stamp:.3f}",
        18,
        29,
        0.48,
    )
    put_text(panel, "replayed algorithm: wheel + IMU; visual is diagnostic only", 18, 52, 0.40)
    put_text(panel, pose_text("fused*", poses.get("replayed_fused")), 18, 77, 0.40)
    put_text(panel, pose_text("local", poses.get("replayed_local")), 18, 98, 0.40)
    put_text(panel, pose_text("visual", poses.get("raw_visual")), 18, 119, 0.40)
    state = status_at(statuses, stamp)
    if state:
        put_text(panel, f"status: {state[:74]}", 18, 138, 0.34, (130, 235, 255))
    return panel


def draw_map_frame(
    view: MapView,
    trajectories: list[Trajectory],
    stamp: float,
) -> tuple[np.ndarray, dict[str, correction.PoseSample | None]]:
    panel = view.panel()
    poses: dict[str, correction.PoseSample | None] = {}
    for trajectory in trajectories:
        poses[trajectory.name] = draw_trajectory(panel, view, trajectory, stamp)
    draw_legend(panel, trajectories)
    for trajectory in trajectories:
        pose = poses[trajectory.name]
        if pose is None:
            continue
        px, py = view.pixel(pose.x, pose.y)
        cv2.circle(panel, (px, py), 8, trajectory.color, 1, cv2.LINE_AA)
    put_text(panel, f"t={stamp:.3f}", 14, view.height - 31, 0.36, (35, 35, 35))
    return panel, poses


def open_writer(path: Path, codec: str, fps: float, size: tuple[int, int]):
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*codec),
        fps,
        size,
    )
    if not writer.isOpened():
        writer.release()
        raise RuntimeError(f"failed to open video writer {path} with codec {codec}")
    return writer


def format_pose_row(
    prefix: str,
    pose: correction.PoseSample | None,
) -> dict[str, str]:
    if pose is None:
        return {
            f"{prefix}_x": "",
            f"{prefix}_y": "",
            f"{prefix}_yaw_deg": "",
        }
    return {
        f"{prefix}_x": f"{pose.x:.9f}",
        f"{prefix}_y": f"{pose.y:.9f}",
        f"{prefix}_yaw_deg": f"{math.degrees(pose.yaw):.6f}",
    }


def write_trajectory_png(
    path: Path,
    view: MapView,
    trajectories: list[Trajectory],
    final_stamp: float,
) -> None:
    panel, _ = draw_map_frame(view, trajectories, final_stamp)
    cv2.imwrite(str(path), panel)


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    raw_db_paths = correction.decompress_bag_files(
        args.raw_bag,
        args.out_dir / "raw_decompressed_cache",
    )
    replay_db_paths = correction.decompress_bag_files(
        args.replay_bag,
        args.out_dir / "replay_decompressed_cache",
    )

    replay_fused = correction.load_odometry(replay_db_paths, "/odometry/fused")
    replay_local = correction.load_odometry(replay_db_paths, "/odometry/local")
    replay_local_map = correction.load_odometry(replay_db_paths, "/odometry/local_map")
    raw_wheel = correction.load_odometry(raw_db_paths, "/wheel/odom")
    raw_visual = correction.load_odometry(raw_db_paths, "/odometry/visual_raw")
    raw_fused = correction.load_odometry(raw_db_paths, "/odometry/fused")
    raw_wheel_map = transform_odom_to_map(
        raw_wheel,
        load_map_odom_transforms(raw_db_paths),
    )
    statuses = load_status(replay_db_paths, "/odometry/fusion_status")

    if not replay_fused:
        raise RuntimeError("replay bag has no /odometry/fused")
    anchor = replay_fused[0]
    features = correction.load_map_features(args.map)
    view = MapView(args.map_width, 480, features)

    trajectories = [
        make_trajectory(
            "fused*",
            (0, 0, 235),
            replay_fused,
            anchor,
            width=4,
        ),
        make_trajectory(
            "local",
            (35, 165, 35),
            replay_local,
            anchor,
            width=2,
        ),
        make_trajectory(
            "local_map",
            (220, 100, 20),
            replay_local_map,
            anchor,
            width=2,
        ),
        make_trajectory(
            "wheel->map",
            (105, 105, 105),
            raw_wheel_map,
            anchor,
            dashed=True,
            width=2,
        ),
        make_trajectory(
            "visual raw",
            (255, 145, 0),
            raw_visual,
            anchor,
            dashed=True,
            width=2,
        ),
        make_trajectory(
            "fused original",
            (190, 0, 190),
            raw_fused,
            anchor,
            dashed=True,
            width=2,
        ),
    ]

    image_stamps: list[float] = []
    for timestamp_ns, data in correction.iter_topic(raw_db_paths, args.image_topic):
        message = deserialize_message(data, Image)
        image_stamps.append(stamp_for_message(message, timestamp_ns))
    if not image_stamps:
        raise RuntimeError(f"raw bag has no {args.image_topic}")
    first_image_stamp = image_stamps[0]

    csv_path = args.out_dir / "replayed_current_odometry.csv"
    avi_path = args.out_dir / "replayed_current_odometry.avi"
    mp4_path = args.out_dir / "replayed_current_odometry.mp4"
    png_path = args.out_dir / "replayed_current_odometry_trajectory.png"
    summary_path = args.out_dir / "replayed_current_odometry_summary.json"

    frame_size = (640 + args.map_width, 480)
    avi_writer = open_writer(avi_path, "MJPG", args.fps, frame_size)
    mp4_writer = open_writer(mp4_path, "mp4v", args.fps, frame_size)
    fieldnames = [
        "frame_index",
        "stamp",
        "relative_time",
        "fusion_status",
    ]
    for prefix in (
        "replayed_fused",
        "replayed_local",
        "replayed_local_map",
        "raw_wheel",
        "raw_visual",
        "raw_fused",
    ):
        fieldnames.extend(
            [f"{prefix}_x", f"{prefix}_y", f"{prefix}_yaw_deg"]
        )

    written = 0
    decoded = 0
    skipped = 0
    last_stamp = first_image_stamp
    with csv_path.open("w", newline="", encoding="utf-8") as csv_stream:
        csv_writer = csv.DictWriter(csv_stream, fieldnames=fieldnames)
        csv_writer.writeheader()
        try:
            for timestamp_ns, data in correction.iter_topic(raw_db_paths, args.image_topic):
                if decoded % max(1, args.image_stride) != 0:
                    decoded += 1
                    continue
                decoded += 1
                message = deserialize_message(data, Image)
                stamp = stamp_for_message(message, timestamp_ns)
                bgr = image_to_bgr(message)
                if bgr is None:
                    skipped += 1
                    continue
                if bgr.shape[:2] != (480, 640):
                    bgr = cv2.resize(bgr, (640, 480))

                map_panel, map_poses = draw_map_frame(view, trajectories, stamp)
                poses = {
                    "replayed_fused": map_poses.get("fused*"),
                    "replayed_local": map_poses.get("local"),
                    "replayed_local_map": map_poses.get("local_map"),
                    "raw_wheel": map_poses.get("wheel->map"),
                    "raw_visual": map_poses.get("visual raw"),
                    "raw_fused": map_poses.get("fused original"),
                }
                camera_panel = render_camera_panel(
                    bgr,
                    written,
                    stamp,
                    first_image_stamp,
                    statuses,
                    poses,
                    width=640,
                )
                frame = np.hstack((camera_panel, map_panel))
                avi_writer.write(frame)
                mp4_writer.write(frame)

                row = {
                    "frame_index": written,
                    "stamp": f"{stamp:.9f}",
                    "relative_time": f"{stamp - first_image_stamp:.6f}",
                    "fusion_status": status_at(statuses, stamp),
                }
                for prefix, pose_key in (
                    ("replayed_fused", "replayed_fused"),
                    ("replayed_local", "replayed_local"),
                    ("replayed_local_map", "replayed_local_map"),
                    ("raw_wheel", "raw_wheel"),
                    ("raw_visual", "raw_visual"),
                    ("raw_fused", "raw_fused"),
                ):
                    row.update(format_pose_row(prefix, poses.get(pose_key)))
                csv_writer.writerow(row)

                written += 1
                last_stamp = stamp
                if written % 200 == 0:
                    print(f"rendered {written} frames", flush=True)
                if args.max_frames > 0 and written >= args.max_frames:
                    break
        finally:
            avi_writer.release()
            mp4_writer.release()

    write_trajectory_png(png_path, view, trajectories, last_stamp)
    summary = {
        "raw_bag": str(args.raw_bag),
        "replay_bag": str(args.replay_bag),
        "map": str(args.map),
        "image_topic": args.image_topic,
        "sync_tolerance_s": args.sync_tolerance,
        "fps": args.fps,
        "image_stride": args.image_stride,
        "frames_written": written,
        "frames_decoded": decoded,
        "frames_skipped": skipped,
        "image_first_stamp": first_image_stamp,
        "image_last_stamp": last_stamp,
        "replayed_topic_counts": {
            "odometry/fused": len(replay_fused),
            "odometry/local": len(replay_local),
            "odometry/local_map": len(replay_local_map),
        },
        "raw_topic_counts": {
            "wheel/odom": len(raw_wheel),
            "wheel/odom_transformed_to_map": len(raw_wheel_map),
            "odometry/visual_raw": len(raw_visual),
            "odometry/fused": len(raw_fused),
        },
        "outputs": {
            "avi": str(avi_path),
            "mp4": str(mp4_path),
            "csv": str(csv_path),
            "trajectory_png": str(png_path),
        },
    }
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {written} frames to {avi_path}")
    print(f"wrote {written} frames to {mp4_path}")
    print(f"wrote per-frame data to {csv_path}")
    print(f"wrote trajectory image to {png_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
