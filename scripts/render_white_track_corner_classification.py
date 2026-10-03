#!/usr/bin/env python3
"""Render RGB white-track corner classifications from a rosbag.

The output is an Ubuntu-friendly MJPG AVI plus a CSV containing raw and
temporally stable labels. This is an offline experiment and does not publish
ROS topics or modify the navigation stack.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import Counter, deque
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

import offline_image_track_correction as correction

VISION_SOURCE = Path(__file__).resolve().parents[1] / "src" / "vision_correction"
if str(VISION_SOURCE) not in sys.path:
    sys.path.insert(0, str(VISION_SOURCE))

from vision_correction import road_detector  # noqa: E402
from white_track_corner_detector import (  # noqa: E402
    CROSS_JUNCTION,
    LEFT_CORNER,
    LOST,
    RIGHT_CORNER,
    STRAIGHT,
    T_JUNCTION,
    UNKNOWN,
    WhiteTrackDetection,
    WhiteTrackParameters,
    detect_white_track,
)
from vehicle_physical import VEHICLE_PHYSICAL


DEFAULT_BAG = Path(
    "/media/hjh/E/rosbag_recording/navigation_runs/"
    "20260928T174028.206047Z_163fb8060938/bag"
)
DEFAULT_OUT = Path(
    "/media/hjh/E/rosbag_recording/bag_exports/"
    "20260928T174028.206047Z_163fb8060938/analysis/"
    "white_track_corner_classification.avi"
)


class StableLabel:
    def __init__(self, window: int, minimum_votes: int) -> None:
        self._history: deque[tuple[str, float]] = deque(maxlen=max(3, window))
        self._minimum_votes = max(2, minimum_votes)
        self._lost_streak = 0
        self._unknown_streak = 0

    def update(self, label: str, confidence: float) -> str:
        if label == LOST:
            self._lost_streak += 1
            self._unknown_streak = 0
            if self._lost_streak >= 2:
                self._history.clear()
                return LOST
        elif label == UNKNOWN:
            self._unknown_streak += 1
            self._lost_streak = 0
            if self._unknown_streak >= 2:
                self._history.clear()
                return UNKNOWN
        else:
            self._lost_streak = 0
            self._unknown_streak = 0
        self._history.append((label, confidence))
        counts = Counter(item[0] for item in self._history)
        selected, votes = counts.most_common(1)[0]
        if votes < self._minimum_votes:
            return UNKNOWN
        selected_confidence = [
            confidence
            for item, confidence in self._history
            if item == selected
        ]
        if sum(selected_confidence) / max(1, len(selected_confidence)) < 0.30:
            return UNKNOWN
        return selected


def image_to_bgr(message: Image) -> np.ndarray | None:
    return correction.image_to_bgr(message)


def cache_directory(bag: Path, output: Path) -> Path:
    run_id = bag.parent.name if bag.name == "bag" else ""
    if run_id:
        sibling = (
            bag.parent.parent.parent
            / "bag_exports"
            / run_id
            / "decompressed"
        )
        if sibling.is_dir() and list(sibling.glob("*.db3")):
            return sibling
    return output.parent / "decompressed"


def put_text(
    image: np.ndarray,
    text: str,
    x: int,
    y: int,
    scale: float = 0.48,
    color: tuple[int, int, int] = (255, 255, 255),
) -> None:
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        1,
        cv2.LINE_AA,
    )


def draw_ground_roi(
    image: np.ndarray,
    polygon: list[tuple[int, int]],
) -> None:
    if polygon:
        cv2.polylines(
            image,
            [np.asarray(polygon, dtype=np.int32)],
            True,
            (255, 220, 0),
            2,
            cv2.LINE_AA,
        )


def draw_image_overlay(
    bgr: np.ndarray,
    detection: WhiteTrackDetection,
    polygon: list[tuple[int, int]],
    camera: road_detector.CameraGeometry,
    params: WhiteTrackParameters,
    stable: str,
    frame_index: int,
    stamp: float,
) -> np.ndarray:
    panel = bgr.copy()
    white_overlay = np.zeros_like(panel)
    white_overlay[:, :, 2] = 255
    panel = np.where(
        detection.mask[:, :, None] != 0,
        cv2.addWeighted(panel, 0.55, white_overlay, 0.45, 0),
        panel,
    )
    draw_ground_roi(panel, polygon)

    if detection.component_mask.size:
        contours, _ = cv2.findContours(
            detection.component_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        for contour in contours:
            points: list[tuple[int, int]] = []
            for point in contour[:, 0, :]:
                row, column = int(point[1]), int(point[0])
                grid_x = params.min_ground_x_m + (
                    row + 0.5
                ) * params.ground_grid_resolution_m
                grid_y = -params.max_abs_ground_y_m + (
                    column + 0.5
                ) * params.ground_grid_resolution_m
                pixel = road_detector.ground_to_pixel(camera, grid_x, grid_y)
                if pixel is not None:
                    points.append(
                        (int(round(pixel[0])), int(round(pixel[1])))
                    )
            if len(points) >= 3:
                cv2.polylines(
                    panel,
                    [np.asarray(points, dtype=np.int32)],
                    True,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

    color = (0, 255, 0) if stable in (STRAIGHT, LEFT_CORNER, RIGHT_CORNER, T_JUNCTION, CROSS_JUNCTION) else (0, 180, 255)
    put_text(panel, f"raw={detection.state} stable={stable}", 12, 26, 0.62, color)
    put_text(
        panel,
        (
            f"frame={frame_index} t={stamp:.2f}s "
            f"conf={detection.confidence:.2f} "
            f"green={detection.green_fraction:.2f}"
        ),
        12,
        52,
    )
    put_text(
        panel,
        (
            f"near={detection.near_center_pixels}/{detection.near_support_rows} "
            f"branch_x={detection.branch_x:.2f} "
            f"L={detection.left_extent:.2f} R={detection.right_extent:.2f} "
            f"forward={detection.forward_continuation_rows}"
        ),
        12,
        78,
        0.42,
    )
    return panel


def ground_panel(
    detection: WhiteTrackDetection,
    stable: str,
    params: WhiteTrackParameters,
    size: int = 480,
) -> np.ndarray:
    panel = np.zeros((detection.ground_mask.shape[0], detection.ground_mask.shape[1], 3), dtype=np.uint8)
    panel[:, :] = (25, 42, 25)
    panel[detection.green_mask != 0] = (35, 100, 35)
    panel[detection.ground_mask != 0] = (135, 135, 135)
    panel[detection.component_mask != 0] = (240, 240, 240)

    grid_height, grid_width = panel.shape[:2]
    zero_column = int(round((0.0 + params.max_abs_ground_y_m) / params.ground_grid_resolution_m))
    zero_column = max(0, min(grid_width - 1, zero_column))
    cv2.line(panel, (zero_column, 0), (zero_column, grid_height - 1), (0, 220, 255), 1)
    for x_value in (0.2, 0.4, 0.6, 0.8, 1.0, 1.2):
        row = int(round((x_value - params.min_ground_x_m) / params.ground_grid_resolution_m))
        if 0 <= row < grid_height:
            cv2.line(panel, (0, row), (grid_width - 1, row), (80, 80, 80), 1)
    if math.isfinite(detection.branch_x):
        row = int(round((detection.branch_x - params.min_ground_x_m) / params.ground_grid_resolution_m))
        if 0 <= row < grid_height:
            cv2.line(panel, (0, row), (grid_width - 1, row), (0, 165, 255), 2)

    panel = cv2.resize(panel, (size, size), interpolation=cv2.INTER_NEAREST)
    color = (0, 255, 0) if stable not in (UNKNOWN, LOST) else (0, 180, 255)
    put_text(panel, stable, 10, 24, 0.62, color)
    put_text(
        panel,
        "top=far  bottom=near  yellow=x=0",
        10,
        size - 12,
        0.38,
    )
    return panel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--fused-topic", default="/odometry/fused")
    parser.add_argument("--sync-tolerance", type=float, default=0.12)
    parser.add_argument("--image-stride", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--stable-window", type=int, default=7)
    parser.add_argument("--stable-votes", type=int, default=4)
    parser.add_argument(
        "--camera-height", type=float, default=VEHICLE_PHYSICAL.camera_z_m
    )
    parser.add_argument("--camera-x", type=float, default=VEHICLE_PHYSICAL.camera_x_m)
    parser.add_argument("--camera-y", type=float, default=VEHICLE_PHYSICAL.camera_y_m)
    parser.add_argument("--ground-min-x", type=float, default=0.10)
    parser.add_argument("--ground-max-x", type=float, default=1.35)
    parser.add_argument("--ground-max-y", type=float, default=0.75)
    parser.add_argument("--ground-resolution", type=float, default=0.01)
    parser.add_argument("--max-near-center-offset", type=float, default=0.18)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    cache = cache_directory(args.bag, args.out)
    db_paths = correction.decompress_bag_files(args.bag, cache)
    fused = correction.load_odometry(db_paths, args.fused_topic)
    camera_model = correction.load_camera_model(
        db_paths,
        args.camera_info_topic,
        correction.CameraModel(
            x=args.camera_x,
            y=args.camera_y,
            z=args.camera_height,
        ),
        tf_topic="/tf_static",
    )
    camera = correction.camera_geometry(camera_model)
    params = WhiteTrackParameters(
        min_ground_x_m=args.ground_min_x,
        max_ground_x_m=args.ground_max_x,
        max_abs_ground_y_m=args.ground_max_y,
        ground_grid_resolution_m=args.ground_resolution,
        max_near_center_offset_m=args.max_near_center_offset,
    )
    stable_label = StableLabel(args.stable_window, args.stable_votes)
    writer = None
    csv_file = args.out.with_suffix(".csv")
    csv_stream = csv_file.open("w", newline="", encoding="utf-8")
    csv_writer = csv.DictWriter(
        csv_stream,
        fieldnames=[
            "frame_index",
            "stamp",
            "fused_x",
            "fused_y",
            "fused_yaw",
            "raw_state",
            "stable_state",
            "confidence",
            "white_threshold",
            "green_baseline",
            "green_fraction",
            "component_area",
            "near_center_pixels",
            "near_support_rows",
            "near_center_y",
            "branch_x",
            "left_extent",
            "right_extent",
            "forward_continuation_rows",
        ],
    )
    csv_writer.writeheader()
    processed = 0
    written = 0
    stable_counts: Counter[str] = Counter()
    try:
        for timestamp_ns, data in correction.iter_topic(db_paths, args.image_topic):
            if processed % max(1, args.image_stride) != 0:
                processed += 1
                continue
            processed += 1
            message = deserialize_message(data, Image)
            stamp = correction.stamp_seconds(message, timestamp_ns)
            bgr = image_to_bgr(message)
            if bgr is None:
                continue
            bgr = road_detector.undistort_bgr(bgr, camera)
            detection = detect_white_track(bgr, camera, params)
            stable = stable_label.update(detection.state, detection.confidence)
            stable_counts[stable] += 1
            pose = correction.interpolate_pose(fused, stamp, args.sync_tolerance)
            if pose is None:
                pose = correction.PoseSample(stamp, 0.0, 0.0, 0.0)

            _, polygon = road_detector.ground_roi_mask(bgr.shape, camera, params)
            camera_panel = draw_image_overlay(
                bgr,
                detection,
                polygon,
                camera,
                params,
                stable,
                written,
                stamp,
            )
            bird_panel = ground_panel(detection, stable, params)
            frame = np.hstack((camera_panel, bird_panel))
            if writer is None:
                writer = cv2.VideoWriter(
                    str(args.out),
                    cv2.VideoWriter_fourcc(*"MJPG"),
                    args.fps,
                    (frame.shape[1], frame.shape[0]),
                )
                if not writer.isOpened():
                    raise RuntimeError(f"failed to open {args.out}")
            writer.write(frame)
            csv_writer.writerow(
                {
                    "frame_index": written,
                    "stamp": f"{stamp:.9f}",
                    "fused_x": f"{pose.x:.6f}",
                    "fused_y": f"{pose.y:.6f}",
                    "fused_yaw": f"{pose.yaw:.6f}",
                    "raw_state": detection.state,
                    "stable_state": stable,
                    "confidence": f"{detection.confidence:.4f}",
                    "white_threshold": f"{detection.white_threshold:.3f}",
                    "green_baseline": f"{detection.green_baseline:.3f}",
                    "green_fraction": f"{detection.green_fraction:.4f}",
                    "component_area": detection.component_area,
                    "near_center_pixels": detection.near_center_pixels,
                    "near_support_rows": detection.near_support_rows,
                    "near_center_y": f"{detection.near_center_y:.4f}",
                    "branch_x": f"{detection.branch_x:.4f}",
                    "left_extent": f"{detection.left_extent:.4f}",
                    "right_extent": f"{detection.right_extent:.4f}",
                    "forward_continuation_rows": detection.forward_continuation_rows,
                }
            )
            written += 1
            if written % 200 == 0:
                print(f"wrote {written} frames")
            if args.max_frames > 0 and written >= args.max_frames:
                break
    finally:
        if writer is not None:
            writer.release()
        csv_stream.close()
    print(f"wrote {written} frames to {args.out}")
    print("stable labels:", dict(stable_counts))


if __name__ == "__main__":
    main()
