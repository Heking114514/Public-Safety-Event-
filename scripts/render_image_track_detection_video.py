#!/usr/bin/env python3
"""Render image track/map detection overlays from a recorded rosbag."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

import offline_image_track_correction as correction


DEFAULT_BAG = Path("/media/hjh/Data/rosbag_recording/latest_navigation_bag")
DEFAULT_MAP = Path("src/arena_path_planner/config/arena_map.yaml")
DEFAULT_OUT = Path(
    "analysis_bags/20260921T171415.029504Z_9ead9ca47d9b/rgb_road_detection_overlay.webm"
)


@dataclass
class DetectedSegment:
    image_line: tuple[int, int, int, int]
    ground: correction.GroundSegment
    label: str = ""
    kind: str = ""
    distance: float = float("inf")
    yaw_error: float = float("inf")


def feature_is_allowed(feature: correction.LineFeature, args) -> bool:
    if args.feature_mode == "all":
        return True
    if args.feature_mode == "physical":
        return feature.kind != "road_center"
    return feature.kind == "road_center"


def image_to_bgr(message: Image) -> np.ndarray | None:
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding == "rgb8":
        rgb = data.reshape((message.height, message.step // 3, 3))[:, : message.width]
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if message.encoding == "bgr8":
        return data.reshape((message.height, message.step // 3, 3))[:, : message.width].copy()
    gray = correction.image_to_gray(message)
    if gray is not None:
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return None


def filter_components(mask: np.ndarray, args) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    output = np.zeros_like(mask)
    max_area = args.max_component_area_fraction * float(mask.shape[0] * mask.shape[1])
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        if area < args.min_component_area or area > max_area:
            continue
        if width < args.min_component_width and height < args.min_component_height:
            continue
        output[labels == label] = 255
    return output


def detect_mask(bgr: np.ndarray, args) -> np.ndarray:
    if args.detector == "rgb_dark":
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mask = np.all(rgb <= args.rgb_threshold, axis=2).astype(np.uint8) * 255
    elif args.detector == "local_dark":
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        sigma = max(1.0, args.local_sigma)
        background = cv2.GaussianBlur(gray, (0, 0), sigmaX=sigma, sigmaY=sigma)
        difference = cv2.subtract(background, gray)
        mask = cv2.threshold(difference, args.local_delta, 255, cv2.THRESH_BINARY)[1]
    elif args.detector in ("rgb_and_local", "rgb_or_local"):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb_mask = np.all(rgb <= args.rgb_threshold, axis=2).astype(np.uint8) * 255
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        sigma = max(1.0, args.local_sigma)
        background = cv2.GaussianBlur(gray, (0, 0), sigmaX=sigma, sigmaY=sigma)
        difference = cv2.subtract(background, gray)
        local_mask = cv2.threshold(difference, args.local_delta, 255, cv2.THRESH_BINARY)[1]
        if args.detector == "rgb_and_local":
            mask = cv2.bitwise_and(rgb_mask, local_mask)
        else:
            mask = cv2.bitwise_or(rgb_mask, local_mask)
    else:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        denoised = cv2.bilateralFilter(gray, 7, 45, 45)
        blackhat_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (args.blackhat_size, args.blackhat_size)
        )
        blackhat = cv2.morphologyEx(denoised, cv2.MORPH_BLACKHAT, blackhat_kernel)
        mask = cv2.threshold(blackhat, args.blackhat_threshold, 255, cv2.THRESH_BINARY)[1]

    roi_top = int(round(mask.shape[0] * args.roi_top_fraction))
    mask[:roi_top, :] = 0
    roi_bottom = int(round(mask.shape[0] * args.roi_bottom_fraction))
    if roi_bottom < mask.shape[0]:
        mask[max(0, roi_bottom) :, :] = 0

    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (args.morphology_size, args.morphology_size)
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return filter_components(mask, args)


def detect_segments(bgr: np.ndarray, camera: correction.CameraModel, args) -> tuple[list[DetectedSegment], np.ndarray]:
    mask = detect_mask(bgr, args)

    lines = cv2.HoughLinesP(
        mask,
        1.0,
        np.pi / 180.0,
        args.hough_threshold,
        minLineLength=args.min_line_length_px,
        maxLineGap=args.max_line_gap_px,
    )
    if lines is None:
        return [], mask

    limits = {
        "min_ray_down_z": args.min_ray_down_z,
        "min_x": args.min_ground_x,
        "max_x": args.max_ground_x,
        "max_abs_y": args.max_abs_ground_y,
    }
    detected: list[DetectedSegment] = []
    for raw in lines[:, 0, :]:
        p1 = correction.project_pixel(camera, float(raw[0]), float(raw[1]), limits)
        p2 = correction.project_pixel(camera, float(raw[2]), float(raw[3]), limits)
        if p1 is None or p2 is None:
            continue
        length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if length < args.min_ground_segment_length:
            continue
        ground = correction.GroundSegment(
            p1[0],
            p1[1],
            p2[0],
            p2[1],
            length,
            math.atan2(p2[1] - p1[1], p2[0] - p1[0]),
        )
        detected.append(
            DetectedSegment((int(raw[0]), int(raw[1]), int(raw[2]), int(raw[3])), ground)
        )
    detected.sort(key=lambda segment: segment.ground.length, reverse=True)
    return detected[: args.max_segments], mask


def match_segments(
    pose: correction.PoseSample,
    detected: list[DetectedSegment],
    features: list[correction.LineFeature],
    args,
) -> None:
    for detected_segment in detected:
        observed = correction.transform_segment(pose, detected_segment.ground)
        mx = 0.5 * (observed.ax + observed.bx)
        my = 0.5 * (observed.ay + observed.by)
        best = None
        best_score = float("inf")
        for feature in features:
            if not feature_is_allowed(feature, args):
                continue
            px, py, distance = correction.nearest_on_feature(mx, my, feature)
            if distance > args.max_match_distance:
                continue
            feature_yaw = math.atan2(feature.by - feature.ay, feature.bx - feature.ax)
            yaw_error = correction.undirected_angle_error(feature_yaw, observed.yaw)
            if abs(yaw_error) > args.max_match_angle:
                continue
            score = distance + args.angle_weight * abs(yaw_error)
            if score < best_score:
                best_score = score
                best = (feature, distance, yaw_error)
        if best is None:
            continue
        feature, distance, yaw_error = best
        detected_segment.label = feature.label
        detected_segment.kind = feature.kind
        detected_segment.distance = distance
        detected_segment.yaw_error = yaw_error


def segment_color(segment: DetectedSegment) -> tuple[int, int, int]:
    if not segment.label:
        return 0, 220, 255
    if segment.kind == "obstacle_edge":
        return 0, 180, 0
    if segment.kind == "free_edge":
        return 255, 160, 0
    return 255, 80, 80


def put_text(image: np.ndarray, text: str, x: int, y: int, scale: float = 0.45) -> None:
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


def draw_camera_panel(
    bgr: np.ndarray,
    mask: np.ndarray,
    detected: list[DetectedSegment],
    pose: correction.PoseSample,
    stamp: float,
    frame_index: int,
    args,
) -> np.ndarray:
    panel = bgr.copy()
    red = np.zeros_like(panel)
    red[:, :, 2] = 255
    panel = np.where(mask[:, :, None] > 0, cv2.addWeighted(panel, 0.45, red, 0.55, 0), panel)

    roi_top = int(round(mask.shape[0] * args.roi_top_fraction))
    roi_bottom = int(round(mask.shape[0] * args.roi_bottom_fraction))
    cv2.line(panel, (0, roi_top), (panel.shape[1] - 1, roi_top), (255, 255, 0), 1)
    cv2.line(panel, (0, roi_bottom), (panel.shape[1] - 1, roi_bottom), (255, 255, 0), 1)

    for segment in detected:
        x1, y1, x2, y2 = segment.image_line
        cv2.line(panel, (x1, y1), (x2, y2), segment_color(segment), 2, cv2.LINE_AA)
        if segment.label and args.draw_labels:
            put_text(panel, segment.label.split(":")[0], x1, max(18, y1 - 4), 0.35)

    matched = sum(1 for segment in detected if segment.label)
    put_text(panel, f"frame={frame_index} t={stamp:.3f}", 12, 24)
    put_text(
        panel,
        f"segments={len(detected)} matched={matched} detector={args.detector} rgb<={args.rgb_threshold}",
        12,
        48,
    )
    put_text(panel, f"fused x={pose.x:.3f} y={pose.y:.3f} yaw={math.degrees(pose.yaw):.1f}", 12, 72)
    return panel


def map_to_pixel(x: float, y: float, center: correction.PoseSample, scale: float, size: int) -> tuple[int, int]:
    return int(round(size * 0.5 + (x - center.x) * scale)), int(round(size * 0.5 - (y - center.y) * scale))


def draw_map_line(
    panel: np.ndarray,
    ax: float,
    ay: float,
    bx: float,
    by: float,
    center: correction.PoseSample,
    scale: float,
    color: tuple[int, int, int],
    thickness: int,
) -> None:
    p1 = map_to_pixel(ax, ay, center, scale, panel.shape[0])
    p2 = map_to_pixel(bx, by, center, scale, panel.shape[0])
    cv2.line(panel, p1, p2, color, thickness, cv2.LINE_AA)


def draw_map_panel(
    pose: correction.PoseSample,
    detected: list[DetectedSegment],
    features: list[correction.LineFeature],
    args,
) -> np.ndarray:
    size = args.map_panel_size
    panel = np.full((size, size, 3), 245, dtype=np.uint8)
    scale = size / args.map_width_m

    for feature in features:
        if feature.kind == "road_center":
            color = (215, 205, 195)
            thickness = 1
        elif feature.kind == "obstacle_edge":
            color = (0, 165, 255)
            thickness = 1
        else:
            color = (180, 170, 155)
            thickness = 1
        draw_map_line(panel, feature.ax, feature.ay, feature.bx, feature.by, pose, scale, color, thickness)

    for segment in detected:
        observed = correction.transform_segment(pose, segment.ground)
        thickness = 3 if segment.label else 2
        draw_map_line(
            panel,
            observed.ax,
            observed.ay,
            observed.bx,
            observed.by,
            pose,
            scale,
            segment_color(segment),
            thickness,
        )

    car = np.array(
        [
            map_to_pixel(pose.x + 0.11 * math.cos(pose.yaw), pose.y + 0.11 * math.sin(pose.yaw), pose, scale, size),
            map_to_pixel(
                pose.x + 0.06 * math.cos(pose.yaw + 2.35),
                pose.y + 0.06 * math.sin(pose.yaw + 2.35),
                pose,
                scale,
                size,
            ),
            map_to_pixel(
                pose.x + 0.06 * math.cos(pose.yaw - 2.35),
                pose.y + 0.06 * math.sin(pose.yaw - 2.35),
                pose,
                scale,
                size,
            ),
        ],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(panel, car, (50, 50, 230), cv2.LINE_AA)
    cv2.circle(panel, (size // 2, size // 2), 3, (0, 0, 0), -1)
    put_text(panel, f"local map {args.map_width_m:.1f}m", 10, 22, 0.42)
    put_text(panel, "green/blue=matched  yellow=unmatched", 10, size - 12, 0.34)
    return panel


def make_video_frame(
    bgr: np.ndarray,
    mask: np.ndarray,
    detected: list[DetectedSegment],
    pose: correction.PoseSample,
    features: list[correction.LineFeature],
    stamp: float,
    frame_index: int,
    args,
) -> np.ndarray:
    camera_panel = draw_camera_panel(bgr, mask, detected, pose, stamp, frame_index, args)
    map_panel = draw_map_panel(pose, detected, features, args)
    if map_panel.shape[0] != camera_panel.shape[0]:
        map_panel = cv2.resize(map_panel, (camera_panel.shape[0], camera_panel.shape[0]))
    return np.hstack((camera_panel, map_panel))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--fused-topic", default="/odometry/fused")
    parser.add_argument("--left-image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--left-camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--sync-tolerance", type=float, default=0.12)
    parser.add_argument("--image-stride", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--codec", default="VP80")
    parser.add_argument("--camera-x", type=float, default=0.055)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=0.121)
    parser.add_argument("--camera-pitch-down", type=float, default=0.0)
    parser.add_argument("--roi-top-fraction", type=float, default=0.60)
    parser.add_argument("--roi-bottom-fraction", type=float, default=0.94)
    parser.add_argument(
        "--detector",
        choices=("rgb_dark", "local_dark", "rgb_and_local", "rgb_or_local", "blackhat"),
        default="rgb_or_local",
    )
    parser.add_argument("--rgb-threshold", type=int, default=60)
    parser.add_argument("--local-delta", type=int, default=20)
    parser.add_argument("--local-sigma", type=float, default=18.0)
    parser.add_argument("--min-component-area", type=int, default=8)
    parser.add_argument("--max-component-area-fraction", type=float, default=0.18)
    parser.add_argument("--min-component-width", type=int, default=4)
    parser.add_argument("--min-component-height", type=int, default=4)
    parser.add_argument("--max-dark-value", type=int, default=92)
    parser.add_argument("--use-fixed-dark", action="store_true")
    parser.add_argument("--use-adaptive-dark", action="store_true")
    parser.add_argument("--adaptive-block-size", type=int, default=31)
    parser.add_argument("--adaptive-c", type=float, default=6.0)
    parser.add_argument("--blackhat-size", type=int, default=31)
    parser.add_argument("--blackhat-threshold", type=int, default=20)
    parser.add_argument("--morphology-size", type=int, default=3)
    parser.add_argument("--hough-threshold", type=int, default=14)
    parser.add_argument("--min-line-length-px", type=int, default=12)
    parser.add_argument("--max-line-gap-px", type=int, default=12)
    parser.add_argument("--min-ray-down-z", type=float, default=0.025)
    parser.add_argument("--min-ground-x", type=float, default=0.05)
    parser.add_argument("--max-ground-x", type=float, default=1.50)
    parser.add_argument("--max-abs-ground-y", type=float, default=0.80)
    parser.add_argument("--min-ground-segment-length", type=float, default=0.035)
    parser.add_argument("--max-segments", type=int, default=40)
    parser.add_argument("--max-match-distance", type=float, default=0.08)
    parser.add_argument("--max-match-angle", type=float, default=0.42)
    parser.add_argument("--angle-weight", type=float, default=0.05)
    parser.add_argument("--feature-mode", choices=("physical", "all", "road_center"), default="physical")
    parser.add_argument("--map-panel-size", type=int, default=480)
    parser.add_argument("--map-width-m", type=float, default=1.8)
    parser.add_argument("--draw-labels", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    cache_dir = args.out.parent / "decompressed"
    db_paths = correction.decompress_bag_files(args.bag, cache_dir)
    fused = correction.load_odometry(db_paths, args.fused_topic)
    if not fused:
        raise RuntimeError(f"no fused odometry on {args.fused_topic}")
    camera = correction.load_camera_model(
        db_paths,
        args.left_camera_info_topic,
        correction.CameraModel(
            x=args.camera_x,
            y=args.camera_y,
            z=args.camera_z,
            pitch_down=args.camera_pitch_down,
        ),
    )
    camera.x = args.camera_x
    camera.y = args.camera_y
    camera.z = args.camera_z
    camera.pitch_down = args.camera_pitch_down
    features = correction.load_map_features(args.map)

    writer = None
    processed = 0
    written = 0
    try:
        for timestamp_ns, data in correction.iter_topic(db_paths, args.left_image_topic):
            if processed % args.image_stride != 0:
                processed += 1
                continue
            processed += 1
            msg = deserialize_message(data, Image)
            stamp = correction.stamp_seconds(msg, timestamp_ns)
            pose = correction.interpolate_pose(fused, stamp, args.sync_tolerance)
            if pose is None:
                continue
            bgr = image_to_bgr(msg)
            if bgr is None:
                continue
            detected, mask = detect_segments(bgr, camera, args)
            match_segments(pose, detected, features, args)
            frame = make_video_frame(bgr, mask, detected, pose, features, stamp, written, args)
            if writer is None:
                fourcc = cv2.VideoWriter_fourcc(*args.codec)
                writer = cv2.VideoWriter(str(args.out), fourcc, args.fps, (frame.shape[1], frame.shape[0]))
                if not writer.isOpened():
                    raise RuntimeError(f"failed to open video writer for {args.out}")
            writer.write(frame)
            written += 1
            if written % 200 == 0:
                print(f"wrote {written} frames to {args.out}")
            if args.max_frames > 0 and written >= args.max_frames:
                break
    finally:
        if writer is not None:
            writer.release()
    print(f"wrote {written} frames to {args.out}")


if __name__ == "__main__":
    main()
