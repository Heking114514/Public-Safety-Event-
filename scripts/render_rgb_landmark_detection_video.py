#!/usr/bin/env python3
"""Detect ground-plane right-angle landmarks from recorded RGB images."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

import offline_image_track_correction as correction
import render_image_track_detection_video as detector


DEFAULT_BAG = Path("/media/hjh/Data/rosbag_recording/latest_navigation_bag")
DEFAULT_MAP = Path("src/arena_path_planner/config/arena_map.yaml")
DEFAULT_OUT = Path(
    "analysis_bags/20260921T171415.029504Z_9ead9ca47d9b/rgb_landmark_detection.webm"
)


@dataclass(frozen=True)
class GroundLine:
    image_line: tuple[int, int, int, int]
    ground: correction.GroundSegment


@dataclass
class Landmark:
    x: float
    y: float
    image_u: float
    image_v: float
    kind: str
    arms: int
    support_lines: int
    map_x: float = 0.0
    map_y: float = 0.0
    matched_label: str = ""
    matched_distance: float = float("inf")


@dataclass(frozen=True)
class MapLandmark:
    x: float
    y: float
    label: str
    kind: str


def line_intersection(
    first: correction.GroundSegment,
    second: correction.GroundSegment,
) -> tuple[float, float] | None:
    px, py = first.ax, first.ay
    rx, ry = first.bx - first.ax, first.by - first.ay
    qx, qy = second.ax, second.ay
    sx, sy = second.bx - second.ax, second.by - second.ay
    denominator = rx * sy - ry * sx
    if abs(denominator) < 1.0e-8:
        return None
    qmpx, qmpy = qx - px, qy - py
    t = (qmpx * sy - qmpy * sx) / denominator
    return px + t * rx, py + t * ry


def point_segment_distance(x: float, y: float, segment: correction.GroundSegment) -> float:
    _, _, distance = correction.nearest_on_feature(
        x,
        y,
        correction.LineFeature(
            segment.ax,
            segment.ay,
            segment.bx,
            segment.by,
            "",
            "",
        ),
    )
    return distance


def angle_distance(first: float, second: float) -> float:
    return abs(correction.wrap_angle(first - second))


def orthogonal_error(first: float, second: float) -> float:
    difference = abs(correction.undirected_angle_error(first, second))
    return abs(difference - 0.5 * math.pi)


def cluster_angles(angles: list[float], tolerance: float) -> int:
    if not angles:
        return 0
    clusters: list[list[float]] = []
    for angle in (correction.wrap_angle(value) for value in angles):
        target = None
        target_distance = float("inf")
        for index, cluster in enumerate(clusters):
            sine = sum(math.sin(value) for value in cluster)
            cosine = sum(math.cos(value) for value in cluster)
            center = math.atan2(sine, cosine)
            distance = angle_distance(angle, center)
            if distance <= tolerance and distance < target_distance:
                target = index
                target_distance = distance
        if target is None:
            clusters.append([angle])
        else:
            clusters[target].append(angle)

    if len(clusters) > 1:
        first_center = math.atan2(
            sum(math.sin(value) for value in clusters[0]),
            sum(math.cos(value) for value in clusters[0]),
        )
        last_center = math.atan2(
            sum(math.sin(value) for value in clusters[-1]),
            sum(math.cos(value) for value in clusters[-1]),
        )
        if angle_distance(first_center, last_center) <= tolerance:
            clusters[0].extend(clusters.pop())
    return min(len(clusters), 4)


def connected_arm_angles(
    x: float,
    y: float,
    lines: list[GroundLine],
    args,
) -> tuple[int, int]:
    angles: list[float] = []
    support_lines = 0
    for line in lines:
        segment = line.ground
        distance = point_segment_distance(x, y, segment)
        if distance > args.landmark_connection_radius:
            continue
        support_lines += 1
        length = max(segment.length, 1.0e-9)
        dx = segment.bx - segment.ax
        dy = segment.by - segment.ay
        projection = ((x - segment.ax) * dx + (y - segment.ay) * dy) / (length * length)
        if args.landmark_endpoint_fraction <= projection <= 1.0 - args.landmark_endpoint_fraction:
            theta = math.atan2(dy, dx)
            angles.extend((theta, theta + math.pi))
            continue
        if projection < 0.5:
            endpoint_x, endpoint_y = segment.ax, segment.ay
        else:
            endpoint_x, endpoint_y = segment.bx, segment.by
        theta = math.atan2(endpoint_y - y, endpoint_x - x)
        angles.append(theta)
    return cluster_angles(angles, args.arm_angle_cluster_tolerance), support_lines


def landmark_kind(arms: int, args) -> str:
    if arms >= 4:
        return "CROSS"
    if arms == 3:
        return "T"
    if arms == 2:
        return "L"
    return "POINT"


def detect_ground_lines(
    bgr: np.ndarray,
    camera: correction.CameraModel,
    args,
) -> tuple[list[GroundLine], np.ndarray]:
    mask = detector.detect_mask(bgr, args)
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
    result: list[GroundLine] = []
    for raw in lines[:, 0, :]:
        x1, y1, x2, y2 = map(float, raw)
        first = correction.project_pixel(camera, x1, y1, limits)
        second = correction.project_pixel(camera, x2, y2, limits)
        if first is None or second is None:
            continue
        length = math.hypot(second[0] - first[0], second[1] - first[1])
        if length < args.min_ground_segment_length:
            continue
        result.append(
            GroundLine(
                (int(x1), int(y1), int(x2), int(y2)),
                correction.GroundSegment(
                    first[0],
                    first[1],
                    second[0],
                    second[1],
                    length,
                    math.atan2(second[1] - first[1], second[0] - first[0]),
                ),
            )
        )
    result.sort(key=lambda line: line.ground.length, reverse=True)
    return result[: args.max_lines], mask


def merge_landmarks(candidates: list[Landmark], args) -> list[Landmark]:
    merged: list[Landmark] = []
    for candidate in sorted(
        candidates,
        key=lambda landmark: (landmark.arms, landmark.support_lines),
        reverse=True,
    ):
        existing = next(
            (
                landmark
                for landmark in merged
                if math.hypot(landmark.x - candidate.x, landmark.y - candidate.y)
                <= args.landmark_merge_radius
            ),
            None,
        )
        if existing is None:
            merged.append(candidate)
            continue
        if candidate.arms > existing.arms or candidate.support_lines > existing.support_lines:
            merged[merged.index(existing)] = candidate
    return merged[: args.max_landmarks]


def detect_landmarks(lines: list[GroundLine], camera: correction.CameraModel, args) -> list[Landmark]:
    candidates: list[Landmark] = []
    for index, first in enumerate(lines):
        for second in lines[index + 1 :]:
            if orthogonal_error(first.ground.yaw, second.ground.yaw) > args.corner_angle_tolerance:
                continue
            intersection = line_intersection(first.ground, second.ground)
            if intersection is None:
                continue
            x, y = intersection
            if not (args.min_ground_x <= x <= args.max_ground_x and abs(y) <= args.max_abs_ground_y):
                continue
            if point_segment_distance(x, y, first.ground) > args.landmark_line_extension:
                continue
            if point_segment_distance(x, y, second.ground) > args.landmark_line_extension:
                continue
            arms, support_lines = connected_arm_angles(x, y, lines, args)
            if arms < args.min_landmark_arms:
                continue
            image_point = ground_to_pixel(camera, x, y)
            if image_point is None:
                continue
            kind = landmark_kind(arms, args)
            candidates.append(
                Landmark(
                    x=x,
                    y=y,
                    image_u=image_point[0],
                    image_v=image_point[1],
                    kind=kind,
                    arms=arms,
                    support_lines=support_lines,
                )
            )
    return merge_landmarks(candidates, args)


def ground_to_pixel(camera: correction.CameraModel, x: float, y: float) -> tuple[float, float] | None:
    base_x = x - camera.x
    base_y = y - camera.y
    base_z = -camera.z
    if base_x <= 1.0e-5:
        return None
    cp = math.cos(camera.pitch_down)
    sp = math.sin(camera.pitch_down)
    optical_x = cp * base_x - sp * base_z
    optical_z = sp * base_x + cp * base_z
    if optical_x <= 1.0e-5:
        return None
    optical_y = base_y
    u = camera.cx - camera.fx * optical_y / optical_x
    v = camera.cy - camera.fy * optical_z / optical_x
    return u, v


def map_landmarks_from_features(features: list[correction.LineFeature], args) -> list[MapLandmark]:
    result: list[MapLandmark] = []
    for feature in features:
        if feature.kind not in ("obstacle_edge", "free_edge"):
            continue
        for x, y in ((feature.ax, feature.ay), (feature.bx, feature.by)):
            if any(math.hypot(x - landmark.x, y - landmark.y) < args.map_landmark_merge_radius for landmark in result):
                continue
            result.append(MapLandmark(x, y, f"{feature.label}_corner", "physical_corner"))
    return result


def transform_base_to_map(pose: correction.PoseSample, x: float, y: float) -> tuple[float, float]:
    return correction.base_point_to_map(pose, x, y)


def match_map_landmarks(
    pose: correction.PoseSample,
    landmarks: list[Landmark],
    map_landmarks: list[MapLandmark],
    args,
) -> None:
    for landmark in landmarks:
        landmark.map_x, landmark.map_y = transform_base_to_map(pose, landmark.x, landmark.y)
        nearest = min(
            map_landmarks,
            key=lambda candidate: math.hypot(
                landmark.map_x - candidate.x,
                landmark.map_y - candidate.y,
            ),
            default=None,
        )
        if nearest is None:
            continue
        distance = math.hypot(landmark.map_x - nearest.x, landmark.map_y - nearest.y)
        if distance <= args.max_map_landmark_distance:
            landmark.matched_label = nearest.label
            landmark.matched_distance = distance


def landmark_color(landmark: Landmark) -> tuple[int, int, int]:
    if landmark.matched_label:
        if landmark.kind == "CROSS":
            return 0, 180, 0
        if landmark.kind == "T":
            return 255, 160, 0
        return 255, 80, 80
    return 0, 220, 255


def draw_landmark_camera_panel(
    bgr: np.ndarray,
    mask: np.ndarray,
    lines: list[GroundLine],
    landmarks: list[Landmark],
    pose: correction.PoseSample,
    stamp: float,
    frame_index: int,
    args,
) -> np.ndarray:
    panel = bgr.copy()
    red = np.zeros_like(panel)
    red[:, :, 2] = 255
    panel = np.where(
        mask[:, :, None] > 0,
        cv2.addWeighted(panel, 0.45, red, 0.55, 0),
        panel,
    )
    roi_top = int(round(mask.shape[0] * args.roi_top_fraction))
    roi_bottom = int(round(mask.shape[0] * args.roi_bottom_fraction))
    cv2.line(panel, (0, roi_top), (panel.shape[1] - 1, roi_top), (255, 255, 0), 1)
    cv2.line(panel, (0, roi_bottom), (panel.shape[1] - 1, roi_bottom), (255, 255, 0), 1)

    for line in lines:
        x1, y1, x2, y2 = line.image_line
        cv2.line(panel, (x1, y1), (x2, y2), (110, 220, 255), 1, cv2.LINE_AA)
    for landmark in landmarks:
        center = (int(round(landmark.image_u)), int(round(landmark.image_v)))
        color = landmark_color(landmark)
        cv2.circle(panel, center, 8, color, 2, cv2.LINE_AA)
        cv2.drawMarker(panel, center, color, cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA)
        text = landmark.kind
        if landmark.matched_label:
            text += f" {landmark.matched_distance:.2f}m"
        detector.put_text(panel, text, center[0] + 8, max(18, center[1] - 8), 0.38)

    matched = sum(1 for landmark in landmarks if landmark.matched_label)
    detector.put_text(panel, f"frame={frame_index} t={stamp:.3f}", 12, 24)
    detector.put_text(
        panel,
        f"lines={len(lines)} landmarks={len(landmarks)} matched={matched}",
        12,
        48,
    )
    detector.put_text(
        panel,
        f"fused x={pose.x:.3f} y={pose.y:.3f} yaw={math.degrees(pose.yaw):.1f}",
        12,
        72,
    )
    return panel


def draw_landmark_map_panel(
    pose: correction.PoseSample,
    landmarks: list[Landmark],
    features: list[correction.LineFeature],
    args,
) -> np.ndarray:
    size = args.map_panel_size
    panel = np.full((size, size, 3), 245, dtype=np.uint8)
    scale = size / args.map_width_m
    for feature in features:
        if feature.kind == "road_center":
            color, thickness = (215, 205, 195), 1
        elif feature.kind == "obstacle_edge":
            color, thickness = (0, 165, 255), 1
        else:
            color, thickness = (180, 170, 155), 1
        detector.draw_map_line(
            panel,
            feature.ax,
            feature.ay,
            feature.bx,
            feature.by,
            pose,
            scale,
            color,
            thickness,
        )

    for landmark in landmarks:
        px, py = detector.map_to_pixel(landmark.map_x, landmark.map_y, pose, scale, size)
        color = landmark_color(landmark)
        cv2.circle(panel, (px, py), 7, color, 2, cv2.LINE_AA)
        cv2.drawMarker(panel, (px, py), color, cv2.MARKER_CROSS, 12, 2, cv2.LINE_AA)
        if landmark.matched_label:
            detector.put_text(panel, landmark.kind, px + 8, py - 8, 0.38)

    cv2.circle(panel, (size // 2, size // 2), 3, (0, 0, 0), -1)
    detector.put_text(panel, f"landmarks: L/T/CROSS", 10, 22, 0.38)
    detector.put_text(panel, "green/blue=matched yellow=unmatched", 10, size - 12, 0.34)
    return panel


def write_landmark_rows(rows: list[dict], out: Path) -> None:
    if not rows:
        return
    with out.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = [
            "frame_index",
            "stamp",
            "fused_x",
            "fused_y",
            "fused_yaw",
            "landmark_index",
            "kind",
            "arms",
            "support_lines",
            "base_x",
            "base_y",
            "map_x",
            "map_y",
            "matched_label",
            "matched_distance",
            "image_u",
            "image_v",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--fused-topic", default="/odometry/fused")
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
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
    parser.add_argument("--min-component-area", type=int, default=6)
    parser.add_argument("--max-component-area-fraction", type=float, default=0.18)
    parser.add_argument("--min-component-width", type=int, default=4)
    parser.add_argument("--min-component-height", type=int, default=4)
    parser.add_argument("--morphology-size", type=int, default=3)
    parser.add_argument("--blackhat-size", type=int, default=31)
    parser.add_argument("--blackhat-threshold", type=int, default=20)
    parser.add_argument("--hough-threshold", type=int, default=14)
    parser.add_argument("--min-line-length-px", type=int, default=12)
    parser.add_argument("--max-line-gap-px", type=int, default=12)
    parser.add_argument("--min-ray-down-z", type=float, default=0.025)
    parser.add_argument("--min-ground-x", type=float, default=0.05)
    parser.add_argument("--max-ground-x", type=float, default=1.50)
    parser.add_argument("--max-abs-ground-y", type=float, default=0.80)
    parser.add_argument("--min-ground-segment-length", type=float, default=0.035)
    parser.add_argument("--max-lines", type=int, default=60)
    parser.add_argument("--corner-angle-tolerance", type=float, default=0.35)
    parser.add_argument("--landmark-line-extension", type=float, default=0.08)
    parser.add_argument("--landmark-connection-radius", type=float, default=0.07)
    parser.add_argument("--landmark-endpoint-fraction", type=float, default=0.18)
    parser.add_argument("--arm-angle-cluster-tolerance", type=float, default=0.50)
    parser.add_argument("--min-landmark-arms", type=int, default=2)
    parser.add_argument("--landmark-merge-radius", type=float, default=0.08)
    parser.add_argument("--max-landmarks", type=int, default=12)
    parser.add_argument("--map-landmark-merge-radius", type=float, default=0.035)
    parser.add_argument("--max-map-landmark-distance", type=float, default=0.25)
    parser.add_argument("--map-panel-size", type=int, default=480)
    parser.add_argument("--map-width-m", type=float, default=1.8)
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
        args.camera_info_topic,
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
    map_landmarks = map_landmarks_from_features(features, args)

    writer = None
    csv_rows: list[dict] = []
    processed = 0
    written = 0
    try:
        for timestamp_ns, data in correction.iter_topic(db_paths, args.image_topic):
            if processed % args.image_stride != 0:
                processed += 1
                continue
            processed += 1
            message = deserialize_message(data, Image)
            stamp = correction.stamp_seconds(message, timestamp_ns)
            pose = correction.interpolate_pose(fused, stamp, args.sync_tolerance)
            if pose is None:
                continue
            bgr = detector.image_to_bgr(message)
            if bgr is None:
                continue
            lines, mask = detect_ground_lines(bgr, camera, args)
            landmarks = detect_landmarks(lines, camera, args)
            match_map_landmarks(pose, landmarks, map_landmarks, args)
            camera_panel = draw_landmark_camera_panel(
                bgr,
                mask,
                lines,
                landmarks,
                pose,
                stamp,
                written,
                args,
            )
            map_panel = draw_landmark_map_panel(pose, landmarks, features, args)
            frame = np.hstack((camera_panel, map_panel))
            if writer is None:
                writer = cv2.VideoWriter(
                    str(args.out),
                    cv2.VideoWriter_fourcc(*args.codec),
                    args.fps,
                    (frame.shape[1], frame.shape[0]),
                )
                if not writer.isOpened():
                    raise RuntimeError(f"failed to open video writer for {args.out}")
            writer.write(frame)
            for landmark_index, landmark in enumerate(landmarks):
                csv_rows.append(
                    {
                        "frame_index": written,
                        "stamp": f"{stamp:.9f}",
                        "fused_x": f"{pose.x:.9f}",
                        "fused_y": f"{pose.y:.9f}",
                        "fused_yaw": f"{pose.yaw:.9f}",
                        "landmark_index": landmark_index,
                        "kind": landmark.kind,
                        "arms": landmark.arms,
                        "support_lines": landmark.support_lines,
                        "base_x": f"{landmark.x:.6f}",
                        "base_y": f"{landmark.y:.6f}",
                        "map_x": f"{landmark.map_x:.6f}",
                        "map_y": f"{landmark.map_y:.6f}",
                        "matched_label": landmark.matched_label,
                        "matched_distance": f"{landmark.matched_distance:.6f}",
                        "image_u": f"{landmark.image_u:.3f}",
                        "image_v": f"{landmark.image_v:.3f}",
                    }
                )
            written += 1
            if written % 200 == 0:
                print(f"wrote {written} frames to {args.out}")
            if args.max_frames > 0 and written >= args.max_frames:
                break
    finally:
        if writer is not None:
            writer.release()
    landmark_csv = args.out.with_name(f"{args.out.stem}_landmarks.csv")
    write_landmark_rows(csv_rows, landmark_csv)
    print(f"wrote {written} frames to {args.out}")
    print(f"wrote {len(csv_rows)} landmark rows to {landmark_csv}")


if __name__ == "__main__":
    main()
