#!/usr/bin/env python3
"""Correct fused odometry from RGB right-angle landmarks and a known map.

This tool is deliberately offline and independent from the navigation stack.
It uses only physical map edges (obstacle/free-region boundaries), never the
road center graph, so an observed off-road vehicle position is not projected
back onto a planned route.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import matplotlib
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

import offline_image_track_correction as correction
import render_image_track_detection_video as detector
import depth_image_utils as depth_utils

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


DEFAULT_BAG = Path("/media/hjh/Data/rosbag_recording/latest_navigation_bag")
DEFAULT_MAP = Path("src/arena_path_planner/config/arena_map.yaml")
DEFAULT_OUT = Path(
    "analysis_bags/20260921T171415.029504Z_9ead9ca47d9b/"
    "landmark_odom_corrected"
)


@dataclass
class CameraCalibration:
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: np.ndarray
    camera_x: float
    camera_y: float
    camera_z: float
    color_to_base: np.ndarray


@dataclass(frozen=True)
class DepthCalibration:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class DepthFrameMatcher:
    """Lazily match depth frames to RGB frames by message header time."""

    iterator: object
    previous: tuple[float, Image] | None = None
    following: tuple[float, Image] | None = None
    exhausted: bool = False

    def _read_next(self) -> tuple[float, Image] | None:
        if self.exhausted:
            return None
        try:
            timestamp_ns, data = next(self.iterator)
        except StopIteration:
            self.exhausted = True
            return None
        message = deserialize_message(data, Image)
        return correction.stamp_seconds(message, timestamp_ns), message

    def match(self, stamp: float, tolerance: float) -> Image | None:
        while self.following is None and not self.exhausted:
            self.following = self._read_next()
        while self.following is not None and self.following[0] < stamp:
            self.previous = self.following
            self.following = self._read_next()

        candidates = [
            item
            for item in (self.previous, self.following)
            if item is not None
        ]
        if not candidates:
            return None
        best_stamp, best_message = min(
            candidates,
            key=lambda item: abs(item[0] - stamp),
        )
        if abs(best_stamp - stamp) > tolerance:
            return None
        return best_message


@dataclass(frozen=True)
class GroundLine:
    image_line: tuple[int, int, int, int]
    ground: correction.GroundSegment


@dataclass
class ObservedLandmark:
    x: float
    y: float
    dirs: tuple[float, ...]
    kind: str
    arms: int
    support_lines: int
    image_u: float
    image_v: float
    matched_map_index: int = -1
    matched_score: float = float("inf")
    inlier: bool = False


@dataclass(frozen=True)
class MapLandmark:
    x: float
    y: float
    dirs: tuple[float, ...]
    kind: str
    arms: int
    labels: str


@dataclass(frozen=True)
class LandmarkMatch:
    observed_index: int
    map_index: int
    score: float
    position_distance: float
    direction_error: float
    yaw_delta: float
    proposed_x: float
    proposed_y: float
    proposed_yaw: float
    weight: float


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
    detected_lines: int
    detected_landmarks: int
    candidate_matches: int
    accepted_matches: int
    matched_labels: str
    correction_confidence: float
    temporal_residual: float
    residual_x: float
    residual_y: float
    residual_yaw: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def wrap_undirected(angle: float) -> float:
    """Wrap a line-angle difference to [-pi/2, pi/2)."""

    return (angle + 0.5 * math.pi) % math.pi - 0.5 * math.pi


def undirected_error(first: float, second: float) -> float:
    return abs(wrap_undirected(first - second))


def angle_mod_pi(angle: float) -> float:
    result = angle % math.pi
    return result if result >= 0.0 else result + math.pi


def yaw_from_quaternion(q) -> float:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 1.0e-12:
        return 0.0
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def set_yaw_quaternion(q, yaw: float) -> None:
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)


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


def quaternion_rotation_from_static(
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


def load_static_transforms(
    db_paths: list[Path],
    topic: str,
) -> dict[tuple[str, str], tuple[np.ndarray, np.ndarray]]:
    result: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}
    for timestamp_ns, data in correction.iter_topic(db_paths, topic):
        del timestamp_ns
        message = deserialize_message(data, TFMessage)
        for transform in message.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
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
            rotation = quaternion_matrix(transform.transform.rotation)
            result[(parent, child)] = (translation, rotation)
    return result


def load_camera_calibration(
    db_paths: list[Path],
    camera_info_topic: str,
    tf_topic: str,
    args,
) -> CameraCalibration:
    info = None
    for timestamp_ns, data in correction.iter_topic(db_paths, camera_info_topic):
        del timestamp_ns
        info = deserialize_message(data, CameraInfo)
        break
    if info is None:
        raise RuntimeError(f"no camera info on {camera_info_topic}")

    fx = float(info.k[0])
    fy = float(info.k[4])
    cx = float(info.k[2])
    cy = float(info.k[5])
    if info.p[0] > 0.0:
        fx = float(info.p[0])
    if info.p[5] > 0.0:
        fy = float(info.p[5])
    if info.p[2] != 0.0:
        cx = float(info.p[2])
    if info.p[6] != 0.0:
        cy = float(info.p[6])

    static = load_static_transforms(db_paths, tf_topic)
    base_to_link = quaternion_rotation_from_static(static, "base_link", "camera_link")
    link_to_color = quaternion_rotation_from_static(
        static,
        "camera_link",
        "camera_color_frame",
    )

    # camera_x/camera_y/camera_z are fallback physical coordinates. When the
    # bag contains the measured base_link -> camera_link TF, use that TF for
    # XY and retain the independently measured camera height for Z.
    camera_translation = np.array(
        [args.camera_x, args.camera_y, args.camera_z],
        dtype=np.float64,
    )
    color_to_base = np.eye(3, dtype=np.float64)
    if base_to_link is not None:
        link_translation, link_rotation = base_to_link
        camera_translation = np.array(
            [
                link_translation[0],
                link_translation[1],
                args.camera_z + link_translation[2],
            ],
            dtype=np.float64,
        )
        color_to_base = link_rotation
    if link_to_color is not None:
        color_translation, color_rotation = link_to_color
        camera_translation += color_to_base @ color_translation
        color_to_base = color_to_base @ color_rotation

    return CameraCalibration(
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        distortion=np.asarray(info.d, dtype=np.float64),
        camera_x=float(camera_translation[0]),
        camera_y=float(camera_translation[1]),
        camera_z=float(camera_translation[2]),
        color_to_base=color_to_base,
    )


def load_depth_calibration(
    db_paths: list[Path],
    depth_camera_info_topic: str,
    fallback: CameraCalibration,
) -> DepthCalibration:
    """Load depth intrinsics, falling back to RGB intrinsics for aligned depth."""

    if depth_camera_info_topic:
        for timestamp_ns, data in correction.iter_topic(
            db_paths,
            depth_camera_info_topic,
        ):
            del timestamp_ns
            info = deserialize_message(data, CameraInfo)
            fx = float(info.p[0] if info.p[0] > 0.0 else info.k[0])
            fy = float(info.p[5] if info.p[5] > 0.0 else info.k[4])
            cx = float(info.p[2] if info.p[2] != 0.0 else info.k[2])
            cy = float(info.p[6] if info.p[6] != 0.0 else info.k[5])
            if fx > 0.0 and fy > 0.0:
                return DepthCalibration(fx, fy, cx, cy)
            break
    return DepthCalibration(fallback.fx, fallback.fy, fallback.cx, fallback.cy)


def project_pixel_with_depth(
    depth_m: np.ndarray,
    calibration: CameraCalibration,
    depth_calibration: DepthCalibration,
    u: float,
    v: float,
    args,
) -> tuple[float, float] | None:
    """Project an aligned depth pixel into base coordinates.

    The D455 depth value is the optical-axis distance, not Euclidean ray
    length.  The existing camera-to-base rotation is then used to obtain the
    same ground-frame convention as the height-only projection.
    """

    depth = depth_utils.sample_valid_depth(
        depth_m,
        u,
        v,
        args.depth_sample_radius_px,
        args.min_depth_m,
        args.max_depth_m,
    )
    if depth is None:
        return None
    ray_color = np.array(
        [
            depth,
            -(u - depth_calibration.cx) / depth_calibration.fx * depth,
            -(v - depth_calibration.cy) / depth_calibration.fy * depth,
        ],
        dtype=np.float64,
    )
    point_base = np.asarray(calibration.color_to_base, dtype=np.float64) @ ray_color
    x = calibration.camera_x + point_base[0]
    y = calibration.camera_y + point_base[1]
    if not (
        math.isfinite(x)
        and math.isfinite(y)
        and args.min_ground_x <= x <= args.max_ground_x
        and abs(y) <= args.max_abs_ground_y
    ):
        return None
    return float(x), float(y)


def undistort_bgr(
    bgr: np.ndarray,
    calibration: CameraCalibration,
) -> np.ndarray:
    camera_matrix = np.array(
        [
            [calibration.fx, 0.0, calibration.cx],
            [0.0, calibration.fy, calibration.cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    if not np.any(np.abs(calibration.distortion) > 1.0e-12):
        return bgr
    return cv2.undistort(
        bgr,
        camera_matrix,
        calibration.distortion,
        None,
        camera_matrix,
    )


def project_pixel(
    calibration: CameraCalibration,
    u: float,
    v: float,
    args,
) -> tuple[float, float] | None:
    ray_color = np.array(
        [
            1.0,
            -(u - calibration.cx) / calibration.fx,
            -(v - calibration.cy) / calibration.fy,
        ],
        dtype=np.float64,
    )
    ray_base = calibration.color_to_base @ ray_color
    if ray_base[2] >= -args.min_ray_down_z:
        return None
    scale = -calibration.camera_z / ray_base[2]
    if scale <= 0.0 or not math.isfinite(scale):
        return None
    x = calibration.camera_x + scale * ray_base[0]
    y = calibration.camera_y + scale * ray_base[1]
    if not (
        args.min_ground_x <= x <= args.max_ground_x
        and abs(y) <= args.max_abs_ground_y
    ):
        return None
    return float(x), float(y)


def ground_to_pixel(
    calibration: CameraCalibration,
    x: float,
    y: float,
) -> tuple[float, float] | None:
    ray_base = np.array(
        [
            x - calibration.camera_x,
            y - calibration.camera_y,
            -calibration.camera_z,
        ],
        dtype=np.float64,
    )
    ray_color = calibration.color_to_base.T @ ray_base
    if ray_color[0] <= 1.0e-6:
        return None
    u = calibration.cx - calibration.fx * ray_color[1] / ray_color[0]
    v = calibration.cy - calibration.fy * ray_color[2] / ray_color[0]
    return float(u), float(v)


def point_segment_distance(
    x: float,
    y: float,
    segment: correction.GroundSegment,
) -> float:
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


def line_intersection(
    first: correction.GroundSegment,
    second: correction.GroundSegment,
) -> tuple[float, float] | None:
    px, py = first.ax, first.ay
    rx, ry = first.bx - first.ax, first.by - first.ay
    qx, qy = second.ax, second.ay
    sx, sy = second.bx - second.ax, second.by - second.ay
    denominator = rx * sy - ry * sx
    if abs(denominator) < 1.0e-9:
        return None
    qmpx, qmpy = qx - px, qy - py
    t = (qmpx * sy - qmpy * sx) / denominator
    return px + t * rx, py + t * ry


def line_projection(
    segment: correction.GroundSegment,
    x: float,
    y: float,
) -> float:
    length2 = max(segment.length * segment.length, 1.0e-12)
    return (
        (x - segment.ax) * (segment.bx - segment.ax)
        + (y - segment.ay) * (segment.by - segment.ay)
    ) / length2


def cluster_directed_angles(
    angles: list[float],
    tolerance: float,
) -> list[float]:
    clusters: list[list[float]] = []
    for angle in angles:
        target = None
        target_distance = float("inf")
        for index, cluster in enumerate(clusters):
            center = math.atan2(
                sum(math.sin(value) for value in cluster),
                sum(math.cos(value) for value in cluster),
            )
            distance = abs(wrap_angle(angle - center))
            if distance <= tolerance and distance < target_distance:
                target = index
                target_distance = distance
        if target is None:
            clusters.append([angle])
        else:
            clusters[target].append(angle)

    # Hough endpoints around the -pi/pi seam describe the same arm. Merge the
    # closest circular clusters repeatedly so that a physical L corner does
    # not become a synthetic T/CROSS merely because of angle wrapping.
    while len(clusters) > 1:
        centers = [
            math.atan2(
                sum(math.sin(value) for value in cluster),
                sum(math.cos(value) for value in cluster),
            )
            for cluster in clusters
        ]
        best_pair = None
        best_distance = float("inf")
        for first in range(len(centers)):
            for second in range(first + 1, len(centers)):
                distance = abs(wrap_angle(centers[first] - centers[second]))
                if distance < best_distance:
                    best_pair = (first, second)
                    best_distance = distance
        if best_pair is None or best_distance > tolerance:
            break
        first, second = best_pair
        clusters[first].extend(clusters.pop(second))

    return [
        wrap_angle(
            math.atan2(
                sum(math.sin(value) for value in cluster),
                sum(math.cos(value) for value in cluster),
            )
        )
        for cluster in clusters
    ]


def cluster_undirected_angles(
    angles: list[float],
    tolerance: float,
) -> list[float]:
    doubled = cluster_directed_angles(
        [2.0 * angle_mod_pi(angle) for angle in angles],
        2.0 * tolerance,
    )
    result = [angle_mod_pi(0.5 * value) for value in doubled]
    return sorted(result)


def classify_landmark(arms: int) -> str:
    if arms >= 4:
        return "CROSS"
    if arms == 3:
        return "T"
    if arms == 2:
        return "L"
    return "POINT"


def canonical_line_params(
    segment: correction.GroundSegment,
) -> tuple[float, float, float, float, float]:
    theta = angle_mod_pi(segment.yaw)
    direction = np.array([math.cos(theta), math.sin(theta)], dtype=np.float64)
    normal = np.array([-math.sin(theta), math.cos(theta)], dtype=np.float64)
    midpoint = np.array(
        [0.5 * (segment.ax + segment.bx), 0.5 * (segment.ay + segment.by)],
        dtype=np.float64,
    )
    rho = float(normal @ midpoint)
    first = float(direction @ np.array([segment.ax, segment.ay]))
    second = float(direction @ np.array([segment.bx, segment.by]))
    return theta, rho, min(first, second), max(first, second), segment.length


def interval_gap(
    first_min: float,
    first_max: float,
    second_min: float,
    second_max: float,
) -> float:
    return max(first_min - second_max, second_min - first_max, 0.0)


def merge_ground_lines(
    lines: list[GroundLine],
    args,
) -> list[GroundLine]:
    clusters: list[list[GroundLine]] = []
    for line in sorted(lines, key=lambda item: item.ground.length, reverse=True):
        theta, rho, line_min, line_max, _ = canonical_line_params(line.ground)
        target = None
        target_score = float("inf")
        for index, cluster in enumerate(clusters):
            params = [canonical_line_params(item.ground) for item in cluster]
            total_weight = sum(item[4] for item in params)
            center_theta = angle_mod_pi(
                0.5
                * math.atan2(
                    sum(item[4] * math.sin(2.0 * item[0]) for item in params),
                    sum(item[4] * math.cos(2.0 * item[0]) for item in params),
                )
            )
            center_rho = sum(item[1] * item[4] for item in params) / total_weight
            cluster_min = min(item[2] for item in params)
            cluster_max = max(item[3] for item in params)
            angle_error = undirected_error(theta, center_theta)
            rho_error = abs(rho - center_rho)
            gap = interval_gap(line_min, line_max, cluster_min, cluster_max)
            if (
                angle_error <= args.line_merge_angle
                and rho_error <= args.line_merge_distance
                and gap <= args.line_merge_gap
            ):
                score = angle_error + rho_error + 0.2 * gap
                if score < target_score:
                    target = index
                    target_score = score
        if target is None:
            clusters.append([line])
        else:
            clusters[target].append(line)

    result: list[GroundLine] = []
    for cluster in clusters:
        params = [canonical_line_params(item.ground) for item in cluster]
        total_weight = sum(item[4] for item in params)
        theta = angle_mod_pi(
            0.5
            * math.atan2(
                sum(item[4] * math.sin(2.0 * item[0]) for item in params),
                sum(item[4] * math.cos(2.0 * item[0]) for item in params),
            )
        )
        direction = np.array([math.cos(theta), math.sin(theta)], dtype=np.float64)
        normal = np.array([-math.sin(theta), math.cos(theta)], dtype=np.float64)
        rho = sum(item[1] * item[4] for item in params) / total_weight
        projections = []
        for item in cluster:
            projections.extend(
                [
                    float(direction @ np.array([item.ground.ax, item.ground.ay])),
                    float(direction @ np.array([item.ground.bx, item.ground.by])),
                ]
            )
        first_value = min(projections)
        second_value = max(projections)
        first = normal * rho + direction * first_value
        second = normal * rho + direction * second_value
        length = float(np.linalg.norm(second - first))
        if length < args.min_ground_segment_length:
            continue
        ground = correction.GroundSegment(
            float(first[0]),
            float(first[1]),
            float(second[0]),
            float(second[1]),
            length,
            theta,
        )
        result.append(GroundLine((0, 0, 0, 0), ground))
    result.sort(key=lambda item: item.ground.length, reverse=True)
    return result[: args.max_merged_lines]


def detect_ground_lines(
    bgr: np.ndarray,
    calibration: CameraCalibration,
    args,
    depth_m: np.ndarray | None = None,
    depth_calibration: DepthCalibration | None = None,
) -> tuple[list[GroundLine], np.ndarray]:
    mask = detector.detect_mask(bgr, args)
    raw_lines = cv2.HoughLinesP(
        mask,
        1.0,
        np.pi / 180.0,
        args.hough_threshold,
        minLineLength=args.min_line_length_px,
        maxLineGap=args.max_line_gap_px,
    )
    if raw_lines is None:
        return [], mask

    result: list[GroundLine] = []
    for raw in raw_lines[:, 0, :]:
        x1, y1, x2, y2 = map(float, raw)
        first = None
        second = None
        if depth_m is not None and depth_calibration is not None:
            first = project_pixel_with_depth(
                depth_m,
                calibration,
                depth_calibration,
                x1,
                y1,
                args,
            )
            second = project_pixel_with_depth(
                depth_m,
                calibration,
                depth_calibration,
                x2,
                y2,
                args,
            )
        if first is None:
            first = project_pixel(calibration, x1, y1, args)
        if second is None:
            second = project_pixel(calibration, x2, y2, args)
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
    result.sort(key=lambda item: item.ground.length, reverse=True)
    return merge_ground_lines(result[: args.max_raw_lines], args), mask


def connected_arm_directions(
    x: float,
    y: float,
    lines: list[GroundLine],
    args,
) -> tuple[tuple[float, ...], int]:
    angles: list[float] = []
    support_lines = 0
    for line in lines:
        segment = line.ground
        projection = line_projection(segment, x, y)
        extension_fraction = args.landmark_connection_radius / max(segment.length, 1.0e-6)
        if projection < -extension_fraction or projection > 1.0 + extension_fraction:
            continue
        if point_segment_distance(x, y, segment) > args.landmark_connection_radius:
            continue
        support_lines += 1
        if args.landmark_endpoint_fraction <= projection <= 1.0 - args.landmark_endpoint_fraction:
            angles.extend((segment.yaw, wrap_angle(segment.yaw + math.pi)))
        elif projection < 0.5:
            angles.append(math.atan2(segment.ay - y, segment.ax - x))
        else:
            angles.append(math.atan2(segment.by - y, segment.bx - x))
    return (
        tuple(
            sorted(
                cluster_directed_angles(
                    angles,
                    args.arm_angle_cluster_tolerance,
                )
            )
        ),
        support_lines,
    )


def merge_observed_landmarks(
    candidates: list[ObservedLandmark],
    args,
) -> list[ObservedLandmark]:
    merged: list[ObservedLandmark] = []
    for candidate in sorted(
        candidates,
        key=lambda item: (item.arms, item.support_lines),
        reverse=True,
    ):
        existing = next(
            (
                item
                for item in merged
                if math.hypot(item.x - candidate.x, item.y - candidate.y)
                <= args.landmark_merge_radius
            ),
            None,
        )
        if existing is None:
            merged.append(candidate)
            continue
        if (candidate.arms, candidate.support_lines) > (
            existing.arms,
            existing.support_lines,
        ):
            merged[merged.index(existing)] = candidate
    return merged[: args.max_landmarks]


def detect_observed_landmarks(
    lines: list[GroundLine],
    calibration: CameraCalibration,
    args,
) -> list[ObservedLandmark]:
    candidates: list[ObservedLandmark] = []
    for first_index, first in enumerate(lines):
        for second in lines[first_index + 1 :]:
            if abs(
                undirected_error(
                    first.ground.yaw,
                    second.ground.yaw,
                )
                - 0.5 * math.pi
            ) > args.corner_angle_tolerance:
                continue
            intersection = line_intersection(first.ground, second.ground)
            if intersection is None:
                continue
            x, y = intersection
            if not (
                args.min_ground_x <= x <= args.max_ground_x
                and abs(y) <= args.max_abs_ground_y
            ):
                continue
            if point_segment_distance(x, y, first.ground) > args.landmark_line_extension:
                continue
            if point_segment_distance(x, y, second.ground) > args.landmark_line_extension:
                continue
            dirs, support_lines = connected_arm_directions(x, y, lines, args)
            if len(dirs) < args.min_landmark_arms:
                continue
            image_point = ground_to_pixel(calibration, x, y)
            if image_point is None:
                continue
            candidates.append(
                ObservedLandmark(
                    x=x,
                    y=y,
                    dirs=dirs,
                    kind=classify_landmark(len(dirs)),
                    arms=len(dirs),
                    support_lines=support_lines,
                    image_u=image_point[0],
                    image_v=image_point[1],
                )
            )
    return merge_observed_landmarks(candidates, args)


def map_landmark_kind(arms: int) -> str:
    return classify_landmark(arms)


def build_map_landmarks(
    features: list[correction.LineFeature],
    args,
) -> list[MapLandmark]:
    endpoints: list[tuple[float, float, float, str]] = []
    for feature in features:
        if feature.kind not in ("obstacle_edge", "free_edge"):
            continue
        theta = math.atan2(feature.by - feature.ay, feature.bx - feature.ax)
        endpoints.append((feature.ax, feature.ay, theta, feature.label))
        endpoints.append(
            (
                feature.bx,
                feature.by,
                wrap_angle(theta + math.pi),
                feature.label,
            )
        )

    clusters: list[list[tuple[float, float, float, str]]] = []
    for endpoint in endpoints:
        target = next(
            (
                index
                for index, cluster in enumerate(clusters)
                if math.hypot(
                    endpoint[0] - cluster[0][0],
                    endpoint[1] - cluster[0][1],
                )
                <= args.map_landmark_merge_radius
            ),
            None,
        )
        if target is None:
            clusters.append([endpoint])
        else:
            clusters[target].append(endpoint)

    result: list[MapLandmark] = []
    for cluster in clusters:
        x = sum(item[0] for item in cluster) / len(cluster)
        y = sum(item[1] for item in cluster) / len(cluster)
        dirs = tuple(
            cluster_undirected_angles(
                [item[2] for item in cluster],
                args.map_arm_angle_tolerance,
            )
        )
        if len(dirs) < args.min_map_landmark_arms:
            continue
        labels = ",".join(sorted({item[3] for item in cluster}))
        result.append(
            MapLandmark(
                x=x,
                y=y,
                dirs=dirs,
                kind=map_landmark_kind(len(dirs)),
                arms=len(dirs),
                labels=labels,
            )
        )
    return result


def base_point_to_map(
    pose: correction.PoseSample,
    x: float,
    y: float,
) -> tuple[float, float]:
    return correction.base_point_to_map(pose, x, y)


def orientation_alignment(
    observed_dirs: tuple[float, ...],
    map_dirs: tuple[float, ...],
    current_yaw: float,
    estimate_yaw: bool,
) -> tuple[float, float]:
    if not observed_dirs or not map_dirs:
        return 0.0, float("inf")
    if not estimate_yaw:
        transformed = [current_yaw + value for value in observed_dirs]
        observed_error = sum(
            min(undirected_error(value, target) for target in map_dirs)
            for value in transformed
        ) / len(transformed)
        map_error = sum(
            min(undirected_error(value, target) for target in transformed)
            for value in map_dirs
        ) / len(map_dirs)
        return 0.0, 0.5 * (
            observed_error
            + map_error
            + 0.16 * abs(len(observed_dirs) - len(map_dirs))
        )
    candidate_deltas = [
        wrap_undirected(map_dir - (current_yaw + observed_dir))
        for observed_dir in observed_dirs
        for map_dir in map_dirs
    ]
    best_delta = 0.0
    best_error = float("inf")
    for delta in candidate_deltas:
        transformed = [
            current_yaw + observed_dir + delta
            for observed_dir in observed_dirs
        ]
        observed_error = sum(
            min(undirected_error(value, target) for target in map_dirs)
            for value in transformed
        ) / len(transformed)
        map_error = sum(
            min(undirected_error(value, target) for target in transformed)
            for value in map_dirs
        ) / len(map_dirs)
        arms_penalty = 0.08 * abs(len(observed_dirs) - len(map_dirs))
        error = 0.5 * (observed_error + map_error) + arms_penalty
        if error < best_error:
            best_error = error
            best_delta = delta
    return best_delta, best_error


def pose_from_landmark(
    current: correction.PoseSample,
    observed: ObservedLandmark,
    mapped: MapLandmark,
    yaw_delta: float,
) -> correction.PoseSample:
    yaw = wrap_angle(current.yaw + yaw_delta)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return correction.PoseSample(
        current.stamp,
        mapped.x - c * observed.x + s * observed.y,
        mapped.y - s * observed.x - c * observed.y,
        yaw,
    )


def make_landmark_matches(
    current: correction.PoseSample,
    observed: list[ObservedLandmark],
    mapped: list[MapLandmark],
    args,
) -> list[LandmarkMatch]:
    result: list[LandmarkMatch] = []
    for observed_index, landmark in enumerate(observed):
        observed_map = base_point_to_map(current, landmark.x, landmark.y)
        for map_index, candidate in enumerate(mapped):
            position_distance = math.hypot(
                observed_map[0] - candidate.x,
                observed_map[1] - candidate.y,
            )
            if position_distance > args.max_landmark_match_distance:
                continue
            yaw_delta, direction_error = orientation_alignment(
                landmark.dirs,
                candidate.dirs,
                current.yaw,
                args.estimate_yaw_from_landmarks,
            )
            if direction_error > args.max_direction_error:
                continue
            if landmark.arms > args.max_observed_arms:
                continue
            arms_penalty = args.arms_penalty * abs(
                landmark.arms - candidate.arms
            )
            kind_penalty = args.kind_penalty if landmark.kind != candidate.kind else 0.0
            score = (
                position_distance
                + args.direction_weight * direction_error
                + arms_penalty
                + kind_penalty
            )
            proposal = pose_from_landmark(
                current,
                landmark,
                candidate,
                yaw_delta,
            )
            # A physical map corner has two arms. Extra arms are usually
            # duplicate Hough detections, so they reduce confidence instead
            # of increasing it.
            shape_weight = math.exp(
                -args.extra_arm_weight * abs(landmark.arms - candidate.arms)
            )
            support_weight = 1.0 / (
                1.0
                + args.extra_support_penalty
                * max(0, landmark.support_lines - 2)
            )
            score_weight = math.exp(-score / args.match_score_scale)
            weight = shape_weight * support_weight * score_weight
            result.append(
                LandmarkMatch(
                    observed_index=observed_index,
                    map_index=map_index,
                    score=score,
                    position_distance=position_distance,
                    direction_error=direction_error,
                    yaw_delta=yaw_delta,
                    proposed_x=proposal.x,
                    proposed_y=proposal.y,
                    proposed_yaw=proposal.yaw,
                    weight=weight,
                )
            )
    return sorted(result, key=lambda item: item.score)


def pose_difference(
    first: LandmarkMatch,
    second: LandmarkMatch,
) -> tuple[float, float]:
    return (
        math.hypot(first.proposed_x - second.proposed_x, first.proposed_y - second.proposed_y),
        abs(wrap_angle(first.proposed_yaw - second.proposed_yaw)),
    )


def pose_distance(
    first: correction.PoseSample,
    second: correction.PoseSample,
) -> tuple[float, float]:
    return (
        math.hypot(first.x - second.x, first.y - second.y),
        abs(wrap_angle(first.yaw - second.yaw)),
    )


def propagate_measurement(
    target: correction.PoseSample,
    source_fused: correction.PoseSample,
    current_fused: correction.PoseSample,
) -> correction.PoseSample:
    """Move the last landmark pose with the short-term fused increment."""

    return correction.PoseSample(
        current_fused.stamp,
        target.x + current_fused.x - source_fused.x,
        target.y + current_fused.y - source_fused.y,
        wrap_angle(
            target.yaw
            + wrap_angle(current_fused.yaw - source_fused.yaw)
        ),
    )


def temporal_match_gate(
    previous_stamp: float,
    current_stamp: float,
    args,
) -> float:
    elapsed = max(0.0, current_stamp - previous_stamp)
    return min(
        args.max_temporal_position_gate,
        args.temporal_position_gate
        + args.temporal_gate_growth_per_second * elapsed,
    )


def filter_temporal_matches(
    matches: list[LandmarkMatch],
    expected: correction.PoseSample | None,
    previous_stamp: float | None,
    current_stamp: float,
    args,
) -> tuple[list[LandmarkMatch], float]:
    if expected is None or previous_stamp is None:
        return matches, 0.0
    gate = temporal_match_gate(previous_stamp, current_stamp, args)
    filtered = []
    smallest = float("inf")
    for match in matches:
        candidate = correction.PoseSample(
            current_stamp,
            match.proposed_x,
            match.proposed_y,
            match.proposed_yaw,
        )
        distance, yaw_error = pose_distance(candidate, expected)
        smallest = min(smallest, distance)
        if (
            distance <= gate
            and yaw_error <= args.temporal_yaw_gate
        ):
            filtered.append(match)
    return filtered, (smallest if math.isfinite(smallest) else 0.0)


def select_consensus_matches(
    matches: list[LandmarkMatch],
    args,
) -> tuple[list[LandmarkMatch], float]:
    if not matches:
        return [], 0.0
    best: list[LandmarkMatch] = []
    best_score = -1.0
    best_seed_score = float("inf")
    for seed in matches[: args.max_ransac_seeds]:
        selected: list[LandmarkMatch] = []
        used_observed: set[int] = set()
        used_map: set[int] = set()
        for candidate in matches:
            if candidate.observed_index in used_observed:
                continue
            if candidate.map_index in used_map:
                continue
            distance, yaw_error = pose_difference(seed, candidate)
            if (
                distance <= args.consensus_position_tolerance
                and yaw_error <= args.consensus_yaw_tolerance
            ):
                selected.append(candidate)
                used_observed.add(candidate.observed_index)
                used_map.add(candidate.map_index)
        score = sum(item.weight for item in selected)
        if score > best_score or (
            math.isclose(score, best_score) and seed.score < best_seed_score
        ):
            best = selected
            best_score = score
            best_seed_score = seed.score
    if len(best) < args.min_accepted_matches:
        return [], 0.0
    return best, best_score


def average_consensus_pose(
    current: correction.PoseSample,
    matches: list[LandmarkMatch],
) -> tuple[float, float, float]:
    weights = [item.weight for item in matches]
    total = sum(weights)
    x = sum(item.proposed_x * weight for item, weight in zip(matches, weights)) / total
    y = sum(item.proposed_y * weight for item, weight in zip(matches, weights)) / total
    yaw = math.atan2(
        sum(math.sin(item.proposed_yaw) * weight for item, weight in zip(matches, weights)),
        sum(math.cos(item.proposed_yaw) * weight for item, weight in zip(matches, weights)),
    )
    del current
    return x, y, yaw


def clamp_vector(x: float, y: float, maximum: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= maximum or norm <= 1.0e-12:
        return x, y
    scale = maximum / norm
    return x * scale, y * scale


def symmetric_lowpass(
    values: list[float],
    stamps: list[float],
    tau: float,
) -> list[float]:
    """Apply a short, zero-phase low-pass filter to an offline signal."""

    if len(values) < 3 or tau <= 0.0:
        return list(values)

    forward = [float(values[0])]
    for index in range(1, len(values)):
        dt = max(0.0, stamps[index] - stamps[index - 1])
        alpha = 1.0 if dt <= 0.0 else 1.0 - math.exp(-dt / tau)
        forward.append(forward[-1] + alpha * (values[index] - forward[-1]))

    backward = [0.0] * len(values)
    backward[-1] = float(values[-1])
    for index in range(len(values) - 2, -1, -1):
        dt = max(0.0, stamps[index + 1] - stamps[index])
        alpha = 1.0 if dt <= 0.0 else 1.0 - math.exp(-dt / tau)
        backward[index] = (
            backward[index + 1]
            + alpha * (values[index] - backward[index + 1])
        )

    result = [
        0.5 * (before + after)
        for before, after in zip(forward, backward)
    ]
    result[0] = float(values[0])
    result[-1] = float(values[-1])
    return result


def smooth_pose_samples(
    samples: list[correction.PoseSample],
    args,
) -> list[correction.PoseSample]:
    """Remove short visual-odometry jumps without route projection."""

    if len(samples) < 3 or args.pose_smoothing_tau <= 0.0:
        return samples

    stamps = [sample.stamp for sample in samples]
    x_values = symmetric_lowpass(
        [sample.x for sample in samples],
        stamps,
        args.pose_smoothing_tau,
    )
    y_values = symmetric_lowpass(
        [sample.y for sample in samples],
        stamps,
        args.pose_smoothing_tau,
    )
    yaw_values = np.unwrap(
        np.asarray([sample.yaw for sample in samples], dtype=np.float64)
    )
    yaw_smooth = symmetric_lowpass(
        yaw_values.tolist(),
        stamps,
        args.pose_smoothing_yaw_tau,
    )
    return [
        correction.PoseSample(
            sample.stamp,
            x,
            y,
            wrap_angle(yaw),
        )
        for sample, x, y, yaw in zip(
            samples,
            x_values,
            y_values,
            yaw_smooth,
        )
    ]


def advance_offset_filter(
    offset_x: float,
    offset_y: float,
    offset_yaw: float,
    velocity_x: float,
    velocity_y: float,
    velocity_yaw: float,
    target_x: float,
    target_y: float,
    target_yaw: float,
    dt: float,
    args,
) -> tuple[float, float, float, float, float, float]:
    """Move the map correction toward its target with bounded dynamics."""

    if dt <= 0.0:
        return (
            offset_x,
            offset_y,
            offset_yaw,
            velocity_x,
            velocity_y,
            velocity_yaw,
        )

    omega = 1.0 / max(args.correction_tau, 1.0e-3)
    acceleration_x = (
        omega * omega * (target_x - offset_x)
        - 2.0 * args.correction_damping * omega * velocity_x
    )
    acceleration_y = (
        omega * omega * (target_y - offset_y)
        - 2.0 * args.correction_damping * omega * velocity_y
    )
    acceleration_x, acceleration_y = clamp_vector(
        acceleration_x,
        acceleration_y,
        args.max_correction_acceleration,
    )
    velocity_x += acceleration_x * dt
    velocity_y += acceleration_y * dt
    velocity_x, velocity_y = clamp_vector(
        velocity_x,
        velocity_y,
        args.max_correction_speed,
    )
    offset_x += velocity_x * dt
    offset_y += velocity_y * dt

    acceleration_yaw = (
        omega * omega * wrap_angle(target_yaw - offset_yaw)
        - 2.0 * args.correction_damping * omega * velocity_yaw
    )
    acceleration_yaw = max(
        -args.max_correction_yaw_acceleration,
        min(args.max_correction_yaw_acceleration, acceleration_yaw),
    )
    velocity_yaw += acceleration_yaw * dt
    velocity_yaw = max(
        -args.max_correction_yaw_speed,
        min(args.max_correction_yaw_speed, velocity_yaw),
    )
    offset_yaw = wrap_angle(offset_yaw + velocity_yaw * dt)
    return (
        offset_x,
        offset_y,
        offset_yaw,
        velocity_x,
        velocity_y,
        velocity_yaw,
    )


def interpolate_offset(
    rows: list[CorrectionRow],
    stamp: float,
) -> tuple[float, float, float]:
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
        before.offset_x + ratio * (after.offset_x - before.offset_x),
        before.offset_y + ratio * (after.offset_y - before.offset_y),
        wrap_angle(
            before.offset_yaw
            + ratio * wrap_angle(after.offset_yaw - before.offset_yaw)
        ),
    )


def image_to_bgr(message: Image) -> np.ndarray | None:
    return detector.image_to_bgr(message)


def draw_line_on_camera(
    panel: np.ndarray,
    line: GroundLine,
    calibration: CameraCalibration,
    color: tuple[int, int, int],
    thickness: int,
) -> None:
    first = ground_to_pixel(
        calibration,
        line.ground.ax,
        line.ground.ay,
    )
    second = ground_to_pixel(
        calibration,
        line.ground.bx,
        line.ground.by,
    )
    if first is None or second is None:
        return
    cv2.line(
        panel,
        (int(round(first[0])), int(round(first[1]))),
        (int(round(second[0])), int(round(second[1]))),
        color,
        thickness,
        cv2.LINE_AA,
    )


def map_to_pixel(
    x: float,
    y: float,
    center: correction.PoseSample,
    scale: float,
    size: int,
) -> tuple[int, int]:
    return (
        int(round(size * 0.5 + (x - center.x) * scale)),
        int(round(size * 0.5 - (y - center.y) * scale)),
    )


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
    cv2.line(
        panel,
        map_to_pixel(ax, ay, center, scale, panel.shape[0]),
        map_to_pixel(bx, by, center, scale, panel.shape[0]),
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_vehicle(
    panel: np.ndarray,
    pose: correction.PoseSample,
    scale: float,
    color: tuple[int, int, int],
    size: int,
) -> None:
    points = np.array(
        [
            map_to_pixel(
                pose.x + 0.11 * math.cos(pose.yaw),
                pose.y + 0.11 * math.sin(pose.yaw),
                pose,
                scale,
                size,
            ),
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
    cv2.fillConvexPoly(panel, points, color, cv2.LINE_AA)


def render_frame(
    bgr: np.ndarray,
    mask: np.ndarray,
    lines: list[GroundLine],
    observed: list[ObservedLandmark],
    map_landmarks: list[MapLandmark],
    features: list[correction.LineFeature],
    fused: correction.PoseSample,
    corrected: correction.PoseSample,
    calibration: CameraCalibration,
    stamp: float,
    frame_index: int,
    accepted_matches: list[LandmarkMatch],
    args,
) -> np.ndarray:
    camera_panel = bgr.copy()
    red = np.zeros_like(camera_panel)
    red[:, :, 2] = 255
    camera_panel = np.where(
        mask[:, :, None] > 0,
        cv2.addWeighted(camera_panel, 0.45, red, 0.55, 0),
        camera_panel,
    )
    roi_top = int(round(mask.shape[0] * args.roi_top_fraction))
    roi_bottom = int(round(mask.shape[0] * args.roi_bottom_fraction))
    cv2.line(
        camera_panel,
        (0, roi_top),
        (camera_panel.shape[1] - 1, roi_top),
        (255, 255, 0),
        1,
    )
    cv2.line(
        camera_panel,
        (0, roi_bottom),
        (camera_panel.shape[1] - 1, roi_bottom),
        (255, 255, 0),
        1,
    )

    accepted_by_observed = {
        item.observed_index for item in accepted_matches
    }
    for line in lines:
        draw_line_on_camera(
            camera_panel,
            line,
            calibration,
            (110, 220, 255),
            2,
        )
    for index, landmark in enumerate(observed):
        center = (int(round(landmark.image_u)), int(round(landmark.image_v)))
        matched = index in accepted_by_observed
        color = (0, 190, 0) if matched else (0, 220, 255)
        cv2.circle(camera_panel, center, 8, color, 2, cv2.LINE_AA)
        cv2.drawMarker(
            camera_panel,
            center,
            color,
            cv2.MARKER_CROSS,
            14,
            2,
            cv2.LINE_AA,
        )
        detector.put_text(
            camera_panel,
            landmark.kind,
            center[0] + 8,
            max(18, center[1] - 8),
            0.38,
        )

    offset_x = corrected.x - fused.x
    offset_y = corrected.y - fused.y
    offset_yaw = wrap_angle(corrected.yaw - fused.yaw)
    detector.put_text(
        camera_panel,
        f"frame={frame_index} t={stamp:.3f}",
        12,
        24,
    )
    detector.put_text(
        camera_panel,
        f"lines={len(lines)} landmarks={len(observed)} "
        f"accepted={len(accepted_matches)}",
        12,
        48,
    )
    detector.put_text(
        camera_panel,
        f"fused x={fused.x:.3f} y={fused.y:.3f} "
        f"yaw={math.degrees(fused.yaw):.1f}",
        12,
        72,
    )
    detector.put_text(
        camera_panel,
        f"corrected x={corrected.x:.3f} y={corrected.y:.3f} "
        f"yaw={math.degrees(corrected.yaw):.1f}",
        12,
        96,
    )
    detector.put_text(
        camera_panel,
        f"offset dx={offset_x:.3f} dy={offset_y:.3f} "
        f"dyaw={math.degrees(offset_yaw):.1f}",
        12,
        120,
    )

    map_size = args.map_panel_size
    map_panel = np.full((map_size, map_size, 3), 245, dtype=np.uint8)
    scale = map_size / args.map_width_m
    for feature in features:
        if feature.kind == "road_center":
            color, thickness = (215, 205, 195), 1
        elif feature.kind == "obstacle_edge":
            color, thickness = (0, 165, 255), 1
        else:
            color, thickness = (180, 170, 155), 1
        draw_map_line(
            map_panel,
            feature.ax,
            feature.ay,
            feature.bx,
            feature.by,
            corrected,
            scale,
            color,
            thickness,
        )
    for landmark in map_landmarks:
        px, py = map_to_pixel(
            landmark.x,
            landmark.y,
            corrected,
            scale,
            map_size,
        )
        cv2.circle(map_panel, (px, py), 2, (165, 165, 165), -1)
    for index, landmark in enumerate(observed):
        observed_map = base_point_to_map(corrected, landmark.x, landmark.y)
        matched = index in accepted_by_observed
        color = (0, 170, 0) if matched else (0, 210, 255)
        px, py = map_to_pixel(
            observed_map[0],
            observed_map[1],
            corrected,
            scale,
            map_size,
        )
        cv2.circle(map_panel, (px, py), 7, color, 2, cv2.LINE_AA)
        if matched:
            match = next(
                item for item in accepted_matches if item.observed_index == index
            )
            target = map_landmarks[match.map_index]
            tx, ty = map_to_pixel(
                target.x,
                target.y,
                corrected,
                scale,
                map_size,
            )
            cv2.circle(map_panel, (tx, ty), 9, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.line(map_panel, (px, py), (tx, ty), color, 1, cv2.LINE_AA)
    draw_vehicle(map_panel, fused, scale, (0, 0, 220), map_size)
    draw_vehicle(map_panel, corrected, scale, (220, 80, 20), map_size)
    cv2.circle(map_panel, (map_size // 2, map_size // 2), 3, (0, 0, 0), -1)
    detector.put_text(map_panel, "red=fused blue=corrected", 10, 22, 0.38)
    detector.put_text(
        map_panel,
        "green=landmark consensus yellow=unmatched",
        10,
        map_size - 12,
        0.32,
    )
    if map_panel.shape[0] != camera_panel.shape[0]:
        map_panel = cv2.resize(
            map_panel,
            (camera_panel.shape[0], camera_panel.shape[0]),
        )
    return np.hstack((camera_panel, map_panel))


def write_landmark_csv(
    output: Path,
    frame_rows: list[dict],
) -> None:
    if not frame_rows:
        return
    fieldnames = list(frame_rows[0].keys())
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(frame_rows)


def write_odom_bag(
    db_paths: list[Path],
    rows: list[CorrectionRow],
    fused: list[correction.PoseSample],
    args,
) -> None:
    if args.output_bag is None:
        return
    import rosbag2_py

    output_bag = args.output_bag
    if output_bag.exists():
        if not args.overwrite_output_bag:
            raise FileExistsError(
                f"{output_bag} exists; pass --overwrite-output-bag"
            )
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
    records = []
    for source_timestamp_ns, data in correction.iter_topic(
        db_paths,
        args.fused_topic,
    ):
        message = deserialize_message(data, Odometry)
        stamp = correction.stamp_seconds(message, source_timestamp_ns)
        baseline = correction.interpolate_pose(
            fused,
            stamp,
            args.sync_tolerance,
        )
        if baseline is None:
            pose = message.pose.pose
            baseline = correction.PoseSample(
                stamp,
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
            )
        offset_x, offset_y, offset_yaw = interpolate_offset(rows, stamp)
        corrected = correction.PoseSample(
            stamp,
            baseline.x + offset_x,
            baseline.y + offset_y,
            wrap_angle(baseline.yaw + offset_yaw),
        )
        message.pose.pose.position.x = corrected.x
        message.pose.pose.position.y = corrected.y
        set_yaw_quaternion(
            message.pose.pose.orientation,
            corrected.yaw,
        )
        # The storage timestamp can be much later than the odometry header
        # timestamp when a publisher delivers a delayed message.  Use the
        # measurement time for the corrected trajectory; otherwise two
        # physically adjacent poses can be written only microseconds apart.
        output_timestamp_ns = int(round(stamp * 1.0e9))
        records.append(
            (
                output_timestamp_ns,
                message,
                corrected,
                int(source_timestamp_ns),
            )
        )
    records.sort(key=lambda item: (item[0], item[3]))
    normalized_records = []
    last_timestamp_ns: int | None = None
    for timestamp_ns, message, pose, source_timestamp_ns in records:
        if (
            last_timestamp_ns is not None
            and timestamp_ns <= last_timestamp_ns
        ):
            timestamp_ns = last_timestamp_ns + 1
        normalized_records.append(
            (timestamp_ns, message, pose, source_timestamp_ns)
        )
        last_timestamp_ns = timestamp_ns
    records = normalized_records

    def valid_neighbor(index: int, direction: int) -> int | None:
        cursor = index + direction
        current_time = records[index][0] * 1.0e-9
        while 0 <= cursor < len(records):
            dt = abs(records[cursor][0] * 1.0e-9 - current_time)
            if dt >= 0.001:
                return cursor
            cursor += direction
        return None

    for index, (timestamp_ns, message, pose, _) in enumerate(records):
        before_index = valid_neighbor(index, -1)
        after_index = valid_neighbor(index, 1)
        if before_index is not None and after_index is not None:
            before = records[before_index][2]
            after = records[after_index][2]
            dt = records[after_index][0] * 1.0e-9 - records[before_index][0] * 1.0e-9
            if dt > 0.0:
                world_vx = (after.x - before.x) / dt
                world_vy = (after.y - before.y) / dt
                angular_z = wrap_angle(after.yaw - before.yaw) / dt
            else:
                world_vx = world_vy = angular_z = 0.0
        elif before_index is not None:
            before = records[before_index][2]
            dt = timestamp_ns * 1.0e-9 - records[before_index][0] * 1.0e-9
            if dt > 0.0:
                world_vx = (pose.x - before.x) / dt
                world_vy = (pose.y - before.y) / dt
                angular_z = wrap_angle(pose.yaw - before.yaw) / dt
            else:
                world_vx = world_vy = angular_z = 0.0
        elif after_index is not None:
            after = records[after_index][2]
            dt = records[after_index][0] * 1.0e-9 - timestamp_ns * 1.0e-9
            if dt > 0.0:
                world_vx = (after.x - pose.x) / dt
                world_vy = (after.y - pose.y) / dt
                angular_z = wrap_angle(after.yaw - pose.yaw) / dt
            else:
                world_vx = world_vy = angular_z = 0.0
        else:
            world_vx = world_vy = angular_z = 0.0
        if before_index is not None or after_index is not None:
            c = math.cos(pose.yaw)
            s = math.sin(pose.yaw)
            message.twist.twist.linear.x = c * world_vx + s * world_vy
            message.twist.twist.linear.y = -s * world_vx + c * world_vy
            message.twist.twist.angular.z = angular_z
        writer.write(
            args.output_odom_topic,
            serialize_message(message),
            int(timestamp_ns),
        )
    written = len(records)
    print(
        f"wrote {written} corrected odometry messages to "
        f"{output_bag} ({args.output_odom_topic})"
    )


def plot_outputs(
    output: Path,
    rows: list[CorrectionRow],
    features: list[correction.LineFeature],
) -> None:
    if not rows:
        return
    figure, axis = plt.subplots(figsize=(8, 8))
    for feature in features:
        if feature.kind == "road_center":
            color, width = "#cbd5e1", 1.0
        elif feature.kind == "obstacle_edge":
            color, width = "#f59e0b", 1.0
        else:
            color, width = "#94a3b8", 0.8
        axis.plot(
            [feature.ax, feature.bx],
            [feature.ay, feature.by],
            color=color,
            linewidth=width,
            alpha=0.8,
        )
    axis.plot(
        [row.fused_x for row in rows],
        [row.fused_y for row in rows],
        color="#dc2626",
        linewidth=1.2,
        label="fused",
    )
    axis.plot(
        [row.corrected_x for row in rows],
        [row.corrected_y for row in rows],
        color="#2563eb",
        linewidth=1.2,
        label="landmark corrected",
    )
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, linewidth=0.3)
    axis.legend()
    axis.set_title("Landmark corrected odometry; no route snap")
    figure.tight_layout()
    figure.savefig(output / "landmark_corrected_trajectory.png", dpi=180)
    plt.close(figure)

    t0 = rows[0].stamp
    times = [row.stamp - t0 for row in rows]
    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(times, [row.offset_x for row in rows], label="offset x")
    axes[0].plot(times, [row.offset_y for row in rows], label="offset y")
    axes[0].set_ylabel("offset [m]")
    axes[0].legend()
    axes[0].grid(True, linewidth=0.3)
    axes[1].plot(
        times,
        [math.degrees(row.offset_yaw) for row in rows],
        label="offset yaw",
    )
    axes[1].set_ylabel("yaw offset [deg]")
    axes[1].grid(True, linewidth=0.3)
    axes[2].plot(
        times,
        [row.accepted_matches for row in rows],
        label="accepted landmarks",
        color="#16a34a",
    )
    axes[2].set_ylabel("matches")
    axes[2].set_xlabel("time [s]")
    axes[2].grid(True, linewidth=0.3)
    figure.tight_layout()
    figure.savefig(output / "landmark_correction_timeseries.png", dpi=180)
    plt.close(figure)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--cache-dir", type=Path)
    parser.add_argument("--fused-topic", default="/odometry/fused")
    parser.add_argument(
        "--image-topic",
        default="/camera/camera/color/image_raw",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/camera/camera/color/camera_info",
    )
    parser.add_argument(
        "--depth-image-topic",
        default="",
        help=(
            "Optional aligned depth image topic, for example "
            "/camera/camera/aligned_depth_to_color/image_raw. "
            "Empty disables depth processing."
        ),
    )
    parser.add_argument(
        "--depth-camera-info-topic",
        default="",
        help="Optional CameraInfo topic for the depth image.",
    )
    parser.add_argument(
        "--depth-scale",
        type=float,
        default=0.001,
        help="Scale for 16UC1 depth values to metres; 32FC1 is already metres.",
    )
    parser.add_argument("--depth-sync-tolerance", type=float)
    parser.add_argument("--depth-sample-radius-px", type=int, default=3)
    parser.add_argument("--min-depth-m", type=float, default=0.05)
    parser.add_argument("--max-depth-m", type=float, default=5.0)
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--sync-tolerance", type=float, default=0.12)
    parser.add_argument("--image-stride", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--codec", default="VP80")
    parser.add_argument("--camera-x", type=float, default=0.055)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=0.121)
    parser.add_argument("--roi-top-fraction", type=float, default=0.58)
    parser.add_argument("--roi-bottom-fraction", type=float, default=0.96)
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
    parser.add_argument("--max-ground-x", type=float, default=1.60)
    parser.add_argument("--max-abs-ground-y", type=float, default=0.90)
    parser.add_argument("--min-ground-segment-length", type=float, default=0.035)
    parser.add_argument("--max-raw-lines", type=int, default=100)
    parser.add_argument("--max-merged-lines", type=int, default=45)
    parser.add_argument("--line-merge-angle", type=float, default=0.14)
    parser.add_argument("--line-merge-distance", type=float, default=0.045)
    parser.add_argument("--line-merge-gap", type=float, default=0.22)
    parser.add_argument("--corner-angle-tolerance", type=float, default=0.30)
    parser.add_argument("--landmark-line-extension", type=float, default=0.075)
    parser.add_argument("--landmark-connection-radius", type=float, default=0.055)
    parser.add_argument("--landmark-endpoint-fraction", type=float, default=0.18)
    parser.add_argument("--arm-angle-cluster-tolerance", type=float, default=0.30)
    parser.add_argument("--min-landmark-arms", type=int, default=2)
    parser.add_argument("--landmark-merge-radius", type=float, default=0.065)
    parser.add_argument("--max-landmarks", type=int, default=10)
    parser.add_argument("--map-landmark-merge-radius", type=float, default=0.035)
    parser.add_argument("--map-arm-angle-tolerance", type=float, default=0.20)
    parser.add_argument("--min-map-landmark-arms", type=int, default=2)
    parser.add_argument("--max-landmark-match-distance", type=float, default=0.30)
    parser.add_argument("--max-direction-error", type=float, default=0.40)
    parser.add_argument("--direction-weight", type=float, default=0.22)
    parser.add_argument("--max-observed-arms", type=int, default=4)
    parser.add_argument("--extra-arm-weight", type=float, default=1.25)
    parser.add_argument("--extra-support-penalty", type=float, default=0.22)
    parser.add_argument("--match-score-scale", type=float, default=0.12)
    parser.add_argument(
        "--estimate-yaw-from-landmarks",
        action="store_true",
        help="Allow landmark line directions to change yaw; off by default.",
    )
    parser.add_argument("--arms-penalty", type=float, default=0.035)
    parser.add_argument("--kind-penalty", type=float, default=0.025)
    parser.add_argument("--max-ransac-seeds", type=int, default=80)
    parser.add_argument("--consensus-position-tolerance", type=float, default=0.16)
    parser.add_argument("--consensus-yaw-tolerance", type=float, default=0.22)
    parser.add_argument("--min-accepted-matches", type=int, default=1)
    parser.add_argument(
        "--min-single-match-confidence",
        type=float,
        default=0.08,
        help="Reject a one-landmark update when its consensus weight is below this value.",
    )
    parser.add_argument("--temporal-position-gate", type=float, default=0.20)
    parser.add_argument(
        "--max-temporal-position-gate",
        type=float,
        default=0.45,
    )
    parser.add_argument(
        "--temporal-gate-growth-per-second",
        type=float,
        default=0.04,
    )
    parser.add_argument("--temporal-yaw-gate", type=float, default=0.25)
    parser.add_argument("--correction-tau", type=float, default=0.70)
    parser.add_argument("--correction-damping", type=float, default=1.0)
    parser.add_argument("--max-correction-speed", type=float, default=0.20)
    parser.add_argument(
        "--max-correction-acceleration",
        type=float,
        default=0.60,
    )
    parser.add_argument(
        "--max-correction-yaw-speed",
        type=float,
        default=0.60,
    )
    parser.add_argument(
        "--max-correction-yaw-acceleration",
        type=float,
        default=1.50,
    )
    parser.add_argument("--translation-gain", type=float, default=0.65)
    parser.add_argument("--yaw-gain", type=float, default=0.55)
    parser.add_argument("--max-step-m", type=float, default=0.035)
    parser.add_argument("--max-step-yaw", type=float, default=0.10)
    parser.add_argument("--max-total-offset-m", type=float, default=0.80)
    parser.add_argument("--max-total-yaw", type=float, default=0.75)
    parser.add_argument("--pose-smoothing-tau", type=float, default=0.12)
    parser.add_argument("--pose-smoothing-yaw-tau", type=float, default=0.05)
    parser.add_argument("--map-panel-size", type=int, default=480)
    parser.add_argument("--map-width-m", type=float, default=2.20)
    parser.add_argument(
        "--output-bag",
        type=Path,
        help="Optional sqlite3 rosbag containing the corrected odom topic",
    )
    parser.add_argument(
        "--output-odom-topic",
        default="/odometry/landmark_corrected",
    )
    parser.add_argument("--overwrite-output-bag", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    cache_dir = args.cache_dir or (args.out / "decompressed")
    db_paths = correction.decompress_bag_files(args.bag, cache_dir)
    raw_fused = correction.load_odometry(db_paths, args.fused_topic)
    if not raw_fused:
        raise RuntimeError(f"no fused odometry on {args.fused_topic}")
    fused = smooth_pose_samples(raw_fused, args)
    calibration = load_camera_calibration(
        db_paths,
        args.camera_info_topic,
        args.tf_static_topic,
        args,
    )
    depth_calibration = None
    depth_matcher = None
    depth_tolerance = (
        args.sync_tolerance
        if args.depth_sync_tolerance is None
        else args.depth_sync_tolerance
    )
    if args.depth_image_topic:
        depth_calibration = load_depth_calibration(
            db_paths,
            args.depth_camera_info_topic,
            calibration,
        )
        depth_matcher = DepthFrameMatcher(
            correction.iter_topic(db_paths, args.depth_image_topic)
        )
        print(
            "optional depth enabled "
            f"topic={args.depth_image_topic}, "
            f"tolerance={depth_tolerance:.3f}s"
        )
    features = correction.load_map_features(args.map)
    map_landmarks = build_map_landmarks(features, args)
    print(
        "camera base position "
        f"({calibration.camera_x:.4f}, {calibration.camera_y:.4f}, "
        f"{calibration.camera_z:.4f}), "
        f"map landmarks={len(map_landmarks)}"
    )

    writer = None
    rows: list[CorrectionRow] = []
    landmark_rows: list[dict] = []
    offset_x = 0.0
    offset_y = 0.0
    offset_yaw = 0.0
    velocity_x = 0.0
    velocity_y = 0.0
    velocity_yaw = 0.0
    target_offset_x = 0.0
    target_offset_y = 0.0
    target_offset_yaw = 0.0
    last_filter_stamp: float | None = None
    last_measurement_stamp: float | None = None
    last_measurement_fused: correction.PoseSample | None = None
    last_measurement_target: correction.PoseSample | None = None
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
            fused_pose = correction.interpolate_pose(
                fused,
                stamp,
                args.sync_tolerance,
            )
            if fused_pose is None:
                continue
            bgr = image_to_bgr(message)
            if bgr is None:
                continue
            bgr = undistort_bgr(bgr, calibration)
            depth_m = None
            if depth_matcher is not None:
                depth_message = depth_matcher.match(stamp, depth_tolerance)
                if depth_message is not None:
                    try:
                        depth_m = depth_utils.decode_depth_image(
                            depth_message,
                            args.depth_scale,
                        )
                    except (TypeError, ValueError):
                        depth_m = None
            lines, mask = detect_ground_lines(
                bgr,
                calibration,
                args,
                depth_m,
                depth_calibration,
            )
            observed = detect_observed_landmarks(lines, calibration, args)
            current = correction.PoseSample(
                stamp,
                fused_pose.x + offset_x,
                fused_pose.y + offset_y,
                wrap_angle(fused_pose.yaw + offset_yaw),
            )
            matches = make_landmark_matches(
                current,
                observed,
                map_landmarks,
                args,
            )
            expected_target = None
            if (
                last_measurement_stamp is not None
                and last_measurement_fused is not None
                and last_measurement_target is not None
            ):
                expected_target = propagate_measurement(
                    last_measurement_target,
                    last_measurement_fused,
                    fused_pose,
                )
            temporal_matches, temporal_residual = filter_temporal_matches(
                matches,
                expected_target,
                last_measurement_stamp,
                stamp,
                args,
            )
            accepted, confidence = select_consensus_matches(
                temporal_matches,
                args,
            )
            if (
                len(accepted) == 1
                and confidence < args.min_single_match_confidence
            ):
                accepted = []
                confidence = 0.0
            residual_x = residual_y = residual_yaw = 0.0
            labels: list[str] = []
            if accepted:
                target_x, target_y, target_yaw = average_consensus_pose(
                    current,
                    accepted,
                )
                desired_offset_x = target_x - fused_pose.x
                desired_offset_y = target_y - fused_pose.y
                desired_offset_yaw = wrap_angle(target_yaw - fused_pose.yaw)
                residual_x = desired_offset_x - offset_x
                residual_y = desired_offset_y - offset_y
                residual_yaw = wrap_angle(desired_offset_yaw - offset_yaw)
                target_offset_x = desired_offset_x
                target_offset_y = desired_offset_y
                target_offset_yaw = desired_offset_yaw
                last_measurement_stamp = stamp
                last_measurement_fused = fused_pose
                last_measurement_target = correction.PoseSample(
                    stamp,
                    target_x,
                    target_y,
                    target_yaw,
                )
                for item in accepted:
                    observed[item.observed_index].matched_map_index = item.map_index
                    observed[item.observed_index].matched_score = item.score
                    observed[item.observed_index].inlier = True
                    labels.append(map_landmarks[item.map_index].labels)

            filter_dt = (
                0.0
                if last_filter_stamp is None
                else max(0.0, stamp - last_filter_stamp)
            )
            (
                offset_x,
                offset_y,
                offset_yaw,
                velocity_x,
                velocity_y,
                velocity_yaw,
            ) = advance_offset_filter(
                offset_x,
                offset_y,
                offset_yaw,
                velocity_x,
                velocity_y,
                velocity_yaw,
                target_offset_x,
                target_offset_y,
                target_offset_yaw,
                filter_dt,
                args,
            )
            offset_x, offset_y = clamp_vector(
                offset_x,
                offset_y,
                args.max_total_offset_m,
            )
            offset_yaw = max(
                -args.max_total_yaw,
                min(args.max_total_yaw, offset_yaw),
            )
            last_filter_stamp = stamp

            corrected = correction.PoseSample(
                stamp,
                fused_pose.x + offset_x,
                fused_pose.y + offset_y,
                wrap_angle(fused_pose.yaw + offset_yaw),
            )
            for index, landmark in enumerate(observed):
                landmark_rows.append(
                    {
                        "frame_index": written,
                        "stamp": f"{stamp:.9f}",
                        "fused_x": f"{fused_pose.x:.9f}",
                        "fused_y": f"{fused_pose.y:.9f}",
                        "fused_yaw": f"{fused_pose.yaw:.9f}",
                        "landmark_index": index,
                        "kind": landmark.kind,
                        "arms": landmark.arms,
                        "support_lines": landmark.support_lines,
                        "base_x": f"{landmark.x:.6f}",
                        "base_y": f"{landmark.y:.6f}",
                        "image_u": f"{landmark.image_u:.3f}",
                        "image_v": f"{landmark.image_v:.3f}",
                        "matched_map_index": landmark.matched_map_index,
                        "matched_labels": (
                            map_landmarks[landmark.matched_map_index].labels
                            if landmark.matched_map_index >= 0
                            else ""
                        ),
                        "matched_score": (
                            f"{landmark.matched_score:.6f}"
                            if math.isfinite(landmark.matched_score)
                            else ""
                        ),
                        "inlier": int(landmark.inlier),
                    }
                )
            rows.append(
                CorrectionRow(
                    stamp=stamp,
                    fused_x=fused_pose.x,
                    fused_y=fused_pose.y,
                    fused_yaw=fused_pose.yaw,
                    corrected_x=corrected.x,
                    corrected_y=corrected.y,
                    corrected_yaw=corrected.yaw,
                    offset_x=offset_x,
                    offset_y=offset_y,
                    offset_yaw=offset_yaw,
                    detected_lines=len(lines),
                    detected_landmarks=len(observed),
                    candidate_matches=len(matches),
                    accepted_matches=len(accepted),
                    matched_labels=";".join(sorted(set(labels))),
                    correction_confidence=confidence,
                    temporal_residual=temporal_residual,
                    residual_x=residual_x,
                    residual_y=residual_y,
                    residual_yaw=residual_yaw,
                )
            )

            frame = render_frame(
                bgr,
                mask,
                lines,
                observed,
                map_landmarks,
                features,
                fused_pose,
                corrected,
                calibration,
                stamp,
                written,
                accepted,
                args,
            )
            if writer is None:
                writer = cv2.VideoWriter(
                    str(args.out / "landmark_correction.webm"),
                    cv2.VideoWriter_fourcc(*args.codec),
                    args.fps,
                    (frame.shape[1], frame.shape[0]),
                )
                if not writer.isOpened():
                    raise RuntimeError("failed to open WebM writer")
            writer.write(frame)
            written += 1
            if written % 200 == 0:
                print(f"processed {written} image frames")
            if args.max_frames > 0 and written >= args.max_frames:
                break
    finally:
        if writer is not None:
            writer.release()

    frame_csv = args.out / "landmark_corrected_odom.csv"
    with frame_csv.open("w", newline="", encoding="utf-8") as stream:
        writer_csv = csv.DictWriter(
            stream,
            fieldnames=list(CorrectionRow.__dataclass_fields__),
        )
        writer_csv.writeheader()
        for row in rows:
            writer_csv.writerow(row.__dict__)
    write_landmark_csv(
        args.out / "landmark_observations.csv",
        landmark_rows,
    )
    plot_outputs(args.out, rows, features)
    summary = {
        "image_frames": written,
        "frames_with_landmarks": sum(
            row.detected_landmarks > 0 for row in rows
        ),
        "frames_with_accepted_correction": sum(
            row.accepted_matches > 0 for row in rows
        ),
        "accepted_landmark_observations": sum(
            row.accepted_matches for row in rows
        ),
        "mean_accepted_matches": (
            sum(row.accepted_matches for row in rows) / len(rows)
            if rows
            else 0.0
        ),
        "max_offset_m": (
            max(math.hypot(row.offset_x, row.offset_y) for row in rows)
            if rows
            else 0.0
        ),
        "final_offset": {
            "x": rows[-1].offset_x if rows else 0.0,
            "y": rows[-1].offset_y if rows else 0.0,
            "yaw": rows[-1].offset_yaw if rows else 0.0,
        },
        "camera_position_base_m": [
            calibration.camera_x,
            calibration.camera_y,
            calibration.camera_z,
        ],
        "map_landmarks": len(map_landmarks),
    }
    (args.out / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_odom_bag(db_paths, rows, fused, args)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"wrote {frame_csv}")
    print(f"wrote {args.out / 'landmark_correction.webm'}")


if __name__ == "__main__":
    main()
