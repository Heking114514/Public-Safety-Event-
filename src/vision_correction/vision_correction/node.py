"""Realtime RGB landmark correction for planar fused odometry.

The node intentionally keeps the runtime path small:

* RGB images are converted to a dark-line mask and short Hough segments.
* Perpendicular segment intersections are compared with physical map corners.
* A causal spring-damper filter moves the odometry correction toward matches.
* Without a valid match, the corrected odometry is a causal-smoothed fused
  odometry stream and the old correction decays after visual_timeout_s.

This is a ROS 2 runtime implementation and does not read rosbag files.
"""

from __future__ import annotations

import copy
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from . import road_detector

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - only useful outside a ROS install
    get_package_share_directory = None


@dataclass(frozen=True)
class Segment:
    ax: float
    ay: float
    bx: float
    by: float
    length: float
    yaw: float


@dataclass(frozen=True)
class Landmark:
    x: float
    y: float
    dirs: tuple[float, ...]
    arms: int


@dataclass(frozen=True)
class Pose:
    stamp: float
    x: float
    y: float
    yaw: float


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def undirected_error(first: float, second: float) -> float:
    value = wrap_angle(first - second)
    if value > 0.5 * math.pi:
        value -= math.pi
    elif value < -0.5 * math.pi:
        value += math.pi
    return abs(value)


def angle_mod_pi(value: float) -> float:
    return value % math.pi


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


def stamp_seconds(message, fallback: float = 0.0) -> float:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return fallback
    value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return value if value > 0.0 else fallback


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
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def inverse_transform(
    transform: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    translation, rotation = transform
    inverse_rotation = rotation.T
    return -inverse_rotation @ translation, inverse_rotation


def line_projection(segment: Segment, x: float, y: float) -> float:
    length2 = max(segment.length * segment.length, 1.0e-12)
    return (
        (x - segment.ax) * (segment.bx - segment.ax)
        + (y - segment.ay) * (segment.by - segment.ay)
    ) / length2


def point_segment_distance(x: float, y: float, segment: Segment) -> float:
    projection = max(0.0, min(1.0, line_projection(segment, x, y)))
    px = segment.ax + projection * (segment.bx - segment.ax)
    py = segment.ay + projection * (segment.by - segment.ay)
    return math.hypot(x - px, y - py)


def line_intersection(first: Segment, second: Segment) -> Optional[tuple[float, float]]:
    rx = first.bx - first.ax
    ry = first.by - first.ay
    sx = second.bx - second.ax
    sy = second.by - second.ay
    denominator = rx * sy - ry * sx
    if abs(denominator) < 1.0e-9:
        return None
    qpx = second.ax - first.ax
    qpy = second.ay - first.ay
    t = (qpx * sy - qpy * sx) / denominator
    return first.ax + t * rx, first.ay + t * ry


def cluster_angles(angles: list[float], tolerance: float) -> tuple[float, ...]:
    clusters: list[list[float]] = []
    for angle in angles:
        target = None
        for index, cluster in enumerate(clusters):
            center = math.atan2(
                sum(math.sin(item) for item in cluster),
                sum(math.cos(item) for item in cluster),
            )
            if undirected_error(angle, center) <= tolerance:
                target = index
                break
        if target is None:
            clusters.append([angle])
        else:
            clusters[target].append(angle)
    result = []
    for cluster in clusters:
        result.append(
            angle_mod_pi(
                0.5
                * math.atan2(
                    sum(math.sin(2.0 * item) for item in cluster),
                    sum(math.cos(2.0 * item) for item in cluster),
                )
            )
        )
    return tuple(sorted(result))


class VisionCorrectionNode(Node):
    """Publish a corrected odometry stream without requiring a rosbag."""

    def __init__(self) -> None:
        super().__init__("vision_correction_node")
        self._declare_parameters()
        self._lock = threading.Lock()

        self._map_file = self._resolve_map_file(
            str(self.get_parameter("map_file").value)
        )
        self._features = self._load_map_landmarks(self._map_file)
        if not self._features:
            self.get_logger().warning(
                "No physical map landmarks loaded; node will smooth and relay fused odometry."
            )

        self._fx = 0.0
        self._fy = 0.0
        self._cx = 0.0
        self._cy = 0.0
        self._distortion = np.zeros(5, dtype=np.float64)
        self._have_camera_info = False
        self._transforms: dict[
            tuple[str, str], tuple[np.ndarray, np.ndarray]
        ] = {}
        # CameraInfo for the RGB stream uses the ROS optical convention.
        self._color_to_base = road_detector.DEFAULT_OPTICAL_TO_BASE.copy()
        self._camera_frame_id = "camera_color_optical_frame"
        self._camera_x = float(self.get_parameter("camera_x_m").value)
        self._camera_y = float(self.get_parameter("camera_y_m").value)
        self._camera_z = float(self.get_parameter("camera_height_m").value)

        self._latest_image: Optional[Image] = None
        self._latest_image_stamp = 0.0
        self._last_processed_image_stamp = 0.0
        self._latest_fused: Optional[Odometry] = None
        self._latest_fused_stamp = 0.0
        self._latest_fused_receipt = self.get_clock().now()
        self._last_visual_stamp = 0.0
        self._visual_worker_stop = threading.Event()
        self._target_offset = np.zeros(3, dtype=np.float64)
        self._visual_candidate_offset: Optional[np.ndarray] = None
        self._visual_candidate_since = 0.0
        self._offset = np.zeros(3, dtype=np.float64)
        self._offset_velocity = np.zeros(3, dtype=np.float64)
        self._smoothed_fused: Optional[Pose] = None
        self._last_output_pose: Optional[Pose] = None
        self._last_output_stamp = 0.0
        self._pending_output: Optional[Odometry] = None
        self._last_log_time = self.get_clock().now()

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        fused_topic = str(self.get_parameter("fused_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        tf_static_topic = str(self.get_parameter("tf_static_topic").value)

        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self._image_subscription = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self._fused_subscription = self.create_subscription(
            Odometry,
            fused_topic,
            self._on_fused,
            20,
        )
        tf_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._tf_subscription = self.create_subscription(
            TFMessage,
            tf_static_topic,
            self._on_tf_static,
            tf_qos,
        )

        publish_rate = float(self.get_parameter("publish_rate").value)
        self._timer = None
        if publish_rate > 0.0:
            self._timer = self.create_timer(1.0 / publish_rate, self._publish_pending)
        self._visual_worker = threading.Thread(
            target=self._visual_worker_loop,
            name="vision_correction_worker",
            daemon=True,
        )
        self._visual_worker.start()

        self.get_logger().info(
            f"RGB correction: {image_topic} + {fused_topic} -> {output_topic}; "
            f"map={self._map_file}"
        )

    def _declare_parameters(self) -> None:
        declarations = {
            "map_file": "",
            "image_topic": "/camera/camera/color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "fused_topic": "/odometry/fused",
            "output_topic": "/odometry/landmark_corrected",
            "tf_static_topic": "/tf_static",
            "camera_height_m": 0.121,
            "camera_x_m": 0.055,
            "camera_y_m": 0.0,
            "sync_tolerance_s": 0.12,
            "publish_rate": 0.0,
            "timeout": 0.5,
            "visual_timeout_s": 1.0,
            "visual_stability_s": 0.5,
            "visual_stability_position_m": 0.08,
            "image_processing_interval_s": 0.15,
            "map_frame": "map",
            "base_frame": "base_link",
            "detector": "rgb_and_local",
            "rgb_threshold": 60,
            "rgb_dark_percentile": 88.0,
            "rgb_dark_margin": 30.0,
            "rgb_dark_min_threshold": 18.0,
            "rgb_dark_max_threshold": 90.0,
            "local_delta": 16.0,
            "local_dark_delta": 16.0,
            "local_sigma": 11.0,
            "local_dark_sigma": 11.0,
            "blackhat_size": 25,
            "blackhat_threshold": 10.0,
            "max_color_spread": 35.0,
            "morphology_size": 3,
            "min_ground_x_m": 0.08,
            "roi_top_fraction": 0.0,
            "roi_bottom_fraction": 1.0,
            "ground_grid_resolution_m": 0.01,
            "ground_morphology_size_px": 3,
            "ground_hough_threshold": 10,
            "ground_max_line_gap_m": 0.10,
            "min_line_support_fraction": 0.50,
            "hough_threshold": 14,
            "min_line_length_px": 12,
            "max_line_gap_px": 12,
            "min_ground_segment_length_m": 0.035,
            "line_merge_angle_rad": 0.14,
            "line_merge_distance_m": 0.045,
            "line_merge_gap_m": 0.18,
            "max_ground_x_m": 1.60,
            "max_abs_ground_y_m": 0.65,
            "min_ray_down_z": 0.025,
            "min_component_area": 8,
            "max_component_area_fraction": 0.04,
            "min_component_width": 3,
            "min_component_height": 2,
            "max_segments": 60,
            "max_landmark_match_distance_m": 0.30,
            "corner_angle_tolerance_rad": 0.30,
            "landmark_line_extension_m": 0.075,
            "landmark_connection_radius_m": 0.055,
            "landmark_merge_radius_m": 0.065,
            "min_landmark_arm_length_m": 0.10,
            "max_observed_landmark_arms": 2,
            "map_landmark_merge_radius_m": 0.035,
            "max_correction_speed_mps": 0.20,
            "max_correction_acceleration_mps2": 0.60,
            "max_correction_yaw_speed_rps": 0.60,
            "max_correction_yaw_acceleration_rps2": 1.50,
            "correction_tau_s": 0.70,
            "correction_damping": 1.0,
            "fused_pose_smoothing_tau_s": 0.05,
            "fused_yaw_smoothing_tau_s": 0.04,
        }
        for name, value in declarations.items():
            self.declare_parameter(name, value)

    def _resolve_map_file(self, value: str) -> str:
        if value:
            return str(Path(value).expanduser())
        if get_package_share_directory is not None:
            try:
                return str(
                    Path(get_package_share_directory("arena_path_planner"))
                    / "config"
                    / "arena_map.yaml"
                )
            except Exception:
                pass
        return ""

    def _load_map_landmarks(self, path: str) -> list[Landmark]:
        if not path:
            return []
        try:
            with Path(path).open("r", encoding="utf-8") as stream:
                root = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().error(f"Could not load map {path}: {error}")
            return []

        start_x, start_y = map(float, root["start"]["position_m"])
        start_yaw = math.radians(float(root["start"]["heading_deg"]))
        c = math.cos(start_yaw)
        s = math.sin(start_yaw)

        def to_map(x: float, y: float) -> tuple[float, float]:
            dx = x - start_x
            dy = y - start_y
            return c * dx + s * dy, -s * dx + c * dy

        endpoints: list[tuple[float, float, float]] = []

        def add_rectangle(rect) -> None:
            x1, y1, x2, y2 = map(float, rect)
            corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
            for first, second in zip(corners, corners[1:] + corners[:1]):
                ax, ay = to_map(*first)
                bx, by = to_map(*second)
                yaw = math.atan2(by - ay, bx - ax)
                endpoints.append((ax, ay, yaw))
                endpoints.append((bx, by, wrap_angle(yaw + math.pi)))

        for rectangle in root.get("arena", {}).get("obstacles", []):
            add_rectangle(rectangle)
        for rectangle in root.get("arena", {}).get("free_regions", []):
            add_rectangle(rectangle)

        clusters: list[list[tuple[float, float, float]]] = []
        merge_radius = float(self.get_parameter("map_landmark_merge_radius_m").value)
        for endpoint in endpoints:
            cluster = next(
                (
                    item
                    for item in clusters
                    if math.hypot(
                        endpoint[0] - item[0][0],
                        endpoint[1] - item[0][1],
                    )
                    <= merge_radius
                ),
                None,
            )
            if cluster is None:
                clusters.append([endpoint])
            else:
                cluster.append(endpoint)

        landmarks = []
        for cluster in clusters:
            x = sum(item[0] for item in cluster) / len(cluster)
            y = sum(item[1] for item in cluster) / len(cluster)
            dirs = cluster_angles(
                [item[2] for item in cluster],
                float(self.get_parameter("corner_angle_tolerance_rad").value),
            )
            if len(dirs) >= 2:
                landmarks.append(Landmark(x, y, dirs, len(dirs)))
        return landmarks

    def _on_camera_info(self, message: CameraInfo) -> None:
        # The subscribed topic is image_raw, so K/D describe the actual input
        # image. P belongs to the rectified image model and is only valid for
        # image_rect topics.
        fx = float(message.k[0])
        fy = float(message.k[4])
        cx = float(message.k[2])
        cy = float(message.k[5])
        if fx > 0.0 and fy > 0.0:
            self._fx, self._fy, self._cx, self._cy = fx, fy, cx, cy
            self._distortion = np.asarray(message.d, dtype=np.float64)
            if message.header.frame_id:
                self._camera_frame_id = message.header.frame_id.lstrip("/")
            self._have_camera_info = True
            self._update_camera_transform()

    def _on_tf_static(self, message: TFMessage) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if not parent or not child:
                continue
            translation = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ],
                dtype=np.float64,
            )
            rotation = quaternion_matrix(transform.transform.rotation)
            self._transforms[(parent, child)] = (translation, rotation)
        self._update_camera_transform()

    def _lookup_transform(
        self,
        parent: str,
        child: str,
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        direct = self._transforms.get((parent, child))
        if direct is not None:
            return direct
        reverse = self._transforms.get((child, parent))
        if reverse is not None:
            return inverse_transform(reverse)
        return None

    def _update_camera_transform(self) -> None:
        base = str(self.get_parameter("base_frame").value).lstrip("/")
        camera_link = self._lookup_transform(base, "camera_link")
        if camera_link is None:
            return
        link_translation, link_rotation = camera_link
        camera_translation = np.array(
            [
                link_translation[0],
                link_translation[1],
                float(self.get_parameter("camera_height_m").value)
                + link_translation[2],
            ],
            dtype=np.float64,
        )
        optical_to_base = (
            link_rotation @ road_detector.DEFAULT_OPTICAL_TO_BASE
        )

        color = self._lookup_transform("camera_link", "camera_color_frame")
        if color is not None:
            color_translation, color_rotation = color
            camera_translation += link_rotation @ color_translation
            optical_to_base = (
                link_rotation
                @ color_rotation
                @ road_detector.DEFAULT_OPTICAL_TO_BASE
            )
            color_optical = self._lookup_transform(
                "camera_color_frame",
                "camera_color_optical_frame",
            )
            if color_optical is not None:
                optical_translation, optical_rotation = color_optical
                camera_translation += (
                    link_rotation
                    @ color_rotation
                    @ optical_translation
                )
                optical_to_base = (
                    link_rotation
                    @ color_rotation
                    @ optical_rotation
                )

        self._camera_x = float(camera_translation[0])
        self._camera_y = float(camera_translation[1])
        self._camera_z = float(camera_translation[2])
        self._color_to_base = optical_to_base

    def _on_image(self, message: Image) -> None:
        stamp = stamp_seconds(message, self.get_clock().now().nanoseconds * 1.0e-9)
        with self._lock:
            self._latest_image = message
            self._latest_image_stamp = stamp

    def _visual_worker_loop(self) -> None:
        interval = max(
            0.02,
            float(self.get_parameter("image_processing_interval_s").value),
        )
        while not self._visual_worker_stop.wait(interval):
            with self._lock:
                image = self._latest_image
                stamp = self._latest_image_stamp
            if image is not None:
                self._process_image(image, stamp)

    def _process_image(self, message: Image, stamp: float) -> None:
        """Consume one image once it can be paired with the newest fused pose."""
        with self._lock:
            fused = self._latest_fused
            fused_stamp = self._latest_fused_stamp
            if fused is None or abs(stamp - fused_stamp) > float(
                self.get_parameter("sync_tolerance_s").value
            ):
                return
            if not self._have_camera_info or not self._features:
                return
            if stamp <= self._last_processed_image_stamp:
                return
            self._last_processed_image_stamp = stamp
        try:
            bgr = self._image_to_bgr(message)
            if bgr is None:
                return
            bgr = road_detector.undistort_bgr(bgr, self._camera_geometry())
            segments = self._detect_segments(bgr)
            observed = self._detect_landmarks(segments)
            if not observed:
                with self._lock:
                    self._visual_candidate_offset = None
                    self._visual_candidate_since = 0.0
                return
            fused_pose = self._pose_from_odom(fused, fused_stamp)
            match = self._match_landmarks(fused_pose, observed)
            if match is None:
                with self._lock:
                    self._visual_candidate_offset = None
                    self._visual_candidate_since = 0.0
                return
            target_x, target_y = match
            candidate_offset = np.array(
                [target_x - fused_pose.x, target_y - fused_pose.y, 0.0],
                dtype=np.float64,
            )
            with self._lock:
                stability_distance = float(
                    self.get_parameter("visual_stability_position_m").value
                )
                if (
                    self._visual_candidate_offset is None
                    or np.linalg.norm(
                        candidate_offset[:2]
                        - self._visual_candidate_offset[:2]
                    )
                    > stability_distance
                ):
                    self._visual_candidate_offset = candidate_offset
                    self._visual_candidate_since = stamp
                    return
                if stamp - self._visual_candidate_since < float(
                    self.get_parameter("visual_stability_s").value
                ):
                    return
                self._target_offset[:] = candidate_offset
                self._last_visual_stamp = stamp
        except (ValueError, cv2.error) as error:
            self._throttled_warning(f"visual processing skipped: {error}")

    def _on_fused(self, message: Odometry) -> None:
        receipt = self.get_clock().now()
        fallback_stamp = receipt.nanoseconds * 1.0e-9
        stamp = stamp_seconds(message, fallback_stamp)
        current = self._pose_from_odom(message, stamp)
        with self._lock:
            # Navigation consumers require a causal, monotonic stream. A
            # replayed or duplicated fused sample must not move the output
            # timestamp backward or recompute velocity with a zero interval.
            if self._last_output_stamp > 0.0 and stamp <= self._last_output_stamp:
                return
            self._latest_fused = message
            self._latest_fused_stamp = stamp
            self._latest_fused_receipt = receipt
            if self._smoothed_fused is None:
                smoothed = current
            else:
                dt = max(0.0, stamp - self._smoothed_fused.stamp)
                smoothed = self._smooth_pose(self._smoothed_fused, current, dt)
            self._smoothed_fused = smoothed

            if (
                self._last_visual_stamp <= 0.0
                or stamp - self._last_visual_stamp
                > float(self.get_parameter("visual_timeout_s").value)
            ):
                self._target_offset *= 0.0
                self._visual_candidate_offset = None
                self._visual_candidate_since = 0.0

            dt_filter = (
                0.0
                if self._last_output_stamp <= 0.0
                else max(0.0, stamp - self._last_output_stamp)
            )
            self._advance_correction(dt_filter)
            corrected_pose = Pose(
                stamp,
                smoothed.x + self._offset[0],
                smoothed.y + self._offset[1],
                wrap_angle(smoothed.yaw + self._offset[2]),
            )
            output = self._make_output(message, corrected_pose)
            self._last_output_pose = corrected_pose
            self._last_output_stamp = stamp
            self._pending_output = output

        if float(self.get_parameter("publish_rate").value) <= 0.0:
            self._publisher.publish(output)

    def destroy_node(self) -> bool:
        self._visual_worker_stop.set()
        if getattr(self, "_visual_worker", None) is not None:
            self._visual_worker.join(timeout=1.0)
        return super().destroy_node()

    def _publish_pending(self) -> None:
        with self._lock:
            output = self._pending_output
            receipt = self._latest_fused_receipt
        if output is None:
            return
        timeout = float(self.get_parameter("timeout").value)
        if timeout > 0.0:
            age = (self.get_clock().now() - receipt).nanoseconds * 1.0e-9
            if age > timeout:
                return
        self._publisher.publish(output)

    def _pose_from_odom(self, message: Odometry, stamp: float) -> Pose:
        pose = message.pose.pose
        return Pose(
            stamp,
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    def _smooth_pose(self, previous: Pose, current: Pose, dt: float) -> Pose:
        position_tau = float(self.get_parameter("fused_pose_smoothing_tau_s").value)
        yaw_tau = float(self.get_parameter("fused_yaw_smoothing_tau_s").value)
        alpha_position = (
            1.0
            if position_tau <= 0.0 or dt <= 0.0
            else 1.0 - math.exp(-dt / position_tau)
        )
        alpha_yaw = (
            1.0
            if yaw_tau <= 0.0 or dt <= 0.0
            else 1.0 - math.exp(-dt / yaw_tau)
        )
        return Pose(
            current.stamp,
            previous.x + alpha_position * (current.x - previous.x),
            previous.y + alpha_position * (current.y - previous.y),
            wrap_angle(
                previous.yaw + alpha_yaw * wrap_angle(current.yaw - previous.yaw)
            ),
        )

    def _advance_correction(self, dt: float) -> None:
        if dt <= 0.0:
            return
        tau = max(0.02, float(self.get_parameter("correction_tau_s").value))
        damping = float(self.get_parameter("correction_damping").value)
        omega = 1.0 / tau
        acceleration = (
            omega * omega * (self._target_offset - self._offset)
            - 2.0 * damping * omega * self._offset_velocity
        )
        max_acceleration = float(
            self.get_parameter("max_correction_acceleration_mps2").value
        )
        acceleration_xy = np.asarray(acceleration[:2], dtype=np.float64)
        norm = float(np.linalg.norm(acceleration_xy))
        if norm > max_acceleration > 0.0:
            acceleration_xy *= max_acceleration / norm
        self._offset_velocity[:2] += acceleration_xy * dt
        max_speed = float(self.get_parameter("max_correction_speed_mps").value)
        speed = float(np.linalg.norm(self._offset_velocity[:2]))
        if speed > max_speed > 0.0:
            self._offset_velocity[:2] *= max_speed / speed
        self._offset[:2] += self._offset_velocity[:2] * dt

        yaw_acceleration = float(acceleration[2])
        max_yaw_acceleration = float(
            self.get_parameter("max_correction_yaw_acceleration_rps2").value
        )
        yaw_acceleration = max(
            -max_yaw_acceleration,
            min(max_yaw_acceleration, yaw_acceleration),
        )
        self._offset_velocity[2] += yaw_acceleration * dt
        max_yaw_speed = float(
            self.get_parameter("max_correction_yaw_speed_rps").value
        )
        self._offset_velocity[2] = max(
            -max_yaw_speed,
            min(max_yaw_speed, self._offset_velocity[2]),
        )
        self._offset[2] = wrap_angle(self._offset[2] + self._offset_velocity[2] * dt)

    def _make_output(self, source: Odometry, pose: Pose) -> Odometry:
        output = copy.deepcopy(source)
        output.header.frame_id = str(self.get_parameter("map_frame").value)
        output.child_frame_id = str(self.get_parameter("base_frame").value)
        output.pose.pose.position.x = pose.x
        output.pose.pose.position.y = pose.y
        set_yaw_quaternion(output.pose.pose.orientation, pose.yaw)

        if self._last_output_pose is None:
            return output
        dt = pose.stamp - self._last_output_pose.stamp
        if dt <= 1.0e-4:
            return output
        world_vx = (pose.x - self._last_output_pose.x) / dt
        world_vy = (pose.y - self._last_output_pose.y) / dt
        yaw_rate = wrap_angle(pose.yaw - self._last_output_pose.yaw) / dt
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        output.twist.twist.linear.x = c * world_vx + s * world_vy
        output.twist.twist.linear.y = -s * world_vx + c * world_vy
        output.twist.twist.angular.z = yaw_rate
        return output

    def _image_to_bgr(self, message: Image) -> Optional[np.ndarray]:
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
            code = cv2.COLOR_RGBA2BGR if message.encoding == "rgba8" else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(image, code)
        return None

    def _camera_geometry(self) -> road_detector.CameraGeometry:
        return road_detector.CameraGeometry(
            fx=self._fx,
            fy=self._fy,
            cx=self._cx,
            cy=self._cy,
            camera_x=self._camera_x,
            camera_y=self._camera_y,
            camera_z=self._camera_z,
            distortion=self._distortion,
            color_to_base=self._color_to_base,
        )

    def _detector_parameters(self) -> dict[str, object]:
        names = (
            "detector",
            "rgb_threshold",
            "rgb_dark_percentile",
            "rgb_dark_margin",
            "rgb_dark_min_threshold",
            "rgb_dark_max_threshold",
            "local_delta",
            "local_dark_delta",
            "local_sigma",
            "local_dark_sigma",
            "blackhat_size",
            "blackhat_threshold",
            "max_color_spread",
            "morphology_size",
            "min_ground_x_m",
            "roi_top_fraction",
            "roi_bottom_fraction",
            "ground_grid_resolution_m",
            "ground_morphology_size_px",
            "ground_hough_threshold",
            "ground_max_line_gap_m",
            "min_line_support_fraction",
            "hough_threshold",
            "min_line_length_px",
            "max_line_gap_px",
            "min_ray_down_z",
            "min_ground_segment_length_m",
            "line_merge_angle_rad",
            "line_merge_distance_m",
            "line_merge_gap_m",
            "max_ground_x_m",
            "max_abs_ground_y_m",
            "min_component_area",
            "max_component_area_fraction",
            "min_component_width",
            "min_component_height",
            "max_segments",
        )
        return {name: self.get_parameter(name).value for name in names}

    def _detect_segments(self, bgr: np.ndarray) -> list[Segment]:
        detected, _, _, _ = road_detector.detect_ground_segments(
            bgr,
            self._camera_geometry(),
            self._detector_parameters(),
        )
        return [
            Segment(
                item.ax,
                item.ay,
                item.bx,
                item.by,
                item.length,
                item.yaw,
            )
            for item in detected
        ]

    def _detect_landmarks(self, segments: list[Segment]) -> list[Landmark]:
        candidates: list[Landmark] = []
        angle_tolerance = float(
            self.get_parameter("corner_angle_tolerance_rad").value
        )
        extension = float(
            self.get_parameter("landmark_line_extension_m").value
        )
        connection = float(
            self.get_parameter("landmark_connection_radius_m").value
        )
        min_arm_length = float(
            self.get_parameter("min_landmark_arm_length_m").value
        )
        max_arms = int(
            self.get_parameter("max_observed_landmark_arms").value
        )
        for index, first in enumerate(segments):
            for second in segments[index + 1 :]:
                if abs(undirected_error(first.yaw, second.yaw) - 0.5 * math.pi) > angle_tolerance:
                    continue
                intersection = line_intersection(first, second)
                if intersection is None:
                    continue
                x, y = intersection
                if point_segment_distance(x, y, first) > extension:
                    continue
                if point_segment_distance(x, y, second) > extension:
                    continue
                nearby = [
                    segment
                    for segment in segments
                    if point_segment_distance(x, y, segment) <= connection
                    and -0.2 <= line_projection(segment, x, y) <= 1.2
                ]
                arm_angles: list[float] = []
                arm_lengths: list[float] = []
                for segment in nearby:
                    projection = line_projection(segment, x, y)
                    if projection < 0.5:
                        endpoint_x, endpoint_y = segment.ax, segment.ay
                    else:
                        endpoint_x, endpoint_y = segment.bx, segment.by
                    arm_length = math.hypot(endpoint_x - x, endpoint_y - y)
                    if arm_length < min_arm_length:
                        continue
                    arm_angles.append(math.atan2(endpoint_y - y, endpoint_x - x))
                    arm_lengths.append(arm_length)
                dirs = cluster_angles(arm_angles, angle_tolerance)
                if len(dirs) < 2:
                    continue
                if len(dirs) > max_arms or len(arm_lengths) < 2:
                    continue
                candidate = Landmark(x, y, dirs, len(dirs))
                if any(
                    math.hypot(item.x - x, item.y - y)
                    <= float(self.get_parameter("landmark_merge_radius_m").value)
                    for item in candidates
                ):
                    continue
                candidates.append(candidate)
        return candidates[:10]

    def _match_landmarks(
        self,
        fused: Pose,
        observed: list[Landmark],
    ) -> Optional[tuple[float, float]]:
        best_score = float("inf")
        best_target = None
        max_distance = float(
            self.get_parameter("max_landmark_match_distance_m").value
        )
        for item in observed:
            predicted_x = fused.x + math.cos(fused.yaw) * item.x - math.sin(fused.yaw) * item.y
            predicted_y = fused.y + math.sin(fused.yaw) * item.x + math.cos(fused.yaw) * item.y
            for mapped in self._features:
                position_distance = math.hypot(
                    predicted_x - mapped.x,
                    predicted_y - mapped.y,
                )
                if position_distance > max_distance:
                    continue
                transformed = [fused.yaw + direction for direction in item.dirs]
                direction_error = sum(
                    min(undirected_error(direction, target) for target in mapped.dirs)
                    for direction in transformed
                ) / max(1, len(transformed))
                if direction_error > 0.45:
                    continue
                score = position_distance + 0.20 * direction_error
                score += 0.035 * abs(item.arms - mapped.arms)
                if score >= best_score:
                    continue
                best_score = score
                best_target = (
                    mapped.x - math.cos(fused.yaw) * item.x + math.sin(fused.yaw) * item.y,
                    mapped.y - math.sin(fused.yaw) * item.x - math.cos(fused.yaw) * item.y,
                )
        return best_target

    def _throttled_warning(self, text: str) -> None:
        now = self.get_clock().now()
        if (now - self._last_log_time) >= Duration(seconds=2.0):
            self.get_logger().warning(text)
            self._last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionCorrectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
