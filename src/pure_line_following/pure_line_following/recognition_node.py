"""ROS 2 node for camera-only pure-line path recognition."""

from __future__ import annotations

import dataclasses
import json
import math
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vehicle_config import parameters_for, rpy_rotation_matrix

from . import geometry as road_detector
from .tracker import LOST, UNSAFE_EDGE, TrackObservation, TrackerConfig, observe


def _image_to_bgr(message: Image) -> Optional[np.ndarray]:
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


def _stamp_to_seconds(message: Image) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _image_to_depth_m(message: Image, depth_scale: float) -> Optional[np.ndarray]:
    encoding = str(message.encoding).strip().lower()
    if encoding in ("16uc1", "mono16"):
        itemsize = 2
        dtype_code = "u2"
        scale = float(depth_scale)
        if not math.isfinite(scale) or scale <= 0.0:
            raise ValueError("depth_scale must be positive and finite")
    elif encoding == "32fc1":
        itemsize = 4
        dtype_code = "f4"
        scale = 1.0
    else:
        return None

    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height < 0 or width < 0 or step < width * itemsize:
        raise ValueError("invalid depth image dimensions")
    data = memoryview(message.data)
    if data.nbytes < height * step:
        raise ValueError("depth image data is shorter than height * step")
    byte_order = ">" if bool(message.is_bigendian) else "<"
    dtype = np.dtype(byte_order + dtype_code)
    depth = np.empty((height, width), dtype=np.float32)
    row_bytes = width * itemsize
    for row in range(height):
        start = row * step
        depth[row, :] = np.frombuffer(
            data[start : start + row_bytes],
            dtype=dtype,
            count=width,
        )
    depth *= np.float32(scale)
    return depth


def _camera_model_from_parameters(node: Node) -> road_detector.CameraGeometry:
    link_to_base = np.asarray(
        rpy_rotation_matrix(
            float(node.get_parameter("camera_roll_rad").value),
            float(node.get_parameter("camera_pitch_rad").value),
            float(node.get_parameter("camera_yaw_rad").value),
        ),
        dtype=np.float64,
    )
    return road_detector.CameraGeometry(
        fx=float(node.get_parameter("fallback_fx").value),
        fy=float(node.get_parameter("fallback_fy").value),
        cx=float(node.get_parameter("fallback_cx").value),
        cy=float(node.get_parameter("fallback_cy").value),
        camera_x=float(node.get_parameter("camera_x_m").value),
        camera_y=float(node.get_parameter("camera_y_m").value),
        camera_z=float(node.get_parameter("camera_height_m").value),
        color_to_base=link_to_base @ road_detector.DEFAULT_OPTICAL_TO_BASE,
    )


class PureLineRecognitionNode(Node):
    """Recognize the visible white track without publishing drive commands."""

    def __init__(self) -> None:
        super().__init__("pure_line_recognition")
        self._declare_parameters()
        self._camera = _camera_model_from_parameters(self)
        self._have_camera_info = False
        self._latest_depth_m: Optional[np.ndarray] = None
        self._latest_depth_stamp_s = 0.0
        self._last_depth_time = 0.0
        self._last_image_time = 0.0
        self._last_observation: Optional[TrackObservation] = None
        self._last_processing_error = ""
        self._require_camera_info = bool(
            self.get_parameter("require_camera_info").value
        )
        self._require_depth = bool(self.get_parameter("require_depth").value)
        self._use_depth_projection = bool(
            self.get_parameter("use_depth_projection").value
        )
        self._image_is_rectified = bool(
            self.get_parameter("image_is_rectified").value
        )
        self._tracker_config = TrackerConfig(
            **{
                field.name: self.get_parameter(field.name).value
                for field in dataclasses.fields(TrackerConfig)
                if self.has_parameter(field.name)
            }
        )

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        depth_image_topic = str(self.get_parameter("depth_image_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)

        self._status_publisher = self.create_publisher(String, status_topic, 10)
        self._debug_publisher = (
            self.create_publisher(Image, debug_topic, 10)
            if bool(self.get_parameter("publish_debug_image").value)
            else None
        )
        self._image_subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._camera_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self._depth_subscription = (
            self.create_subscription(
                Image,
                depth_image_topic,
                self._depth_callback,
                qos_profile_sensor_data,
            )
            if self._use_depth_projection
            else None
        )
        self.get_logger().info(
            f"pure line recognition ready: image={image_topic}; "
            f"status={status_topic}; depth="
            f"{'enabled' if self._use_depth_projection else 'disabled'}"
        )
        self._publish_status(
            None,
            "waiting_for_camera_info"
            if self._require_camera_info
            else "waiting_for_image",
        )

    def _declare_parameters(self) -> None:
        physical = parameters_for("pure_line_following")
        declarations = {
            "image_topic": "/camera/camera/color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "depth_image_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "status_topic": "/pure_line_recognition/status",
            "debug_image_topic": "/pure_line_recognition/debug_image",
            "publish_debug_image": True,
            "require_camera_info": True,
            "require_depth": False,
            "image_is_rectified": False,
            "camera_x_m": physical["camera_x_m"],
            "camera_y_m": physical["camera_y_m"],
            "camera_height_m": physical["camera_height_m"],
            "camera_roll_rad": physical["camera_roll_rad"],
            "camera_pitch_rad": physical["camera_pitch_rad"],
            "camera_yaw_rad": physical["camera_yaw_rad"],
            "fallback_fx": 383.0,
            "fallback_fy": 382.5,
            "fallback_cx": 318.0,
            "fallback_cy": 247.0,
            "depth_scale": 0.001,
            "depth_sync_tolerance_s": 0.12,
            "image_timeout_s": 0.50,
        }
        for field in dataclasses.fields(TrackerConfig):
            declarations.setdefault(field.name, getattr(TrackerConfig(), field.name))
        self.declare_parameters("", list(declarations.items()))

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if self._image_is_rectified and len(message.p) >= 7:
            fx = float(message.p[0])
            fy = float(message.p[5])
            cx = float(message.p[2])
            cy = float(message.p[6])
        else:
            fx = float(message.k[0])
            fy = float(message.k[4])
            cx = float(message.k[2])
            cy = float(message.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return
        distortion = (
            None
            if self._image_is_rectified
            else np.asarray(message.d, dtype=np.float64)
        )
        self._camera = road_detector.CameraGeometry(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            camera_x=self._camera.camera_x,
            camera_y=self._camera.camera_y,
            camera_z=self._camera.camera_z,
            distortion=distortion,
            color_to_base=self._camera.color_to_base,
        )
        self._have_camera_info = True

    def _depth_callback(self, message: Image) -> None:
        try:
            depth = _image_to_depth_m(
                message,
                float(self.get_parameter("depth_scale").value),
            )
            if depth is None or depth.size == 0:
                raise ValueError(f"unsupported depth encoding: {message.encoding}")
        except (ValueError, TypeError, OverflowError) as exc:
            error = f"depth frame ignored: {exc}"
            if error != self._last_processing_error:
                self.get_logger().warning(error)
                self._last_processing_error = error
            return
        self._latest_depth_m = depth
        self._latest_depth_stamp_s = _stamp_to_seconds(message)
        self._last_depth_time = time.monotonic()

    def _depth_for_image(self, message: Image) -> Optional[np.ndarray]:
        if not self._use_depth_projection or self._latest_depth_m is None:
            return None
        tolerance = max(
            0.0,
            float(self.get_parameter("depth_sync_tolerance_s").value),
        )
        image_stamp_s = _stamp_to_seconds(message)
        if (
            self._latest_depth_stamp_s > 0.0
            and image_stamp_s > 0.0
            and abs(self._latest_depth_stamp_s - image_stamp_s) > tolerance
        ):
            return None
        depth = self._latest_depth_m
        if not self._image_is_rectified:
            depth = road_detector.undistort_depth(depth, self._camera)
        return depth

    def _image_callback(self, message: Image) -> None:
        if self._require_camera_info and not self._have_camera_info:
            self._publish_status(None, "waiting_for_camera_info")
            return
        depth_m = self._depth_for_image(message)
        if self._require_depth and depth_m is None:
            self._publish_status(None, "waiting_for_depth")
            return

        try:
            image = _image_to_bgr(message)
            if image is None or image.size == 0:
                raise ValueError(f"unsupported image encoding: {message.encoding}")
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError("decoded image is not a 3-channel image")
            if not self._image_is_rectified:
                image = road_detector.undistort_bgr(image, self._camera)
            observation = observe(
                image,
                self._camera,
                self._tracker_config,
                depth_m=depth_m,
            )
        except (ValueError, cv2.error, IndexError, TypeError, OverflowError) as exc:
            self._last_observation = None
            self._publish_status(None, f"image_error:{type(exc).__name__}")
            error = f"image processing failed: {exc}"
            if error != self._last_processing_error:
                self.get_logger().warning(error)
                self._last_processing_error = error
            return

        self._last_processing_error = ""
        self._last_image_time = time.monotonic()
        self._last_observation = observation
        self._publish_status(observation, self._status_reason(observation))
        if self._debug_publisher is not None:
            self._publish_debug(image, observation)

    @staticmethod
    def _status_reason(observation: TrackObservation) -> str:
        if observation.state == UNSAFE_EDGE:
            return "road_unsafe"
        if not observation.valid or observation.state == LOST:
            return "track_invalid"
        return "recognition_only"

    def _publish_status(
        self,
        observation: Optional[TrackObservation],
        reason: str = "",
    ) -> None:
        if observation is None:
            state = LOST
            valid = False
            confidence = 0.0
            lateral_error = 0.0
            heading_error = 0.0
            green_fraction = 0.0
            road_left = 0.0
            road_right = 0.0
            road_clearance = 0.0
            lateral_source = "none"
            depth_lateral = 0.0
            depth_control_pixels = 0
            depth_control_rows = 0
            depth_valid_fraction = 0.0
            depth_reject_reason = ""
            branch_x = 0.0
            branch_left = 0.0
            branch_right = 0.0
            can_recover = False
        else:
            state = observation.state
            valid = observation.valid
            confidence = observation.confidence
            lateral_error = observation.lateral_error_m
            heading_error = observation.heading_error_rad
            green_fraction = observation.green_fraction
            road_left = observation.road_left_y_m
            road_right = observation.road_right_y_m
            road_clearance = observation.road_clearance_m
            lateral_source = observation.lateral_source
            depth_lateral = observation.depth_lateral_error_m
            depth_control_pixels = observation.depth_control_pixels
            depth_control_rows = observation.depth_control_rows
            depth_valid_fraction = observation.depth_valid_fraction
            depth_reject_reason = observation.depth_reject_reason
            branch_x = observation.branch_x_m
            branch_left = observation.branch_left_extent_m
            branch_right = observation.branch_right_extent_m
            can_recover = observation.can_recover
        now = time.monotonic()
        depth_age = (
            now - self._last_depth_time
            if self._last_depth_time > 0.0
            else float("inf")
        )
        image_age = (
            now - self._last_image_time
            if self._last_image_time > 0.0
            else float("inf")
        )
        payload = {
            "state": state,
            "valid": valid,
            "confidence": round(confidence, 3),
            "lateral_error_m": round(lateral_error, 4),
            "lateral_source": lateral_source,
            "heading_error_rad": round(heading_error, 4),
            "green_fraction": round(green_fraction, 3),
            "road_left_y_m": round(road_left, 4),
            "road_right_y_m": round(road_right, 4),
            "road_clearance_m": round(road_clearance, 4),
            "branch_x_m": round(branch_x, 4),
            "branch_left_extent_m": round(branch_left, 4),
            "branch_right_extent_m": round(branch_right, 4),
            "depth_enabled": self._use_depth_projection,
            "depth_recent": (
                bool(depth_age <= float(self.get_parameter("image_timeout_s").value))
                if math.isfinite(depth_age)
                else False
            ),
            "depth_age_s": (
                round(depth_age, 3) if math.isfinite(depth_age) else None
            ),
            "image_age_s": (
                round(image_age, 3) if math.isfinite(image_age) else None
            ),
            "depth_lateral_error_m": round(depth_lateral, 4),
            "depth_control_pixels": int(depth_control_pixels),
            "depth_control_rows": int(depth_control_rows),
            "depth_valid_fraction": round(depth_valid_fraction, 3),
            "depth_reject_reason": depth_reject_reason,
            "can_recover": bool(can_recover),
            "camera_info": self._have_camera_info,
            "camera_height_m": round(float(self._camera.camera_z), 4),
            "camera_y_m": round(float(self._camera.camera_y), 4),
            "lateral_error_bias_m": round(
                float(self._tracker_config.lateral_error_bias_m),
                4,
            ),
            "recognition_only": True,
        }
        if reason:
            payload["reason"] = reason
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._status_publisher.publish(message)

    def _publish_debug(
        self,
        image: np.ndarray,
        observation: TrackObservation,
    ) -> None:
        panel = image.copy()
        panel[observation.mask != 0] = (
            0.55 * panel[observation.mask != 0]
            + 0.45 * np.array([0, 0, 255], dtype=np.float32)
        ).astype(np.uint8)
        label = (
            "RECOGNITION "
            f"{observation.state} valid={int(observation.valid)} "
            f"src={observation.lateral_source} "
            f"e={observation.lateral_error_m:+.3f} "
            f"h={math.degrees(observation.heading_error_rad):+.1f}deg "
            f"w={observation.road_clearance_m:.3f} "
            f"bx={observation.branch_x_m:.2f}"
        )
        cv2.putText(
            panel,
            label,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            panel,
            label,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 0) if observation.valid else (0, 180, 255),
            1,
            cv2.LINE_AA,
        )
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_color_optical_frame"
        message.height = int(panel.shape[0])
        message.width = int(panel.shape[1])
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = int(panel.shape[1] * 3)
        message.data = panel.tobytes()
        self._debug_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PureLineRecognitionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
