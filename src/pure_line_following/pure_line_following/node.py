"""ROS 2 adapter for odometry-independent white-track following."""

from __future__ import annotations

import json
import math
import time
import dataclasses
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .controller import ControllerConfig, command_for, limit_angular_rate
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
        code = (
            cv2.COLOR_RGBA2BGR
            if message.encoding == "rgba8"
            else cv2.COLOR_BGRA2BGR
        )
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
    return road_detector.CameraGeometry(
        fx=float(node.get_parameter("fallback_fx").value),
        fy=float(node.get_parameter("fallback_fy").value),
        cx=float(node.get_parameter("fallback_cx").value),
        cy=float(node.get_parameter("fallback_cy").value),
        camera_x=float(node.get_parameter("camera_x_m").value),
        camera_y=float(node.get_parameter("camera_y_m").value),
        camera_z=float(node.get_parameter("camera_height_m").value),
        color_to_base=road_detector.DEFAULT_OPTICAL_TO_BASE.copy(),
    )


class PureLineFollowingNode(Node):
    """Line-following node with optional aligned-depth geometry."""

    def __init__(self) -> None:
        super().__init__("pure_line_following")
        self._declare_parameters()
        self._camera = _camera_model_from_parameters(self)
        self._have_camera_info = False
        self._last_image_time = 0.0
        self._latest_depth_m: Optional[np.ndarray] = None
        self._latest_depth_stamp_s = 0.0
        self._last_depth_time = 0.0
        self._last_valid_observation_time = 0.0
        self._last_observation: Optional[TrackObservation] = None
        self._last_command = Twist()
        self._last_command_update_time = time.monotonic()
        self._lateral_integral_m_s = 0.0
        self._last_integral_update_time = self._last_command_update_time
        self._valid_observation_streak = 0
        self._last_shared_topic_check_time = 0.0
        self._shared_topic_publishers: tuple[str, ...] = ()
        self._last_shared_topic_warning: tuple[str, ...] = ()
        self._last_status = ""
        self._last_processing_error = ""
        self._enabled = bool(self.get_parameter("enabled").value)
        self._require_camera_info = bool(
            self.get_parameter("require_camera_info").value
        )
        self._use_depth_projection = bool(
            self.get_parameter("use_depth_projection").value
        )
        self._require_depth = bool(self.get_parameter("require_depth").value)
        self._image_is_rectified = bool(
            self.get_parameter("image_is_rectified").value
        )
        self._unsafe_command_topic = (
            str(self.get_parameter("cmd_vel_topic").value) == "/cmd_vel_nav"
            and not bool(self.get_parameter("allow_navigation_cmd_vel_topic").value)
        )
        if self._unsafe_command_topic:
            self.get_logger().error(
                "Refusing /cmd_vel_nav: pure line following must use its "
                "isolated command topic so waypoint navigation cannot compete"
            )

        tracker_config = TrackerConfig(
            **{
                field.name: self.get_parameter(field.name).value
                for field in dataclasses.fields(TrackerConfig)
                if self.has_parameter(field.name)
            }
        )
        controller_config = ControllerConfig(
            **{
                field.name: self.get_parameter(field.name).value
                for field in dataclasses.fields(ControllerConfig)
                if self.has_parameter(field.name)
            }
        )
        self._tracker_config = tracker_config
        self._controller_config = controller_config

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        depth_image_topic = str(self.get_parameter("depth_image_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._cmd_vel_topic = cmd_vel_topic
        status_topic = str(self.get_parameter("status_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)

        self._cmd_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
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
        rate = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate, self._publish_watchdog)
        self.get_logger().info(
            f"pure line following ready: image={image_topic} "
            f"cmd_vel={cmd_vel_topic}; depth="
            f"{'enabled' if self._use_depth_projection else 'disabled'}"
        )
        self._publish_status(
            None,
            "waiting_for_camera_info"
            if self._require_camera_info
            else "waiting_for_image",
        )

    def _declare_parameters(self) -> None:
        declarations = {
            "image_topic": "/camera/camera/color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "depth_image_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "cmd_vel_topic": "/cmd_vel_line_following",
            "status_topic": "/pure_line_following/status",
            "debug_image_topic": "/pure_line_following/debug_image",
            "publish_debug_image": True,
            "require_camera_info": True,
            "require_depth": False,
            "image_is_rectified": False,
            "camera_x_m": 0.055,
            # The RGB optical head is 59.1 mm to the right of camera_link in
            # the recorded D455 static transform (base_link + camera_link).
            "camera_y_m": -0.020,
            "camera_height_m": 0.130,
            "fallback_fx": 383.0,
            "fallback_fy": 382.5,
            "fallback_cx": 318.0,
            "fallback_cy": 247.0,
            "depth_scale": 0.001,
            "depth_sync_tolerance_s": 0.12,
            "image_timeout_s": 0.50,
            "invalid_stop_s": 0.40,
            "command_rate_hz": 30.0,
            "enabled": True,
            "allow_navigation_cmd_vel_topic": False,
            "reject_shared_cmd_vel": True,
            "shared_cmd_vel_check_period_s": 0.25,
            "min_valid_observations_before_motion": 2,
        }
        for field in dataclasses.fields(TrackerConfig):
            declarations.setdefault(field.name, getattr(TrackerConfig(), field.name))
        for field in dataclasses.fields(ControllerConfig):
            declarations.setdefault(field.name, getattr(ControllerConfig(), field.name))
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
            self._valid_observation_streak = 0
            self._decay_lateral_integral(time.monotonic())
            self._stop_command(publish=True)
            self._publish_status(None, "waiting_for_camera_info")
            return
        depth_m = self._depth_for_image(message)
        if self._require_depth and depth_m is None:
            self._valid_observation_streak = 0
            self._decay_lateral_integral(time.monotonic())
            self._stop_command(publish=True)
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
            self._valid_observation_streak = 0
            self._decay_lateral_integral(time.monotonic())
            self._stop_command(publish=True)
            self._publish_status(None, f"image_error:{type(exc).__name__}")
            error = f"image processing stopped command: {exc}"
            if error != self._last_processing_error:
                self.get_logger().warning(error)
                self._last_processing_error = error
            return

        self._last_processing_error = ""
        self._last_observation = observation
        now = time.monotonic()
        self._last_image_time = now
        shared_publishers = self._shared_command_publishers(now)
        normal_track_ready = (
            observation.valid
            and observation.confidence >= self._controller_config.min_confidence
        )
        recovery_track_ready = (
            observation.can_recover
            and observation.confidence
            >= self._controller_config.recovery_min_confidence
        )
        command_allowed = (
            self._enabled
            and not self._unsafe_command_topic
            and (normal_track_ready or recovery_track_ready)
            and not shared_publishers
        )
        if command_allowed:
            self._valid_observation_streak += 1
            # Keep the watchdog alive while the short consecutive-frame
            # confirmation is in progress. Otherwise the watchdog resets the
            # streak before the next camera frame can confirm it.
            self._last_valid_observation_time = now
            minimum_streak = max(
                1,
                int(
                    self.get_parameter(
                        "min_valid_observations_before_motion"
                    ).value
                ),
            )
            raw_command = command_for(
                observation,
                self._controller_config,
                lateral_integral_m_s=self._update_lateral_integral(
                    observation,
                    now,
                    normal_track_ready,
                ),
            )
            if self._valid_observation_streak >= minimum_streak:
                previous_angular = self._last_command.angular.z
                self._last_command = raw_command
                self._last_command.angular.z = limit_angular_rate(
                    raw_command.angular.z,
                    previous_angular,
                    now - self._last_command_update_time,
                    self._controller_config,
                )
                self._last_command_update_time = now
                self._last_valid_observation_time = now
            else:
                self._stop_command(publish=True)
                self._last_valid_observation_time = now
        else:
            self._valid_observation_streak = 0
            self._decay_lateral_integral(now)
            self._stop_command(publish=True)
        reason = ""
        if not self._enabled:
            reason = "disabled"
        elif self._unsafe_command_topic:
            reason = "unsafe_cmd_vel_topic"
        elif shared_publishers:
            reason = "shared_cmd_vel_topic"
        elif observation.can_recover:
            reason = "recovering_unsafe_edge"
        elif observation.state == UNSAFE_EDGE:
            reason = "road_unsafe_stop"
        elif not observation.valid:
            reason = "track_invalid"
        elif observation.confidence < self._controller_config.min_confidence:
            reason = "confidence_below_threshold"
        elif self._valid_observation_streak < max(
            1,
            int(
                self.get_parameter("min_valid_observations_before_motion").value
            ),
        ):
            reason = "waiting_for_stable_track"
        self._publish_status(observation, reason)
        if self._debug_publisher is not None:
            self._publish_debug(image, observation)

    def _publish_watchdog(self) -> None:
        now = time.monotonic()
        image_timeout = float(self.get_parameter("image_timeout_s").value)
        invalid_stop = max(
            0.0,
            float(self.get_parameter("invalid_stop_s").value),
        )
        shared_publishers = self._shared_command_publishers(now)
        if (
            not self._enabled
            or self._unsafe_command_topic
            or self._last_image_time <= 0.0
            or now - self._last_image_time > image_timeout
            or self._last_valid_observation_time <= 0.0
            or now - self._last_valid_observation_time > invalid_stop
            or shared_publishers
        ):
            self._valid_observation_streak = 0
            self._decay_lateral_integral(now)
            self._stop_command()
        self._cmd_publisher.publish(self._last_command)

    def _update_lateral_integral(
        self,
        observation: TrackObservation,
        now: float,
        normal_track_ready: bool,
    ) -> float:
        elapsed = max(
            0.0,
            min(0.25, now - self._last_integral_update_time),
        )
        self._last_integral_update_time = now
        params = self._controller_config
        width = float(observation.road_clearance_m)
        eligible = (
            normal_track_ready
            and observation.state in ("TRACK", "JUNCTION")
            and math.isfinite(float(observation.lateral_error_m))
            and abs(float(observation.lateral_error_m))
            <= max(0.0, params.lateral_integral_max_error_m)
            and math.isfinite(width)
            and width >= max(0.0, params.min_observed_road_width_m)
            and width <= max(
                params.min_observed_road_width_m,
                params.heading_ignore_road_width_m,
            )
            and abs(float(observation.heading_error_rad))
            <= max(0.0, params.lateral_integral_max_heading_rad)
        )
        if not eligible or params.lateral_integral_gain <= 0.0:
            return self._decay_lateral_integral(now, elapsed=elapsed)
        limit = max(0.0, params.lateral_integral_limit_m_s)
        self._lateral_integral_m_s = max(
            -limit,
            min(
                limit,
                self._lateral_integral_m_s
                + float(observation.lateral_error_m) * elapsed,
            ),
        )
        return self._lateral_integral_m_s

    def _decay_lateral_integral(
        self,
        now: float,
        elapsed: float | None = None,
    ) -> float:
        if elapsed is None:
            elapsed = max(
                0.0,
                min(0.5, now - self._last_integral_update_time),
            )
            self._last_integral_update_time = now
        decay = max(0.0, self._controller_config.lateral_integral_leak_per_s)
        if decay <= 0.0 or elapsed <= 0.0:
            return self._lateral_integral_m_s
        self._lateral_integral_m_s *= math.exp(-decay * elapsed)
        if abs(self._lateral_integral_m_s) < 1.0e-5:
            self._lateral_integral_m_s = 0.0
        return self._lateral_integral_m_s

    def _stop_command(self, publish: bool = False) -> None:
        self._last_command = Twist()
        self._last_command_update_time = time.monotonic()
        self._last_valid_observation_time = 0.0
        if publish:
            self._cmd_publisher.publish(self._last_command)

    def _shared_command_publishers(self, now: float) -> tuple[str, ...]:
        if not bool(self.get_parameter("reject_shared_cmd_vel").value):
            return ()
        period = max(
            0.05,
            float(
                self.get_parameter("shared_cmd_vel_check_period_s").value
            ),
        )
        if now - self._last_shared_topic_check_time < period:
            return self._shared_topic_publishers
        self._last_shared_topic_check_time = now
        try:
            endpoint_infos = self.get_publishers_info_by_topic(
                self._cmd_vel_topic
            )
        except RuntimeError:
            endpoint_infos = []
        own_name = self.get_name()
        own_namespace = self.get_namespace()
        publishers = {
            self._endpoint_name(info)
            for info in endpoint_infos
            if not (
                info.node_name == own_name
                and info.node_namespace == own_namespace
            )
        }
        self._shared_topic_publishers = tuple(sorted(publishers))
        if self._shared_topic_publishers != self._last_shared_topic_warning:
            if self._shared_topic_publishers:
                self.get_logger().warning(
                    "Command output is shared; refusing to drive "
                    f"{self._cmd_vel_topic} because other publishers are "
                    f"present: {', '.join(self._shared_topic_publishers)}"
                )
            elif self._last_shared_topic_warning:
                self.get_logger().info(
                    f"Command topic {self._cmd_vel_topic} is exclusive again"
                )
            self._last_shared_topic_warning = self._shared_topic_publishers
        return self._shared_topic_publishers

    @staticmethod
    def _endpoint_name(endpoint_info: object) -> str:
        namespace = str(getattr(endpoint_info, "node_namespace", "")).rstrip(
            "/"
        )
        name = str(getattr(endpoint_info, "node_name", "unknown")).lstrip(
            "/"
        )
        return f"{namespace}/{name}" if namespace else f"/{name}"

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
        now = time.monotonic()
        depth_age = (
            now - self._last_depth_time
            if self._last_depth_time > 0.0
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
            "depth_enabled": self._use_depth_projection,
            "depth_recent": (
                bool(depth_age <= float(self.get_parameter("image_timeout_s").value))
                if math.isfinite(depth_age)
                else False
            ),
            "depth_age_s": (
                round(depth_age, 3) if math.isfinite(depth_age) else None
            ),
            "depth_lateral_error_m": round(depth_lateral, 4),
            "depth_control_pixels": int(depth_control_pixels),
            "depth_control_rows": int(depth_control_rows),
            "depth_valid_fraction": round(depth_valid_fraction, 3),
            "depth_reject_reason": depth_reject_reason,
            "can_recover": (
                bool(observation.can_recover)
                if observation is not None
                else False
            ),
            "camera_info": self._have_camera_info,
            "camera_height_m": round(float(self._camera.camera_z), 4),
            "camera_y_m": round(float(self._camera.camera_y), 4),
            "lateral_error_bias_m": round(
                float(self._tracker_config.lateral_error_bias_m),
                4,
            ),
            "lateral_integral_m_s": round(self._lateral_integral_m_s, 5),
        }
        if reason:
            payload["reason"] = reason
        text = json.dumps(payload, separators=(",", ":"))
        if text == self._last_status:
            return
        self._last_status = text
        message = String()
        message.data = text
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
            f"{observation.state} valid={int(observation.valid)} "
            f"recover={int(observation.can_recover)} "
            f"src={observation.lateral_source} "
            f"e={observation.lateral_error_m:+.3f} "
            f"h={math.degrees(observation.heading_error_rad):+.1f}deg "
            f"w={observation.road_clearance_m:.3f}"
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
    node = PureLineFollowingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._last_command = Twist()
            try:
                node._cmd_publisher.publish(node._last_command)
            except RuntimeError:
                # The launch system may have invalidated the context first.
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
