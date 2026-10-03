"""OpenCV GUI for live camera and pure-line-following debug images."""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


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


def _format_float(value: object, digits: int = 3) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(number):
        return "-"
    return f"{number:.{digits}f}"


def _put_lines(
    image: np.ndarray,
    lines: list[tuple[str, tuple[int, int, int]]],
    origin: tuple[int, int],
    line_height: int,
) -> None:
    x, y = origin
    for text, color in lines:
        cv2.putText(
            image,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            1,
            cv2.LINE_AA,
        )
        y += line_height


class PureLineDebugViewer(Node):
    """Display the camera feed, detector overlay, current status and command."""

    def __init__(self) -> None:
        super().__init__("pure_line_debug_viewer")
        self.declare_parameters(
            "",
            [
                ("debug_image_topic", "/pure_line_following/debug_image"),
                ("camera_image_topic", "/camera/camera/color/image_raw"),
                ("status_topic", "/pure_line_following/status"),
                ("cmd_vel_topic", "/cmd_vel_line_following"),
                ("window_name", "pure line following"),
                ("view_mode", "side_by_side"),
                ("show_command", True),
                ("display_scale", 1.0),
                ("console_period_s", 1.0),
                ("status_panel_width_px", 360),
                ("refresh_rate_hz", 30.0),
            ],
        )

        self._latest_debug_image: Optional[np.ndarray] = None
        self._latest_debug_image_time = 0.0
        self._latest_camera_image: Optional[np.ndarray] = None
        self._latest_camera_image_time = 0.0
        self._status: dict[str, object] = {}
        self._status_time = 0.0
        self._cmd = Twist()
        self._cmd_time = 0.0
        self._last_console_time = 0.0
        self._window_ready = False
        self._window_name = str(self.get_parameter("window_name").value)
        self._show_command = bool(self.get_parameter("show_command").value)
        self._debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self._camera_image_topic = str(
            self.get_parameter("camera_image_topic").value
        )
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._debug_subscription = self.create_subscription(
            Image,
            self._debug_image_topic,
            self._debug_image_callback,
            qos_profile_sensor_data,
        )
        self._camera_subscription = self.create_subscription(
            Image,
            self._camera_image_topic,
            self._camera_image_callback,
            qos_profile_sensor_data,
        )
        self._status_subscription = self.create_subscription(
            String,
            self._status_topic,
            self._status_callback,
            10,
        )
        self._cmd_subscription = (
            self.create_subscription(
                Twist,
                self._cmd_vel_topic,
                self._cmd_callback,
                10,
            )
            if self._show_command
            else None
        )
        rate = max(1.0, float(self.get_parameter("refresh_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate, self._draw)
        self.get_logger().info(
            "debug viewer ready: camera=%s debug=%s status=%s cmd=%s"
            % (
                self._camera_image_topic,
                self._debug_image_topic,
                self._status_topic,
                self._cmd_vel_topic if self._show_command else "disabled",
            )
        )

    def _debug_image_callback(self, message: Image) -> None:
        image = _image_to_bgr(message)
        if image is None or image.size == 0:
            self.get_logger().warning(
                f"ignored unsupported debug image encoding: {message.encoding}"
            )
            return
        self._latest_debug_image = image
        self._latest_debug_image_time = time.monotonic()

    def _camera_image_callback(self, message: Image) -> None:
        image = _image_to_bgr(message)
        if image is None or image.size == 0:
            self.get_logger().warning(
                f"ignored unsupported camera image encoding: {message.encoding}"
            )
            return
        self._latest_camera_image = image
        self._latest_camera_image_time = time.monotonic()

    def _status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {"raw": message.data}
        self._status = payload
        self._status_time = time.monotonic()

    def _cmd_callback(self, message: Twist) -> None:
        self._cmd = message
        self._cmd_time = time.monotonic()

    def _draw(self) -> None:
        now = time.monotonic()
        if not self._window_ready:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            self._window_ready = True

        frame = self._compose(now)
        scale = max(0.2, float(self.get_parameter("display_scale").value))
        if abs(scale - 1.0) > 1.0e-3:
            frame = cv2.resize(
                frame,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
            )
        cv2.imshow(self._window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            self.get_logger().info("debug viewer closed by keyboard")
            rclpy.shutdown()
            return

        console_period = max(0.0, float(self.get_parameter("console_period_s").value))
        if console_period > 0.0 and now - self._last_console_time >= console_period:
            self._last_console_time = now
            self.get_logger().info(self._summary_line(now))

    def _compose(self, now: float) -> np.ndarray:
        mode = str(self.get_parameter("view_mode").value).strip().lower()
        if mode not in {"side_by_side", "camera", "debug"}:
            mode = "side_by_side"

        image_panels: list[np.ndarray] = []
        if mode in {"side_by_side", "camera"}:
            image_panels.append(
                self._image_panel(
                    self._latest_camera_image,
                    self._latest_camera_image_time,
                    "camera",
                    f"waiting for {self._camera_image_topic}",
                    now,
                )
            )
        if mode in {"side_by_side", "debug"}:
            image_panels.append(
                self._image_panel(
                    self._latest_debug_image,
                    self._latest_debug_image_time,
                    "tracking overlay",
                    f"waiting for {self._debug_image_topic}",
                    now,
                )
            )

        base = self._join_image_panels(image_panels)

        panel_width = max(240, int(self.get_parameter("status_panel_width_px").value))
        panel = np.full((base.shape[0], panel_width, 3), (28, 28, 28), dtype=np.uint8)
        _put_lines(panel, self._panel_lines(now), (14, 28), 24)
        return np.concatenate([base, panel], axis=1)

    def _image_panel(
        self,
        image: Optional[np.ndarray],
        image_time: float,
        title: str,
        waiting_text: str,
        now: float,
    ) -> np.ndarray:
        if image is None:
            panel = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                panel,
                waiting_text,
                (32, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 220, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            panel = image.copy()

        age = now - image_time if image_time else float("inf")
        if math.isfinite(age):
            label = f"{title}  age={age:.2f}s"
        else:
            label = title
        color = (0, 230, 90) if age <= 0.5 else (0, 190, 255)
        cv2.rectangle(panel, (0, 0), (min(panel.shape[1], 360), 30), (0, 0, 0), -1)
        cv2.putText(
            panel,
            label,
            (10, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            color,
            1,
            cv2.LINE_AA,
        )
        return panel

    @staticmethod
    def _join_image_panels(image_panels: list[np.ndarray]) -> np.ndarray:
        if not image_panels:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        if len(image_panels) == 1:
            return image_panels[0]

        target_height = max(panel.shape[0] for panel in image_panels)
        resized = []
        for panel in image_panels:
            if panel.shape[0] == target_height:
                resized.append(panel)
                continue
            scale = target_height / max(1, panel.shape[0])
            width = max(1, int(round(panel.shape[1] * scale)))
            resized.append(
                cv2.resize(
                    panel,
                    (width, target_height),
                    interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
                )
            )
        return np.concatenate(resized, axis=1)

    def _panel_lines(self, now: float) -> list[tuple[str, tuple[int, int, int]]]:
        status = self._status
        corner_value = status.get("corner_turn")
        corner = corner_value if isinstance(corner_value, dict) else {}
        state = str(status.get("state", "WAITING"))
        reason = str(status.get("reason", ""))
        valid = bool(status.get("valid", False))
        good = (0, 230, 90) if valid else (0, 190, 255)
        stale = (110, 110, 110)
        debug_age = (
            now - self._latest_debug_image_time
            if self._latest_debug_image_time
            else float("inf")
        )
        camera_age = (
            now - self._latest_camera_image_time
            if self._latest_camera_image_time
            else float("inf")
        )
        status_age = now - self._status_time if self._status_time else float("inf")
        cmd_age = now - self._cmd_time if self._cmd_time else float("inf")
        lines = [
            ("PURE LINE DEBUG", (230, 230, 230)),
            (f"state: {state}", good),
            (f"reason: {reason or '-'}", (210, 210, 210)),
            (f"source: {status.get('lateral_source', '-')}", (210, 210, 210)),
            (
                f"valid/conf: {int(valid)} / "
                f"{_format_float(status.get('confidence'))}",
                good,
            ),
            (
                f"lat err: {_format_float(status.get('lateral_error_m'), 4)} m",
                (180, 230, 255),
            ),
            (
                f"heading: {_format_float(status.get('heading_error_rad'), 4)} rad",
                (180, 230, 255),
            ),
            (
                f"road w: {_format_float(status.get('road_clearance_m'), 4)} m",
                (180, 230, 255),
            ),
            (
                f"branch x: {_format_float(status.get('branch_x_m'), 4)} m",
                (180, 230, 255),
            ),
            (
                "branch L/R: %s / %s"
                % (
                    _format_float(status.get("branch_left_extent_m"), 3),
                    _format_float(status.get("branch_right_extent_m"), 3),
                ),
                (180, 230, 255),
            ),
            (
                f"corner: {corner.get('state', '-')} "
                f"{corner.get('direction', '')}",
                (90, 220, 255),
            ),
            (
                f"candidate: {corner.get('candidate_direction', '-')} "
                f"{corner.get('candidate_frames', 0)}",
                (90, 220, 255),
            ),
            (
                f"camera age: {_format_float(camera_age, 2)} s",
                stale if camera_age > 0.5 else (180, 180, 180),
            ),
            (
                f"debug age: {_format_float(debug_age, 2)} s",
                stale if debug_age > 0.5 else (180, 180, 180),
            ),
            (
                f"status age: {_format_float(status_age, 2)} s",
                stale if status_age > 0.5 else (180, 180, 180),
            ),
        ]
        if self._show_command:
            lines.extend(
                [
                    (
                        f"cmd lin: {self._cmd.linear.x:+.3f} m/s",
                        (120, 255, 140),
                    ),
                    (
                        f"cmd ang: {self._cmd.angular.z:+.3f} rad/s",
                        (120, 255, 140),
                    ),
                    (
                        f"cmd age: {_format_float(cmd_age, 2)} s",
                        stale if cmd_age > 0.5 else (180, 180, 180),
                    ),
                ]
            )
        else:
            lines.append(("cmd: disabled", (160, 160, 160)))
        lines.extend(
            [
                (
                    "mode: recognition only"
                    if status.get("recognition_only")
                    else "mode: line following",
                    (160, 160, 160),
                ),
                ("q/Esc: close viewer", (160, 160, 160)),
            ]
        )
        return lines

    def _summary_line(self, now: float) -> str:
        status = self._status
        corner_value = status.get("corner_turn")
        corner = corner_value if isinstance(corner_value, dict) else {}
        camera_age = (
            now - self._latest_camera_image_time
            if self._latest_camera_image_time
            else float("inf")
        )
        debug_age = (
            now - self._latest_debug_image_time
            if self._latest_debug_image_time
            else float("inf")
        )
        cmd_text = (
            "(%.3f, %.3f)" % (self._cmd.linear.x, self._cmd.angular.z)
            if self._show_command
            else "disabled"
        )
        return (
            "state=%s reason=%s src=%s lat=%s head=%s bx=%s corner=%s/%s "
            "cmd=%s camera_age=%s debug_age=%s"
            % (
                status.get("state", "WAITING"),
                status.get("reason", "-"),
                status.get("lateral_source", "-"),
                _format_float(status.get("lateral_error_m"), 4),
                _format_float(status.get("heading_error_rad"), 4),
                _format_float(status.get("branch_x_m"), 3),
                corner.get("state", "-"),
                corner.get("direction", ""),
                cmd_text,
                _format_float(camera_age, 2),
                _format_float(debug_age, 2),
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PureLineDebugViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
