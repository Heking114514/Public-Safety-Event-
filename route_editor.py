#!/usr/bin/env python3

import math
import time
import tkinter as tk
from tkinter import messagebox

import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_srvs.srv import Trigger


GRID_METERS = 0.6
PIXELS_PER_METER = 100.0
POINT_HIT_RADIUS = 13.0


def quaternion_to_yaw(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        return None
    squared_norm = sum(value * value for value in values)
    if squared_norm <= 1e-12 or not math.isfinite(squared_norm):
        return None
    x, y, z, w = (value / math.sqrt(squared_norm) for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def path_fingerprint(path):
    result = []
    for stamped_pose in path.poses:
        yaw = quaternion_to_yaw(stamped_pose.pose.orientation)
        if yaw is None:
            return None
        result.append(
            (
                round(stamped_pose.pose.position.x, 6),
                round(stamped_pose.pose.position.y, 6),
                round(yaw, 6),
            )
        )
    return tuple(result)


class RouteEditorNode(Node):
    def __init__(self, odom_callback):
        super().__init__("waypoint_route_editor")
        self.declare_parameter("odom_topic", "/odometry/fused")
        self.declare_parameter(
            "route_input_topic", "/waypoint_navigation/route_input"
        )
        self.declare_parameter("route_feedback_topic", "/waypoint_path")
        self.declare_parameter("start_service", "/waypoint_navigator/start")
        self.declare_parameter("route_frame", "map")
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("activation_timeout", 3.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.route_input_topic = self.get_parameter("route_input_topic").value
        self.route_feedback_topic = self.get_parameter("route_feedback_topic").value
        self.start_service_name = self.get_parameter("start_service").value
        self.route_frame = self.get_parameter("route_frame").value
        self.odom_timeout = max(0.05, float(self.get_parameter("odom_timeout").value))
        self.activation_timeout = max(
            0.5, float(self.get_parameter("activation_timeout").value)
        )
        self._odom_callback = odom_callback
        self._activation_phase = "IDLE"
        self._activation_deadline = 0.0
        self._pending_fingerprint = None
        self._pending_stamp_ns = 0
        self._start_future = None
        self._activation_result = None

        route_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.route_publisher = self.create_publisher(
            Path, self.route_input_topic, route_qos
        )
        self.route_feedback_subscription = self.create_subscription(
            Path, self.route_feedback_topic, self._handle_route_feedback, route_qos
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self._handle_odom, qos_profile_sensor_data
        )
        self.start_client = self.create_client(Trigger, self.start_service_name)

    def _handle_odom(self, message):
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        yaw = quaternion_to_yaw(message.pose.pose.orientation)
        frame = message.header.frame_id
        valid_frame = not frame or frame == self.route_frame
        if not math.isfinite(x) or not math.isfinite(y) or yaw is None or not valid_frame:
            self._odom_callback(None)
            return
        self._odom_callback((x, y, yaw, time.monotonic()))

    @property
    def activation_busy(self):
        return self._activation_phase not in ("IDLE", "SUCCEEDED", "FAILED")

    def publish_and_activate(self, waypoints):
        if self.activation_busy:
            raise RuntimeError("路线正在启用，请等待当前操作结束")
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.route_frame
        for x, y, yaw in waypoints:
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError("route contains a non-finite waypoint")
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            path.poses.append(pose)

        self._pending_fingerprint = path_fingerprint(path)
        self._pending_stamp_ns = (
            path.header.stamp.sec * 1_000_000_000 + path.header.stamp.nanosec
        )
        self._activation_deadline = time.monotonic() + self.activation_timeout
        self._activation_phase = "WAIT_ACK"
        self._activation_result = None
        self.route_publisher.publish(path)

    def _handle_route_feedback(self, path):
        if self._activation_phase != "WAIT_ACK":
            return
        stamp_ns = path.header.stamp.sec * 1_000_000_000 + path.header.stamp.nanosec
        if stamp_ns < self._pending_stamp_ns:
            return
        if path.header.frame_id and path.header.frame_id != self.route_frame:
            return
        if path_fingerprint(path) != self._pending_fingerprint:
            return
        self._activation_phase = "WAIT_SERVICE"

    def poll_activation(self):
        if not self.activation_busy:
            return
        if time.monotonic() > self._activation_deadline:
            self._finish_activation(False, "导航器未及时确认并启动路线")
            return
        if self._activation_phase == "WAIT_SERVICE":
            if not self.start_client.service_is_ready():
                return
            self._start_future = self.start_client.call_async(Trigger.Request())
            self._activation_phase = "WAIT_RESPONSE"
            return
        if self._activation_phase == "WAIT_RESPONSE" and self._start_future.done():
            try:
                response = self._start_future.result()
            except Exception as exception:
                self._finish_activation(False, f"启动服务调用失败: {exception}")
                return
            self._finish_activation(bool(response.success), response.message)

    def _finish_activation(self, success, message):
        self._activation_phase = "SUCCEEDED" if success else "FAILED"
        self._activation_result = (success, message)
        self._pending_fingerprint = None
        self._start_future = None

    def take_activation_result(self):
        result = self._activation_result
        self._activation_result = None
        return result


class RouteEditorWindow:
    def __init__(self, root):
        self.root = root
        self.root.title("Waypoint Route Editor")
        self.root.geometry("980x720")
        self.root.minsize(640, 480)

        self.waypoints = []
        self.current_pose = None
        self.drag_index = None
        self.odom_ready = False

        toolbar = tk.Frame(root, padx=10, pady=8, bg="#f3f4f6")
        toolbar.pack(fill=tk.X)
        self.odom_label = tk.Label(
            toolbar,
            text="NO ODOM",
            fg="#991b1b",
            bg="#f3f4f6",
            font=("TkDefaultFont", 11, "bold"),
        )
        self.odom_label.pack(side=tk.LEFT)
        self.publish_label = tk.Label(
            toolbar, text="未发布", fg="#4b5563", bg="#f3f4f6"
        )
        self.publish_label.pack(side=tk.LEFT, padx=20)
        self.publish_button = tk.Button(
            toolbar,
            text="发布并启用路线",
            command=self.publish_route,
            state=tk.DISABLED,
            padx=14,
            pady=4,
        )
        self.publish_button.pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(root, bg="#ffffff", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self.on_left_press)
        self.canvas.bind("<B1-Motion>", self.on_left_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_left_release)
        self.canvas.bind("<Button-3>", self.on_right_click)

        self.node = RouteEditorNode(self.on_odometry)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(20, self.spin_ros)
        self.root.after(100, self.refresh_status)

    @property
    def origin(self):
        return self.canvas.winfo_width() * 0.5, self.canvas.winfo_height() * 0.5

    def world_to_canvas(self, x, y):
        origin_x, origin_y = self.origin
        return origin_x + x * PIXELS_PER_METER, origin_y - y * PIXELS_PER_METER

    def canvas_to_world(self, canvas_x, canvas_y):
        origin_x, origin_y = self.origin
        return (
            (canvas_x - origin_x) / PIXELS_PER_METER,
            (origin_y - canvas_y) / PIXELS_PER_METER,
        )

    def spin_ros(self):
        if not rclpy.ok():
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self.spin_ros)

    def on_odometry(self, pose):
        self.current_pose = pose
        self.redraw()

    def refresh_status(self):
        self.node.poll_activation()
        activation_result = self.node.take_activation_result()
        if activation_result is not None:
            success, message = activation_result
            if success:
                self.publish_label.configure(text="路线已启用", fg="#166534")
            else:
                self.publish_label.configure(text="启用失败", fg="#991b1b")
                messagebox.showerror("路线启用失败", message)
            self.update_publish_button()

        ready = (
            self.current_pose is not None
            and time.monotonic() - self.current_pose[3] <= self.node.odom_timeout
        )
        if ready != self.odom_ready:
            self.odom_ready = ready
            self.odom_label.configure(
                text="ODOM READY" if ready else "NO ODOM",
                fg="#166534" if ready else "#991b1b",
            )
            self.update_publish_button()
            self.redraw()
        self.root.after(100, self.refresh_status)

    def update_publish_button(self):
        state = (
            tk.NORMAL
            if self.odom_ready and self.waypoints and not self.node.activation_busy
            else tk.DISABLED
        )
        self.publish_button.configure(state=state)

    def nearest_waypoint(self, canvas_x, canvas_y):
        best_index = None
        best_distance = POINT_HIT_RADIUS
        for index, (x, y, _yaw) in enumerate(self.waypoints):
            point_x, point_y = self.world_to_canvas(x, y)
            distance = math.hypot(canvas_x - point_x, canvas_y - point_y)
            if distance <= best_distance:
                best_index = index
                best_distance = distance
        return best_index

    def on_left_press(self, event):
        if self.node.activation_busy:
            return
        self.drag_index = self.nearest_waypoint(event.x, event.y)
        if self.drag_index is None:
            x, y = self.canvas_to_world(event.x, event.y)
            self.waypoints.append([x, y, 0.0])
            self.drag_index = len(self.waypoints) - 1
            self.publish_label.configure(text="未发布", fg="#4b5563")
            self.update_publish_button()
            self.redraw()

    def on_left_drag(self, event):
        if self.drag_index is None or self.node.activation_busy:
            return
        point = self.waypoints[self.drag_index]
        cursor_x, cursor_y = self.canvas_to_world(event.x, event.y)
        delta_x = cursor_x - point[0]
        delta_y = cursor_y - point[1]
        if math.hypot(delta_x, delta_y) > 0.03:
            point[2] = math.atan2(delta_y, delta_x)
            self.publish_label.configure(text="未发布", fg="#4b5563")
            self.redraw()

    def on_left_release(self, _event):
        self.drag_index = None

    def on_right_click(self, _event):
        if self.waypoints and not self.node.activation_busy:
            self.waypoints.pop()
            self.drag_index = None
            self.publish_label.configure(text="未发布", fg="#4b5563")
            self.update_publish_button()
            self.redraw()

    def publish_route(self):
        if not self.waypoints:
            messagebox.showwarning("路线为空", "至少添加一个航点后才能发布。")
            return
        if not self.odom_ready:
            messagebox.showwarning("NO ODOM", "里程计就绪后才能启用路线。")
            return
        try:
            self.node.publish_and_activate(self.waypoints)
        except (RuntimeError, ValueError) as exception:
            messagebox.showerror("路线无效", str(exception))
            return
        self.publish_label.configure(
            text=f"正在启用 {len(self.waypoints)} 点...", fg="#92400e"
        )
        self.update_publish_button()

    def draw_arrow(self, x, y, yaw, color, width=3, length=34):
        start_x, start_y = self.world_to_canvas(x, y)
        end_x = start_x + length * math.cos(yaw)
        end_y = start_y - length * math.sin(yaw)
        self.canvas.create_line(
            start_x,
            start_y,
            end_x,
            end_y,
            fill=color,
            width=width,
            arrow=tk.LAST,
            arrowshape=(10, 12, 5),
        )

    def redraw(self):
        self.canvas.delete("all")
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        if width <= 1 or height <= 1:
            return
        origin_x, origin_y = self.origin
        grid_pixels = GRID_METERS * PIXELS_PER_METER

        offset = origin_x % grid_pixels
        x = offset
        while x < width:
            self.canvas.create_line(x, 0, x, height, fill="#e5e7eb")
            x += grid_pixels
        offset = origin_y % grid_pixels
        y = offset
        while y < height:
            self.canvas.create_line(0, y, width, y, fill="#e5e7eb")
            y += grid_pixels

        self.canvas.create_line(0, origin_y, width, origin_y, fill="#374151", width=2)
        self.canvas.create_line(origin_x, height, origin_x, 0, fill="#374151", width=2)
        self.canvas.create_text(width - 14, origin_y - 14, text="+X", fill="#111827")
        self.canvas.create_text(origin_x + 16, 12, text="+Y", fill="#111827")
        self.draw_arrow(0.0, 0.0, 0.0, "#111827", width=2, length=28)

        if len(self.waypoints) > 1:
            route_points = []
            for x, y, _yaw in self.waypoints:
                route_points.extend(self.world_to_canvas(x, y))
            self.canvas.create_line(*route_points, fill="#2563eb", width=2)

        for index, (x, y, yaw) in enumerate(self.waypoints):
            point_x, point_y = self.world_to_canvas(x, y)
            self.canvas.create_oval(
                point_x - 7,
                point_y - 7,
                point_x + 7,
                point_y + 7,
                fill="#2563eb",
                outline="#ffffff",
                width=2,
            )
            self.canvas.create_text(
                point_x + 12,
                point_y + 12,
                text=str(index + 1),
                fill="#1e3a8a",
                anchor=tk.NW,
            )
            self.draw_arrow(x, y, yaw, "#1d4ed8", width=2, length=27)

        if self.odom_ready and self.current_pose is not None:
            x, y, yaw, _arrival = self.current_pose
            point_x, point_y = self.world_to_canvas(x, y)
            self.canvas.create_oval(
                point_x - 9,
                point_y - 9,
                point_x + 9,
                point_y + 9,
                fill="#16a34a",
                outline="#ffffff",
                width=2,
            )
            self.draw_arrow(x, y, yaw, "#15803d", width=4, length=42)

    def close(self):
        if rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    RouteEditorWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
