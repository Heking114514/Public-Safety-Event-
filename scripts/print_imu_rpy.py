#!/usr/bin/env python3

import argparse
from collections import deque
import math
from statistics import median
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def optical_to_body(x, y, z):
    # ROS optical (right, down, forward) -> camera_link (forward, left, up).
    return z, -x, -y


def quaternion_yaw(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class VectorMedianFilter:
    def __init__(self, window_size):
        self.samples = deque(maxlen=window_size)

    def update(self, sample):
        self.samples.append(sample)
        return tuple(median(axis) for axis in zip(*self.samples))


class VectorMeanFilter:
    def __init__(self, window_size):
        self.samples = deque(maxlen=window_size)

    def update(self, sample):
        self.samples.append(sample)
        sample_count = len(self.samples)
        return tuple(sum(axis) / sample_count for axis in zip(*self.samples))


class StationaryDetector:
    def __init__(self, dwell_time=0.5):
        self.dwell_time = dwell_time
        self.candidate_since = None
        self.active = False

    def update(self, stamp, acceleration_error, gyro_norm):
        if self.active:
            if acceleration_error > 0.35 or gyro_norm > 0.05:
                self.active = False
                self.candidate_since = None
            return self.active

        if acceleration_error <= 0.20 and gyro_norm <= 0.02:
            if self.candidate_since is None:
                self.candidate_since = stamp
            elif stamp - self.candidate_since >= self.dwell_time:
                self.active = True
        else:
            self.candidate_since = None
        return self.active


class YawBiasKalman:
    def __init__(self):
        self.yaw = 0.0
        self.bias = 0.0
        self.p00 = math.radians(5.0) ** 2
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = math.radians(1.0) ** 2
        self.gyro_noise = math.radians(0.6)
        self.bias_walk = math.radians(0.03)

    def predict(self, measured_yaw_rate, dt):
        self.yaw = wrap_angle(self.yaw + (measured_yaw_rate - self.bias) * dt)

        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11
        self.p00 = (
            old_p00
            - dt * (old_p01 + old_p10)
            + dt * dt * old_p11
            + (self.gyro_noise * dt) ** 2
        )
        self.p01 = old_p01 - dt * old_p11
        self.p10 = old_p10 - dt * old_p11
        self.p11 = old_p11 + self.bias_walk * self.bias_walk * dt

    def update(self, measured_yaw, measurement_std):
        innovation = wrap_angle(measured_yaw - self.yaw)
        innovation_covariance = self.p00 + measurement_std * measurement_std
        gain_yaw = self.p00 / innovation_covariance
        gain_bias = self.p10 / innovation_covariance

        self.yaw = wrap_angle(self.yaw + gain_yaw * innovation)
        self.bias += gain_bias * innovation
        self.bias = max(math.radians(-5.0), min(math.radians(5.0), self.bias))

        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11
        self.p00 = max((1.0 - gain_yaw) * old_p00, 1e-12)
        self.p01 = (1.0 - gain_yaw) * old_p01
        self.p10 = old_p10 - gain_bias * old_p00
        self.p11 = max(old_p11 - gain_bias * old_p01, 1e-12)
        off_diagonal = 0.5 * (self.p01 + self.p10)
        self.p01 = off_diagonal
        self.p10 = off_diagonal


class ImuRpyPrinter(Node):
    def __init__(
        self,
        topic,
        output_rate,
        correction_time,
        low_pass_cutoff,
        mean_window,
        yaw_reference_topic,
        tracking_topic,
        use_visual_yaw,
    ):
        super().__init__("imu_rpy_printer")
        self.output_period = 1.0 / output_rate
        self.correction_time = correction_time
        self.low_pass_time_constant = (
            1.0 / (2.0 * math.pi * low_pass_cutoff)
            if low_pass_cutoff > 0.0
            else 0.0
        )
        self.last_stamp = None
        self.last_output = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_filter = YawBiasKalman()
        self.initialized = False
        self.stationary_anchor = None
        self.last_stationary_update = None
        self.latest_reference_yaw = None
        self.reference_sequence = 0
        self.applied_reference_sequence = 0
        self.reference_offset = None
        self.rejected_reference_count = 0
        self.reference_received_at = None
        self.tracking_state = None
        self.tracking_received_at = None
        self.correction_source = "gyro"
        self.stationary_detector = StationaryDetector()
        self.gyro_median = VectorMedianFilter(5)
        self.accel_median = VectorMedianFilter(5)
        self.gyro_mean = VectorMeanFilter(mean_window)
        self.accel_mean = VectorMeanFilter(mean_window)
        self.filtered_gyro = None
        self.filtered_accel = None
        self.imu_subscription = self.create_subscription(
            Imu, topic, self.imu_callback, qos_profile_sensor_data
        )
        if use_visual_yaw:
            self.odometry_subscription = self.create_subscription(
                Odometry,
                yaw_reference_topic,
                self.odometry_callback,
                qos_profile_sensor_data,
            )
            self.tracking_subscription = self.create_subscription(
                Int32,
                tracking_topic,
                self.tracking_callback,
                qos_profile_sensor_data,
            )

    def odometry_callback(self, message):
        self.latest_reference_yaw = quaternion_yaw(message.pose.pose.orientation)
        self.reference_sequence += 1
        self.reference_received_at = time.monotonic()

    def tracking_callback(self, message):
        self.tracking_state = message.data
        self.tracking_received_at = time.monotonic()

    def imu_callback(self, message):
        gx, gy, gz = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        ax, ay, az = (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )

        if message.header.frame_id.endswith("_optical_frame"):
            gx, gy, gz = optical_to_body(gx, gy, gz)
            ax, ay, az = optical_to_body(ax, ay, az)

        gx, gy, gz = self.gyro_median.update((gx, gy, gz))
        ax, ay, az = self.accel_median.update((ax, ay, az))
        gx, gy, gz = self.gyro_mean.update((gx, gy, gz))
        ax, ay, az = self.accel_mean.update((ax, ay, az))

        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

        if not self.initialized:
            self.filtered_gyro = (gx, gy, gz)
            self.filtered_accel = (ax, ay, az)
            roll_acc = math.atan2(ay, az)
            pitch_acc = math.atan2(-ax, math.hypot(ay, az))
            self.roll = roll_acc
            self.pitch = pitch_acc
            self.last_stamp = stamp
            self.initialized = True
            return

        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 0.0 or dt > 0.1:
            return

        low_pass_weight = (
            dt / (self.low_pass_time_constant + dt)
            if self.low_pass_time_constant > 0.0
            else 1.0
        )
        self.filtered_gyro = tuple(
            previous + low_pass_weight * (current - previous)
            for previous, current in zip(self.filtered_gyro, (gx, gy, gz))
        )
        self.filtered_accel = tuple(
            previous + low_pass_weight * (current - previous)
            for previous, current in zip(self.filtered_accel, (ax, ay, az))
        )
        gx, gy, gz = self.filtered_gyro
        ax, ay, az = self.filtered_accel

        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.hypot(ay, az))

        cos_pitch = math.cos(self.pitch)
        if abs(cos_pitch) < 1e-3:
            cos_pitch = math.copysign(1e-3, cos_pitch)

        tan_pitch = math.sin(self.pitch) / cos_pitch
        roll_rate = gx + math.sin(self.roll) * tan_pitch * gy + math.cos(
            self.roll
        ) * tan_pitch * gz
        pitch_rate = math.cos(self.roll) * gy - math.sin(self.roll) * gz
        yaw_rate = (
            math.sin(self.roll) / cos_pitch * gy
            + math.cos(self.roll) / cos_pitch * gz
        )

        self.roll = wrap_angle(self.roll + roll_rate * dt)
        self.pitch = wrap_angle(self.pitch + pitch_rate * dt)
        self.yaw_filter.predict(yaw_rate, dt)

        acceleration_error = abs(accel_norm - 9.80665)
        acceleration_confidence = max(
            0.0, min(1.0, (0.75 - acceleration_error) / 0.5)
        )
        if acceleration_confidence > 0.0:
            gyro_weight = self.correction_time / (self.correction_time + dt)
            accel_weight = (1.0 - gyro_weight) * acceleration_confidence
            self.roll = wrap_angle(
                self.roll + accel_weight * wrap_angle(roll_acc - self.roll)
            )
            self.pitch = wrap_angle(
                self.pitch + accel_weight * wrap_angle(pitch_acc - self.pitch)
            )

        corrected_yaw_rate = yaw_rate - self.yaw_filter.bias
        gyro_norm = math.sqrt(
            roll_rate * roll_rate
            + pitch_rate * pitch_rate
            + corrected_yaw_rate * corrected_yaw_rate
        )
        stationary = self.stationary_detector.update(
            stamp, acceleration_error, gyro_norm
        )
        if stationary:
            if self.stationary_anchor is None:
                self.stationary_anchor = self.yaw_filter.yaw
                self.last_stationary_update = stamp
            elif stamp - self.last_stationary_update >= 0.05:
                self.yaw_filter.update(
                    self.stationary_anchor, math.radians(0.5)
                )
                self.last_stationary_update = stamp
                self.correction_source = "stationary"
        elif self.stationary_detector.candidate_since is not None:
            if self.stationary_anchor is None:
                self.stationary_anchor = self.yaw_filter.yaw
                self.last_stationary_update = stamp
            self.correction_source = "gyro"
        else:
            self.stationary_anchor = None
            self.last_stationary_update = None
            self.correction_source = "gyro"

        has_new_reference = (
            self.latest_reference_yaw is not None
            and self.reference_sequence != self.applied_reference_sequence
        )
        now = time.monotonic()
        reference_is_fresh = (
            self.reference_received_at is not None
            and now - self.reference_received_at <= 0.2
        )
        tracking_is_fresh = (
            self.tracking_received_at is not None
            and now - self.tracking_received_at <= 0.5
        )
        if (
            self.tracking_state == 2
            and tracking_is_fresh
            and reference_is_fresh
            and has_new_reference
        ):
            if self.reference_offset is None:
                self.reference_offset = wrap_angle(
                    self.latest_reference_yaw - self.yaw_filter.yaw
                )
            relative_reference = wrap_angle(
                self.latest_reference_yaw - self.reference_offset
            )
            innovation = wrap_angle(relative_reference - self.yaw_filter.yaw)
            if abs(innovation) <= math.radians(15.0):
                self.yaw_filter.update(relative_reference, math.radians(2.0))
                self.rejected_reference_count = 0
                self.correction_source = "visual"
            else:
                self.rejected_reference_count += 1
                self.correction_source = "visual_rej"
                if self.rejected_reference_count >= 10:
                    self.reference_offset = wrap_angle(
                        self.latest_reference_yaw - self.yaw_filter.yaw
                    )
                    self.rejected_reference_count = 0
                    self.correction_source = "visual_base"
            self.applied_reference_sequence = self.reference_sequence

        if now - self.last_output < self.output_period:
            return
        self.last_output = now

        scale = 180.0 / math.pi
        print(
            f"RPY [deg]  roll={self.roll * scale:8.3f}  "
            f"pitch={self.pitch * scale:8.3f}  "
            f"yaw={self.yaw_filter.yaw * scale:8.3f}  "
            f"bias={self.yaw_filter.bias * scale:7.3f} deg/s  "
            f"source={self.correction_source:10s}  "
            f"|a|={accel_norm:6.3f} m/s^2",
            flush=True,
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Print RPY with Kalman-filtered IMU/visual yaw."
    )
    parser.add_argument("--topic", default="/camera/camera/imu")
    parser.add_argument("--rate", type=float, default=10.0, help="print rate in Hz")
    parser.add_argument(
        "--correction-time",
        type=float,
        default=0.5,
        help="accelerometer tilt-correction time constant in seconds",
    )
    parser.add_argument(
        "--low-pass-cutoff",
        type=float,
        default=5.0,
        help="IMU low-pass cutoff frequency in Hz; 0 disables it",
    )
    parser.add_argument(
        "--mean-window",
        type=int,
        default=5,
        help="moving-average window in samples; 1 disables it",
    )
    parser.add_argument("--yaw-reference-topic", default="/odom")
    parser.add_argument("--tracking-topic", default="/tracking_state")
    parser.add_argument(
        "--no-visual-yaw",
        action="store_true",
        help="disable odometry yaw corrections and use stationary updates only",
    )
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.correction_time <= 0.0:
        parser.error("--correction-time must be positive")
    if args.low_pass_cutoff < 0.0:
        parser.error("--low-pass-cutoff cannot be negative")
    if args.mean_window < 1:
        parser.error("--mean-window must be at least 1")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = ImuRpyPrinter(
        args.topic,
        args.rate,
        args.correction_time,
        args.low_pass_cutoff,
        args.mean_window,
        args.yaw_reference_topic,
        args.tracking_topic,
        not args.no_visual_yaw,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
