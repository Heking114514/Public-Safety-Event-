#!/usr/bin/env python3

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Continuously publish a test Twist command like the waypoint navigator."
    )
    parser.add_argument("--topic", default="/cmd_vel_nav")
    parser.add_argument("--linear", type=float, default=0.0)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to publish; 0 means run until Ctrl+C.",
    )
    parser.add_argument(
        "--enable-output",
        action="store_true",
        help="Required before publishing a non-zero command.",
    )
    arguments = parser.parse_args()

    if arguments.rate <= 0.0:
        parser.error("--rate must be positive")
    if arguments.duration < 0.0:
        parser.error("--duration cannot be negative")
    if (arguments.linear != 0.0 or arguments.angular != 0.0) and not arguments.enable_output:
        parser.error("add --enable-output to publish a non-zero command")

    return arguments


def make_twist(linear_x: float, angular_z: float) -> Twist:
    message = Twist()
    message.linear.x = linear_x
    message.angular.z = angular_z
    return message


def main():
    arguments = parse_arguments()
    rclpy.init()
    node = rclpy.create_node("cmd_vel_nav_test_publisher")
    publisher = node.create_publisher(Twist, arguments.topic, 10)
    command = make_twist(arguments.linear, arguments.angular)
    stop_command = make_twist(0.0, 0.0)
    period = 1.0 / arguments.rate
    start_time = time.monotonic()

    node.get_logger().info(
        f"Publishing {arguments.topic}: linear.x={arguments.linear:.3f} m/s, "
        f"angular.z={arguments.angular:.3f} rad/s at {arguments.rate:.1f} Hz"
    )
    if arguments.duration == 0.0:
        node.get_logger().info("Press Ctrl+C to stop and send zero velocity")

    try:
        while rclpy.ok():
            if arguments.duration > 0.0 and time.monotonic() - start_time >= arguments.duration:
                break
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Sending zero velocity before exit")
        for _ in range(5):
            publisher.publish(stop_command)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
