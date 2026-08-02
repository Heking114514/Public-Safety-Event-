from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device = LaunchConfiguration("device")
    baud_rate = LaunchConfiguration("baud_rate")
    topic = LaunchConfiguration("topic")
    send_rate_hz = LaunchConfiguration("send_rate_hz")
    command_timeout_s = LaunchConfiguration("command_timeout_s")

    serial_node = Node(
        package="cup_car_serial",
        executable="cmd_vel_serial_node",
        name="cmd_vel_serial_node",
        output="screen",
        parameters=[
            {
                "device": ParameterValue(device, value_type=str),
                "baud_rate": ParameterValue(baud_rate, value_type=int),
                "topic": ParameterValue(topic, value_type=str),
                "send_rate_hz": ParameterValue(send_rate_hz, value_type=float),
                "command_timeout_s": ParameterValue(
                    command_timeout_s, value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("send_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.4"),
            serial_node,
        ]
    )
