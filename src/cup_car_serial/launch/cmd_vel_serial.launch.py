from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device = LaunchConfiguration("device")
    baud_rate = LaunchConfiguration("baud_rate")
    topic = LaunchConfiguration("topic")
    rpy_topic = LaunchConfiguration("rpy_topic")
    rfid_topic = LaunchConfiguration("rfid_topic")
    rfid_number_topic = LaunchConfiguration("rfid_number_topic")
    rfid_uid_topic = LaunchConfiguration("rfid_uid_topic")
    speak_text_service = LaunchConfiguration("speak_text_service")
    send_rate_hz = LaunchConfiguration("send_rate_hz")
    command_timeout_s = LaunchConfiguration("command_timeout_s")
    rpy_timeout_s = LaunchConfiguration("rpy_timeout_s")
    control_telemetry_timeout_s = LaunchConfiguration(
        "control_telemetry_timeout_s"
    )
    config_file = LaunchConfiguration("config_file")

    serial_node = Node(
        package="cup_car_serial",
        executable="cmd_vel_serial_node",
        name="cmd_vel_serial_node",
        output="screen",
        parameters=[
            config_file,
            {
                "device": ParameterValue(device, value_type=str),
                "baud_rate": ParameterValue(baud_rate, value_type=int),
                "topic": ParameterValue(topic, value_type=str),
                "rpy_topic": ParameterValue(rpy_topic, value_type=str),
                "rfid_topic": ParameterValue(rfid_topic, value_type=str),
                "rfid_number_topic": ParameterValue(
                    rfid_number_topic, value_type=str
                ),
                "rfid_uid_topic": ParameterValue(rfid_uid_topic, value_type=str),
                "speak_text_service": ParameterValue(
                    speak_text_service, value_type=str
                ),
                "speech_protocol": ParameterValue(
                    LaunchConfiguration("speech_protocol"), value_type=str
                ),
                "send_rate_hz": ParameterValue(send_rate_hz, value_type=float),
                "command_timeout_s": ParameterValue(
                    command_timeout_s, value_type=float
                ),
                "rpy_timeout_s": ParameterValue(rpy_timeout_s, value_type=float),
                "control_telemetry_timeout_s": ParameterValue(
                    control_telemetry_timeout_s, value_type=float
                ),
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("rpy_topic", default_value="/imu/rpy"),
            DeclareLaunchArgument("rfid_topic", default_value="/cup_car_serial/rfid"),
            DeclareLaunchArgument(
                "rfid_number_topic", default_value="/cup_car_serial/rfid_number"
            ),
            DeclareLaunchArgument(
                "rfid_uid_topic", default_value="/cup_car_serial/rfid_uid"
            ),
            DeclareLaunchArgument(
                "speak_text_service", default_value="/cup_car_serial/speak_text"
            ),
            DeclareLaunchArgument("speech_protocol", default_value="legacy"),
            DeclareLaunchArgument("send_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.4"),
            DeclareLaunchArgument("rpy_timeout_s", default_value="0.4"),
            DeclareLaunchArgument(
                "control_telemetry_timeout_s", default_value="0.90"
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("cup_car_serial"), "config", "cmd_vel_serial.yaml"]
                ),
            ),
            serial_node,
        ]
    )
