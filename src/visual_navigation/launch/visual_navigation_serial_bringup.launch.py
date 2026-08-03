from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    visual_navigation_share = FindPackageShare("visual_navigation")

    route_file = LaunchConfiguration("route_file")
    route_frame = LaunchConfiguration("route_frame")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    autostart = LaunchConfiguration("autostart")
    serial_no = LaunchConfiguration("serial_no")
    initial_reset = LaunchConfiguration("initial_reset")
    body_frame_id = LaunchConfiguration("body_frame_id")
    visualization = LaunchConfiguration("visualization")
    use_imu = LaunchConfiguration("use_imu")
    serial_device = LaunchConfiguration("serial_device")
    serial_baud_rate = LaunchConfiguration("serial_baud_rate")
    serial_send_rate_hz = LaunchConfiguration("serial_send_rate_hz")
    serial_command_timeout_s = LaunchConfiguration("serial_command_timeout_s")

    default_route = PathJoinSubstitution(
        [visual_navigation_share, "routes", "example_route.csv"]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    visual_navigation_share,
                    "launch",
                    "visual_navigation_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "route_file": route_file,
            "route_frame": route_frame,
            "cmd_vel_topic": cmd_vel_topic,
            "autostart": autostart,
            "serial_no": serial_no,
            "initial_reset": initial_reset,
            "body_frame_id": body_frame_id,
            "visualization": visualization,
            "use_imu": use_imu,
        }.items(),
    )

    serial_bridge = Node(
        package="cup_car_serial",
        executable="cmd_vel_serial_node",
        name="cmd_vel_serial_node",
        output="screen",
        parameters=[
            {
                "device": ParameterValue(serial_device, value_type=str),
                "baud_rate": ParameterValue(serial_baud_rate, value_type=int),
                "topic": ParameterValue(cmd_vel_topic, value_type=str),
                "send_rate_hz": ParameterValue(
                    serial_send_rate_hz, value_type=float
                ),
                "command_timeout_s": ParameterValue(
                    serial_command_timeout_s, value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("route_file", default_value=default_route),
            DeclareLaunchArgument("route_frame", default_value="map"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("serial_no", default_value="_038122250473"),
            DeclareLaunchArgument("initial_reset", default_value="false"),
            DeclareLaunchArgument("body_frame_id", default_value="camera_link"),
            DeclareLaunchArgument("visualization", default_value="false"),
            DeclareLaunchArgument("use_imu", default_value="true"),
            DeclareLaunchArgument("serial_device", default_value="auto"),
            DeclareLaunchArgument("serial_baud_rate", default_value="115200"),
            DeclareLaunchArgument("serial_send_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("serial_command_timeout_s", default_value="0.4"),
            navigation,
            serial_bridge,
        ]
    )
