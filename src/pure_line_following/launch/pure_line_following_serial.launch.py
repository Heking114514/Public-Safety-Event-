from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vehicle_config import parameters_for


def generate_launch_description():
    package_share = Path(get_package_share_directory("pure_line_following"))
    serial_share = Path(get_package_share_directory("cup_car_serial"))
    default_parameters = str(package_share / "config" / "pure_line_following.yaml")
    default_serial_parameters = str(serial_share / "config" / "cmd_vel_serial.yaml")
    physical_parameters = parameters_for("pure_line_following")

    parameters_file = LaunchConfiguration("parameters_file")
    serial_parameters_file = LaunchConfiguration("serial_parameters_file")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    enable_depth = LaunchConfiguration("enable_depth")
    align_depth = LaunchConfiguration("align_depth")

    camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
        parameters=[
            {
                "serial_no": ParameterValue(
                    LaunchConfiguration("camera_serial"),
                    value_type=str,
                ),
                "enable_color": True,
                "rgb_camera.color_profile": ParameterValue(
                    LaunchConfiguration("color_profile"),
                    value_type=str,
                ),
                "rgb_camera.color_format": "RGB8",
                "enable_depth": ParameterValue(enable_depth, value_type=bool),
                "depth_module.depth_profile": ParameterValue(
                    LaunchConfiguration("depth_profile"),
                    value_type=str,
                ),
                "depth_module.depth_format": "Z16",
                "align_depth.enable": ParameterValue(
                    align_depth,
                    value_type=bool,
                ),
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_gyro": False,
                "enable_accel": False,
                "enable_sync": True,
                "publish_tf": False,
            }
        ],
    )

    pure_line_node = Node(
        package="pure_line_following",
        executable="pure_line_following_node",
        name="pure_line_following",
        output="screen",
        parameters=[
            parameters_file,
            physical_parameters,
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                "cmd_vel_topic": cmd_vel_topic,
                "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                "publish_debug_image": ParameterValue(
                    LaunchConfiguration("publish_debug_image"),
                    value_type=bool,
                ),
                "require_camera_info": ParameterValue(
                    LaunchConfiguration("require_camera_info"),
                    value_type=bool,
                ),
                "require_depth": ParameterValue(
                    LaunchConfiguration("require_depth"),
                    value_type=bool,
                ),
                "use_depth_projection": ParameterValue(
                    LaunchConfiguration("use_depth_projection"),
                    value_type=bool,
                ),
                "image_is_rectified": ParameterValue(
                    LaunchConfiguration("image_is_rectified"),
                    value_type=bool,
                ),
                "enabled": ParameterValue(
                    LaunchConfiguration("enabled"),
                    value_type=bool,
                ),
            },
        ],
    )

    serial_node = Node(
        package="cup_car_serial",
        executable="cmd_vel_serial_node",
        name="cmd_vel_serial_node",
        output="screen",
        parameters=[
            serial_parameters_file,
            {
                "device": ParameterValue(
                    LaunchConfiguration("serial_device"),
                    value_type=str,
                ),
                "baud_rate": ParameterValue(
                    LaunchConfiguration("serial_baud_rate"),
                    value_type=int,
                ),
                "topic": ParameterValue(cmd_vel_topic, value_type=str),
                "rpy_topic": ParameterValue(
                    LaunchConfiguration("rpy_topic"),
                    value_type=str,
                ),
                "send_rate_hz": ParameterValue(
                    LaunchConfiguration("send_rate_hz"),
                    value_type=float,
                ),
                "command_timeout_s": ParameterValue(
                    LaunchConfiguration("command_timeout_s"),
                    value_type=float,
                ),
                "rpy_timeout_s": ParameterValue(
                    LaunchConfiguration("rpy_timeout_s"),
                    value_type=float,
                ),
                "control_telemetry_timeout_s": ParameterValue(
                    LaunchConfiguration("control_telemetry_timeout_s"),
                    value_type=float,
                ),
                "allow_generic_auto_device": ParameterValue(
                    LaunchConfiguration("allow_generic_auto_device"),
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
            ),
            DeclareLaunchArgument(
                "serial_parameters_file",
                default_value=default_serial_parameters,
            ),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument(
                "camera_serial",
                default_value="_038122250473",
            ),
            DeclareLaunchArgument("color_profile", default_value="640x480x15"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("align_depth", default_value="true"),
            DeclareLaunchArgument("depth_profile", default_value="640x480x15"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel_line_following",
            ),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value="/pure_line_following/debug_image",
            ),
            DeclareLaunchArgument("publish_debug_image", default_value="false"),
            DeclareLaunchArgument("show_debug_gui", default_value="false"),
            DeclareLaunchArgument("viewer_mode", default_value="side_by_side"),
            DeclareLaunchArgument("require_camera_info", default_value="true"),
            DeclareLaunchArgument("require_depth", default_value="false"),
            DeclareLaunchArgument("use_depth_projection", default_value="true"),
            DeclareLaunchArgument("image_is_rectified", default_value="false"),
            DeclareLaunchArgument("enabled", default_value="true"),
            DeclareLaunchArgument("serial_device", default_value="auto"),
            DeclareLaunchArgument("serial_baud_rate", default_value="115200"),
            DeclareLaunchArgument("allow_generic_auto_device", default_value="false"),
            DeclareLaunchArgument("rpy_topic", default_value="/imu/rpy"),
            DeclareLaunchArgument("send_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.4"),
            DeclareLaunchArgument("rpy_timeout_s", default_value="0.4"),
            DeclareLaunchArgument(
                "control_telemetry_timeout_s",
                default_value="0.90",
            ),
            camera_node,
            pure_line_node,
            Node(
                package="pure_line_following",
                executable="pure_line_debug_viewer",
                name="pure_line_debug_viewer",
                output="screen",
                condition=IfCondition(LaunchConfiguration("show_debug_gui")),
                parameters=[
                    {
                        "camera_image_topic": LaunchConfiguration("image_topic"),
                        "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                        "status_topic": "/pure_line_following/status",
                        "cmd_vel_topic": cmd_vel_topic,
                        "view_mode": LaunchConfiguration("viewer_mode"),
                    }
                ],
            ),
            serial_node,
        ]
    )
