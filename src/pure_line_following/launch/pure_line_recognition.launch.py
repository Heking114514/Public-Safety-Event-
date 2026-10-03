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
    default_parameters = str(package_share / "config" / "pure_line_following.yaml")
    parameters_file = LaunchConfiguration("parameters_file")
    physical_parameters = parameters_for("pure_line_following")

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
                "enable_depth": ParameterValue(
                    LaunchConfiguration("enable_depth"),
                    value_type=bool,
                ),
                "depth_module.depth_profile": ParameterValue(
                    LaunchConfiguration("depth_profile"),
                    value_type=str,
                ),
                "depth_module.depth_format": "Z16",
                "align_depth.enable": ParameterValue(
                    LaunchConfiguration("align_depth"),
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

    recognition_node = Node(
        package="pure_line_following",
        executable="pure_line_recognition_node",
        # Reuse the tuned pure_line_following YAML section.
        name="pure_line_following",
        output="screen",
        parameters=[
            parameters_file,
            physical_parameters,
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
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
            },
        ],
    )

    viewer_node = Node(
        package="pure_line_following",
        executable="pure_line_debug_viewer",
        name="pure_line_recognition_viewer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_debug_gui")),
        parameters=[
            {
                "camera_image_topic": LaunchConfiguration("image_topic"),
                "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
                "cmd_vel_topic": "/cmd_vel_line_following",
                "show_command": False,
                "view_mode": LaunchConfiguration("viewer_mode"),
                "window_name": "pure line recognition",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
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
                "status_topic",
                default_value="/pure_line_recognition/status",
            ),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value="/pure_line_recognition/debug_image",
            ),
            DeclareLaunchArgument("publish_debug_image", default_value="true"),
            DeclareLaunchArgument("show_debug_gui", default_value="true"),
            DeclareLaunchArgument("viewer_mode", default_value="side_by_side"),
            DeclareLaunchArgument("require_camera_info", default_value="true"),
            DeclareLaunchArgument("require_depth", default_value="false"),
            DeclareLaunchArgument("use_depth_projection", default_value="true"),
            DeclareLaunchArgument("image_is_rectified", default_value="false"),
            camera_node,
            recognition_node,
            viewer_node,
        ]
    )
