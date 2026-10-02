from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("pure_line_following"))
    default_parameters = str(package_share / "config" / "pure_line_following.yaml")
    parameters_file = LaunchConfiguration("parameters_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
            ),
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
            DeclareLaunchArgument("publish_debug_image", default_value="true"),
            DeclareLaunchArgument("require_camera_info", default_value="true"),
            DeclareLaunchArgument("require_depth", default_value="false"),
            DeclareLaunchArgument("use_depth_projection", default_value="true"),
            DeclareLaunchArgument("image_is_rectified", default_value="false"),
            DeclareLaunchArgument("enabled", default_value="true"),
            Node(
                package="pure_line_following",
                executable="pure_line_following_node",
                name="pure_line_following",
                output="screen",
                parameters=[
                    parameters_file,
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
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
            ),
        ]
    )
