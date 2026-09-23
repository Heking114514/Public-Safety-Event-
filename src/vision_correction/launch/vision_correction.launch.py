from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("vision_correction"))
    default_parameters = str(package_share / "config" / "vision_correction.yaml")
    parameters_file = LaunchConfiguration("parameters_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("parameters_file", default_value=default_parameters),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument("fused_topic", default_value="/odometry/fused"),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/odometry/landmark_corrected",
            ),
            DeclareLaunchArgument("tf_static_topic", default_value="/tf_static"),
            DeclareLaunchArgument("camera_height_m", default_value="0.121"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("sync_tolerance_s", default_value="0.12"),
            DeclareLaunchArgument("publish_rate", default_value="0.0"),
            DeclareLaunchArgument("timeout", default_value="0.5"),
            Node(
                package="vision_correction",
                executable="vision_correction_node",
                name="vision_correction_node",
                output="screen",
                parameters=[
                    parameters_file,
                    {
                        "map_file": LaunchConfiguration("map_file"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "fused_topic": LaunchConfiguration("fused_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "tf_static_topic": LaunchConfiguration("tf_static_topic"),
                        "camera_height_m": LaunchConfiguration("camera_height_m"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "sync_tolerance_s": LaunchConfiguration("sync_tolerance_s"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "timeout": LaunchConfiguration("timeout"),
                    },
                ],
            ),
        ]
    )
