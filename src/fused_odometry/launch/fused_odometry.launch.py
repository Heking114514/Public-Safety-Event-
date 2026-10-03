from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from vehicle_config import parameters_for
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("fused_odometry"),
        "config",
        "fused_odometry.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    physical_parameters = parameters_for("fused_odometry")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="fused_odometry",
            executable="fusion_gate_node",
            name="fused_odometry_gate",
            output="screen",
            parameters=[config, physical_parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="fused_ekf",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "/odometry/local")],
        ),
        Node(
            package="fused_odometry",
            executable="map_odom_correction_node",
            name="map_odom_correction",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="fused_odometry",
            executable="local_map_odometry_node",
            name="local_map_odometry",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        ),
    ])
