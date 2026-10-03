from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from vehicle_config import parameters_for


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    physical_parameters = parameters_for("wheel_odometry")
    default_config = PathJoinSubstitution(
        [FindPackageShare("wheel_odometry"), "config", "wheel_odometry.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="wheel_odometry",
                executable="wheel_odometry_node",
                name="wheel_odometry_node",
                output="screen",
                parameters=[config_file, physical_parameters],
            ),
        ]
    )
