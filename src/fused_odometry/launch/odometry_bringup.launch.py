from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def launch_file(package, name):
    return os.path.join(get_package_share_directory(package), "launch", name)


def generate_launch_description():
    serial_no = LaunchConfiguration("serial_no")
    initial_reset = LaunchConfiguration("initial_reset")
    visualization = LaunchConfiguration("visualization")
    use_imu = LaunchConfiguration("use_imu")
    use_slam_imu = LaunchConfiguration("use_slam_imu")
    equalize = LaunchConfiguration("equalize")
    use_wheel = LaunchConfiguration("use_wheel")
    use_sim_time = LaunchConfiguration("use_sim_time")

    wheel_config = os.path.join(
        get_package_share_directory("wheel_odometry"),
        "config",
        "wheel_odometry.yaml",
    )
    imu_config = os.path.join(
        get_package_share_directory("imu_rpy_filter"),
        "config",
        "imu_rpy_filter.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_no", default_value="_038122250473"),
        DeclareLaunchArgument("initial_reset", default_value="false"),
        DeclareLaunchArgument("visualization", default_value="false"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("use_slam_imu", default_value="false"),
        DeclareLaunchArgument("equalize", default_value="false"),
        DeclareLaunchArgument("use_wheel", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file("orbslam3", "realsense_d455_stereo_inertial.launch.py")
            ),
            launch_arguments={
                "serial_no": serial_no,
                "body_frame_id": "camera_link",
                "map_frame_id": "map",
                "publish_tf": "false",
                "visualization": visualization,
                "equalize": equalize,
                "initial_reset": initial_reset,
                "use_slam_imu": use_slam_imu,
                "enable_imu": use_imu,
            }.items(),
        ),
        Node(
            package="imu_rpy_filter",
            executable="imu_rpy_filter_node",
            name="imu_rpy_filter",
            output="screen",
            condition=IfCondition(use_imu),
            parameters=[imu_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="wheel_odometry",
            executable="wheel_odometry_node",
            name="wheel_odometry_node",
            output="screen",
            condition=IfCondition(use_wheel),
            parameters=[
                wheel_config,
                {
                    "base_frame": "camera_link",
                    "publish_tf": False,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file("fused_odometry", "fused_odometry.launch.py")
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
    ])
