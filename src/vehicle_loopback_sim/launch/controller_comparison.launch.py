"""兼容入口：只启动 final_navigation Contrast 控制器闭环。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_file = (
        Path(get_package_share_directory("vehicle_loopback_sim"))
        / "launch"
        / "final_navigation_comparison.launch.py"
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)))
    ])
