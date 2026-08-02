from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    visual_navigation_share = FindPackageShare("visual_navigation")
    orbslam3_share = FindPackageShare("orbslam3")

    route_file = LaunchConfiguration("route_file")
    route_frame = LaunchConfiguration("route_frame")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    autostart = LaunchConfiguration("autostart")
    serial_no = LaunchConfiguration("serial_no")
    body_frame_id = LaunchConfiguration("body_frame_id")
    visualization = LaunchConfiguration("visualization")
    use_imu = LaunchConfiguration("use_imu")

    default_route = PathJoinSubstitution(
        [visual_navigation_share, "routes", "example_route.csv"]
    )

    orbslam3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    orbslam3_share,
                    "launch",
                    "realsense_d455_stereo_inertial.launch.py",
                ]
            )
        ),
        launch_arguments={
            "serial_no": serial_no,
            "body_frame_id": body_frame_id,
            "map_frame_id": route_frame,
            "visualization": visualization,
            "use_imu": use_imu,
        }.items(),
    )

    waypoint_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    visual_navigation_share,
                    "launch",
                    "waypoint_navigation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "route_file": route_file,
            "route_frame": route_frame,
            "cmd_vel_topic": cmd_vel_topic,
            "autostart": autostart,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("route_file", default_value=default_route),
            DeclareLaunchArgument("route_frame", default_value="map"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("serial_no", default_value="_038122250473"),
            DeclareLaunchArgument("body_frame_id", default_value="camera_link"),
            DeclareLaunchArgument("visualization", default_value="false"),
            DeclareLaunchArgument("use_imu", default_value="true"),
            orbslam3,
            waypoint_navigation,
        ]
    )
