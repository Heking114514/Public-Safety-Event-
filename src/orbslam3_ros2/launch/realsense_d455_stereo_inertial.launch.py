from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("orbslam3")
    vocabulary = PathJoinSubstitution([package_share, "vocabulary", "ORBvoc.txt"])
    settings = PathJoinSubstitution(
        [package_share, "config", "stereo-inertial", "RealSense_D455.yaml"]
    )

    serial_no = LaunchConfiguration("serial_no")
    body_frame_id = LaunchConfiguration("body_frame_id")
    map_frame_id = LaunchConfiguration("map_frame_id")
    visualization = LaunchConfiguration("visualization")
    equalize = LaunchConfiguration("equalize")
    initial_reset = LaunchConfiguration("initial_reset")
    use_imu = LaunchConfiguration("use_imu")

    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[
            {
                "serial_no": ParameterValue(serial_no, value_type=str),
                "initial_reset": ParameterValue(initial_reset, value_type=bool),
                "enable_color": False,
                "enable_depth": False,
                "enable_infra1": True,
                "enable_infra2": True,
                "depth_module.emitter_enabled": 0,
                "depth_module.infra_profile": "848x480x30",
                "depth_module.infra1_format": "Y8",
                "depth_module.infra2_format": "Y8",
                "enable_gyro": ParameterValue(use_imu, value_type=bool),
                "enable_accel": ParameterValue(use_imu, value_type=bool),
                "gyro_fps": 200,
                "accel_fps": 63,
                "unite_imu_method": ParameterValue(
                    PythonExpression(["2 if '", use_imu, "'.lower() == 'true' else 0"]),
                    value_type=int,
                ),
                "enable_sync": True,
                "publish_tf": True,
            }
        ],
    )

    orbslam = Node(
        package="orbslam3",
        executable="stereo-inertial",
        name="orbslam3_stereo_inertial",
        output="screen",
        arguments=[vocabulary, settings, "false", equalize, visualization, use_imu],
        parameters=[
            {
                "map_frame_id": map_frame_id,
                "body_frame_id": body_frame_id,
                "publish_tf": True,
                "publish_path": True,
                "save_trajectory": True,
                "trajectory_file": "KeyFrameTrajectory.txt",
                "max_stereo_time_diff": 0.01,
                "max_pending_stereo_pairs": 10,
                "path_max_poses": 2000,
                "diagnostics_period": 1.0,
                "camera_warmup_seconds": 2.0,
            }
        ],
        remappings=[
            ("camera/left", "/camera/camera/infra1/image_rect_raw"),
            ("camera/right", "/camera/camera/infra2/image_rect_raw"),
            ("imu", "/camera/camera/imu"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_no",
                default_value="_038122250473",
                description="RealSense serial prefixed with '_' so ROS keeps it as a string",
            ),
            DeclareLaunchArgument("body_frame_id", default_value="camera_link"),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("visualization", default_value="false"),
            DeclareLaunchArgument(
                "equalize",
                default_value="false",
                description="Enable CLAHE preprocessing; disabled by default for stable D455 infrared tracking",
            ),
            DeclareLaunchArgument(
                "initial_reset",
                default_value="false",
                description="Reset this D455 at startup; keep disabled in stereo-inertial mode to preserve the IMU IIO device",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="true",
                description="Use stereo-inertial mode; requires startup motion for IMU initialization",
            ),
            camera,
            orbslam,
        ]
    )
