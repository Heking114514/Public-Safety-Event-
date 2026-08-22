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
    use_slam_imu = LaunchConfiguration("use_slam_imu")
    enable_imu = LaunchConfiguration("enable_imu")
    publish_tf = LaunchConfiguration("publish_tf")
    odom_topic = LaunchConfiguration("odom_topic")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    legacy_odom_topic = LaunchConfiguration("legacy_odom_topic")
    legacy_raw_odom_topic = LaunchConfiguration("legacy_raw_odom_topic")

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
                "enable_gyro": ParameterValue(enable_imu, value_type=bool),
                "enable_accel": ParameterValue(enable_imu, value_type=bool),
                "gyro_fps": 200,
                "accel_fps": 63,
                "unite_imu_method": ParameterValue(
                    PythonExpression(["2 if '", enable_imu, "'.lower() == 'true' else 0"]),
                    value_type=int,
                ),
                "enable_sync": True,
                # Keep the D455's internal camera transforms available. The
                # publish_tf launch argument controls only ORB's live map TF.
                "publish_tf": True,
            }
        ],
    )

    orbslam = Node(
        package="orbslam3",
        executable="stereo-inertial",
        name="orbslam3_stereo_inertial",
        output="screen",
        arguments=[vocabulary, settings, "false", equalize, visualization, use_slam_imu],
        parameters=[
            {
                "map_frame_id": map_frame_id,
                "body_frame_id": body_frame_id,
                "publish_tf": ParameterValue(publish_tf, value_type=bool),
                "odom_topic": ParameterValue(odom_topic, value_type=str),
                "raw_odom_topic": ParameterValue(raw_odom_topic, value_type=str),
                "legacy_odom_topic": ParameterValue(legacy_odom_topic, value_type=str),
                "legacy_raw_odom_topic": ParameterValue(legacy_raw_odom_topic, value_type=str),
                "map_change_topic": "/orbslam3/map_change",
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
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Publish the live map-to-body transform",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/visual_continuous",
                description="Continuity-aligned visual odometry",
            ),
            DeclareLaunchArgument(
                "raw_odom_topic",
                default_value="/odometry/visual_raw",
                description="Fixed-origin visual odometry that preserves ORB map corrections",
            ),
            DeclareLaunchArgument(
                "legacy_odom_topic",
                default_value="/odom",
                description="Compatibility alias for continuous visual odometry; empty disables it",
            ),
            DeclareLaunchArgument(
                "legacy_raw_odom_topic",
                default_value="/odom/orb_raw",
                description="Compatibility alias for raw visual odometry; empty disables it",
            ),
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
                "use_slam_imu",
                default_value="false",
                description="Fuse raw D455 IMU measurements inside ORB-SLAM3",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="Publish D455 IMU streams for external filtering and odometry fusion",
            ),
            camera,
            orbslam,
        ]
    )
