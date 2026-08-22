# imu_rpy_filter

ROS 2 C++ IMU attitude estimator for the RealSense D455. It converts the
camera optical IMU axes to `camera_link`, rejects spikes with median and moving
average windows, applies a low-pass filter, and estimates roll, pitch, yaw and
gyroscope bias. Roll and pitch are corrected with gravity. A debounced
stationary detector preserves the yaw anchor through short IMU spikes, while a
robust zero-rate update continuously estimates the z-gyro bias.

## Topics

Input:

- `/camera/camera/imu` (`sensor_msgs/msg/Imu`)
- `/odometry/visual_continuous` (`nav_msgs/msg/Odometry`): optional visual yaw
  reference when `use_yaw_reference` is enabled
- `/cmd_vel_nav` (`geometry_msgs/msg/Twist`) and `/wheel/odom`
  (`nav_msgs/msg/Odometry`): corroborate stationary state when both are fresh
  and zero; IMU-only detection remains available when either topic is absent

Outputs:

- `/imu/rpy` (`geometry_msgs/msg/Vector3Stamped`): roll, pitch and yaw in radians
- `/imu/rpy_degrees` (`geometry_msgs/msg/Vector3Stamped`): values in degrees
- `/imu/filtered` (`sensor_msgs/msg/Imu`): filtered acceleration/angular rate,
  estimated quaternion and covariance for later odometry fusion
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): stationary state,
  estimated z bias, uncertainty and rejected transient count

All output data uses the `camera_link` convention: x forward, y left, z up.
Yaw starts at zero because a D455 has no absolute heading sensor. While moving,
Yaw can still drift slowly; leave `use_yaw_reference` disabled when feeding
`/imu/filtered` into an odometry fusion node to avoid a feedback loop.

## Run

Start the D455 driver with the unified IMU topic, then run:

```bash
ros2 launch imu_rpy_filter imu_rpy_filter.launch.py
```

The workspace visual-navigation launch starts this node automatically whenever
`use_imu:=true`.
