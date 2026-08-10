# fused_odometry

Independent ROS 2 Humble odometry fusion package. It contains no navigator and
never publishes a velocity command.

## Interfaces

Inputs:

- `/odom` (`nav_msgs/Odometry`): ORB pose; only planar `x`, `y`, and yaw are used.
- `/odom/orb_raw` (`nav_msgs/Odometry`): ORB pose in the fixed first-body map
  frame, before tracking-loss continuity correction. It is preferred for
  checking relocalization consistency; continuous `/odom` remains the fallback.
- `/tracking_state` (`std_msgs/Int32`): ORB `OK=2`, `OK_KLT=5` are healthy.
- `/wheel/odom` (`nav_msgs/Odometry`): body-forward `vx`; `wz` is accepted only
  after sustained agreement with IMU and remains deliberately low weight.
- `/imu/filtered` (`sensor_msgs/Imu`): only `angular_velocity.z` is used.
- `/cmd_vel_nav` (`geometry_msgs/Twist`): observation-only input for classifying
  slip, mechanical stall, and encoder failure; it is never forwarded or modified.

Outputs:

- `/odometry/fused` (`nav_msgs/Odometry`): continuous EKF estimate.
- `/odometry/fusion_status` (`std_msgs/String`, transient local): `FULL`,
  `DEGRADED_NO_VISION`, `DEGRADED_NO_WHEEL`, `DEGRADED_NO_IMU`,
  `DEGRADED_VISION_ONLY`, `DEGRADED_WHEEL_ONLY`,
  `DEGRADED_VISUAL_REALIGNED`, `FAULT_STALLED`, or `FAULT`.
- `/diagnostics`: freshness, residuals, rejection counters and dead-reckoning limits.

The gate publishes private sanitized inputs below `/fusion/input/*`; consumers
should not use those as the public odometry interface.

## Behavior

`robot_localization` performs the planar EKF. The gate independently validates
finite values, expected frames, monotonic timestamps and source freshness. It
uses median/MAD residual windows, recovery hysteresis, and increases measurement
covariance as wheel/IMU residuals approach their rejection thresholds. IMU yaw
rate is compared with the latest visual yaw rate so rapid direction reversals do
not compare a current sample with a delayed 0.75-second visual median. Three
consecutive hard robust residuals reject a source; ten consistent comparisons
are required to restore it. Wheel covariance supplied by the encoder node is
retained as the lower bound before this adaptive inflation.

Command, wheel, and visual motion are classified only after a configurable dwell:
wheel motion without visual motion is `WHEEL_SLIP`; commanded motion with neither
wheel nor visual motion is `MECHANICAL_STALL`; visual motion without wheel motion
is `ENCODER_FAILURE`. A stall publishes `FAULT_STALLED`. Slip and encoder failure
remove wheel measurements from the EKF, leaving visual/IMU degradation where
available. Recovery also has a separate dwell to prevent rapid state toggling.
Angular stall detection requires at least two fresh angular-rate sources and
publishes `FAULT_STALLED` after a turn command produces no measured rotation for
0.8 seconds.

During visual loss, wheel `vx` plus IMU `wz` keep the EKF predicting in
`DEGRADED_NO_VISION`; navigation applies a reduced speed until vision returns.
The stricter time and distance window below applies when IMU is also absent.

If IMU disappears after wheel yaw rate has agreed with it for at least 2 seconds,
wheel-only prediction is permitted for at most 2 seconds, 0.30 m, and only at
or below 0.30 m/s. It otherwise becomes `FAULT`. Linear acceleration is never
fused.

After five good recovery frames, the gate prefers `/odom/orb_raw`. An existing
raw-to-fused SE(2) alignment is checked against the current prediction. A
reasonable recovery enters a 0.75-second covariance ramp so correction is
gradual. A recovery beyond 1.50 m or 1.00 rad is rejected instead of moving the
visual origin onto the wheel prediction. The established visual map transform
is preserved across an outage, allowing accepted visual recovery to correct
wheel dead-reckoning drift.

Raw ORB pose is accepted only when its body Z axis remains within 0.50 rad of
the map Z axis. This rejects the optical-world coordinates produced by older
builds or a malformed camera transform instead of projecting them into a false
planar yaw.

## Frames and current limitation

The repository has no measured `base_link -> camera_link` extrinsic. The default
fusion body is therefore `camera_link`, matching ORB and filtered IMU. The wheel
node must also be started with `base_frame: camera_link`; messages still labeled
`base_link` are deliberately rejected. This treats the encoder forward velocity
as measured at the camera origin and is only a temporary planar approximation.

After measuring the extrinsic, change the fusion body to `base_link`, keep the
wheel output in `base_link`, publish the measured static transform, and let the
filter transform camera measurements normally. Do not invent a zero transform.

The EKF publishes the validated fused message and the gate is the only component
that broadcasts its live `map -> camera_link` transform. Disable ORB TF
(`publish_tf:=false`) and wheel TF (`publish_tf: false`) when using this package.

## Start

First start ORB, the IMU filter, and wheel odometry with TF disabled and wheel
`base_frame` overridden to `camera_link`. Then run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fused_odometry fused_odometry.launch.py
```

Alternatively, start the complete odometry chain (camera, ORB, optional filtered
IMU, optional wheel odometry, gate, and EKF) without any navigator or motor output:

```bash
ros2 launch fused_odometry odometry_bringup.launch.py
```

Bringup arguments are `serial_no`, `initial_reset`, `visualization`, `use_imu`,
`use_slam_imu`, `equalize`, `use_wheel`, and `use_sim_time`. Their defaults match
the current D455 setup. `use_imu:=false` also disables the D455 IMU streams;
`use_wheel:=false` leaves wheel odometry out and produces a degraded status.

No physical motion is started by this command. Inspect health before connecting
the output to navigation:

```bash
ros2 topic echo /odometry/fusion_status
ros2 topic hz /odometry/fused
```

All gates and conservative limits are in `config/fused_odometry.yaml`. They are
starting values and must be tuned from recorded bags, especially wheel scale,
left/right imbalance, wheel/vision speed residuals, and IMU/vision yaw residuals.
