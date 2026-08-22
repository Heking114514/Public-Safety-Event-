# fused_odometry

Independent ROS 2 Humble odometry fusion package. It contains no navigator and
never publishes a velocity command.

## Interfaces

Inputs:

- `/odometry/visual_raw` (`nav_msgs/Odometry`): canonical ORB observation in
  the fixed first-body map frame. It preserves relocalization and map-correction
  jumps so the gate can validate them instead of silently hiding them.
- `/odom/orb_raw` (`nav_msgs/Odometry`): compatibility alias carrying the same
  messages as `/odometry/visual_raw`; it is not used by the default fusion profile.
- `/tracking_state` (`std_msgs/Int32`): ORB `OK=2`, `OK_KLT=5` are healthy.
- `/orbslam3/map_change` (`std_msgs/UInt64`): verified ORB loop-closure or
  global-BA map correction sequence.
- `/wheel/odom` (`nav_msgs/Odometry`): body-forward `vx`; `wz` remains available
  for consistency diagnostics. Neither component enters the default EKF.
- `/imu/filtered` (`sensor_msgs/Imu`): only `angular_velocity.z` is used.
- `/cmd_vel_nav` (`geometry_msgs/Twist`): observation-only input for classifying
  slip, mechanical stall, and encoder failure; it is never forwarded or modified.

Outputs:

- `/odometry/local` (`nav_msgs/Odometry`): continuous local EKF estimate in
  `odom`, driven by gated visual forward velocity and IMU yaw rate.
- `/odometry/fused` (`nav_msgs/Odometry`): smoothed map-frame pose used by the
  existing navigator and route editor.
- `/odometry/fusion_status` (`std_msgs/String`, transient local): `FULL`,
  `DEGRADED_NO_VISION`, `DEGRADED_NO_WHEEL`, `DEGRADED_NO_IMU`,
  `DEGRADED_VISION_ONLY`, `DEGRADED_WHEEL_ONLY`,
  `DEGRADED_VISUAL_REALIGNED`, `FAULT_STALLED`, or `FAULT`.
- `/diagnostics`: freshness, residuals, rejection counters and dead-reckoning limits.

The gate publishes private sanitized inputs below `/fusion/input/*`; consumers
should not use those as the public odometry interface.

## Behavior

`robot_localization` performs the local planar EKF from gated visual forward
velocity and IMU yaw rate. Encoder odometry never modifies this state. A separate
correction node combines gated ORB pose with the local
estimate to publish `map -> odom` and `/odometry/fused`. Normal visual drift is
corrected with a 0.75-second time constant; a verified ORB map change uses a
1.25-second time constant so PID control never receives a loop-closure jump.
The gate independently validates
finite values, expected frames, monotonic timestamps and source freshness. It
uses median/MAD residual windows, recovery hysteresis, and adaptive measurement
covariance. IMU yaw rate is the primary short-term rotation source: it is removed
only for an invalid frame/value, implausible magnitude, non-monotonic timestamp,
or timeout. IMU/visual disagreement increases visual yaw covariance instead of
interrupting `/fusion/input/imu`, because visual angular rate can lag during a
rapid turn. Wheel residuals retain hard rejection and recovery hysteresis. Wheel
covariance supplied by the encoder node remains the lower bound before adaptive
inflation.
During healthy commanded straight motion, a slow bounded gyro-z bias estimator
uses the robust visual yaw rate to suppress long-run lateral drift in
`/odometry/local`; it freezes during turns and visual outages, so it does not
replace the high-frequency IMU turn signal.

Command, wheel, and visual motion are classified only after a configurable dwell:
wheel motion without visual motion is `WHEEL_SLIP`; commanded motion with neither
wheel nor visual motion is `MECHANICAL_STALL`; visual motion without wheel motion
is `ENCODER_FAILURE`. A stall publishes `FAULT_STALLED`. Slip and encoder failure
change health and degraded-operation authorization, but wheel measurements never
enter the EKF. Recovery has a separate dwell to prevent rapid state toggling.
Angular stall detection requires at least two fresh angular-rate sources and
publishes `FAULT_STALLED` after a turn command produces no measured rotation for
0.8 seconds.

During visual loss, the local EKF receives no new translation measurement; its
short prediction continues from the last accepted visual velocity while IMU `wz`
keeps angular motion live. Validated wheel speed only measures the outage-distance
budget and participates in health checks. Navigation applies a reduced speed until
vision returns, and the configured time/distance limits bound this open-loop period.

Wheel yaw validation requires sustained agreement with IMU during actual
rotation; stationary zero-rate agreement cannot validate it. Because the latest
bag showed a large encoder turn-scale error, `fuse_wheel_yaw` remains false.
Wheel yaw is diagnostic-only in the default EKF profile. If both vision and IMU
disappear, the fusion becomes `FAULT` instead of attempting an unreliable blind
turn. Linear acceleration is never fused.

After five good recovery frames, the gate accepts the configured raw visual
topic. An existing raw-to-fused SE(2) alignment is checked against the current
prediction. A
reasonable recovery enters a 0.75-second covariance ramp so correction is
gradual. A recovery beyond 1.50 m or 1.00 rad is rejected instead of moving the
visual origin onto the open-loop local prediction. The established visual map
transform is preserved across an outage, allowing accepted visual recovery to
correct local prediction drift.

The ROS ORB wrapper publishes `/odometry/visual_raw` as planar `x`, `y`, and yaw
while retaining the full SE(3) map internally. `/odom/orb_raw` is its legacy
alias. The gate still checks the configured input's quaternion and frame before
using it. `/odometry/visual_continuous` and its `/odom` alias deliberately hide
relocalization jumps and are not odometry-fusion pose inputs.

## Frames

The current planar measurement places `camera_link` 0.096 m forward of the
drive-wheel center: `base_link -> camera_link = (0.096, 0, 0)`. ORB and wheel
odometry publish the vehicle center as `base_link`; the IMU filter keeps its
camera frame but already converts gyro measurements to Euler yaw rate before the
gate republishes that scalar in `base_link`.

The local EKF broadcasts `odom -> base_link`; the correction node broadcasts
`map -> odom`. ORB and wheel TF publication remain disabled to avoid duplicate
TF parents.

During a commanded in-place turn, the correction node holds the global vehicle
center in XY while the local IMU yaw remains live. When the turn ends, the
visual translation is rebased to that anchor, so later ORB translation
increments remain available without accepting the 4-9 cm apparent translation
observed during zero-linear-speed turns.

## Start

First start ORB with body frame `base_link`, the IMU filter, and wheel odometry
in `base_link`, with their live TF outputs disabled. Then run:

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
