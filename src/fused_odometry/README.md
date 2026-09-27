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
  for consistency diagnostics. Only `vx` enters the default EKF.
- `/cup_car_serial/bmi088_attitude` (`sensor_msgs/Imu`): lower-controller
  processed BMI088 ATT yaw and gyro-z input.

Outputs:

- `/odometry/local` (`nav_msgs/Odometry`): continuous local EKF estimate in
  `odom`, driven by sanitized wheel body velocity and BMI088 yaw rate. ORB
  frame-to-frame velocity is not integrated into this state.
- `/odometry/local_map` (`nav_msgs/Odometry`): start-aligned copy of
  `/odometry/local` expressed as `map -> base_link`; this is the temporary
  default navigation input while visual global correction is being recalibrated.
- `/odometry/fused` (`nav_msgs/Odometry`): public map-frame pose. By default it
  is only the start-aligned local EKF pose; absolute ORB XY correction is
  opt-in and remains available for a separately validated global source.
- `/odometry/fusion_status` (`std_msgs/String`, transient local):
  `WAITING_FOR_INITIALIZATION`, `FULL`,
  `DEGRADED_NO_VISION`, `DEGRADED_NO_IMU`, `DEGRADED_NO_WHEEL`,
  `DEGRADED_VISION_ONLY`, `DEGRADED_WHEEL_ONLY`,
  `DEGRADED_VISUAL_REALIGNED`, `FAULT_STALLED`, `FAULT_INIT_TIMEOUT`, or
  `FAULT`.
- `/diagnostics`: freshness, residuals, rejection counters and dead-reckoning limits.
- `/imu/control` (`sensor_msgs/Imu`): gate-validated, base-link-frame processed
  BMI088 yaw/yaw-rate. This is the single IMU feedback stream for both the local
  EKF and navigation control.

The gate publishes sanitized visual and wheel inputs below `/fusion/input/*`.
The visual stream is retained for tracking health and future trusted landmark
correction; it is not a local translation source.
The control IMU is intentionally public at `/imu/control` so that
navigation damping and EKF prediction use the same yaw-rate sample.

Wheel, IMU, and raw visual callbacks reject unset, stale, future, and
non-monotonic measurement timestamps. All periodic status/publish timers use
the node ROS clock, so simulated-time pause and jumps do not mix with wall-time
control scheduling. When `use_slam_imu=true`, the bringup launch omits the
D455 IMU stream from ORB input; the fusion gate still expects processed BMI088
ATT from `cup_car_serial`.

`/odometry/fused` publishes a map-frame pose. Its position and yaw covariance
include both the local odometry covariance and the accepted visual correction
covariance; the local covariance is not relabeled as global covariance.

`use_absolute_visual_correction` is `false` by default. This is intentional:
the current ORB absolute XY stream is not an arena-map truth measurement and can
report false translation during in-place turns. With the default, `/odometry/fused`
keeps the same start alignment and local motion as `/odometry/local_map`, while
preserving the fused topic and `map -> odom` TF contract. Enable the parameter
only after a trusted landmark/corner source has been validated.

Turn-position holding is enabled by default (`hold_global_xy_during_turn:
true`). It is released if local odometry measures more than
`turn_hold_max_translation_m` of translation, so a misclassified in-place turn
cannot silently erase real chassis motion.

The map correction node pairs visual poses with local EKF poses at the visual
measurement timestamp. Timestamps between two retained local samples use SE(2)
interpolation, including shortest-path yaw. A visual sample newer than the
local history is held briefly until a matching local sample arrives; stale,
out-of-order, or over-tolerance samples are rejected instead of using a
nearest-sample scheduling approximation.

Map correction also subscribes to `/odometry/fusion_status`. This latched status
is the authoritative global permission for visual correction: `FAULT_*`,
`DEGRADED_NO_VISION`, `DEGRADED_WHEEL_ONLY`, unknown, stale, or future-dated
statuses block correction. The map node still checks its own visual/local pair
freshness because that is a local timestamp-matching requirement, not a second
global vision-health decision.

## Behavior

`robot_localization` performs the local planar EKF from gated wheel `vx`/`vy`
and processed BMI088 yaw/yaw-rate. ORB frame-to-frame velocity is not fused:
the latest arena bags contain false forward motion during in-place turns.
Wheel yaw does not modify this state. The correction node publishes `map ->
odom` and `/odometry/fused`; by default that transform is fixed at the start
alignment, so the public stream is a wheel/IMU local pose with the existing
map-frame contract.
When `use_absolute_visual_correction` is explicitly enabled, the node can
smooth a trusted global pose source with the configured correction time
constants. The current ORB absolute pose is not trusted by default.
The gate independently validates
finite values, expected frames, monotonic timestamps and source freshness. It
uses median/MAD residual windows for diagnostics. IMU yaw rate is the primary
short-term rotation source: it is removed only for an invalid frame/value,
implausible magnitude, non-monotonic timestamp, or timeout. IMU/visual
disagreement increases visual yaw covariance instead of interrupting
`/imu/control`, because visual angular rate can lag during a rapid turn. The
upper computer does not relearn BMI088 gyro bias; it trusts the lower-controller
processed ATT sample and only validates the frame, timestamp and bounds.

Wheel and visual forward motion are classified only after a configurable dwell:
wheel motion without visual motion is `WHEEL_SLIP`; visual motion without wheel
motion is `ENCODER_FAILURE`. These remain diagnostics only. A fresh,
frame-valid wheel sample is still published to the local EKF, including a
zeroed forward velocity plus the 50 mm lever-arm lateral velocity during an
in-place turn. Visual residuals do not suppress the wheel stream or inflate its
EKF covariance. With healthy visual odometry but no IMU, the mode is always
`DEGRADED_NO_IMU`, independent of wheel health. Wheel faults and loss still set
the diagnostic level to `WARN`; recovery has a separate dwell to prevent rapid
state toggling.

During visual loss, the local EKF continues from wheel `vx`/`vy` and BMI088
yaw/yaw-rate while the configured time/distance limits bound this open-loop
period. Fusion status is infrastructure health only: allowed degraded states do
not lower navigation speed. `FAULT`, stale status, invalid odometry, or
exceeded dead-reckoning limits stop navigation and wait for recovery.

At startup the gate publishes `WAITING_FOR_INITIALIZATION` while ORB warms up
and accumulates coherent visual increments. Navigation treats this state as a
zero-velocity wait, but it is not a latched fault. If no visual initialization
arrives within `initialization_timeout_s` (10 s by default), the status becomes
`FAULT_INIT_TIMEOUT`; navigation can then latch it for manual recovery.

Wheel yaw validation requires sustained agreement with IMU during actual
rotation; stationary zero-rate agreement cannot validate it. Because the latest
bag showed a large encoder turn-scale error, `fuse_wheel_yaw` remains false.
Wheel yaw is diagnostic-only in the default EKF profile. If both vision and IMU
disappear, the fusion becomes `FAULT` instead of attempting an unreliable blind
turn. Linear acceleration is never fused.

The wheel gate only zeroes forward velocity for a true near-stationary pivot.
The default `wheel_in_place_max_linear_speed_mps` is `0.02`, so slow moving arcs
such as `0.06 m/s` rectangle turns keep their encoder distance instead of being
misclassified as in-place rotation.

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

The current planar measurement places `camera_link` 0.055 m forward of the
geometric vehicle center: `base_link -> camera_link = (0.055, 0, 0)`. The wheel
axle midpoint is measured 0.050 m behind that center, so wheel odometry converts
the axle pose to `base_link` with `base_offset_x_m: 0.050`. ORB, wheel odometry
and processed BMI088 ATT publish the vehicle center as `base_link`.

The local EKF broadcasts `odom -> base_link`; the correction node broadcasts
`map -> odom`. ORB and wheel TF publication remain disabled to avoid duplicate
TF parents.

During an observed in-place turn, the correction node holds the global vehicle
center in XY while the local IMU yaw remains live. When the turn ends, the
visual translation is rebased to that anchor, so later ORB translation
increments remain available without accepting the 4-9 cm apparent translation
observed during zero-linear-speed turns.

Motion states are intentionally contextual rather than one shared boolean:
map correction uses measured local speed to select its correction time constant,
navigation braking uses measured position-window speed and a dwell, and the IMU
filter's `stationary` state is its own gyro-bias/yaw-hold detector. These states
must not be substituted for one another because they answer different questions
and use different trustworthy sensors.

## Start

First start ORB with body frame `base_link`, and start cup-car serial ATT plus
wheel odometry when those inputs are selected, with their live TF outputs
disabled. Then run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fused_odometry fused_odometry.launch.py
```

Alternatively, start the camera, ORB, optional wheel odometry, gate, and EKF
without any navigator or motor output:

```bash
ros2 launch fused_odometry odometry_bringup.launch.py
```

Bringup arguments are `serial_no`, `orb_settings_file`, `infra_profile`,
`initial_reset`, `visualization`, `use_imu`, `use_slam_imu`, `equalize`,
`use_wheel`, and `use_sim_time`. Their defaults match the current D455 setup.
`infra_profile:=848x480x15` plus the RK3588 ORB settings YAML is the low-load
trial profile. `use_imu:=false` also disables the D455 IMU streams;
`use_wheel:=false` leaves wheel odometry out and produces a degraded status.
Start `cup_car_serial cmd_vel_serial.launch.py` separately, or use the
serial-navigation bringup, when the gate should receive BMI088 ATT.

No physical motion is started by this command. Inspect health before connecting
the output to navigation:

```bash
ros2 topic echo /odometry/fusion_status
ros2 topic hz /odometry/fused
```

All gates and conservative limits are in `config/fused_odometry.yaml`. They are
starting values and must be tuned from recorded bags, especially wheel scale,
left/right imbalance, wheel/vision speed residuals, and IMU/vision yaw residuals.
