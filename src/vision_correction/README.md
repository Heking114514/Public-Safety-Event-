# vision_correction

Realtime RGB landmark correction for `/odometry/fused`.

Not part of the default navigation path. The unified entry
(`scripts/start_visual_navigation.sh`) defaults to `odom_topic=/odometry/fused`
and only starts this node when explicitly asked for the corrected stream, so the
correction runs solely through the opt-in paths documented below.

The node subscribes to:

- `/camera/camera/color/image_raw`
- `/camera/camera/color/camera_info`
- `/odometry/fused`
- `/tf_static`

It publishes:

- `/odometry/landmark_corrected`

The runtime path does not read a rosbag. The correction detector is intentionally
small for the first vehicle test: RGB dark/local-dark line masking, Hough line
segments, right-angle intersections, and matching against physical obstacle/free
region corners from `arena_map.yaml`. It does not snap the vehicle to the
planning route.

With no valid visual match, the node publishes a causal-smoothed fused pose.
After `visual_timeout_s`, the old visual offset decays toward zero. The
correction and odometry smoothing filters are causal and do not use future
samples. The output keeps the source header, frame IDs, covariances, and all
non-planar fields; planar pose and twist are updated consistently.

## Build

```bash
colcon build --packages-select vision_correction
source install/setup.bash
```

## Run the correction node

```bash
ros2 launch vision_correction vision_correction.launch.py \
  map_file:=/home/hjh/cup_corporate/Public-Safety-Event-/src/arena_path_planner/config/arena_map.yaml
```

The empty `map_file` default resolves to the installed
`arena_path_planner/config/arena_map.yaml`.

## Run navigation with the corrected odometry topic

```bash
ros2 launch vision_correction vision_correction_navigation.launch.py \
  map_file:=/home/hjh/cup_corporate/Public-Safety-Event-/src/arena_path_planner/config/arena_map.yaml
```

This wrapper starts the correction node and launches `visual_navigation` with
`odom_topic=/odometry/landmark_corrected`. It does not replace the existing
sensor/fused-odometry bringup.

## Current limitations

- The first realtime detector uses the existing dark-line RGB assumptions.
- It currently applies translation correction and leaves landmark yaw correction
  at zero.
- Camera geometry uses the measured `camera_height_m` and optional static TF.
- No depth image or stereo disparity is required or consumed.
