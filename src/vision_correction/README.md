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

The runtime path does not read a rosbag. RGB `image_raw` frames are rectified
with `K/D`, converted to a black-line mask in RGB space, restricted to a
ground-plane ROI projected from the measured camera geometry, and converted to
base-link ground segments. Right-angle intersections are then matched against
physical obstacle/free-region corners from `arena_map.yaml`. The node does not
snap the vehicle to the planning route.

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

## Detector geometry

The current RGB defaults are intentionally conservative:

- `detector: rgb_and_local`
- adaptive RGB darkness per image row, capped at `90`
- low RGB channel spread to reject colored objects
- projected ground ROI from `camera_height_m`, `CameraInfo`, and `/tf_static`
- `camera_color_optical_frame -> base_link` optical-ray conversion
- connected-component filtering followed by Hough segments

The realtime node requires a landmark match to remain spatially stable for
`visual_stability_s` before changing the correction target. Without a stable
corner match it relays causal-smoothed fused odometry. It does not consume
depth or stereo disparity.
