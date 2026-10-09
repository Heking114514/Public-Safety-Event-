# pure_line_following

An independent white-track follower for the differential-drive car.

The runtime node subscribes to:

- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/CameraInfo`
- optionally, RealSense aligned depth on
  `/camera/camera/aligned_depth_to_color/image_raw`

It does not subscribe to odometry, localization, TF, a route, a planner, or a
map. Camera geometry is fixed by parameters and camera intrinsics are refreshed
from `CameraInfo`; the node stays stopped until valid camera calibration is
received.

The detector:

1. Builds a white-road mask from RGB relations against the green floor.
2. Projects the mask onto a metric ground grid using the configured camera
   height (`0.121 m` by default).
3. When aligned depth is available, projects the white-road pixels through the
   D455 depth image and uses that metric centre instead of the monocular
   ground-plane estimate.
4. Selects the white component connected to the forward camera corridor.
5. Estimates lateral error from the forward trunk while rejecting near-field
   side artifacts and wide crossing rows.
6. Latches the first stable visual corner, keeps following the current trunk
   during approach, then follows the selected branch centreline once the branch
   entrance is still measurable near the configured lookahead.
7. Adds a small bounded integral correction for persistent centre bias.
8. Applies the measured 200 mm road width, 132 mm chassis width, and edge
   margin before allowing forward motion.
9. Publishes a bounded differential-drive `Twist`.

The node publishes zero velocity when the image is stale, the calibration is
missing, the image cannot be decoded, confidence is low, the visible road is
too narrow to be reliable, or the road centre is too far from the camera
corridor. With the default dimensions the physical forward-motion limit is
about 13 mm of centre error: beyond that the controller pivots instead of
driving farther toward the edge. Moderate centre error is corrected by low-speed
forward motion plus steering. A frame marked `UNSAFE_EDGE` may only produce a
bounded recovery turn when both road edges are visible. It deliberately defaults
to `/cmd_vel_line_following` and refuses to drive if another node publishes the
selected command topic. Single-frame `LOST` flicker is held for a short
`invalid_frame_hold_s` window so camera noise does not create a hard zero
command every frame.

The fixed installation comes from
`vehicle_config/config/vehicle_physical.yaml`: `camera_x=0.055 m`,
`camera_y=-0.015 m`, and `camera_height=0.121 m`. In ROS body coordinates,
negative `y` is the measured 15 mm offset to the right of the centreline. Set
`image_is_rectified:=true` when subscribing to an already rectified image;
otherwise the node uses `CameraInfo.K/D` to undistort the raw image.

## Build

```bash
colcon build --packages-select vehicle_config pure_line_following
source install/setup.bash
```

## Run without driving the car

```bash
ros2 launch pure_line_following pure_line_following.launch.py
```

Inspect:

```bash
ros2 topic echo /pure_line_following/status
ros2 topic echo /cmd_vel_line_following
ros2 run pure_line_following pure_line_debug_viewer
```

The debug image is published on `/pure_line_following/debug_image`. The viewer
opens an OpenCV window with the live camera feed, detector overlay, current
status, corner state, and command velocity; press `q` or `Esc` to close only the
viewer. It defaults to a side-by-side camera/overlay view. Use
`viewer_mode:=camera` for the raw camera only or `viewer_mode:=debug` for the
overlay only.

## Push the car for recognition only

When tuning or mapping the visible track by hand, run the recognition-only
script. It starts the camera, path-recognition node, and live GUI, but does not
start the serial bridge and does not publish drive commands:

```bash
scripts/start_pure_line_recognition.sh
```

Useful variants:

```bash
scripts/start_pure_line_recognition.sh viewer_mode:=camera
scripts/start_pure_line_recognition.sh viewer_mode:=debug
scripts/start_pure_line_recognition.sh record_bag:=false
```

The recognition status is published on `/pure_line_recognition/status`, and the
overlay image is published on `/pure_line_recognition/debug_image`. If a writable
recording disk is available, bags are stored under
`path_recognition_runs/<run-id>/bag`; otherwise the live GUI still runs without
recording.

## Collect stationary camera samples

To collect short raw camera bags at manually chosen track positions, use the
interactive sampler. It starts only the D455 camera and never starts the serial
bridge or publishes a drive command:

```bash
scripts/record_camera_samples.sh --duration 8
```

Place the stationary car, enter an optional note such as
`left_corner_40cm`, and press Enter. Move the car and repeat; enter `q` when
finished. Each sample is a separate bag below
`<recording-disk>/rosbag_recording/camera_sample_runs/<session-id>/samples/`,
and `samples.tsv` records its note and capture time. Use `--samples 12` to stop
after a fixed count, `--no-depth` for RGB-only capture, or `--no-camera` when a
camera node is already running.

## Run the car at low speed

Stop waypoint navigation and any other `/cmd_vel_nav` publisher first. Then
launch the line follower together with the serial bridge and an automatic
rosbag recording:

```bash
scripts/start_pure_line_following.sh
```

To open the live debug window in the same launch:

```bash
scripts/start_pure_line_following.sh show_debug_gui:=true
```

To focus on only the raw camera stream:

```bash
scripts/start_pure_line_following.sh show_debug_gui:=true viewer_mode:=camera
```

This starts the RGB + aligned-depth camera, `pure_line_following_node`, and
`cup_car_serial` with an isolated `/cmd_vel_line_following` topic, while
recording the camera streams, `/pure_line_following/status`,
`/cmd_vel_line_following`, and serial feedback. The status JSON reports
`lateral_source=depth_aligned` when the controller is using D455 depth,
otherwise `lateral_source=rgb_ground`. The newest bag is linked at
`/media/hjh/E/rosbag_recording/latest_navigation_bag` when that recording disk
is mounted, and each run is stored under
`/media/hjh/E/rosbag_recording/navigation_runs/<run-id>/bag`. The script does
not fall back to the workspace root; mount `/media/hjh/E` or `/media/hjh/Data`,
or set `PURE_LINE_RUNS_ROOT` explicitly. The plain
`pure_line_following.launch.py` launch is for ROS-side testing only; it does
not open the camera or serial port.

If `device:=auto` cannot find a unique `/dev/serial/by-id/*` controller, pass
the actual device:

```bash
scripts/start_pure_line_following.sh \
  serial_device:=/dev/serial/by-id/your-controller
```

The D455 defaults to serial `_038122250473` and RGB `640x480x15`. Override
`camera_serial` or `color_profile` when using another camera. If a camera is
already running, use `start_camera:=false` and keep the existing image topics.

Start with the configured `0.07 m/s` maximum speed and keep an emergency stop
ready. This package does not implement obstacle detection or full route choice;
it selects only the first stable visual corner by default. At a balanced
T/cross junction the selected branch is followed by the same controller used
for straight road centering. The default preference is left; override
`corner_turn_junction_prefer_direction:=right` for the opposite first turn.
`corner_turn_entry_x_m` defaults to `0.32 m` because the recorded RGB view loses
the branch mouth before `0.18 m`; farther detections are treated as approach
only and do not pull steering toward the branch. `corner_turn_max_s` starts
after that turn-entry point; approach has its own timeout so slow approach does
not mark the corner done before turning.
Do not override the command topic to `/cmd_vel_nav` while waypoint navigation
is running; the node will stop when it detects the shared topic.
