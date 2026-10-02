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
   height (`0.130 m` by default).
3. When aligned depth is available, projects the white-road pixels through the
   D455 depth image and uses that metric centre instead of the monocular
   ground-plane estimate.
4. Selects the white component connected to the forward camera corridor.
5. Estimates lateral error from the forward trunk while rejecting near-field
   side artifacts and wide crossing rows.
6. Adds a small bounded integral correction for persistent centre bias.
7. Publishes a bounded differential-drive `Twist`.

The node publishes zero velocity when the image is stale, the calibration is
missing, the image cannot be decoded, confidence is low, the visible road is
too narrow to be reliable, or the road centre is too far from the camera
corridor. Moderate centre error is corrected by low-speed forward motion plus
steering. A frame marked `UNSAFE_EDGE` may only produce a bounded recovery turn
when both road edges are visible. It deliberately defaults to
`/cmd_vel_line_following` and refuses to drive if another node publishes the
selected command topic.

The default fixed installation is `camera_x=0.0547 m`,
`camera_y=-0.020 m`, and `camera_height=0.130 m`. The lateral value is the
runtime-centre calibration from repeated 20261001 pure-line runs; the raw D455
static transform and the first -0.035 m correction still produced a stable
right drift. The height is the effective RGB optical-centre height from the
same bag sweep: using the measured 121 mm mechanical height projected the
200 mm road too narrow. Set
`image_is_rectified:=true` when subscribing to an already rectified image;
otherwise the node uses `CameraInfo.K/D` to undistort the raw image.

## Build

```bash
colcon build --packages-select pure_line_following
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
```

The debug image is published on `/pure_line_following/debug_image`.

## Run the car at low speed

Stop waypoint navigation and any other `/cmd_vel_nav` publisher first. Then
launch the line follower together with the serial bridge and an automatic
rosbag recording:

```bash
scripts/start_pure_line_following.sh
```

This starts the RGB + aligned-depth camera, `pure_line_following_node`, and
`cup_car_serial` with an isolated `/cmd_vel_line_following` topic, while
recording the camera streams, `/pure_line_following/status`,
`/cmd_vel_line_following`, and serial feedback. The status JSON reports
`lateral_source=depth_aligned` when the controller is using D455 depth,
otherwise `lateral_source=rgb_ground`. The newest bag is linked at
`/media/hjh/E/rosbag_recording/latest_pure_line_bag` when that recording disk
is mounted. The plain `pure_line_following.launch.py` launch is for ROS-side
testing only; it does not open the camera or serial port.

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
ready. This package does not implement obstacle detection or route choice; at
cross junctions it ignores the side branch and drives through using the
near-field road centre. Do not override the command topic to
`/cmd_vel_nav` while waypoint navigation is running; the node will stop when it
detects the shared topic.
