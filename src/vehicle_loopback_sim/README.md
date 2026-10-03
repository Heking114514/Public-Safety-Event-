# vehicle_loopback_sim

`vehicle_loopback_sim` is a lightweight truth-odometry loopback for testing the
arena planner and waypoint controller without the real chassis, wheel odometry,
ORB-SLAM, or the fusion gate.

It subscribes to `/cmd_vel_nav` and publishes:

- `/odometry/fused` as `map -> base_link` truth odometry
- `/imu/control` with truth yaw rate
- `/odometry/fusion_status` as `FULL`
- `/cup_car_serial/actuator_healthy` as `true`
- `/cup_car_serial/control_telemetry` with ideal wheel feedback

The launch files also start a small browser GUI by default. It renders
`arena_map.yaml` road regions and transforms planner `arena` paths into the
same `map` frame used by truth odometry. Open:

```text
http://127.0.0.1:8765/
```

The GUI shows the planner map, planned path, controller route, truth vehicle
footprint, driven trail, command velocity and navigation state.

Run only the vehicle loopback:

```bash
ros2 launch vehicle_loopback_sim vehicle_loopback.launch.py
```

Run planner, the formal waypoint controller and loopback together:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py
```

This full launch sends real `/arena_path_planner/plan` requests for
`layer1,layer2,layer3` by default. Each stage is activated in the formal
`visual_navigation/waypoint_navigator`; after `GOAL_REACHED`, the loopback
mission uses truth odometry as the next planner start and carries confirmed
coverage into the following request. Disable that when you only want idle
topics:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py plan_on_start:=false
```

Run the debug-only simplified loopback route controller explicitly when you need
to isolate the simulator from the production navigator:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py \
  use_waypoint_navigator:=false use_debug_controller:=true
```

Use another planner mode or a shorter comma-separated sequence explicitly when
needed:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py planner_mode:=layer2
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py planner_mode:=layer1,layer3
```

Disable or move the GUI when needed:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py gui:=false
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py gui_port:=8766
```

Publish `/initialpose` to reset the simulated pose during a run. Speed and
acceleration limits are configured in `config/vehicle_loopback.yaml`; setting an
acceleration limit to `0` disables that limiter.
