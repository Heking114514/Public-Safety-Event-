# vehicle_loopback_sim

`vehicle_loopback_sim` is a lightweight truth-odometry loopback for testing the
arena planner and waypoint controller without the real chassis, wheel odometry,
ORB-SLAM, or the fusion gate.

Truth integration, sensor publication, obstacle simulation and the optional
debug route controller are C++17 executables. The browser GUI, launch files and
legacy comparison mission helper remain Python because they are operator/test
tooling rather than the realtime simulation path.

It subscribes to `/cmd_vel_nav` and publishes:

- `/odometry/fused` as `map -> base_link` truth odometry
- `/imu/control` with truth yaw rate
- `/odometry/fusion_status` as `FULL`
- `/cup_car_serial/actuator_healthy` as `true`
- `/cup_car_serial/control_telemetry` with ideal wheel feedback
- `/obstacle/front_range` and `/obstacle/front_blocked` as simulated front
  obstacle recognition

When enabled, simulated obstacle recognition uses only temporary GUI/manual
obstacles. Static obstacles from `arena.yaml` remain known map geometry and are
never published as sensor detections. Each temporary obstacle uses a fixed
`0.25 m` trigger distance and is reported only inside the vehicle's forward
camera corridor. The detector is disabled by default. Enable it with:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py \
  simulate_obstacle_recognition:=true
```

The GUI can also place temporary simulated obstacles without editing the map:
open the loopback page, enable `Obstacle`, then click the arena. `Clear` removes
only these temporary GUI obstacles. GUI obstacles snap to the nearest road
centerline and are placed crosswise to that lane at the configured `0.20 m`
road width, with `0.10 m` depth, so clicking in a passage blocks that passage.
The loopback node publishes GUI obstacles on `/obstacle/front_range`,
`/obstacle/front_blocked` and `/perception/dynamic_obstacle`. The GUI draws a dashed
forward detection fan from the camera position.

The launch files also start a small browser GUI by default. It renders
`new_map_planner/config/arena.yaml` road regions and transforms planner `arena` paths into the
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

Run the same formal controller and truth vehicle with accelerated playback:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py \
  playback_speed:=3.0
```

`playback_speed` accelerates the controller timer and truth integration
together, so calibrated turn geometry is preserved. The accepted range is
`0.1` to `10.0`; the default remains `1.0`.

This full launch starts `new_map_planner`, the formal
`visual_navigation/waypoint_navigator`, the production
`navigation_mission_manager`, and the truth vehicle. By default the mission
manager auto-starts the production staged mission against truth odometry. Static
map obstacle simulation is opt-in. This launch sets
`task_confirmation_mode=route`, so a task is confirmed only after its route is
acknowledged, executed, and reports `GOAL_REACHED`; no simulated RFID reader is
required. Disable mission startup when you only want
idle topics:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py plan_on_start:=false
```

Enable simulated map/manual obstacle reports explicitly:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py \
  simulate_obstacle_recognition:=true
```

Run the debug-only simplified loopback route controller explicitly when you need
to isolate the simulator from the production navigator:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py \
  use_waypoint_navigator:=false use_debug_controller:=true
```

Pass a shorter comma-separated target sequence when needed:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py \
  target_ids:=1,2,3,4,5,6,7,8,9,10,11,12
```

The old `loopback_mission_node` executable remains available as a compatibility
test tool, but `planner_control_loopback.launch.py` no longer starts it by
default.

Disable or move the GUI when needed:

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py gui:=false
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py gui_port:=8766
```

Publish `/initialpose` to reset the simulated pose during a run. Speed and
acceleration limits are configured in `config/vehicle_loopback.yaml`; setting an
acceleration limit to `0` disables that limiter.

## New/legacy controller comparison

The comparison launch runs two isolated truth vehicles against the same route.
The new controller uses `waypoint_navigator`; the legacy controller is the
unchanged pre-shunt code imported from commit `71ddfde` and built as
`waypoint_navigator_legacy`.

```bash
ros2 launch vehicle_loopback_sim controller_comparison.launch.py
```

Open both live views:

- new shunt controller: `http://127.0.0.1:8765/`
- legacy in-place-turn controller: `http://127.0.0.1:8766/`

The two controllers share only the `new_map_planner` route. Commands, truth odometry,
IMU, telemetry, status and trails are under `/comparison/new/*` and
`/comparison/old/*`. The mission waits for both controllers to report
`GOAL_REACHED` before it requests another stage. The default comparison uses
the same complete 12-target closed route.
