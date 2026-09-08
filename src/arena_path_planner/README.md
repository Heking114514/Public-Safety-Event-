# Arena Path Planner

ROS 2 C++ backend for the arena route planner. The package owns map loading,
task ordering, collision-aware planning, path smoothing, and conversion from
arena coordinates to the navigation frame.

这是本工作空间的**自动规划导航模式**。它不读取 `visual_navigation` 的 CSV 路点文件，
而是根据当前车辆位姿、目标点、地图和动态障碍物生成 `nav_msgs/msg/Path`，再按请求决定
是否自动交给 `visual_navigation` 执行。路线执行仍由 `visual_navigation` 负责。

Interfaces are read from the `topics` section of `config/arena_map.yaml`. The
default configuration is the measured 6 x 6 m production map exported at 5 cm
resolution. It includes the configured 217 x 210 mm vehicle footprint, safety
margin, and explicit staging area for the launch lane. The former 3.2 x 4.4 m
narrow-corridor fixture is retained as `config/arena_map_synthetic.yaml` for
collision and incompatible-map rejection tests only; it must not be used for a
vehicle route.

The default configuration uses:

- Service: `/arena_path_planner/plan`
- Arena path: `/arena_path_planner/arena_path`
- Navigation path: `/arena_path_planner/navigation_path`
- Occupancy grid: `/arena_path_planner/map`
- Activated route: `/waypoint_navigation/route_input`

Changing a value under `topics` changes the corresponding publisher or service
when the node starts; no source edit or ROS remapping is required.

The planning request accepts multiple dynamic obstacle polygons. If a complete
blockage makes some targets unreachable, the response still contains a route
through every currently reachable target and lists blocked work in
`deferred_targets`. The caller can replan those deferred targets once from the
vehicle's new position instead of abandoning the mission or retrying forever.
Finite point and line observations are retained as one-cell minimum obstacles
before normal footprint inflation. An empty polygon or one containing NaN/Inf
rejects only that planning request; no replacement route is activated, so the
navigator keeps its last validated route and the planner process remains alive.

Use `scripts/start_arena_planner.sh` from the workspace root to incrementally
build the backend and open the Python frontend. The visual/fusion/navigation
stack must already be running, for example in another terminal:

```bash
./scripts/start_visual_navigation.sh --serial-device auto
```

Then start the planner frontend:

```bash
./scripts/start_arena_planner.sh
```

In the frontend, use `vehicle` mode, confirm the targets and obstacles, and
click `开始导航`. The frontend requests planning with `activate_navigation=true`.
The backend publishes a collision-free route to
`/waypoint_navigation/route_input` whenever it has real movement and new
mission progress, even if some targets are temporarily deferred. The
`visual_navigation` node activates that stage; after `GOAL_REACHED`, the
frontend replans the deferred work from the vehicle's new pose. A result with
no movement or no new progress is reported but is not activated. `开始规划`
only previews a route, and simulation requests never publish to the
activated-route topic.

For a hand-marked route or a fixed CSV route, do not use this package; use the
`visual_navigation` manual route instructions instead.
