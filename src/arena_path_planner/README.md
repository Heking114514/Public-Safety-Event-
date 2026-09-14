# Arena Path Planner

ROS 2 C++ backend for the arena route planner. The package owns map loading,
task ordering, collision-aware planning, path smoothing, and conversion from
arena coordinates to the navigation frame.

这是本工作空间的**自动规划导航模式**。它不读取 `visual_navigation` 的 CSV 路点文件，
而是根据当前车辆位姿、目标点、地图和动态障碍物生成 `nav_msgs/msg/Path`，再按请求决定
是否自动交给 `visual_navigation` 执行。路线执行仍由 `visual_navigation` 负责。

Interfaces are read from the `topics` section of `config/arena_map.yaml`. The
default configuration follows the official diagram: a 3.2 x 3.2 m main field
inside a 3.2 x 4.4 m envelope, ten 0.8 x 0.8 m blocks, nominal 0.2 m roads and
launch box, and four tunnel sections. The 2 cm planning grid keeps the narrow
road centre lines representable. The 12 numbered task points are existing
project mission points on road centres; the supplied diagram itself does not
assign task numbers.

For the official map, heading changes are emitted at configured road
junctions. The navigation path uses `pose.position.z == 0.001` as a 2D-only
junction marker; the arena path keeps `z == 0`. The marker authorizes an
ordinary junction pivot, not a planned U-turn. Official-map routes reject
180 degree reversals and short consecutive 90 degree turn pairs; if the robot
must back out after a blocked road, that is handled by obstacle recovery
returning to a junction before replanning. The marker does not change the
displayed planar route or the map transform.

The active footprint uses the measured 143.4 x 143.7 mm complete vehicle
envelope. With a 15 mm margin on each side, its 173.4 x 173.7 mm translational
envelope fits a nominal 200 mm straight road. The 124.7 mm kinematic wheel
track remains a separate parameter from the measured overall width. In-place
turns remain explicitly enabled for the tight
intersections. The old 6 x 6 m map is retained as
`config/arena_map_6x6_legacy.yaml` for explicit `--map` use.
`config/arena_map_synthetic.yaml` deliberately keeps the incompatible 217 x
210 mm footprint as a rejection-test fixture and is not a vehicle map.

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

Grid connectors use four-direction motion. A 90 degree pivot has a bounded
distance-equivalent cost, so route selection strongly prefers fewer turns and
long straight runs without taking an extreme detour merely to save one turn.
The official 12-point task stage uses exact ordering rather than greedy
nearest-neighbour selection, because the nearest target can be a short but
unexecutable out-and-back at the launch road. Published paths contain control
points only at the start, required task/road positions, real corners, and the
end; straight segments are not filled with closely spaced intermediate
waypoints.

Road progress is represented by `RoadInterval` values. `start_fraction` and
`end_fraction` are in `[0, 1]` along the corresponding `inspection_graph` edge
in its YAML `from -> to` direction. The request's `covered_intervals` records
work confirmed by actual execution, and the response echoes that normalized
state. `planned_intervals` is work expected from the returned route; the caller
promotes it to confirmed coverage only after navigation confirms execution.
`blocked_intervals` is the obstacle body plus the configured 0.05 m planning
reserve, so the planner does not repeatedly approach the same object.
It is waived for the current obstacle snapshot but is never reported as
inspected. `deferred_intervals` contains only traversable work still owed. Thus
a rock in the middle of road AB can leave an A-side and a B-side as separate
work, then allow completion after both sides are traversed without making the
rock itself permanent debt. Legacy `covered_edges` remains supported and means
the complete `[0, 1]` interval.

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

Production missions always run `layer1 -> layer2 -> layer3` as three separate
requests, carrying the measured end pose and confirmed coverage into the next
stage. The legacy one-shot `layered` request is deliberately rejected: joining
independently simplified stages can create a 180 degree reversal that the car
cannot execute in a narrow junction.

For a hand-marked route or a fixed CSV route, do not use this package; use the
`visual_navigation` manual route instructions instead.
