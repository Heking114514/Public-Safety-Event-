# Arena Path Planner

ROS 2 C++ backend for the arena route planner. The package owns map loading,
task ordering, collision-aware planning, path smoothing, and conversion from
arena coordinates to the navigation frame.

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

Use `scripts/start_arena_planner.sh` from the workspace root to incrementally
build the backend and open the Python frontend. Simulation requests never
publish to the activated-route topic.
