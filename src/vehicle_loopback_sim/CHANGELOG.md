# vehicle_loopback_sim Changelog

## Unreleased

- Added simulated front-obstacle recognition that publishes the existing
  `/obstacle/front_range` and `/obstacle/front_blocked` interfaces from arena
  map obstacles.
- Added GUI placement and clearing for temporary simulated obstacles.
- Tuned simulated obstacle recognition to use a fixed `0.25 m` trigger distance.
- Snapped GUI temporary obstacles to the nearest road centerline and oriented
  them crosswise to the lane so they block the clicked passage.
- Matched GUI temporary obstacle width to the `0.20 m` road width, set obstacle
  depth to `0.10 m`, and drew a dashed forward detection fan in the GUI.
- Disabled the simulated obstacle stable-detection delay by default so front
  obstacle reports are published immediately.
- Fixed fixed-distance obstacle detection so obstacles already inside the
  `0.25 m` trigger range are reported instead of requiring an exact boundary
  crossing.
- Published detected simulated obstacles on `/perception/dynamic_obstacle` so
  the production mission manager can replan with an obstacle polygon.
- Switched the default planner/control loopback launch to the production
  `navigation_mission_manager` instead of the compatibility
  `loopback_mission_node`.
