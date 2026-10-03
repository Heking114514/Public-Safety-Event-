# vehicle_config

`config/vehicle_physical.yaml` is the single source of truth for measured car
geometry. It contains the chassis envelope, differential-drive geometry and
calibration, encoder resolution, physical speed ceiling, axle-to-`base_link`
offset, navigation tracking point, and camera mounting pose.

Coordinate convention follows ROS body axes: `+x` is forward, `+y` is left,
and `+z` is up. The camera is 15 mm right of the centreline, so its configured
`y_m` is `-0.015`.

Launch files load this package after their algorithm-specific YAML. Do not copy
these measurements into another package config; edit `vehicle_physical.yaml`
and rebuild `vehicle_config` instead.
