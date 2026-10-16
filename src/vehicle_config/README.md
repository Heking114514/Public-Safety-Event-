# vehicle_config

`config/vehicle_physical.yaml` is the single source of truth for measured car
geometry. It contains the chassis envelope, differential-drive geometry and
calibration, encoder resolution, physical speed ceiling, axle-to-`base_link`
offset, navigation tracking point, and camera mounting pose.

The configured chassis envelope is `132 mm × 132 mm` (length × width).

坐标遵循 ROS 车体约定：`+x` 向前、`+y` 向左、`+z` 向上。
相机横向安装在车身中轴线上，配置为 `camera.y_m: 0.0`。

Launch files load this package after their algorithm-specific YAML. Do not copy
these measurements into another package config; edit `vehicle_physical.yaml`
and rebuild `vehicle_config` instead.
