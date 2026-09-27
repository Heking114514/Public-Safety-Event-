# rectangle_odometry_test

独立的闭环底盘运动测试包。它不实现新的控制器，也不修改里程计；只调用现有
`visual_navigation/waypoint_navigator` 的动态路线接口：

```text
/waypoint_navigation/route_input
```

节点启动后等待 `/odometry/fused` 的有效位姿，以当前车体位置和朝向为矩形路线起点，
生成一个边长 `1.8 m` 的顺时针矩形：

```text
起点 --(+X)--> 右侧
  ^             |
  |             v
左侧 <--(-X)-- 下侧
```

实际坐标方向会按照启动时车辆的航向旋转。路线有四条边，最后回到起始位置；
现有导航器在最终航点完成时不会额外原地转向，因此最终车头方向是最后一条边的方向。
完成后节点会打印 fused 里程计的闭合误差：

```text
dx, dy, position_error, dyaw
```

其中 `dyaw` 是相对启动朝向的角度差；走完一个几何矩形后，理想值约为 `+90 deg`，
不要把它当成闭环后的姿态误差。

## 使用

先启动现有的里程计、融合、导航和串口链路。例如：

```bash
cd /home/hjh/cup_corporate/Public-Safety-Event-
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/start_visual_navigation.sh --fused-odom
```

默认导航启动使用 `tracking_point_offset_x=0.0`，也就是控制 `base_link`
中心沿路线运动。做原地 pivot 对比时可以临时改成：

```bash
./scripts/start_visual_navigation.sh --fused-odom --tracking-point-offset-x 0.05
```

确认车辆周围没有障碍物、急停和串口状态正常后，另开终端启动测试：

```bash
cd /home/hjh/cup_corporate/Public-Safety-Event-
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rectangle_odometry_test rectangle_test.launch.py
```

启动这个测试会自动向已有导航器提交路线，并可能使实车运动。速度和航点容差使用
现有 `waypoint_navigator` 的运行参数，不在动态 `nav_msgs/Path` 中重复配置：

```bash
ros2 launch rectangle_odometry_test rectangle_test.launch.py \
  side_length_m:=1.8
```

默认 `anchor_to_current_pose:=true`。如果要使用固定的 map 坐标起点，可设置：

```bash
ros2 launch rectangle_odometry_test rectangle_test.launch.py \
  anchor_to_current_pose:=false start_x:=0.0 start_y:=0.0 start_yaw:=0.0
```

圆弧过弯测试：

```bash
ros2 launch rectangle_odometry_test rectangle_test.launch.py \
  corner_mode:=arc corner_radius_m:=0.25 corner_samples:=16 straight_step_m:=0.05
```

`corner_mode:=arc` 会发布无 stop marker 的密集圆角路线，让导航器持续输出
`linear.x + angular.z`，用于对比“边走边转”和默认 `corner_mode:=pivot`
的原地转弯效果。圆弧模式控制的是 `base_link` 中心线，启动导航时保持
`tracking_point_offset_x=0.0`。直线段也会按 `straight_step_m` 拆成小航点，
避免导航在第一个远目标前提前套用圆弧曲率。

arc 模式中当前车体位姿是第一条直线的起点，也是圆弧路线的切线起点，
不是几何上的尖角。速度由现有 `waypoint_navigator` 负责，测试包只发布
`/waypoint_navigation/route_input`，不直接发布 `/cmd_vel_nav`。

首轮实车测试按工程规范使用独立慢速参数文件，不改变生产默认导航配置：

```bash
./scripts/start_visual_navigation.sh \
  --fused-odom \
  --navigation-parameters-file \
  src/rectangle_odometry_test/config/slow_arc_navigation.yaml
```

另开终端确认安全条件后，再提交圆弧路线：

```bash
ros2 launch rectangle_odometry_test rectangle_test.launch.py \
  corner_mode:=arc corner_radius_m:=0.35 corner_samples:=16 straight_step_m:=0.05
```

慢速 profile 将直线段线速度限制为 `0.04 m/s`，利用曲率限速让
`0.25 m` 圆角约降到 `0.03 m/s`，路径角速度上限为 `0.12 rad/s`。
每次启动的参数文件绝对路径会写入 `run_manifest.json`，
方便和导出的 rosbag 一起复核。确认轨迹稳定后，再单独提高 profile，
不要直接改 `visual_navigation/config/waypoint_navigation.yaml`。

## 离散动作原语实验

使用用户提供的 A* / 差速动作原语思路做矩形测试时，统一启动脚本提供独立模式：

```bash
./scripts/start_visual_navigation.sh \
  --fused-odom \
  --primitive-rectangle-test
```

该模式使用 `/odometry/fused` 作为实时位姿，按离散 `v`、`w` 和持续时间动作
搜索当前矩形目标；执行过程中根据实测里程计检查偏差，偏差过大时从当前位姿重新
规划。实验控制器直接发布 `/cmd_vel_nav`，启动脚本会自动关闭
`waypoint_navigator`，避免两个控制器同时发布速度。原有融合状态、执行器健康、
障碍和任务驻停安全门仍然生效，rosbag 和 `run_manifest.json` 也会照常记录。

动作单位统一为米、秒和弧度，不直接把原脚本的 `mm/step`、`deg/step` 当作
ROS 速度。默认使用 `30 mm` 平移步长、`3/6/12°` 曲率动作和 `±6°` 原地
动作，并用 `0.50 s` 作为一个真实动作时基，即直线约 `0.06 m/s`。执行器实际
转角落后于命令时，控制器会用实时 fused 航向重新规划；同时在当前转弯目标仍未
达到时保持较慢的角向修正，避免规划动作模型已经变成直行、但车体实际还没有转够。
在直线动作上还会以目标航向做小角度比例闭环：误差小于 `1.5°` 不动作，超过后
按 `1.5` 倍误差修正，角速度限制为 `8°/s`，防止直线逐渐走斜。默认到点航向
容差为 `5°`，弯道补偿从 `6°` 开始，补偿速度限制为 `10~25°/s`。这些阈值可以通过
`primitive_rectangle_test.launch.py` 的参数单独调节。这是独立实验模式，默认启动
脚本仍使用 `waypoint_navigator`。

## 构建

```bash
colcon build --symlink-install --packages-select rectangle_odometry_test
source install/setup.bash
```
