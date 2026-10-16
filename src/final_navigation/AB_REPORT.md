# final_navigation 闭环验证记录

本文件保留迁移前 Contrast 动作的路口验证数据，当前控制器已经统一命名为
`final_navigation`。历史记录不再启动或依赖旧控制器。

## 验证条件

- `final_navigation`、地图规划器和 loopback 真值车辆使用同一份路线、地图和
  `vehicle_config` 物理参数。
- 静态车体碰撞保护保持开启；任务检查点要求路线 ACK 匹配、位置误差不超过 `0.05 m`、
  线速度不超过 `0.005 m/s`、角速度不超过 `0.02 rad/s`，并连续保持一秒。
- 检查点只用于闭环授权，不模拟实物 RFID 读取、定位修正或语音播报。

## 结果

首个右转和左转均完成 `CAPTURE -> APPROACH_ENTRY -> TURN_60 -> DIAGONAL_CREEP ->
TURN_30 -> COMPLETE` 阶段，并进入出口直道。转弯线速度为 `0.035 m/s`，角速度受本包
YAML 的上下限约束；车辆经过完整车体扫掠检查后到达后续检查点。

## 复现

```bash
source install/setup.bash
ros2 launch vehicle_loopback_sim final_navigation_comparison.launch.py gui:=false
```

单方向测试可传入 `planner_mode:=layer1`、`target_ids:=1` 或 `target_ids:=12`，并设置
`return_home:=false`。GUI 默认地址为 `http://127.0.0.1:8765/`。

## 结论边界

该记录只说明当前 loopback 地图、车体和轴心模型下的动作阶段可以完成，不能替代实车
标定、真实电机滞后/打滑、视觉感知、物理 RFID 或定位修正验收。实车应用前仍需低速分
阶段验证，并保留转弯无进展锁停。
