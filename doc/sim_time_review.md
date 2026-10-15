# 仿真时间倍率验收记录

## 2026-10-15：实现与边界

- Contrast 避障闭环使用唯一 `/clock`，控制器、任务总控、真值车辆和消息时间戳统一使用仿真时间。
- GUI 的“仿真时间”滑块支持运行中调节 0.1～10 倍，等待时钟回执后显示实际倍率，不重置车辆或当前动作。
- 线速度、角速度、加速度和固定转弯参数不变；`playback_speed` 与 `control_rate_multiplier` 固定为 1，防止重复加速。
- 当前支持 `obstacle_validation.launch.py` 和 `contrast_comparison.launch.py controllers:=contrast`；生产控制器及双车入口保留原时间机制。

## 动态切换实跑

正式默认起点和任务入口，运行中通过 GUI HTTP 接口执行 1→3→1→3 倍。
实测仿真秒/现实秒分别为 0.999809、3.000165、0.999892、3.000022，时钟没有倒退。
正常首个转弯完成；注入障碍后恢复完成，并实测继续直行 0.200569 m。
按仿真时间计算的直线速度误差最大为 0.000003583 m/s，未出现 FAULT 或碰撞保护介入。
本次运行现实耗时约 28 秒。

证据：`/tmp/sim_scale_verify_20261015_214334/result.json`，同目录保存 `observations.json` 和 `launch.log`。

## 3 倍严格避障回归

- `south_nominal`：通过，恢复后实测直行 0.201062 m。
- `after_normal_90deg_turn`：通过，确认先完成正常转弯，恢复后实测直行 0.200631 m。
- 两场均通过固定入口、实测停车换向、60°/30°特定转弯和恢复交接协议检查，`protocol_error` 为空。

证据：`/tmp/sim_scale_strict_20261015_214438/summary.json`，各场景保留轨迹、里程计、命令和日志。

## 编译与测试限制

使用低优先级、单线程构建四个相关包，构建通过。新增控制时钟 3 项和任务时钟 4 项测试通过，控制核心、任务总控和仿真器原有测试套件通过。
Contrast 的三个 Python 套件通过；原有 `ContrastTurn.FullLeftAndRightAtLaunchWithRealOutputAndCollision` 仍失败，使用生产参数的转弯被真实车身碰撞模型拒绝，未改参数或弱化断言。

本次未验收三层全任务、实车 RFID 和播报，也未完成所有倍率及路口组合的覆盖；0.1～10 是可设范围，正式闭环实跑证据为上述 1 倍与 3 倍。
