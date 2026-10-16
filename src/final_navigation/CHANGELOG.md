# 变更记录

## 未发布

- 将 dm2325 的 Contrast 控制器统一命名为 `final_navigation`，删除旧导航控制器、旧
  可执行文件、旧 launch 和旧参数入口。
- 将路线任务真值检查点、闭环仿真和避障验证收敛为单一 `final_navigation` 节点；删除
  双控制器选择参数，仿真器和检查点服务使用独立的 final_navigation 命名空间。
- 将仿真专用任务脚本和验证测试迁移到 `vehicle_loopback_sim`，避免导航包与仿真包形成
  运行时依赖循环；导航包只保留自身转弯测试和实车启动文件。
- 保留 dm2325 自身参数。普通直线路线速度维持 `0.10 m/s`，并通过已有线速度加减速
  限幅器平滑输出；转弯、读卡、倒车和恢复动作继续使用各自原参数。
- 统一安装和文档中的核心库目标名为
  `contrast_navigation_core::contrast_waypoint_navigator_core`。
- 默认实车入口使用 `final_navigation_serial_bringup.launch.py`，节点名、服务地址和
  控制器可执行文件均为 `final_navigation`。
