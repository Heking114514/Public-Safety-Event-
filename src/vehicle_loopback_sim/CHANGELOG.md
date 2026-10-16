# 变更记录

## 未发布

- 将 loopback 启动、任务辅助节点和契约测试迁移到单一 `final_navigation` 控制器入口，
  删除旧双控制器选择参数及仅服务于退役分支的 `loopback_route_controller`。
- 为仿真包声明 `final_navigation` 运行时依赖；从 `final_navigation` 移除反向仿真依赖，
  让包依赖保持单向。
- 把路线任务检查点辅助脚本改名为 `final_navigation_mission_node`，并同步更新节点、服务、
  话题和验证测试。
- 更新 loopback 使用说明，记录单车 launch、仿真障碍检测和正式闭环的时间倍率要求；
  为独立质点验证入口提供显式静态碰撞开关。
