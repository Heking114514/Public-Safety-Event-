# contrast_navigation_core

这是 `final_navigation` 使用的独立 Contrast 航点导航核心。

源码从原导航核心拆出并由 `final_navigation` 链接，保留 C++ 命名空间、消息接口、参数接口和控制话题接口。旧导航控制器已移除，生产入口只保留本包的 Contrast 核心。

当前包只提供静态核心库，不直接启动 ROS 节点；由 `final_navigation` 的启动文件运行。

道路恢复由 `pose_recovery.cpp`（地图和车身运动学）、`recovery_routes.cpp`（T/X 完整动作预检）、`recovery_execution.cpp`（实时执行和停车确认）及 `turn_recovery.cpp`（正常转弯中遇障返回）组成。只有实际出口通过道路、航向和安全校验后才报告完成；完整结果见 [本轮核对记录](../final_navigation/RECOVERY_REVIEW.md)。
