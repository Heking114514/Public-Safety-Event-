# contrast_navigation_core

本包提供 `final_navigation` 使用的导航核心静态库，包含路径跟踪、障碍恢复、姿态恢复和
转弯动作逻辑。ROS 节点入口与运行参数由 `final_navigation` 功能包负责。

核心库保留现有 C++ 头文件路径及命名空间，避免在迁移控制器入口时扩大接口改动；这些
内部兼容名称不代表运行时还会启动旧导航器。本包不依赖仿真器，`vehicle_loopback_sim`
通过 ROS 运行时依赖启动 `final_navigation`。
