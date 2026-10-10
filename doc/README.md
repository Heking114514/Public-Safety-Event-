# 规划器与控制器问题记录

本目录记录 `new_map_planner` 与 `visual_navigation` 的接口契约、门控核查和 rosbag 证据。

详细记录见：

- [规划器控制器契约审查](规划器控制器契约审查.md)
- [转弯动作重构总设计](转弯动作重构总设计.md)
- [控制器 agent 任务书](控制器_agent任务书.md)
- [规划器 agent 任务书](规划器_agent任务书.md)

最近一次 rosbag 分析产物位于外接盘，不纳入仓库：

- `/media/hjh/E/rosbag_recording/bag_exports/20261010T080931.334638Z_f19974ea9e7e/analysis`
- 对照包：`/media/hjh/E/rosbag_recording/bag_exports/20261010T062140.645065Z_f19974ea9e7e/analysis`

当前文档只记录证据和待验证事项，不代表已经修改了规划器或控制器。生产修改必须在专项实验确认后进行，并同步更新功能包 changelog。

