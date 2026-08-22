# 下一步任务：保证实车运行版本与 rosbag 可追溯

你在仓库 `/home/hjh/cup_corporate/Public-Safety-Event-` 中工作。

## 任务目标

修复 `problem to solve.md` 中的 `P0-02` 和 `P3-08`：

1. 启动实车栈时，不能因为 `install/` 中存在旧文件就跳过当前源码的构建。
2. 使用 `--no-build` 时，不能未经证明就使用已有二进制；必须确认它们与当前源码状态一致，否则明确失败。
3. 每次运行生成独立 rosbag，不覆盖任何历史记录。
4. 每个 rosbag 必须附带足以还原本次运行源码、二进制、配置和启动参数的清单。

这是运行基础设施修复，不是控制算法修改。

## 当前已确认的问题

- `scripts/start_visual_navigation.sh` 的 `runtime_ready()` 只检查文件是否存在。
- 源码修改后，只要旧可执行文件仍在，脚本就可能直接运行旧程序。
- 脚本会删除 `latest_navigation_bag` 后再录制，新一轮运行会丢失上一轮记录。
- 当前 bag 的 `metadata.yaml` 只有 rosbag 标准信息，没有源码版本、dirty 状态、二进制哈希、配置快照或启动参数。
- 仓库经常带有尚未提交的实车调参和代码修改，不能只记录 Git commit；相同 commit 下的两个 dirty 工作树可能具有完全不同的行为。

## 不可违反的边界

- 不修改跟线、转弯、减速、停车、短时失定位、隧道容错和速度参数。
- 不修改编码器是否进入 EKF；这是后续单独任务。
- 不修改 ROS topic、frame、TF 所有权和消息语义。
- 不改下位机代码。
- 不删除、覆盖或清空现有 `latest_navigation_bag/` 及任何历史 bag。
- 不使用 `git reset`、`git checkout` 或其他方式丢弃当前工作树修改。
- 当前工作树已有其他任务留下的修改，必须保留并与其兼容。
- 不以“要求工作树必须 clean”规避 dirty 工作树追踪；实车调试允许 dirty，但必须可识别。
- `problem to solve.md` 只记录仍存在的问题，不写修复方案、完成总结或宣传性内容。

## 要求一：构建一致性

默认启动路径必须在停止现有 ROS 节点和操作相机之前，完成当前源码的增量构建。推荐直接复用现有 CMake/colcon 增量构建：

- 检查并增量构建 ORB-SLAM3 核心库。
- 增量构建脚本实际会启动的 ROS 2 packages。
- 只有构建成功后才停止旧节点并启动新栈。
- 不再用“可执行文件存在”代表“当前源码已经构建”。

保留 `--no-build`，但其语义必须改为：

- 不执行构建。
- 读取上一次成功构建生成的构建清单。
- 比较当前源码指纹、所需文件和实际二进制哈希。
- 任一信息缺失、不一致或无法验证时立即失败，并清楚说明需要重新构建。

源码指纹至少覆盖：

- 当前完整 Git commit。
- staged 和 unstaged tracked diff。
- 会影响运行的 untracked 源码、launch、config、脚本、接口文件。
- 相关 submodule revision 和修改状态。

不得只用 commit、文件存在性或单一目录 mtime 作为一致性证明。指纹计算应排除 `build/`、`install/`、`log/`、bag 目录等生成物。

成功构建后记录实际运行产物的 SHA-256，至少包括：

- ORB-SLAM3 共享库。
- `stereo-inertial`。
- `imu_rpy_filter_node`。
- `wheel_odometry_node`。
- `fusion_gate_node`。
- `map_odom_correction_node`。
- `waypoint_navigator`。
- 启用串口时的 `cmd_vel_serial_node`。

## 要求二：运行目录和 rosbag 保留

为每次启动建立唯一运行目录。推荐结构如下，等价且更简单的结构也可以：

```text
navigation_runs/<时间>_<短commit>[_序号]/
  bag/
  run_manifest.yaml
  parameters/
latest_navigation_run -> navigation_runs/<本次运行>
latest_navigation_bag -> navigation_runs/<本次运行>/bag
```

要求：

- 名称碰撞时生成新名称或明确失败，绝不能覆盖已有目录。
- 现有 `latest_navigation_bag/` 如果是普通目录，必须原样保留或无损迁移到历史目录，不能删除。
- `latest_navigation_bag` 继续作为兼容入口，最终指向最近一次 bag。
- 最近一次链接的更新应是原子的，且不能把失败前的有效记录变成悬空链接。
- 即使运行异常退出，已经产生的 bag 和运行清单也要保留，并记录退出状态。

## 要求三：运行清单

每次运行必须生成机器可读的 YAML 或 JSON 清单。至少记录：

- 唯一 run ID、开始时间、结束时间、运行状态和退出码。
- 仓库绝对路径、branch、完整 commit、short commit、dirty 状态和源码指纹。
- submodule revision/dirty 状态。
- 原始命令行参数及解析后的有效设置。
- ROS distro、相机序列号、串口设置、是否启用 IMU/SLAM IMU/串口等开关。
- 本次构建清单标识和所有关键运行二进制 SHA-256。
- 所有实际使用的 launch/config/route 文件路径及 SHA-256。
- rosbag 输出路径和完整 topic 列表。
- 关键节点的有效参数快照，至少包含 fusion gate、EKF、map correction、IMU filter、navigator；启用串口时包含串口节点。

参数快照必须来自启动后的实际 ROS 节点，例如使用带超时的 `ros2 param dump`。如果某个节点未启动或抓取失败，清单必须明确记录失败节点，不能伪装成完整快照。

清单写入失败属于启动失败，不能在没有基本版本信息的情况下继续实车运行。

## 工程实现要求

- 优先把“源码指纹、构建清单、唯一运行目录、清单写入”做成可独立测试的 shell helper 或小脚本，避免继续扩大主启动脚本。
- 不引入当前系统没有的 `jq`、`shellcheck` 等强制运行依赖。
- 所有路径必须正确处理空格，变量引用完整。
- 更新清单时使用临时文件加原子 rename，避免异常退出留下半个文件。
- 对目录迁移、链接替换和进程退出保持幂等；重复调用 cleanup 不得破坏记录。
- 不要为了本任务重构无关启动逻辑。

## 文档与问题清单

更新根 `README.md`，简洁说明：

- 默认会执行增量构建。
- `--no-build` 只接受已验证匹配的构建。
- 历史运行目录、`latest_navigation_bag` 兼容链接和运行清单的位置。
- 如何从清单确认 commit、dirty 指纹、二进制哈希和有效参数。

完成全部验收前，不得删除 `problem to solve.md` 中的 `P0-02` 或 `P3-08`。全部验收通过后，从问题清单删除这两个已解决条目；不要在问题清单中留下“已修复”说明。

## 必须完成的验证

1. `bash -n scripts/start_visual_navigation.sh` 以及所有新增 shell 脚本通过。
2. 对 helper 做无硬件测试，使用临时目录证明连续两次运行会得到不同目录，第一次记录仍存在。
3. 证明已有普通目录形式的 `latest_navigation_bag` 不会被删除。
4. 证明 dirty tracked 文件变化会改变源码指纹。
5. 证明相关 untracked 源码或配置变化会改变源码指纹。
6. 证明 `build/`、`install/`、`log/` 和 bag 内容变化不会改变源码指纹。
7. 证明 `--no-build` 在源码与构建清单不一致时失败，在完全一致时通过。
8. 证明关键二进制被修改后，`--no-build` 校验失败。
9. 对本任务触及的 ROS packages 执行 `colcon build` 和 `colcon test`。
10. 执行 `git diff --check`。

不连接相机或底盘时，不要为了验证而启动真实硬件。应通过 helper 测试、命令 mock 或明确的 prepare/verify 模式完成自动验证。

## 交付说明

完成后报告：

- 修改文件。
- 构建一致性如何得到证明。
- bag 如何命名和保留。
- 清单字段与参数快照位置。
- 执行过的测试及结果。
- 仍未解决的问题。

不要提交 Git commit，等待主审查者 review。
