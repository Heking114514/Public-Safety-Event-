# rosbag 录制位置说明

## 一句话

rosbag 不在仓库里，在**外接 NTFS 硬盘**上，路径是
`<录制盘>/rosbag_recording/navigation_runs/<run_id>/bag/`。

## 两块录制盘

| 挂载点 | 卷标 / 设备 | 容量 | 说明 |
|---|---|---|---|
| `/media/hjh/E` | `E` / `/dev/nvme1n1p2` | 854G | **启动脚本首选**，新录制写这里 |
| `/media/hjh/Data` | `Data` / `/dev/nvme0n1p4` | 654G | 备用，历史 run 和离线分析产物 |

`scripts/start_visual_navigation.sh` 按 **E → Data** 的顺序挑第一个可写的，选中后新录制就写到它的 `rosbag_recording/` 下。也可以用 `NAVIGATION_RUNS_ROOT` 直接指定。

## 目录结构

```
<录制盘>/rosbag_recording/
├── navigation_runs/
│   └── <run_id>/                     # run_id = 录制起始时间_提交号，如 20260926T033028.634025Z_38ea715c18dd
│       ├── bag/
│       │   ├── bag_0.db3.zstd        # zstd FILE 压缩的 sqlite 分片
│       │   ├── bag_1.db3.zstd
│       │   ├── ...
│       │   └── metadata.yaml         # 话题表 + 消息数 + 各分片起止时间
│       ├── parameters/               # 各节点参数快照
│       └── run_manifest.json         # 状态、退出码、话题、构建指纹
├── latest_navigation_run  ->  navigation_runs/<最新 run>
├── latest_navigation_bag  ->  navigation_runs/<最新 run>/bag
└── bag_exports/                      # scripts/export_rosbag.py 的输出
```

`latest_navigation_run` / `latest_navigation_bag` 是指向最新一次的软链，所以不用记 run_id 也能找到最新的包。

**保留策略**：只保留最近 **3** 份完整 bag（`RETAIN_BAGS=3`），更早的 bag 目录会被删除，但 `run_manifest.json` 和参数快照会留下。要长期留档就趁早拷走。

## 怎么读

包是 zstd 压缩的，本机 rosbag2 没有 zstd 解压插件，**不能直接打开**，先解压：

```bash
# 最新一份，解压 + 分析 + 出视频 + 写 manifest
python3 scripts/export_rosbag.py

# 指定 run_id 前缀
python3 scripts/export_rosbag.py 20260926T033028

# 指定 bag 目录
python3 scripts/export_rosbag.py /media/hjh/E/rosbag_recording/navigation_runs/<run_id>/bag

# 只解压和分析，不出视频
python3 scripts/export_rosbag.py --no-video
```

输出到 `<录制盘>/rosbag_recording/bag_exports/<run_id>/`：

- `decompressed/` —— 解压后的 `.db3`，**带一份重写过的 `metadata.yaml`，可以直接当 bag 打开**
- `analysis/` —— 路线分段、里程计、路口推进、转弯中心的 csv + `summary.json` + png
- `<run_id>.mp4` —— 相机画面，H.264，Ubuntu 直接能播
- `export_manifest.json` —— 话题表、消息数、视频信息、分析状态

解压**不会删除原始 `.db3.zstd`**。分析就是调 `scripts/analyze_navigation_bag.py`，
约 37 秒，纯里程计/路线几何计算，不依赖地图；包里没有路线时自动标记为 skipped。
`--no-analysis` 可跳过。

要单独重跑分析：

```bash
python3 scripts/analyze_navigation_bag.py \
  /media/hjh/E/rosbag_recording/bag_exports/<run_id>/decompressed \
  --output-dir analysis_bags/<run_id>
```

## 挂载（重要）

NTFS 挂载**不跨重启**，重启后盘会掉，启动脚本会报
`no writable external rosbag directory found`。启动脚本和 `export_rosbag.py` 都会自己尝试挂载，也可以手动：

```bash
udisksctl mount -b /dev/disk/by-label/E      # 不需要密码
```

如果脏卷（Windows 快速启动导致）udisks 拒绝，就用：

```bash
sudo mkdir -p /media/hjh/E
sudo mount -t ntfs3 -o force /dev/disk/by-label/E /media/hjh/E
```

`-o force` 只挂载不写盘，比 `ntfsfix` 安全（后者会改盘）。实在不行才 `sudo ntfsfix`。

**坑**：udisks 按卷标命名挂载点。如果 `/media/hjh/E` 已经作为普通目录存在（哪怕是空目录），它会默默挂到 `/media/hjh/E1`，而启动脚本只认规范路径，就会误报「找不到录制盘」。所以**调用 udisks 前不要预先创建挂载点**。真遇到 `E1`，卸载后删掉那个空目录再挂：

```bash
sudo umount /media/hjh/E1 && sudo rmdir /media/hjh/E
udisksctl mount -b /dev/disk/by-label/E
```
