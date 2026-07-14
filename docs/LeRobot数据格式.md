# LeRobot v2.1 / v3 数据格式

## 稳定特征契约

无论物理布局是 v2.1 还是 v3，本项目提交给 `LeRobotDataset` 的 feature schema 相同：

| feature | dtype | shape |
|---|---|---|
| `observation.state` | `float32` | `[23]` |
| `action` | `float32` | `[23]` |
| `observation.images.head_left` | `video` 或 `image` | `[H,W,3]` |
| `observation.images.head_right` | `video` 或 `image` | `[H,W,3]` |
| `observation.images.left_wrist` | `video` 或 `image` | `[H,W,3]` |
| `observation.images.right_wrist` | `video` 或 `image` | `[H,W,3]` |
| `task` | LeRobot task | 每帧继承当前 episode 文本 |

默认 H/W 为 `360/640`，数据集 FPS 为 `15`。`--record_images` 改用 image feature，否则使用 MP4 video feature。

## 23 维顺序

```text
 0 FoldingModularJoint02_Joint
 1 FoldingModularJoint03_Joint
 2 Trunk_Joint
 3 ArmL02_Joint
 4 ArmL03_Joint
 5 ArmL04_Joint
 6 ArmL05_Joint
 7 ArmL06_Joint
 8 ArmL07_Joint
 9 ArmL08_Joint
10 JawBlock01_Joint   physical right gripper
11 JawBlock02_Joint   physical right gripper
12 ArmR02_Joint
13 ArmR03_Joint
14 ArmR04_Joint
15 ArmR05_Joint
16 ArmR06_Joint
17 ArmR07_Joint
18 ArmR08_Joint
19 JawBlock03_Joint   physical left gripper
20 JawBlock04_Joint   physical left gripper
21 Head02_Joint
22 Head03_Joint
```

夹爪索引看似交叉是现有 ABI 的历史约定，不要根据数组邻接位置交换。

## 写入生命周期

```text
start/record_start
  -> LeRobotDataset.create（只在首次需要时）
  -> 每个同步采样 add_frame
pause
  -> 不 add_frame，episode 保持打开
resume
  -> 重新标定后继续 add_frame
stop/record_stop
  -> save_episode
正常退出/record_finalize
  -> finalize（v3）或 consolidate（旧 v2.1 API）
```

录制器使用后台有界队列，写图像或视频不会直接阻塞物理主循环。`/status` 中必须监控：

- `dropped_frames == 0`
- `error == null`
- `episode_frames > 0`
- `record_fps <= camera_hz`

## v2.1 与 v3

本项目不手工伪造 LeRobot 目录；安装的官方 `LeRobotDataset` 决定实际 Parquet、视频和元数据布局。这样可以随 v3 的 shard/metadata writer 变化而保持兼容。

v2.1 常见 episode-based 布局类似：

```text
data/chunk-000/episode_000000.parquet
videos/chunk-000/<camera>/episode_000000.mp4
meta/episodes.jsonl
```

v3 使用聚合的 Parquet/video shards 与新的 metadata 表；不要写脚本假定每个 episode 必有单独文件。训练端应通过官方 `LeRobotDataset` loader 读取。

同一次进程只能按已安装 LeRobot 的原生格式写一个 dataset。要同时交付两种格式：

1. 在 v2.1 环境录制到 `datasets/name_v2`；
2. 用官方转换脚本生成 v3，或在 v3 环境另录到 `datasets/name_v3`；
3. 分别运行本项目校验器。

## 校验

```bash
python -m pico_isaaclab.validate_dataset /path/to/dataset --repo_id local/name
```

校验器会检查：

- `meta/info.json`、data parquet、video/image 文件是否存在且非空；
- 四相机 shape 和 feature key；
- 23 维 dtype、shape、motor names 与顺序；
- 官方 loader 能否打开，并抽查首尾帧的有限值和图像通道。

仅做文件/元数据检查：

```bash
python -m pico_isaaclab.validate_dataset /path/to/dataset --skip_loader
```

## 常见错误

- `format v2 requested, but installed ... v3`：更换 LeRobot 环境或使用 `auto/v3`，不要继续写入。
- `Refusing to save an empty episode`：开始后尚未完成标定或相机没有有效帧。
- `dataset has been finalized`：使用新的 `record_root` 重启，不能在 finalized root 追加。
- `dropped_frames > 0`：降低 `camera_hz/record_fps/分辨率` 或增加 `record_queue_size`，并重新采集该 episode。

