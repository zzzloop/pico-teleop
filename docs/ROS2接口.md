# ROS 2 接口与服务器进程边界

## 1. 唯一通信链路

```text
Windows Unity
  └─ ROS-TCP / TCP 10000
       └─ server: ros_tcp_endpoint
            └─ server-local ROS 2 topics/services
                 └─ server: pico_teleop_bridge
                      └─ UDP 127.0.0.1:9765
                           └─ server: Isaac Lab process
```

ROS 2、endpoint、bridge 和 Isaac Lab 都在服务器。Windows 与 PICO 不加入 DDS domain。Isaac Lab Python 不导入 `rclpy`，bridge 到仿真的 UDP 地址已在代码中硬编码为回环地址，不能配置成跨机地址。

固定网络地址：Windows Unity 为 `192.168.50.61`，服务器为 `192.168.50.227`；Unity 的 ROS Settings 必须连接 `192.168.50.227:10000`。

## 2. 一条命令启动 ROS 层

首次构建：

```bash
cd /home/kemove/zzk_data/pico-teleop/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

每次启动：

```bash
source /opt/ros/humble/setup.bash
source /home/kemove/zzk_data/pico-teleop/ros2_ws/install/setup.bash
ros2 launch pico_teleop_bridge pico_teleop.launch.py
```

launch 会同时启动：

- `ros_tcp_endpoint`，监听 `0.0.0.0:10000`，接收 Windows Unity TCP 连接。
- `pico_teleop_bridge`，订阅 PICO 话题、提供控制服务、以 60 Hz 向 `127.0.0.1:9765` 发送协议包，并接收仿真状态确认。

## 3. 输入话题

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/pico/left_controller/pose` | `geometry_msgs/msg/PoseStamped` | best effort, depth 1 | 左手柄，米，四元数 xyzw |
| `/pico/right_controller/pose` | `geometry_msgs/msg/PoseStamped` | best effort, depth 1 | 右手柄 |
| `/pico/head/pose` | `geometry_msgs/msg/PoseStamped` | best effort, depth 1 | 头显，保留用于状态扩展 |
| `/pico/controllers/joy` | `sensor_msgs/msg/Joy` | reliable, depth 10 | trigger、grip、按键 |
| `/pico_teleop/task` | `std_msgs/msg/String` | reliable, depth 10 | 下一 episode 的任务文本 |

`Joy.axes` 固定为：

```text
0 left_trigger
1 left_grip
2 right_trigger
3 right_grip
```

`Joy.buttons` 默认是：

```text
0 A      start/resume
1 B      pause
2 X      record toggle
3 Y      stop/save
4 menu   calibrate
```

夹爪使用 trigger；jaw target 默认 `0 m` 张开、`0.041 m` 闭合。

## 4. 控制服务

全部使用 `std_srvs/srv/Trigger`：

| 服务 | 行为 |
|---|---|
| `/pico_teleop/start` | 进入 running、清空标定；配置允许时自动开始 episode |
| `/pico_teleop/pause` | 保持机器人、暂停写帧，WebRTC 和状态继续 |
| `/pico_teleop/resume` | 进入 running，下一有效 pose 重新标定 |
| `/pico_teleop/stop` | 保持机器人并保存 active episode |
| `/pico_teleop/reset` | 保存 active episode、恢复初始 qpos、回到 idle |
| `/pico_teleop/calibrate` | 清空 anchor，下一有效 pose 重建标定 |
| `/pico_teleop/record_start` | 只开始新 episode |
| `/pico_teleop/record_stop` | 保存当前 episode |
| `/pico_teleop/record_abort` | 丢弃未保存 buffer，需所装 LeRobot API 支持 |
| `/pico_teleop/record_finalize` | 关闭 dataset writer；本进程不能继续录制 |

服务返回 `success=true` 只表示命令已排队。bridge 为每条命令生成单调 `event_id`，在仿真回传 `teleop.last_event_id` 前持续重发；仿真按 id 去重。因此 start/pause/stop 不依赖某一个 UDP 包是否丢失。

手工控制示例：

```bash
ros2 service call /pico_teleop/start std_srvs/srv/Trigger '{}'
ros2 service call /pico_teleop/pause std_srvs/srv/Trigger '{}'
ros2 service call /pico_teleop/resume std_srvs/srv/Trigger '{}'
ros2 service call /pico_teleop/stop std_srvs/srv/Trigger '{}'
```

## 5. 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/pico_teleop/status` | `std_msgs/msg/String` | JSON，transient-local/reliable |
| `/pico_teleop/joint_states` | `sensor_msgs/msg/JointState` | BRX 公共 23 维顺序 |

关键状态示例：

```json
{
  "ready": true,
  "teleop": {
    "session_state": "running",
    "safety_hold": false,
    "hold_reason": "",
    "calibrated": true,
    "packet_age_ms": 8.2,
    "recording_gate": true,
    "last_event_id": 123
  },
  "recorder": {
    "enabled": true,
    "active": true,
    "episode_frames": 123,
    "dropped_frames": 0,
    "dataset_format_version": "v3.0",
    "error": null
  }
}
```

检查：

```bash
ros2 topic echo --once /pico_teleop/status
ros2 topic hz /pico_teleop/joint_states
```

## 6. 端口和 DDS

| 端口 | 可见范围 | 用途 |
|---|---|---|
| TCP 10000 | Windows 主机 → 服务器 | Unity ROS-TCP |
| UDP 9765 | 仅服务器 `127.0.0.1` | bridge ↔ Isaac Lab |

服务器的 ROS 2 进程位于同一台机器，不需要跨机 DDS discovery、组播转发、ROS_DOMAIN_ID 穿透或在 Windows 安装 ROS 2。若 `ss -lunp | grep 9765` 显示 `0.0.0.0:9765`，说明运行的不是本项目当前代码，应立即停止并检查版本。

## 7. 配置文件

服务器配置文件：

```text
/home/kemove/zzk_data/pico-teleop/ros2_ws/src/pico_teleop_bridge/config/default.yaml
```

允许调整的内容只有 UDP 端口、发包频率、pose timeout、任务文本和按键索引。`sim_host` 已删除，服务器回环边界不可配置。修改 YAML 后如果使用 `--symlink-install`，重启 launch 即可；否则重新 `colcon build`。
