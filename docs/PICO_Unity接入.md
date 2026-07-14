# Windows 主机上的 PICO / Unity 输入端

## 1. 固定工作方式

PICO 通过 USB 和 PICO Connect 接到 Windows 主机，作为 PCVR/OpenXR 追踪设备。Unity 在 Windows 上读取头显和左右手柄位姿，然后通过 TCP `10000` 连接服务器上的 `ros_tcp_endpoint`。

本项目不在 PICO 上安装 Android APK，不在 Windows 上安装 ROS 2，也不把 Isaac Sim WebRTC 画面送入 PICO。WebRTC 画面显示在 Windows 的独立客户端窗口；PICO 只负责输入。Unity XR 程序必须保持运行，脚本已设置 `Application.runInBackground = true`，切换到 WebRTC 窗口后仍继续发布。

## 2. 安装并连接 PICO Connect

1. 在 Windows 主机安装 [PICO Connect 官方 Windows 版](https://www.picoxr.com/cn/software/pico-link)。
2. PICO 系统升级到官方 PICO Connect 要求的版本。
3. PICO 用 USB 数据线连接 Windows，并在 PICO Connect 中启动 PCVR 串流。
4. 在 PICO Connect 设置中把 PICO/OpenXR runtime 设为当前 OpenXR runtime。
5. 保持头显、左右控制器均显示已追踪，再启动 Unity。

官方说明确认 PICO Connect 可把 Windows PCVR/OpenXR 内容和控制器追踪连接到头显。本项目只消费标准 OpenXR 输入，不依赖 PICO Android SDK 私有 API。

## 3. 创建 Unity Windows OpenXR 工程

1. 用 Unity Hub 创建 3D 工程，目标平台保持 `PC, Mac & Linux Standalone`。
2. 在 `Window → Package Manager` 安装 Unity 官方 `OpenXR Plugin`。
3. 进入 `Edit → Project Settings → XR Plug-in Management → PC`，启用 `OpenXR`。
4. 在 OpenXR interaction profiles 中启用控制器可用的通用 profile；运行时由 PICO Connect 提供实际设备。
5. 场景内放置一个 XR Origin/XR Rig 和 Main Camera，确保 OpenXR loader 会启动。
6. `File → Build Settings` 选择 Windows、架构 `x86_64`。

不要切换 Android 平台，也不要为 PICO 打包 APK。

## 4. 安装 ROS-TCP-Connector

在 `Window → Package Manager → + → Add package from git URL` 输入：

```text
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

然后打开 `Robotics → ROS Settings`，只填写以下配置：

| 项目 | 固定值 |
|---|---|
| Protocol | `ROS2` |
| ROS IP Address | `192.168.50.227` |
| ROS TCP Port | `10000` |

这里必须填 `192.168.50.227`，不能填 `127.0.0.1`，因为 Unity 位于 Windows 主机 `192.168.50.61`，endpoint 位于服务器。Unity 官方教程也使用 ROS-TCP-Connector 配合 ROS 2 endpoint，默认端口为 10000。

## 5. 加入发布脚本

复制：

```text
unity/Assets/Scripts/PicoRosPublisher.cs
```

到 Unity 工程的 `Assets/Scripts/`。新建空 GameObject `PicoRosPublisher` 并挂载脚本。脚本以 60 Hz 发布标准消息：

| ROS 2 话题 | 类型 |
|---|---|
| `/pico/left_controller/pose` | `geometry_msgs/msg/PoseStamped` |
| `/pico/right_controller/pose` | `geometry_msgs/msg/PoseStamped` |
| `/pico/head/pose` | `geometry_msgs/msg/PoseStamped` |
| `/pico/controllers/joy` | `sensor_msgs/msg/Joy` |

不需要生成自定义 ROS message。坐标从 Unity 左手 RUF 转成 ROS 右手 FLU：

```text
position_ros = [position_unity.z, -position_unity.x, position_unity.y]
R_ros = C * R_unity * C^-1
```

## 6. Windows 构建和启动

1. 在 Unity Play Mode 中确认 Console 没有 OpenXR loader 或 `ROSConnection` 错误。
2. 构建 Windows x86_64 程序，例如 `PicoTeleopInput.exe`。
3. 启动顺序固定为：PICO Connect 已连接 → 服务器 ROS launch 已运行 → 启动 `PicoTeleopInput.exe`。
4. 不关闭 Unity 输入程序。操作时把焦点切到 Isaac Sim WebRTC Streaming Client 即可。

## 7. 服务器端验收

在服务器执行：

```bash
source /opt/ros/humble/setup.bash
source /home/kemove/zzk_data/pico-teleop/ros2_ws/install/setup.bash

ros2 topic hz /pico/left_controller/pose
ros2 topic hz /pico/right_controller/pose
ros2 topic echo /pico/controllers/joy
```

验收标准：

- 左右 pose 接近 60 Hz，移动手柄时 position 连续变化。
- 左 trigger 使 `axes[0]` 从 0 到 1；右 trigger 使 `axes[2]` 从 0 到 1。
- A/B/X/Y 和 menu 在 `buttons[0..4]` 产生上升沿。
- Unity ROS Console 显示已连接服务器，不反复重连。

## 8. 默认按键

```text
A       start；暂停状态下为 resume
B       pause
X       开始/停止当前 episode 录制
Y       stop 并保存 active episode
menu    calibrate
```

由于不同控制器 runtime 可能改变 `CommonUsages` 映射，第一次必须以服务器收到的 `Joy` 为准。需要修改时只改服务器文件 `ros2_ws/src/pico_teleop_bridge/config/default.yaml` 的 `button_*` 索引，控制链路和架构不变。

## 9. 常见故障

- Unity 没有 pose：先确认 PICO Connect 的 OpenXR runtime 已生效，PICO 和两个控制器均为 tracking 状态，再重启 Unity 程序。
- Unity 显示连接失败：服务器运行 `ss -lntp | grep 10000`；应看到 endpoint 监听 `0.0.0.0:10000`。
- 服务器没有话题：检查 Unity ROS Settings 是 `ROS2`，IP 是 `192.168.50.227`，并确认 `192.168.50.61 → 192.168.50.227:10000/TCP` 可达。
- 切到 WebRTC 后停止发布：确认使用项目内已更新脚本；其 `Start()` 会启用 `Application.runInBackground`。
- pose 正常但按键不对：用 `ros2 topic echo /pico/controllers/joy` 确认实际索引，再改 YAML。
