<p>
  <a href="https://opensource.x-humanoid-cloud.com/plugin.php?id=keke_video_base&ac=course&cid=20">
    <img src="./static/course-tiangong-sdk-banner.png" width="700">
  </a>

</p >

# 具身天工 3.0 SDK 示例

面向 **具身天工 3.0** 人形机器人平台的开源 SDK 示例集，基于 ROS 2 Jazzy 构建。

> **开发平台**：本项目需要在 **具身天工 3.0 机器人本体** 上进行开发和运行，开发环境为 **Ubuntu 24.04**，当前不支持 Mac 和 Windows。
>
> ```bash
> # 登录算力主机（通过网线直连时需配置本机41网段网卡 MTU 为 9000）
> ssh nvidia@192.168.41.2
> 
> # 加载 ROS2 工作空间环境（顺序不能颠倒，缺一不可）
> source /opt/ros/jazzy/setup.bash
> source /opt/humanoid/install/setup.bash
> source ~/xos/setup.bash
> ```

## 简介

本仓库提供一系列开箱即用的 ROS 2 示例包，演示具身天工 3.0 机器人 SDK 的核心能力，涵盖关节控制、传感器数据可视化、语音交互等。每个示例均提供 **Python** 和 **C++** 双版本实现，可独立编译和运行。

## 功能特性

- 单关节位置模式与力位混合模式控制
- 多相机 RGB / 深度图像显示（头部 & 腰部）
- 点云可视化（Livox 激光雷达）
- 多源 IMU 数据显示与曲线绘图
- GPS 数据采集与记录
- 文字转语音（TTS）与语音识别（ASR）
- 麦克风录音与喇叭播放
- 强脑灵巧手手势控制与触觉反馈 *（选配硬件）*

## 快速开始

### 环境要求

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| 构建工具 | colcon |
| 语言 | Python 3.12+ / C++17 |

### 获取代码

```bash
# 登录具身天工 3.0 开发板
ssh nvidia@192.168.41.2

# 克隆仓库到机器人 ~/work 目录下
cd ~/work
git clone https://github.com/Open-X-Humanoid/xhumanoid_sdk.git
```

### 编译

所有示例依赖 xos 工作空间中的 ROS 2 消息包（`ros2_bridge_msgs`、`interaction_msgs` 等），需要先按以下顺序加载环境再编译（`ros2_bridge_msgs` 由 `/opt/humanoid/install` 提供，`~/xos` 里只有新版 `lyre_msgs`，缺一不可；`~/xos` 必须最后加载，否则旧版同名包会压住新版）。机器人算力主机已在 `~/.bashrc` 中配置为登录后自动按此顺序加载，新开终端无需手动 source：

```bash
# 按顺序加载环境：ROS 2 -> 人形机器人 SDK（ros2_bridge_msgs）-> xos 工作空间（新版 lyre_msgs）
source /opt/ros/jazzy/setup.bash
source /opt/humanoid/install/setup.bash
source ~/xos/setup.bash

# 编译单个示例（推荐）
colcon build --packages-select single_joint_control_py

# 或编译某个示例的 Python + C++ 版本
colcon build --packages-select single_joint_control_py single_joint_control_cpp

# 全量编译（补齐全部示例的 Python + C++ 版本）
colcon build
```

### 编译常见问题

- **编译报 `fatal error: xxx.hpp: No such file or directory`**：多为 build 缓存残留了旧依赖路径（如 `lyre_msgs_DIR` 被钉死在旧版 `/opt/humanoid/install/lyre_msgs`，而源码用到的是 `~/xos` 新版消息）。删除该包缓存后重新编译：

  ```bash
  rm -rf build/<包名> install/<包名>
  colcon build --packages-select <包名>
  ```

- **`source install/setup.bash` 报 `not found: .../install/<包名>/share/<包名>/local_setup.bash`**：该包上次构建被中断，install 目录残留不完整产物（缺 `local_setup.bash` 与可执行文件）。同样删除 `install/<包名>` 后重新编译即可，setup.bash 会随之重新生成。

- 编译过程中如遇其他依赖缺失，请确认环境按上述三层顺序加载（顺序影响同名包如 `lyre_msgs` 的解析优先级）。

### 运行示例

```bash
# 在仓库根目录下按顺序加载环境，并额外加载本仓库的编译产物
# （运行时需要；新终端已由 ~/.bashrc 自动加载，无需手动执行）
source /opt/ros/jazzy/setup.bash
source /opt/humanoid/install/setup.bash
source ~/xos/setup.bash
source install/setup.bash

# 示例：启动单关节控制（Python 版）
ros2 launch single_joint_control_py single_joint_control.launch.py

# 示例：启动单关节控制（C++ 版）
ros2 launch single_joint_control_cpp single_joint_control.launch.py
```

> `~/.bashrc` 已自动加载上面四层环境（`/opt/ros/jazzy` → `/opt/humanoid/install` → `~/xos` → 本仓库 `install`，末层带存在性检查），
> 新开终端可直接 `ros2 launch ...` 无需手动 source。只有修改了 `~/.bashrc` 未生效时才需要手动执行上面的命令。

## 示例列表

| 示例 | 说明 | 硬件要求 |
|------|------|----------|
| [single_joint_control](single_joint_control/) | 单关节位置 & 力位混合控制 | 标配 |
| [brainco_hand_gesture_control](brainco_hand_gesture_control/) | 强脑灵巧手手势控制 | 选配 |
| [play_text_demo](play_text_demo/) | 文字转语音播放 | 标配 |
| [speech_recognition_demo](speech_recognition_demo/) | 语音识别（ASR） | 标配 |
| [point_cloud_display](point_cloud_display/) | Livox 激光雷达点云可视化 | 标配 |
| [imu_display](imu_display/) | 多源 IMU 数据显示与绘图 | 标配 |
| [gps_data_display](gps_data_display/) | GPS 数据采集与记录 | 选配 |
| [camera_6v_display](camera_6v_display/) | 6 路相机全景可视化 | 选配 |
| [speaker_play_demo](speaker_play_demo/) | 喇叭音频播放 | 标配 |
| [mic_record_demo](mic_record_demo/) | 麦克风录音 | 标配 |
| [camera_display](camera_display/) | 头部 & 腰部相机 RGB/深度显示 | 标配 |
| [camera_wrist_driver](camera_wrist_driver/) | 腕部 RealSense D405 相机 RGB/深度驱动 | 选配 |
| [brainco_hand_touch_display](brainco_hand_touch_display/) | 强脑手触觉反馈显示 | 选配 |

> **选配** 示例需要额外硬件支持，部分机型可能未配备。

各示例目录下均包含独立的 `README.md`，提供详细的使用说明。

## 项目结构

```
xhumanoid_sdk/
├── README.md                           # 本文件
├── LICENSE                             # Apache-2.0
├── CONTRIBUTING.md                     # 贡献指南
├── single_joint_control/               # 关节控制示例
│   ├── python/                         # Python ROS 2 包
│   ├── cpp/                            # C++ ROS 2 包
│   └── README.md
├── brainco_hand_gesture_control/       # 灵巧手手势控制（选配）
├── play_text_demo/                     # 文字转语音示例
├── speech_recognition_demo/            # 语音识别示例
├── point_cloud_display/                # 点云显示示例
├── imu_display/                        # IMU 显示示例
├── gps_data_display/                   # GPS 数据示例（选配）
├── camera_6v_display/                  # 6 路相机示例（选配）
├── speaker_play_demo/                  # 喇叭播放示例
├── mic_record_demo/                    # 麦克风录音示例
├── camera_display/                     # 头部 & 腰部相机示例
├── camera_wrist_driver/                # 腕部 D405 相机驱动示例（选配）
└── brainco_hand_touch_display/         # 触觉反馈示例（选配）
```

各示例统一遵循以下目录规范：

```
<demo_name>/
├── python/          # Python ROS 2 包 (ament_python)
│   ├── package.xml
│   ├── setup.py
│   ├── launch/
│   └── <pkg_name>/
├── cpp/             # C++ ROS 2 包 (ament_cmake)
│   └── <pkg_name>/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       └── src/
└── README.md
```

## 文档

- 各示例的 `README.md` 提供详细的使用方法、参数说明和预期输出。
- ROS 2 消息和服务定义由 `ros2_bridge_msgs`、`interaction_msgs`、`lyre_msgs`、`brainco_hand_msgs` 提供。

## 路线图

- [ ] 增加多关节、全身控制示例
- [ ] 增加仿真环境支持（Gazebo / Isaac Sim）
- [ ] 提供 Docker 开发环境
- [ ] 增加单元测试和集成测试
- [ ] 配置 CI/CD 流水线

## 贡献

请阅读 [CONTRIBUTING.md](CONTRIBUTING.md) 了解如何参与贡献。

## 社区

- **问题反馈**：通过 [GitHub Issues](https://github.com/Open-X-Humanoid/xhumanoid_sdk/issues) 提交 Bug 或功能建议。
- **讨论交流**：通过 [GitHub Discussions](https://github.com/Open-X-Humanoid/xhumanoid_sdk/discussions) 提问和交流。

## 引用

如果您在研究中使用了本项目，请引用：

```bibtex
@misc{xhumanoid_sdk,
  title  = {具身天工 3.0 SDK 示例},
  author = {Open-X-Humanoid},
  year   = {2025},
  url    = {https://github.com/Open-X-Humanoid/xhumanoid_sdk}
}
```

## 许可证

本项目采用 [Apache License 2.0](LICENSE) 开源协议。

```
Copyright 2025 Open-X-Humanoid

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```
