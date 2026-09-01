# 多源IMU显示示例

> **适用平台**: 具身天工3.0 (Thor)
>
> 本示例需要在 **具身天工3.0 机器人本体** 上进行开发和运行，开发环境为 **Ubuntu 24.04**，当前不支持 Mac 和 Windows。
>
> ```bash
> # 登录算力主机（通过网线直连时需配置本机41网段网卡 MTU 为 9000）
> ssh nvidia@192.168.41.2
>
> # 注意：必须使用 nvidia 用户启动 ROS2 节点，否则消息无法通信
> source ~/xos/setup.bash
> ```

本示例演示如何订阅多种IMU传感器数据并进行实时可视化显示和曲线绘图。

## IMU数据源

Thor平台提供两种IMU数据源，通过 `imu_source` 参数进行选择：

| IMU源 | 参数值 | 话题 | 消息类型 | 频率 | 说明 |
|--------|--------|------|----------|------|------|
| **Livox雷达IMU** | `livox`（默认） | `/livox/imu` | `sensor_msgs/Imu` | 200Hz | 雷达内置IMU |
| Xsens体内IMU | `xsens` | `/robot_state` | `ros2_bridge_msgs/RobotState` | - | 体内IMU，通过robot_state获取 |

提供两个版本：
- **Python版本**: `imu_display_py`
- **C++版本**: `imu_display_cpp`

两个版本功能相同：
- 多源IMU数据订阅与切换
- RViz可视化（IMU姿态显示）
- 图表保存（无显示器环境可用）

## 功能特性

- 支持两种IMU数据源切换（livox / xsens）
- 实时显示欧拉角（Roll、Pitch、Yaw）
- 实时显示角速度（陀螺仪数据）
- 实时显示线性加速度（加速度计数据）
- **自动保存曲线图到文件（无显示器环境可用）**
- RViz可视化IMU姿态

## 依赖

- ROS2 (Jazzy)
- rclcpp / rclpy
- sensor_msgs, visualization_msgs
- ros2_bridge_msgs（xsens IMU支持）
- python3-matplotlib
- python3-numpy
- rviz2

## 编译

### Python版本

```bash
cd ~/xos
colcon build --packages-select imu_display_py
source setup.bash
```

### C++版本

```bash
cd ~/xos
colcon build --packages-select imu_display_cpp
source setup.bash
```

## 使用方法

### 1. 启动IMU显示节点

#### 使用 Livox IMU（默认）

```bash
# Python版本
ros2 launch imu_display_py imu_display.launch.py

# C++版本
ros2 launch imu_display_cpp imu_display.launch.py
```

#### 使用 Xsens 体内IMU

```bash
# Python版本
ros2 launch imu_display_py imu_display.launch.py imu_source:=xsens

# C++版本
ros2 launch imu_display_cpp imu_display.launch.py imu_source:=xsens
```

> Livox IMU 和 Xsens IMU 的数据由系统自动发布，无需手动启动驱动。

#### 自定义参数

```bash
ros2 launch imu_display_py imu_display.launch.py \
    imu_source:=livox \
    history_size:=500 \
    plot_interval:=2.0 \
    save_format:=png \
    dpi:=150
```

## 参数说明

### 通用参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `imu_source` | `livox` | IMU数据源：`livox`、`xsens` |
| `imu_topic` | （自动） | 自定义话题（留空则根据imu_source自动选择） |
| `frame_id` | `base_link` | 坐标系ID |

### Python节点参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `history_size` | `500` | 历史数据存储数量 |
| `plot_interval` | `2.0` | 绘图更新间隔（秒） |
| `save_format` | `png` | 图片格式（png, pdf, svg） |
| `dpi` | `150` | 图片DPI |

### C++节点参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `history_size` | `200` | 历史数据存储数量 |
| `print_interval` | `1.0` | 统计信息打印间隔（秒） |
| `plot_interval` | `2.0` | 绘图更新间隔（秒） |
| `save_plot` | `true` | 是否保存图表 |
| `save_dir` | `/tmp/imu_plots` | 图表保存目录 |
| `save_format` | `png` | 图片格式 |
| `dpi` | `150` | 图片DPI |

## IMU数据源详细对比

### Livox 雷达IMU

- **话题**: `/livox/imu`
- **消息类型**: `sensor_msgs/msg/Imu`
- **频率**: 200Hz
- **特点**: 与激光雷达集成，无需额外硬件
- **适用场景**: 雷达-惯性融合定位（LIO）

### Xsens 体内IMU

- **话题**: `/robot_state`（`ros2_bridge_msgs/msg/RobotState` 的 `imu` 字段）
- **消息类型**: `ros2_bridge_msgs/msg/ImuStatus`
- **字段**: `qx, qy, qz, qw`（四元数）、`roll, pitch, yaw`（欧拉角，度）、`wx, wy, wz`（角速度 rad/s）、`ax, ay, az`（加速度 m/s²）
- **特点**: 集成在机器人本体内部
- **适用场景**: 本体姿态估计、平衡控制

## 输出文件

运行后会在指定目录生成：

- `imu_plot.png` - 实时更新的绘图文件（每次更新覆盖）

绘图包含三部分：
1. **Orientation** - 欧拉角曲线（Roll/红、Pitch/绿、Yaw/蓝）
2. **Gyroscope** - 角速度曲线（X/红、Y/绿、Z/蓝）
3. **Accelerometer** - 加速度曲线（X/红、Y/绿、Z/蓝）

## 无显示器环境

两个版本都支持无显示器环境运行：
- Python版本使用matplotlib的`Agg`后端
- C++版本调用Python脚本生成图表

适合：
- SSH远程连接
- 服务器环境
- Docker容器
- 无图形界面的嵌入式系统

## 目录结构

```
imu_display/
├── README.md
├── python/                              # Python版本 (imu_display_py)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── pyproject.toml
│   ├── resource/
│   │   └── imu_display_py
│   ├── imu_display/
│   │   ├── __init__.py
│   │   ├── imu_display_node.py
│   │   └── imu_display_node_uv.sh
│   ├── launch/
│   │   ├── imu_display.launch.py
│   │   └── imu_display_rviz.launch.py
│   └── rviz/
│       └── imu_display.rviz
└── cpp/                                 # C++版本 (imu_display_cpp)
    └── imu_display/
        ├── CMakeLists.txt
        ├── package.xml
        ├── src/
        │   └── imu_display_node.cpp
        ├── scripts/
        │   └── imu_plot_generator.py
        ├── launch/
        │   ├── imu_display.launch.py
        │   └── imu_display_rviz.launch.py
        └── rviz/
            └── imu_display.rviz
```

## UV 环境配置（可选）

使用 [uv](https://docs.astral.sh/uv/) 创建隔离的 Python 环境：

```bash
# 安装 uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# 创建虚拟环境
cd ~/work/xhumanoid_sdk/imu_display/python
uv venv
uv pip install matplotlib "numpy>=1.21.0,<2.0.0" pyyaml

# 使用 uv 环境运行
./imu_display/imu_display_node_uv.sh --ros-args -r __node:=imu_display_node
```

## 相关链接

- Livox雷达驱动：`livox_ros_driver2`
- 机器人状态消息：`ros2_bridge_msgs`
