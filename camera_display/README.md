# 头部、腰部和腕部相机显示 (Head, Waist & Wrist Camera Display)

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

基于 ROS2 的多相机图像可视化显示节点，支持头部/腰部 Orbbec 相机与腕部 RealSense D405 相机（选配）的 RGB 彩色图像和深度图像实时显示。

提供两个版本：
- **Python版本**: `camera_display_py`
- **C++版本**: `camera_display_cpp`

## 功能特性

- 同时显示头部、腰部、左右腕部相机的 RGB 彩色图像与深度图像
- 可独立启用/禁用每一路相机（头/腰/左腕/右腕）
- 腕部 D405 近距相机使用独立深度量程（默认 100-600mm，避免深度图全黑）
- 多种深度颜色映射（GRAY、JET、RAINBOW、TURBO）
- 深度直方图显示
- 深度统计信息叠加（最小、最大、平均深度，有效像素数）
- 支持自定义深度范围
- 可调节显示比例

## 相机话题

根据 SDK 文档，头部和腰部相机的 ROS2 话题为：

```bash
# 头部相机
/ob_camera_head/color/image_raw        # 彩色原始图像（默认RGB8，分辨率1280×720）
/ob_camera_head/depth/image_raw        # 深度原始图像

# 腰部相机
/ob_camera_waist/color/image_raw       # 彩色原始图像（默认RGB8，分辨率1280×720）
/ob_camera_waist/depth/image_raw       # 深度原始图像

# 腕部相机（选配 D405，由 camera_wrist_driver 驱动发布）
/ob_camera_wrist_left/color/image_raw    # 左腕彩色（bgr8，640×480）
/ob_camera_wrist_left/depth/image_raw    # 左腕深度（16UC1，毫米）
/ob_camera_wrist_right/color/image_raw   # 右腕彩色（bgr8，640×480）
/ob_camera_wrist_right/depth/image_raw   # 右腕深度（16UC1，毫米）

# 腕部相机同时提供压缩话题（driver 默认开启，image_transport 格式）
/ob_camera_wrist_left/color/image_raw/compressed   # 彩色 JPEG（有损，带宽 ~26KB/帧）
/ob_camera_wrist_left/depth/image_raw/compressedDepth  # 深度 PNG（无损，16 位）
```

## 依赖

- ROS2 (Humble/Jazzy)
- OpenCV
- sensor_msgs

## 目录结构

```
camera_display/
├── README.md
├── python/                              # Python版本 (包名: camera_display_py)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   │   └── camera_display_py
│   ├── launch/
│   │   └── camera_display.launch.py
│   └── camera_display/
│       ├── __init__.py
│       └── camera_display_node.py
└── cpp/                                 # C++版本 (包名: camera_display_cpp)
    └── camera_display/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── camera_display.launch.py
        └── src/
            └── camera_display_node.cpp
```

## 编译

### Python版本

```bash
cd ~/xos
colcon build --packages-select camera_display_py
source setup.bash
```

### C++版本

```bash
cd ~/xos
colcon build --packages-select camera_display_cpp
source setup.bash
```

## 使用方法

### 前提条件

确保相机驱动已启动：

```bash
# 启动头部相机
sudo systemctl start orbbec_head.service

# 启动腰部相机
sudo systemctl start orbbec_waist.service
```

### 1. 同时显示头部和腰部相机（默认）

#### Python版本

```bash
ros2 launch camera_display_py camera_display.launch.py
```

#### C++版本

```bash
ros2 launch camera_display_cpp camera_display.launch.py
```

### 2. 仅显示头部相机

```bash
ros2 launch camera_display_py camera_display.launch.py enable_waist:=false
```

### 3. 仅显示腰部相机

```bash
ros2 launch camera_display_py camera_display.launch.py enable_head:=false
```

### 4. 显示腕部相机（选配 D405）

先部署腕部相机驱动（见 `camera_wrist_driver` 示例），再：

```bash
# 全部四路一起显示
ros2 launch camera_display_py camera_display.launch.py \
    enable_wrist_left:=true enable_wrist_right:=true

# 仅显示左右腕（不显示头/腰）
ros2 launch camera_display_py camera_display.launch.py \
    enable_head:=false enable_waist:=false \
    enable_wrist_left:=true enable_wrist_right:=true

# C++ 版本参数相同
ros2 launch camera_display_cpp camera_display.launch.py \
    enable_wrist_left:=true enable_wrist_right:=true
```

> 腕部 D405 为近距离深度相机（工作距离约 0.1–0.5m），默认使用独立的
> `wrist_min_depth=100` / `wrist_max_depth=600` 显示量程；头/腰部仍用全局
> `min_depth` / `max_depth`。

### 5. 自定义深度范围和颜色映射

```bash
ros2 launch camera_display_py camera_display.launch.py \
    min_depth:=500.0 \
    max_depth:=3000.0 \
    colormap:=1
```

### 6. 完整参数配置

```bash
ros2 launch camera_display_py camera_display.launch.py \
    colormap:=2 \
    max_depth:=5000.0 \
    min_depth:=0.0 \
    display_scale:=0.5 \
    show_histogram:=true \
    show_statistics:=true \
    enable_head:=true \
    enable_waist:=true \
    enable_wrist_left:=false \
    enable_wrist_right:=false \
    wrist_min_depth:=100.0 \
    wrist_max_depth:=600.0
```

## 参数说明

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `colormap` | `2` | 深度颜色映射（0=GRAY, 1=JET, 2=RAINBOW, 3=TURBO） |
| `max_depth` | `5000.0` | 最大深度值(mm)用于可视化 |
| `min_depth` | `0.0` | 最小深度值(mm)用于可视化 |
| `display_scale` | `0.5` | 显示比例因子 |
| `show_histogram` | `true` | 显示深度直方图 |
| `show_statistics` | `true` | 显示深度统计信息 |
| `enable_head` | `true` | 启用头部相机显示 |
| `enable_waist` | `true` | 启用腰部相机显示 |
| `enable_wrist_left` | `false` | 启用左腕 D405 相机显示（需已部署 camera_wrist_driver） |
| `enable_wrist_right` | `false` | 启用右腕 D405 相机显示（需已部署 camera_wrist_driver） |
| `wrist_min_depth` | `100.0` | 腕部相机可视化最小深度值(mm)（独立于全局 min_depth） |
| `wrist_max_depth` | `600.0` | 腕部相机可视化最大深度值(mm)（独立于全局 max_depth） |

## 订阅话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/ob_camera_head/depth/image_raw` | `sensor_msgs/msg/Image` | 头部深度图像 |
| `/ob_camera_head/color/image_raw` | `sensor_msgs/msg/Image` | 头部彩色图像 |
| `/ob_camera_waist/depth/image_raw` | `sensor_msgs/msg/Image` | 腰部深度图像 |
| `/ob_camera_waist/color/image_raw` | `sensor_msgs/msg/Image` | 腰部彩色图像 |
| `/ob_camera_wrist_left/depth/image_raw` | `sensor_msgs/msg/Image` | 左腕深度图像（选配） |
| `/ob_camera_wrist_left/color/image_raw` | `sensor_msgs/msg/Image` | 左腕彩色图像（选配） |
| `/ob_camera_wrist_right/depth/image_raw` | `sensor_msgs/msg/Image` | 右腕深度图像（选配） |
| `/ob_camera_wrist_right/color/image_raw` | `sensor_msgs/msg/Image` | 右腕彩色图像（选配） |

### 支持的图像编码

**深度图像：**
- `16UC1` - 16位无符号整数（毫米）
- `mono16` - 16位灰度
- `32FC1` - 32位浮点（米）

**彩色图像：**
- `rgb8` / `RGB8` - RGB 8位
- `bgr8` / `BGR8` - BGR 8位
- `rgba8` / `RGBA8` - RGBA 8位
- `bgra8` / `BGRA8` - BGRA 8位

## 显示窗口

启用多路相机时，将显示以下窗口：

| 窗口名称 | 说明 |
|----------|------|
| Head - Color | 头部彩色图像 |
| Head - Depth | 头部深度图像（颜色映射后） |
| Head - Depth Histogram | 头部深度直方图 |
| Waist - Color | 腰部彩色图像 |
| Waist - Depth | 腰部深度图像（颜色映射后） |
| Waist - Depth Histogram | 腰部深度直方图 |
| Wrist Left/Right - Color | 腕部彩色图像（启用时） |
| Wrist Left/Right - Depth | 腕部深度图像（启用时，100-600mm 量程） |
| Wrist Left/Right - Depth Histogram | 腕部深度直方图（启用时） |

## 键盘快捷键

| 按键 | 功能 |
|------|------|
| `ESC` / `q` | 退出程序 |
| `c` | 切换颜色映射（所有相机同时切换） |
| `h` | 切换直方图显示 |
| `s` | 切换统计信息显示 |

## 颜色映射说明

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | GRAY | 灰度图 |
| 1 | JET | JET颜色映射（蓝-青-黄-红） |
| 2 | RAINBOW | 彩虹颜色映射 |
| 3 | TURBO | Turbo颜色映射（改进的JET） |

## 统计信息说明

深度图像上叠加显示以下统计信息（每个相机独立统计）：
- **Min** - 最小深度值（毫米）
- **Max** - 最大深度值（毫米）
- **Mean** - 平均深度值（毫米）
- **Valid** - 有效像素数量（非零值）

## 注意事项

1. 需要图形界面支持（SSH连接需使用 `-X` 转发或VNC）
2. 确保对应的相机驱动已启动（`orbbec_head.service` / `orbbec_waist.service` / `realsense_wrist_left.service` / `realsense_wrist_right.service`）
3. 无效深度值（0）显示为黑色
4. 显示比例可调整以适应不同屏幕尺寸
5. 同时显示多路相机时建议适当降低 `display_scale`

## 故障排除

### 没有图像显示

1. 检查相机服务是否已启动：
   ```bash
   sudo systemctl status orbbec_head.service
   sudo systemctl status orbbec_waist.service
   sudo systemctl status realsense_wrist_left.service   # 腕部（选配）
   sudo systemctl status realsense_wrist_right.service  # 腕部（选配）
   ```

2. 检查相机话题是否有数据：
   ```bash
   ros2 topic list | grep ob_camera
   ros2 topic hz /ob_camera_head/color/image_raw
   ros2 topic hz /ob_camera_waist/color/image_raw
   ros2 topic hz /ob_camera_wrist_left/color/image_raw   # 腕部（选配）
   ros2 topic hz /ob_camera_wrist_right/color/image_raw  # 腕部（选配）
   ```

3. 检查话题内容：
   ```bash
   ros2 topic echo /ob_camera_head/depth/image_raw --once
   ```

> 腕部话题在服务 active 但列表里看不到时，多为服务启动早于 41 网段网卡就绪
> （journal 有 `All whitelist interfaces were filtered out`），
> `sudo systemctl restart realsense_wrist_<side>` 即可恢复；新版
> `start_wrist.sh` 已内置网络等待。

### OpenCV窗口不显示

确保有显示器或启用了X11转发：
```bash
echo $DISPLAY
export DISPLAY=:0
```

## 相关链接

- Orbbec相机驱动：`hardware/camera/src/OrbbecSDK_ROS2`
- Orbbec SDK文档：https://github.com/orbbec/OrbbecSDK_ROS2
