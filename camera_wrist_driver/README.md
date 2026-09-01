# 腕部相机驱动 (Wrist Camera Driver)

> **适用平台**: 具身天工3.0 (Thor)
>
> 本示例需要在 **具身天工3.0 机器人本体** 上进行开发和运行，开发环境为 **Ubuntu 24.04**，当前不支持 Mac 和 Windows。
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

基于 ROS2 的 **左右腕部 Intel RealSense D405 相机驱动节点**，参考 `orbbec_head.service` / `orbbec_waist.service` 的 systemd 服务模式设计，左右腕各一个独立服务，同时发布彩色图与深度图，话题命名与 `camera_display` 现有 orbbec 相机约定一致，可直接接入显示。

## 功能特性

- 左右腕各一台 D405，独立节点/服务，互不影响
- 同时发布 **彩色图（bgr8）+ 深度图（16UC1，毫米）**
- 同步发布 **压缩话题**（彩色 JPEG、深度无损 PNG，image_transport 格式），节省带宽
- 支持自定义分辨率、帧率、话题前缀，可关闭深度流
- 提供 systemd 一键部署脚本（SN 探测、udev 规则、开机自启）
- QoS 与 `camera_display` 订阅约定完全一致（BEST_EFFORT + VOLATILE）

## 发布话题

| 话题 | 类型 | 编码 |
|------|------|------|
| `/ob_camera_wrist_left/color/image_raw` | `sensor_msgs/Image` | `bgr8` |
| `/ob_camera_wrist_left/depth/image_raw` | `sensor_msgs/Image` | `16UC1` |
| `/ob_camera_wrist_right/color/image_raw` | `sensor_msgs/Image` | `bgr8` |
| `/ob_camera_wrist_right/depth/image_raw` | `sensor_msgs/Image` | `16UC1` |
| `/ob_camera_wrist_<side>/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | `jpeg` |
| `/ob_camera_wrist_<side>/depth/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | `16UC1; compressedDepth png` |

- QoS：BEST_EFFORT, KEEP_LAST 10, VOLATILE
- 压缩话题与头部相机 (`ob_camera_head`) 的 `/compressed`、`/compressedDepth` 格式一致，
  rviz2 / Foxglove / rqt_image_view 可直接订阅；彩色为有损 JPEG（质量可调），
  深度为 16 位无损 PNG，像素级还原
- `frame_id`：`wrist_left` / `wrist_right`
- 深度单位：毫米（mm），无效深度值为 0

## 依赖

- ROS2 (Jazzy)：`rclpy` / `sensor_msgs`（本体出厂环境）
- `pyrealsense2` + `numpy`：**install.sh 会自动检测并以服务用户身份安装**（pip `--user`，失败时回退 apt）
- sensor_msgs

> 手动安装 pyrealsense2 时注意：服务以 `nvidia` 用户运行，须装入该用户环境：
> `sudo -u nvidia pip3 install --user --break-system-packages pyrealsense2`

## 目录结构

```
camera_wrist_driver/
├── README.md                              # 本文档
├── python/                                # Python版本 (包名: camera_wrist_driver_py)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   │   └── camera_wrist_driver_py
│   ├── launch/
│   │   └── camera_wrist_driver.launch.py
│   └── camera_wrist_driver/
│       ├── __init__.py
│       └── camera_wrist_driver_node.py   # D405 驱动 + ROS2 发布节点
├── scripts/
│   ├── install.sh                         # 部署：探测 SN -> env -> udev -> systemd 服务
│   ├── uninstall.sh                       # 卸载 systemd 部署
│   └── start_wrist.sh                     # systemd 包装脚本（加载环境后 exec 节点）
└── service/
    ├── realsense_wrist_left.service       # systemd 服务模板（install.sh 渲染）
    └── realsense_wrist_right.service
```

## 编译

```bash
cd ~/xos
colcon build --packages-select camera_wrist_driver_py
source install/setup.bash
```

## 使用方法

### 前提条件

1. 两台 D405 已通过 USB 连接（`lsusb | grep 8086` 应看到两台，D405 USB ID 为 `8086:0b5b`）；仅插 1 台时脚本会拒绝安装（需左右映射）
2. `pyrealsense2` 未装也没关系，install.sh 会自动装
3. ROS 三层环境（`/opt/ros/jazzy`、`/opt/humanoid/install`、`~/xos`）缺失时脚本会警告
4. 查询序列号：`python3 -c "import pyrealsense2 as rs; [print(d.get_info(rs.camera_info.serial_number)) for d in rs.context().query_devices()]"`

### 方式一：launch 启动（手动调试）

```bash
ros2 launch camera_wrist_driver_py camera_wrist_driver.launch.py side:=left serial:=<LEFT_SN>
ros2 launch camera_wrist_driver_py camera_wrist_driver.launch.py side:=right serial:=<RIGHT_SN>
```

### 方式二：源码直跑（无需编译）

```bash
cd ~/work/xhumanoid_sdk/camera_wrist_driver
export PYTHONPATH=$PWD/python:$PYTHONPATH
python3 -m camera_wrist_driver.camera_wrist_driver_node --ros-args -p side:=left -p serial:=<LEFT_SN>
```

### 方式三：systemd 部署（开机自启，推荐）

```bash
cd ~/work/xhumanoid_sdk/camera_wrist_driver
sudo ./scripts/install.sh
```

脚本会自动：

1. **检查并安装运行依赖**：系统工具（python3/pip3/udevadm/systemctl）、ROS 三层环境（缺失则警告）、`pyrealsense2`/`numpy`（以服务用户身份 `pip --user` 安装，失败回退 apt；`--skip-deps` 可跳过）
2. 探测两台 D405 的序列号（SN），交互式选择左右映射（也可用 `--left-sn/--right-sn` 直接指定；`--left-rotate/--right-rotate 0|90|180|270` 设置画面旋转，`--left-fps/--right-fps` 设置帧率，`--no-compressed` 关闭压缩话题，`--jpeg-quality` 设置 JPEG 质量，均写入 env 文件）
3. 写入 `/etc/realsense-wrist/wrist_left.env`、`wrist_right.env`（`WRIST_SERIAL`/`WRIST_ROTATE`/`WRIST_WIDTH`/`WRIST_HEIGHT`/`WRIST_FPS`/`WRIST_COMPRESSED`/`WRIST_JPEG_QUALITY`/`WRIST_DEPTH_MAX`）
4. 安装 udev 规则 `/etc/udev/rules.d/99-realsense-d405.rules`（0666 权限，非 root 可访问）
5. 渲染 `realsense_wrist_left.service` / `realsense_wrist_right.service` 到 `/etc/systemd/system/`
6. `systemctl daemon-reload` + `enable --now`，并在 30 秒内轮询**验证服务状态**（两台 D405 在同一 SoC 上打开管道需要数秒），未运行则直接打印 journalctl 前 10 行失败线索

仓库不在 `~/work/xhumanoid_sdk` 时无需额外参数，脚本按自身位置自动定位。

## 验证

```bash
# 1. 服务状态（systemd 部署后应为 active (running)）
systemctl status realsense_wrist_left.service
systemctl status realsense_wrist_right.service

# 2. 话题有数据
ros2 topic hz /ob_camera_wrist_left/color/image_raw
ros2 topic hz /ob_camera_wrist_right/depth/image_raw
ros2 topic hz /ob_camera_wrist_left/color/image_raw/compressed
ros2 topic hz /ob_camera_wrist_left/depth/image_raw/compressedDepth

# 3. 日志排查
journalctl -u realsense_wrist_left.service -n 50 --no-pager
```

## 参数说明（节点 ROS 参数 / launch 参数）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial` | `$WRIST_SERIAL` | 相机序列号（必填；systemd 通过 EnvironmentFile 注入） |
| `side` | `left`（或 `$WRIST_SIDE`） | `left` / `right`，决定话题名与 `frame_id` |
| `width` / `height` | `640` / `480` | 分辨率 |
| `fps` | `15` | 帧率 5/15/30（安装默认 15；本机部署实测 640×480 双流 30fps 可用，暗光下会自适应降低） |
| `enable_depth` | `true` | 是否发布深度流 |
| `rotation` | `0`（或 `$WRIST_ROTATE`） | 画面旋转角度：`0`/`90`/`180`/`270`（顺时针）。D405 无 IMU，无法自动检测安装方向，倒装设 `180` |
| `topic_prefix` | `ob_camera_wrist` | 话题前缀 -> `/ob_camera_wrist_<side>/...` |
| `enable_compressed` | `true`（或 `$WRIST_COMPRESSED`） | 是否发布 `/compressed` 与 `/compressedDepth` 话题 |
| `jpeg_quality` | `80`（或 `$WRIST_JPEG_QUALITY`） | 彩色 JPEG 质量 1-100 |
| `depth_max` | `10.0`（或 `$WRIST_DEPTH_MAX`） | compressedDepth 裁剪距离（米），超过视为无效置 0 |

## 常用控制

```bash
sudo systemctl restart realsense_wrist_left.service
sudo systemctl restart realsense_wrist_right.service
sudo systemctl stop realsense_wrist_left.service
```

## 卸载

```bash
sudo ./scripts/uninstall.sh
```

将停止并禁用两个服务、删除 service 文件、`/etc/realsense-wrist/` 环境文件与 udev 规则。

## 故障排查

| 现象 | 可能原因与处理 |
|------|----------------|
| `service` 反复重启，日志 `Failed to capture frames` | USB 接触不良/掉线；连续 5 次采帧失败后节点会自动重启 pipeline，仍失败则退出并由 systemd 重启（重新插拔相机后可自恢复）；检查 `lsusb`，确认 udev 规则已生效（`ls -l /dev/bus/usb` 权限为 0666） |
| 话题 `ros2 topic list` 看不到腕部相机（服务 active 但日志有 `All whitelist interfaces were filtered out`） | 服务启动早于 41 网段网卡就绪，FastDDS UDP 传输已永久失效且不会重试；`sudo systemctl restart realsense_wrist_<side>` 即可恢复（新版 start_wrist.sh 已内置网络等待） |
| 日志 `no camera serial` | `/etc/realsense-wrist/wrist_*.env` 缺失或 `WRIST_SERIAL` 为空，重跑 install.sh；launch 方式则检查 `serial:=` 参数 |
| 节点启动报 `pyrealsense2 is required` | `pip install pyrealsense2`；注意 systemd 服务以 `nvidia` 用户运行，需装在其默认 python3 环境 |
| 只检测到 1 台 D405 | 检查第二台 USB 连接；`udevadm trigger` 后重试 |
| 深度图全黑 | D405 为近距离深度相机，工作距离约 0.1–0.5m；显示时建议 `min_depth≈100`、`max_depth≈600` |
| 彩色与深度视野不一致 | 未启用 depth↔color 对齐（默认关闭以降低 CPU 消耗），属正常现象 |
| 画面上下颠倒（相机倒装） | 安装时加 `--left-rotate 180` / `--right-rotate 180`；或直接编辑 `/etc/realsense-wrist/wrist_<side>.env` 增加 `WRIST_ROTATE=180` 后 `sudo systemctl restart realsense_wrist_<side>` |

## 接入 camera_display 显示

`camera_display` 已内置腕部话题支持，启动时加参数即可：

```bash
# 全部四路一起显示
ros2 launch camera_display_py camera_display.launch.py \
    enable_wrist_left:=true enable_wrist_right:=true

# 仅显示左右腕（不显示头/腰）
ros2 launch camera_display_py camera_display.launch.py \
    enable_head:=false enable_waist:=false \
    enable_wrist_left:=true enable_wrist_right:=true
```

腕部为近距相机，`camera_display` 使用独立的 `wrist_min_depth`（默认 100mm）/
`wrist_max_depth`（默认 600mm）显示量程，不影响头/腰部相机的全局 `min_depth`/`max_depth`。
