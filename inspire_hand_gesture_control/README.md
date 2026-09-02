# 因时灵巧手手势控制节点（选配）

> **适用平台**: 具身天工3.0 (Thor) | **选配功能** - 需要搭配因时灵巧手使用
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

基于 `inspire_hand_msgs` 消息定义的ROS2节点,通过 `SetAngle` 位置控制因时灵巧手实现预设手势。

提供两个版本：
- **Python版本**: `inspire_hand_gesture_control_py`
- **C++版本**: `inspire_hand_gesture_control_cpp`

## 功能

支持以下手势:
- **OK手势** (`ok`): 大拇指和食指捏合,其他手指伸直
- **石头** (`rock`): 所有手指弯曲握拳
- **剪刀** (`scissors`): 食指和中指伸直,其他手指弯曲
- **布** (`paper`): 所有手指伸直张开

## 目录结构

```
inspire_hand_gesture_control/
├── README.md
├── inspire_hand_gesture_interfaces/      # 服务消息定义包
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── srv/
│       └── GestureCommand.srv
├── python/                              # Python版本 (inspire_hand_gesture_control_py)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   │   └── inspire_hand_gesture_control_py
│   ├── inspire_hand_gesture_control/
│   │   ├── __init__.py
│   │   └── hand_gesture_control.py
│   └── launch/
│       └── hand_gesture_control.launch.py
└── cpp/                                 # C++版本 (inspire_hand_gesture_control_cpp)
    └── inspire_hand_gesture_control/
        ├── CMakeLists.txt
        ├── package.xml
        ├── src/
        │   └── hand_gesture_control.cpp
        ├── config/
        │   └── gesture_config.yaml
        └── launch/
            └── hand_gesture_control.launch.py
```

## 依赖

- ROS2 (Jazzy)
- `inspire_hand_msgs` 包
- `inspire_hand_gesture_interfaces` 包 (服务消息定义)

## 编译

### 首先编译消息包

```bash
cd ~/xos
colcon build --packages-select inspire_hand_msgs inspire_hand_gesture_interfaces
source setup.bash
```

### Python版本

```bash
cd ~/xos
colcon build --packages-select inspire_hand_gesture_control_py
source setup.bash
```

### C++版本

```bash
cd ~/xos
colcon build --packages-select inspire_hand_gesture_control_cpp
source setup.bash
```

## 使用方法

### 0. 先启动因时手驱动

手势控制节点只发布控制命令,电机驱动由 `inspire_hand` 包提供:

```bash
# 右手 (socketcan 网卡 can1)
ros2 launch inspire_hand right_hand.launch.py

# 左手 (socketcan 网卡 can0)
ros2 launch inspire_hand left_hand.launch.py
```

### 1. 启动节点

#### Python版本

```bash
# 默认控制右手
ros2 launch inspire_hand_gesture_control_py hand_gesture_control.launch.py

# 控制左手
ros2 launch inspire_hand_gesture_control_py hand_gesture_control.launch.py hand_prefix:=left_hand hand_id:=1
```

#### C++版本

```bash
# 默认控制右手
ros2 launch inspire_hand_gesture_control_cpp hand_gesture_control.launch.py

# 控制左手
ros2 launch inspire_hand_gesture_control_cpp hand_gesture_control.launch.py hand_prefix:=left_hand hand_id:=1
```

### 2. 调用服务触发手势

```bash
# OK手势
ros2 service call /gesture_command inspire_hand_gesture_interfaces/srv/GestureCommand "{gesture: 'ok'}"

# 石头
ros2 service call /gesture_command inspire_hand_gesture_interfaces/srv/GestureCommand "{gesture: 'rock'}"

# 剪刀
ros2 service call /gesture_command inspire_hand_gesture_interfaces/srv/GestureCommand "{gesture: 'scissors'}"

# 布
ros2 service call /gesture_command inspire_hand_gesture_interfaces/srv/GestureCommand "{gesture: 'paper'}"
```

### 3. 查看状态

```bash
# 查看右手关节实际角度 (含 joint_names[13] 真实关节布局)
ros2 topic echo /right_hand/angle_actual

# 查看左手关节实际角度
ros2 topic echo /left_hand/angle_actual

# 查看触觉数据
ros2 topic echo /right_hand/touch_data
```

## 参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `hand_prefix` | `right_hand` | 手的前缀: `right_hand` 或 `left_hand` |
| `hand_id` | `2` | 手编号: `1`=左手, `2`=右手 (vendor demo 07 约定) |

## 服务接口

### 手势控制服务

| 服务名 | 消息类型 | 描述 |
|--------|----------|------|
| `/gesture_command` | inspire_hand_gesture_interfaces/srv/GestureCommand | 触发指定手势 |

**请求字段:**
- `gesture` (string): 手势名称, 支持: `ok`, `rock`, `scissors`, `paper`

**响应字段:**
- `success` (bool): 执行结果
- `message` (string): 结果信息

## 话题接口

### 订阅话题

| 话题 | 消息类型 | 描述 |
|------|----------|------|
| `/{hand_prefix}/angle_actual` | inspire_hand_msgs/GetAngleAct | 关节实际角度 + joint_names[13] |
| `/{hand_prefix}/touch_data` | inspire_hand_msgs/TouchData | 触觉反馈 (选配) |

### 发布话题

| 话题 | 消息类型 | 描述 |
|------|----------|------|
| `/{hand_prefix}/angle_set` | inspire_hand_msgs/SetAngle | 关节角度控制命令 |

## 关节索引

因时手 `SetAngle.joint_values` 为固定 13 个关节。**真实关节布局以 `angle_actual` 话题的 `joint_names[13]` 实测为准**（节点首次收到状态时会打印）。

当前手势表按以下假设填写（前 6 个关节沿用强脑手 6 电机的手指映射）:

| 索引 | 假设手指 | 描述 |
|------|---------|------|
| 0 | 拇指弯曲 | 控制拇指弯曲程度 |
| 1 | 拇指旋转 | 控制拇指旋转角度 |
| 2 | 食指 | |
| 3 | 中指 | |
| 4 | 无名指 | |
| 5 | 小指 | |
| 6-12 | 其余关节 | 手势表中按伸直处理 |

## 位置范围

位置值范围: **0 ~ 1000**（vendor demo 07 约定）

> ⚠️ 0 与 1000 分别对应伸直/弯曲还是相反,需按实际硬件确认。

## 位置校准

**重要**: 代码中的位置值是示例值,需要根据实际因时手硬件进行校准。

校准步骤:
1. 启动因时手驱动和本节点;
2. `ros2 topic echo /{hand_prefix}/angle_actual` 查看 `joint_names[13]` 真实布局;
3. 用 `ros2 topic pub` 单关节逐步测试位置方向与范围;
4. 根据实测结果调整 `initGesturePositions()` 中的位置值。

> 补充: `inspire_hand_msgs` 还提供 `Setgestureno` 服务(固件内置手势编号),若固件支持可直接下发编号,无需本节点的手势表。

## 许可证

Apache-2.0
