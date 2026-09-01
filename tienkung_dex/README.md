# tienkung_dex — TienkungDex 机器人门面库

面向 **具身天工 3.0** 的 SDK 能力统一封装（设计文档：主仓库 `docs/详细设计/09-TienkungDex机器人类设计.md`）。把一个门面类 `TienkungDex` 暴露给上层模块，屏蔽 13 个示例 demo 的约 30 个话题/服务与 5 个消息包；**实机（real）/ 仿真（sim）/ 内存桩（mock）三套后端共用一套接口**，仿真与离机开发不需要 `/opt/humanoid` 消息包。

## 目录

```
tienkung_dex/
├── python/
│   ├── package.xml  setup.py  resource/
│   ├── launch/tienkung_dex.launch.py
│   ├── config/joints.yaml            # 关节 ID 映射（附录 A：占位示例，待实机导出填写）
│   ├── tienkung_dex/
│   │   ├── robot.py                  # TienkungDex 门面 + create_robot 工厂
│   │   ├── demo_node.py              # 健康自检 demo 节点（真机验证用）
│   │   ├── core/                     # 值对象 / 抽象基类 / 话题常量（仅标准消息）
│   │   └── backends/
│   │       ├── real/                 # 真机：SDK 话题适配（唯一 import *_msgs 的层）
│   │       ├── sim/                  # 仿真：ros_gz 桥接同名话题 + 两指手模型
│   │       └── mock.py               # 内存桩：无 ROS 图，单测/故障注入
│   └── tests/                        # pytest（40 例，headless）
└── README.md
```

## 快速开始

```python
import rclpy
from tienkung_dex import create_robot

node = ...                     # 调用方的 rclpy 节点（依赖注入）
robot = create_robot(node, backend='real')   # 'real' | 'sim' | 'mock'
robot.start()                  # 统一建订阅/客户端（幂等）

# 关节控制（位置 / 力位混合，50 Hz 节拍由调用方控制循环持有）
robot.arm.move_to({21: -1.588})              # 右肩 Pitch
robot.arm.impedance({21: -0.5}, kp=50.0, kd=2.0)
robot.arm.wait_until(21, -1.588, tol_deg=5.0, timeout=10.0)
pos = robot.arm.get_state(21)                # JointReading 快照

# 相机（color=BGR / depth=uint16 mm，BEST_EFFORT 已匹配）
cam = robot.cameras['ob_camera_head']
cam.on_frame(print)                          # 或 frame = cam.latest()

# 音频 / 语音
robot.audio.speak('你好')
robot.audio.on_voice_event(cb, event_types={1})     # 1=ASR 结果

# 安全（急停时任何关节指令抛 EstopActiveError）
robot.safety.on_estop(cb)
print(robot.health())                        # 各子系统 is_active 汇总

robot.shutdown()                # 逆序关闭（幂等）
```

## 真机验证（复刻 demo 可观测行为，设计文档 §10 T2–T4）

```bash
# 机器人算力主机上，按顺序加载环境后编译
source /opt/ros/jazzy/setup.bash && source /opt/humanoid/install/setup.bash
source ~/xos/setup.bash
colcon build --packages-select tienkung_dex
source install/setup.bash

# 健康自检：所有子系统 is_active 应在数据时限内翻 True
ros2 launch tienkung_dex tienkung_dex.launch.py
```

关节 ID 表：见 `python/config/joints.yaml`（占位）与设计文档附录 A 的导出核对方法；未填写完整前默认 `strict=false`（未知 ID 仅告警），可用 `create_robot(..., strict_joint_ids=True)` 强制拒绝。

## 单测（任意开发机，无需 ROS 图）

```bash
cd tienkung_dex/python
python3 -m pytest tests -q        # 40 passed（headless mock 后端）
```

## 已知限制（与设计文档 §11 开放问题一一对应）

1. **head/waist/leg 消息类与 `/robot_state` 对应字段**仅有 arm 有 demo 佐证：`HeadCtrl/WaistCtrl/LegCtrl` 缺失时回退 `ArmCtrl`（字段集一致，HWI §7.1），启动日志会告警；
2. **因时 13 维手**无 demo 佐证，仅真机脑手（brainco）实现；`sim`/`mock` 后端不支持 `hand_vendor='inspire'`（工厂直接抛 `BackendUnavailableError`）；
3. **六维力**（HWI 待验证）：话题名需显式传 `force_topic=`；无数据时 `is_active=False`、`latest()=None`，不抛异常；
4. **`RobotState.imu` 单位**：demo 双版本按"度"透传姿态角，库不做换算，`ImuReading` 字段标"待验证"；
5. **全景 6 目相机**（选配）：默认不启用（`enable` 不含 `panorama`），且非 Bi-View 感知源（仅能力封装）；压缩话题需 cv2 才能解码，推荐 raw；
6. **急停消息类名**未被任何 demo 钉死：`_msgs.key_status_msg()` 按候选名探测 `is_estop` 字段；探测失败时安全监测降级（`is_active=False`、拦截关闭，日志告警）；
7. **仿真关节桥接话题**（`/joint_states`、`/tienkung_dex/joint_cmds`）为库默认值，实际 ros_gz 桥接映射以主工程 `simulation` 配置为准（06 §5）。

## 设计模式（详见设计文档 §2）

Facade（`TienkungDex`）｜ Strategy + Abstract Factory（三后端整体创建，禁止混搭）｜ Adapter（消息层与值对象互转，仅 real 后端 import `*_msgs`）｜ Observer（`on_xxx` 回调，须可重入）｜ Template Method（`start/on_start` 生命周期）｜ Proxy（`latest()` 加锁快照）。

## 许可证

Apache-2.0（同 SDK 仓库）。
