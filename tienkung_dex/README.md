# tienkung_dex — TienkungDex 机器人门面库

面向 **具身天工 3.0** 的 SDK 能力统一封装（设计文档：主仓库 `docs/详细设计/09-TienkungDex机器人类设计.md`）。把一个门面类 `TienkungDex` 暴露给上层模块，屏蔽 20 个示例 demo 的约 30 个话题/服务与 5 个消息包；**实机（real）/ 仿真（sim）/ 内存桩（mock）三套后端共用一套接口**，仿真与离机开发不需要 `/opt/humanoid` 消息包。

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
│   │       ├── real/                 # 真机：SDK 话题适配（唯一 import *_msgs 的层；
│   │       │                         #   joint/audio/camera/safety/sensors/
│   │       │                         #   hand(brainco+inspire)/power/light/sbus/serial）
│   │       ├── sim/                  # 仿真：ros_gz 桥接同名话题 + 两指手模型
│   │       └── mock.py               # 内存桩：无 ROS 图，单测/故障注入
│   ├── examples/                     # 20 个手动测试 demo（继承 TienkungDex，
│   │                                 #   mock 默认；详见 examples/README.md）
│   └── tests/                        # pytest（headless，含 examples 结构/运行测试）
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

# 电源 / 灯带 / 遥控器 / 身份
print(robot.power.latest())                  # PowerReading(电压/电流/功率/急停)
robot.light.set_mode('wakeup')               # 灯带预设：battery_normal/charging/...
print(robot.sbus.latest())                   # SbusReading(摇杆轴 + 按键)
print(robot.serial.get_serial_number())      # 机器人序列号

# 安全（急停时任何关节指令抛 EstopActiveError）
robot.safety.on_estop(cb)
print(robot.health())                        # 各子系统 is_active 汇总

robot.shutdown()                # 逆序关闭（幂等）
```

自检终态模式（✅/❌ 报告 + 退出码=异常子系统数，交付验收/CI 用）：

```bash
ros2 run tienkung_dex tienkung_dex_demo --ros-args -p once:=True -p once_duration:=8.0
```

## 真机验证（复刻 demo 可观测行为，设计文档 §10 T2–T4）

```bash
# 机器人算力主机上，按顺序加载环境后编译
source /opt/ros/jazzy/setup.bash && source /opt/humanoid/install/setup.bash
source ~/xos/setup.bash
# 真机注意：/opt/humanoid/install 里的 lyre_msgs/bodyctrl_msgs 等为旧版本，
# 与运行中的生产系统（~/xos 源码工作区）不一致，须让 xos 消息包优先
for p in lyre_msgs interaction_msgs bodyctrl_msgs brainco_hand_msgs navigation_msgs; do
  export PYTHONPATH=$HOME/xos/$p/lib/python3.12/site-packages:$PYTHONPATH
  export LD_LIBRARY_PATH=$HOME/xos/$p/lib:$LD_LIBRARY_PATH
done
# 仓库根有 COLCON_IGNORE（排除第三方示例包），需用 --base-paths 指定本包
colcon build --base-paths tienkung_dex
source install/setup.bash

# 健康自检：所有子系统 is_active 应在数据时限内翻 True
ros2 launch tienkung_dex tienkung_dex.launch.py
```

关节 ID 表：见 `python/config/joints.yaml`（占位）与设计文档附录 A 的导出核对方法；未填写完整前默认 `strict=false`（未知 ID 仅告警），可用 `create_robot(..., strict_joint_ids=True)` 强制拒绝。

## 单测（任意开发机，无需 ROS 图）

```bash
cd tienkung_dex/python
python3 -m pytest tests -q        # headless mock 后端（含 examples 测试）
```

## 示例（examples/，手动测试 SDK 完整接口）

20 个 demo 脚本，全部**继承 TienkungDex**（`DemoBase` 基类复用 `create_robot()` 装配），序号按功能区域重排为真机测试先后顺序（① 状态观测 → ② 感知语音 → ③ 低风险执行 → ④ 躯干关节 → ⑤ 底层模式）。默认 mock 后端（任意开发机可跑），真机加 `--backend real`：

```bash
cd tienkung_dex/python
python3 examples/03_robot_state.py                    # mock：关节状态订阅
python3 examples/18_arm_control_demo.py --mode imp    # mock：阻抗控制
python3 examples/03_robot_state.py --backend real     # 真机（需 xos 环境链）
```

退出码 0 = 全部检查通过（`test_demos.sh` 风格）。完整列表、参数与安全边界见 [examples/README.md](python/examples/README.md)；裁剪说明：六维力由单测覆盖、整库健康自检用 `demo_node` 的 `once` 模式，均不重复设示例。

## 已知限制（与设计文档 §11 开放问题一一对应）

1. **head/waist/leg 消息类与 `/robot_state` 对应字段**仅有 arm 有 demo 佐证：`HeadCtrl/WaistCtrl/LegCtrl` 缺失时回退 `ArmCtrl`（字段集一致，HWI §7.1），启动日志会告警；
2. **因时 13 维手**接口已在真机确认（`angle/force/speed_set` 话题、13 维 `joint_values`、`angle_actual/force_actual/touch_data` 反馈、`SetClearError` 清错服务），real 后端支持 `hand_vendor='inspire'`；`sim` 后端仍不支持（工厂抛 `BackendUnavailableError`，保持两指手模型），`mock` 为内存桩（与手厂商无关）；
3. **六维力**（HWI 待验证）：话题名需显式传 `force_topic=`；无数据时 `is_active=False`、`latest()=None`，不抛异常；
4. **`RobotState.imu` 单位**：demo 双版本按"度"透传姿态角，库不做换算，`ImuReading` 字段标"待验证"；
5. **全景 6 目相机**（选配）：默认不启用（`enable` 不含 `panorama`），且非 Bi-View 感知源（仅能力封装）；压缩话题需 cv2 才能解码，推荐 raw；
6. **急停消息类名**未被任何 demo 钉死：`_msgs.key_status_msg()` 按候选名探测 `is_estop` 字段；探测失败时安全监测降级（`is_active=False`、拦截关闭，日志告警）。`is_active` 为**数据时限语义**（key_status 实测 ~12.6 Hz，默认 0.5 s 时限）：健康自检中 safety 变红 = key_status 流断流（急停源失联），不再是"已订阅即绿"；
7. **电源板/电池**：`power` 子系统只提取已验证的主电池三值（电压/电流/功率）与急停键位；`/power/board/status` 仅作在线性计数，其字段未提取；
8. **SBUS 按键事件**依赖 `bodyctrl_msgs/SbusData`；缺失时摇杆（标准 `Joy`）仍可用，`SbusReading.buttons` 恒为空；
9. **灯带**为纯发布子系统（命令码表见 `core/topics.LIGHT_CMDS`），`is_active` 只反映发布者创建状态（无反馈话题可核）；
10. **头部电机 2 限位**：外部参考脚本写 `[-15, 50]°`，SDK 参考表/joints.yaml 写 `[-15, 58]°`（1.0123 rad），待真机微动核对后再开 strict 模式。
11. **仿真关节桥接话题**（`/joint_states`、`/tienkung_dex/joint_cmds`）为库默认值，实际 ros_gz 桥接映射以主工程 `simulation` 配置为准（06 §5）。
12. **子生命周期是一次性的**：`on_stop` 只释放引用、不销毁 rclpy 实体（订阅/发布器挂在节点上），stop→start 循环会产生重复句柄与重复回调；需要全新生命周期请重建 facade（`create_robot()`），节点销毁是资源回收点（设计决策，见 `core/base.py` SubsystemBase docstring）；
13. **阻塞式服务调用**（`speak/play_file(blocking=True)`、`stop_playback`、`get_serial_number`）内部自行 spin 节点，**必须在 executor spin 线程之外调用**——从订阅/服务回调里调用会死锁或抛 RuntimeError；
14. **sim 后端能力子集**：只支持 `joint/camera/panorama/hand/audio/safety/imu/lidar`，`enable` 里的其余键（power/light/sbus/serial/gps/force）无仿真等价物，对应门面属性为 None（工厂会告警）。

## 设计模式（详见设计文档 §2）

Facade（`TienkungDex`）｜ Strategy + Abstract Factory（三后端整体创建，禁止混搭）｜ Adapter（消息层与值对象互转，仅 real 后端 import `*_msgs`）｜ Observer（`on_xxx` 回调，须可重入）｜ Template Method（`start/on_start` 生命周期）｜ Proxy（`latest()` 加锁快照）。

## 许可证

Apache-2.0（同 SDK 仓库）。
