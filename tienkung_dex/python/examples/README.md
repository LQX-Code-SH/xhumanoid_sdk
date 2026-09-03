# tienkung_dex examples — 手动测试 SDK 完整接口的 demo 集

22 个 demo 脚本，全部**继承 TienkungDex facade**（公共基类 `DemoBase` 复用
`create_robot()` 装配：enable / hand_vendor / topic overrides 全部透传），
脚本内**零** ROS 消息包导入、零直接的 publisher/subscriber 创建——ROS 细节
全部封装在 SDK 的 real 后端内。

- **real（默认）**：真机。机器人算力主机上直接 `python3 examples/xx.py` 即连真机
  （拿到的是真实数据，避免直接运行示例打印 mock 假数据造成误导）；
- **`--backend mock`**：完全 headless 的内存桩，供无真机的开发机、单测/CI 与流程
  走通（所有自动回归都显式 `--backend mock`，与 CLI 默认解耦）；
- **`--backend sim`**：ros_gz 仿真（推荐与 20/09 组合验证）。
- **18~20（关节运动）与 21（矢量行走）默认 real 时**：先打印风险提示并要求**终端回车确认**才下发；
  stdin 无 TTY（AI/脚本/CI 调用）一律拒绝并以退出码 2 终止——真机运动只能由人在
  终端确认触发。

退出码约定（`test_demos.sh` 风格）：`0` = 全部检查通过，`1` = 有失败项，
`130` = 用户中断。

## 列表（按功能区域分组，简易到难 = 真机测试先后顺序）

序号即推荐的真机测试顺序：先纯观测确认各子系统在线，再验证感知与语音，
然后从低风险执行机构到全身关节运动/矢量行走，最后才是底层控制模式。

### ① 基础状态观测（纯订阅/服务调用，零运动风险）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 01 | `01_serial_number_demo.py` | 序列号服务 | |
| 02 | `02_power_status_demo.py` | 电源（电压/电流/功率/急停键） | |
| 03 | `03_robot_state.py` | 关节状态订阅 / `get_state(s)` / `on_state` | |
| 04 | `04_imu_demo.py` | IMU 姿态（xsens/livox 双源） | `--source xsens\|livox` |
| 05 | `05_safety_demo.py` | **项目特有**：急停拦截（`EstopActiveError`）+ 边沿回调；real 纯观测 | |
| 06 | `06_sbus_demo.py` | SBUS 遥控器（axes + A-H 按键 + key_event 事件） | |
| 07 | `07_lidar_demo.py` | livox 点云透传 | |
| 08 | `08_gps_demo.py` | GPS 定位（可选硬件） | |

### ② 感知与语音（只读数据流 / 音频，无关节运动）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 09 | `09_camera_demo.py` | 4 路 RGB-D 相机 + 6 路全景 | `--panorama`、`--view` |
| 10 | `10_play_text_demo.py` | TTS 文字合成 `speak`（对应底层 `play_text_demo`） | `--text`、`--play-wait` |
| 11 | `11_speaker_play_demo.py` | TTS 播报控制 `speak`/`play_file`/`stop_playback`（对应底层 `speaker_play_demo`） | `--text`、`--play-file`、`--play-wait` |
| 12 | `12_speech_recognition_demo.py` | 语音事件解析（`event_type` 1/4/5/6/20，对应底层 `speech_recognition_demo`） | `--observe` |
| 13 | `13_mic_record_demo.py` | 麦克风录制 + 环形缓冲快照 + wave 写盘（对应底层 `mic_record_demo`） | `--seconds N`、`--out PATH` |
| 14 | `14_hand_state_demo.py` | 左右手状态 `get_status` + 触觉 `on_touch`（按关节名称打印最近一帧快照） | `--hand-vendor` |

### ③ 低风险执行（运动幅度小 / 独立于全身关节）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 15 | `15_light_demo.py` | 灯带 `set_mode`/`set_cmd`；默认跑马灯轮播各色灯效 `--cycle` 秒后恢复原状 | `--mode NAME`、`--cmd INT`、`--cycle 秒` |
| 16 | `16_hand_brainco_demo.py` | 强脑 6 电机手：`set_gesture`/`set_positions` + `MotorStatus` 反读 | `--side`、`--gesture`、`--pos` |
| 17 | `17_hand_inspire_demo.py` | 因时 13 关节手：`set_positions`/`set_force`/`set_speed` + `clear_error` | `--side`、`--pos/--force/--speed`、`--clear-error` |
| 18 | `18_head_control_demo.py` | 头部**大幅周期摆头/点头**（平滑插值；pos 用 spd/cur 直发保证肉眼可见）+ 回中 | `--mode pos\|imp`、`--amp-yaw`、`--amp-pitch`、`--freq`、`--cycles`、`--spd`、`--cur` |

### ④ 全身关节/矢量行走控制（运动幅度与风险递增，需先通过 ①②③）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 19 | `19_waist_control_demo.py` | 腰部三轴（pitch→roll→yaw）插值小角度动作 + 回零 | `--mode pos\|imp`、`--ramp s`、`--spd`、`--cur` |
| 20 | `20_arm_control_demo.py` | 手臂控制 + 回零（homing） | `--mode pos\|imp`、`--ramp s`、`--spd`、`--cur` |
| 21 | `21_vector_walk_demo.py` | 矢量行走接口：HRIC `/hric/robot/cmd_vel` 速度矢量流（前进/侧移/原地旋转→归零站立）。leg 关节由行走运控独占，SDK 不直控腿。给了 `--vy/--wz` 而未给 `--vx` 时 vx 自动归 0 | `--vx/--vy/--wz`、`--forward s`、`--hold s` |

### ⑤ 底层控制模式（仅 mock）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 22 | `22_joint_modes_demo.py` | **项目特有**：`ControlMode` 0-5 全集 + `ZERO_CALIB` 解锁（**仅 mock**） | |

## 用法示例

```bash
cd tienkung_dex/python

# 真机（默认 real：机器人主机直接跑；18~21 会先交互回车确认才下发）
python3 examples/03_robot_state.py
python3 examples/02_power_status_demo.py
python3 examples/20_arm_control_demo.py --mode imp    # 回车确认后执行
python3 examples/10_play_text_demo.py                 # 听机器人说话
python3 examples/12_speech_recognition_demo.py        # 唤醒 + 说话观察 ASR

# headless 内存桩（无真机开发机 / 走通流程，显式 --backend mock）
python3 examples/03_robot_state.py --backend mock
python3 examples/16_hand_brainco_demo.py --gesture rock --backend mock
python3 examples/17_hand_inspire_demo.py --pos 500 --force 300 --speed 500 --backend mock
python3 examples/09_camera_demo.py --panorama --backend mock
python3 examples/13_mic_record_demo.py --out /tmp/record.wav --backend mock
```

## 真机运行前提

1. 按仓库 README 的环境链顺序 source（xos 消息包须优先于
   `/opt/humanoid/install` 旧版本）；
2. 14/17 inspire 手：先启动 `inspire_hand` 驱动（can0/can1）；
3. **21 矢量行走**：遥控器切到半身/全身行走策略并 **e 键上拨进入话题控制**
   后才可收到 cmd_vel 速度流；前进/侧移方向各留 1 m+、**涉及转向（--wz，
   原地旋转）时 360° 需留出**无障碍空间（参考 `core/topics.py` 与
   《具身天工DEX-矢量行走接口》文档）；
4. **05 安全**：real 模式纯观测（不发关节指令），按/松急停按钮观察边沿事件；
5. **18~21**：real 默认要求**终端回车确认**（保护：真机运动只能由人
   在终端触发）。stdin 非 TTY 的 AI/脚本/CI 调用会被拒绝（退出码 2），无真机的
   流程验证请用 `--backend mock` 或 `--backend sim`；
6. **22**：拒绝 `--backend real`（低层模式直接下发有硬件风险，退出码 2）；
7. **12 语音识别**：real 需先对机器人说**唤醒词**进入对话，再说测试语，才会出现
   ASR 文本（`event_type=1`）；只触发人脸唤醒/VAD 不算识别成功。

## 裁剪说明

- **六维力**（HWI 未验证）→ 由 `tests/test_mock_backend.py`
  单测覆盖，不设示例；
- **整库健康自检** → `ros2 run tienkung_dex tienkung_dex_demo`（`once` 模式
  自带终态退出码），不重复设示例；
- **wait_ready 类阻塞等待** → 各控制类 demo 已内置 `wait_joint()` 到位判定。

## 自动化测试

`tests/test_examples.py`：22 个 demo 的类结构检查（继承 TienkungDex、`main`
可调用）+ mock 全量运行断言（退出码 0）+ 22 号（joint_modes）拒绝 real 的守卫。
（18~21 的 real 拒绝发生在运行时交互路径——非 TTY 时退出码 2——由人工/脚本
流程覆盖，不在单测内。）

示例 CLI 默认已改为 **real**，回归测试则显式 `cls(backend='mock')` 实例化，
两者互不依赖：CLI 默认值再变也不影响自动测试的确定性。

数字开头的模块名不能 `from examples.01_... import`（语法非法），测试与跨模块
引用一律 `importlib.import_module('examples.01_robot_state')`。
