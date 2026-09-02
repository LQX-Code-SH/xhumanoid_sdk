# tienkung_dex examples — 手动测试 SDK 完整接口的 demo 集

20 个 demo 脚本，全部**继承 TienkungDex facade**（公共基类 `DemoBase` 复用
`create_robot()` 装配：enable / hand_vendor / topic overrides 全部透传），
脚本内**零** ROS 消息包导入、零直接的 publisher/subscriber 创建——ROS 细节
全部封装在 SDK 的 real 后端内。

- **mock（默认）**：完全 headless，任意开发机直接 `python3 examples/xx.py`；
- **`--backend real`**：真机（机器人算力主机，按 README 顶层的环境链顺序 source）；
- **`--backend sim`**：ros_gz 仿真（推荐与 18/09 组合验证）。

退出码约定（`test_demos.sh` 风格）：`0` = 全部检查通过，`1` = 有失败项，
`130` = 用户中断。

## 列表（按功能区域分组，简易到难 = 真机测试先后顺序）

序号即推荐的真机测试顺序：先纯观测确认各子系统在线，再验证感知与语音，
然后从低风险执行机构到躯干关节，最后才是底层控制模式。

### ① 基础状态观测（纯订阅/服务调用，零运动风险）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 01 | `01_serial_number_demo.py` | 序列号服务 | |
| 02 | `02_power_status_demo.py` | 电源（电压/电流/功率/急停键） | |
| 03 | `03_robot_state.py` | 关节状态订阅 / `get_state(s)` / `on_state` | |
| 04 | `04_imu_demo.py` | IMU 姿态（xsens/livox 双源） | `--source xsens\|livox` |
| 05 | `05_safety_demo.py` | **项目特有**：急停拦截（`EstopActiveError`）+ 边沿回调；real 纯观测 | |
| 06 | `06_sbus_demo.py` | SBUS 遥控器（axes + A-F 按键） | |
| 07 | `07_lidar_demo.py` | livox 点云透传 | |
| 08 | `08_gps_demo.py` | GPS 定位（可选硬件） | |

### ② 感知与语音（只读数据流 / 音频，无关节运动）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 09 | `09_camera_demo.py` | 4 路 RGB-D 相机 + 6 路全景 | `--panorama` |
| 10 | `10_audio_ai_demo.py` | TTS `speak`/`play_file` + 语音事件 | `--play-file PATH` |
| 11 | `11_audio_record_demo.py` | **项目特有**：录制 + 环形缓冲快照 + wave 写盘 | `--seconds N`、`--out PATH` |
| 12 | `12_hand_state_demo.py` | 手部状态 `get_status` + 触觉 `on_touch` | `--hand-vendor`、`--side` |

### ③ 低风险执行（运动幅度小 / 独立于全身关节）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 13 | `13_light_demo.py` | 灯带 `set_mode`/`set_cmd` | `--mode NAME`、`--cmd INT` |
| 14 | `14_hand_brainco_demo.py` | 强脑 6 电机手：`set_gesture`/`set_positions` + `MotorStatus` 反读 | `--side`、`--gesture`、`--pos` |
| 15 | `15_hand_inspire_demo.py` | 因时 13 关节手：`set_positions`/`set_force`/`set_speed` + `clear_error` | `--side`、`--pos/--force/--speed`、`--clear-error` |
| 16 | `16_head_control_demo.py` | 头部 `move_to` / `impedance` + 回零 | `--mode pos\|imp` |

### ④ 躯干关节控制（运动幅度与风险递增，需先通过 ①②③）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 17 | `17_waist_control_demo.py` | 腰部控制 + 回零 | `--mode pos\|imp` |
| 18 | `18_arm_control_demo.py` | 手臂控制 + 回零（homing） | `--mode pos\|imp` |
| 19 | `19_leg_control_demo.py` | 腿部控制 + 回零 ⚠️ 需安全支架 | `--mode pos\|imp` |

### ⑤ 底层控制模式（仅 mock）

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 20 | `20_joint_modes_demo.py` | **项目特有**：`ControlMode` 0-5 全集 + `ZERO_CALIB` 解锁（**仅 mock**） | |

## 用法示例

```bash
cd tienkung_dex/python

# mock（默认，任意开发机）
python3 examples/03_robot_state.py
python3 examples/14_hand_brainco_demo.py --gesture rock
python3 examples/15_hand_inspire_demo.py --pos 500 --force 300 --speed 500
python3 examples/09_camera_demo.py --panorama
python3 examples/11_audio_record_demo.py --out /tmp/record.wav

# 真机（机器人主机，先按 README 顶层顺序加载 xos 环境链）
python3 examples/03_robot_state.py --backend real
python3 examples/16_head_control_demo.py --backend real --mode imp
python3 examples/15_hand_inspire_demo.py --backend real --pos 500
```

## 真机运行前提

1. 按仓库 README 的环境链顺序 source（xos 消息包须优先于
   `/opt/humanoid/install` 旧版本）；
2. 12/15 inspire 手：先启动 `inspire_hand` 驱动（can0/can1）；
3. **19 腿部**：机器人必须固定在安全支架上；
4. **05 安全**：real 模式纯观测（不发关节指令），按/松急停按钮观察边沿事件；
5. **20**：拒绝 `--backend real`（低层模式直接下发有硬件风险，退出码 2）。

## 裁剪说明

- **六维力**（HWI 未验证）→ 由 `tests/test_mock_backend.py`
  单测覆盖，不设示例；
- **整库健康自检** → `ros2 run tienkung_dex tienkung_dex_demo`（`once` 模式
  自带终态退出码），不重复设示例；
- **wait_ready 类阻塞等待** → 各控制类 demo 已内置 `wait_joint()` 到位判定。

## 自动化测试

`tests/test_examples.py`：20 个 demo 的类结构检查（继承 TienkungDex、`main`
可调用）+ mock 全量运行断言（退出码 0）+ 20 拒绝 real 的守卫。

数字开头的模块名不能 `from examples.01_... import`（语法非法），测试与跨模块
引用一律 `importlib.import_module('examples.01_robot_state')`。
