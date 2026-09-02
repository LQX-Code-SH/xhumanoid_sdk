# tienkung_dex examples — 手动测试 SDK 完整接口的 demo 集

19 个 demo 脚本，全部**继承 TienkungDex facade**（公共基类 `DemoBase` 复用
`create_robot()` 装配：enable / hand_vendor / topic overrides 全部透传），
脚本内**零** vendor 消息导入、零直接的 publisher/subscriber 创建——ROS 细节
全部封装在 SDK 的 real 后端内。

- **mock（默认）**：完全 headless，任意开发机直接 `python3 examples/xx.py`；
- **`--backend real`**：真机（机器人算力主机，按 README 顶层的环境链顺序 source）；
- **`--backend sim`**：ros_gz 仿真（推荐与 04/12 组合验证）。

退出码约定（`test_demos.sh` 风格）：`0` = 全部检查通过，`1` = 有失败项，
`130` = 用户中断。

## 列表

| # | 脚本 | 测试接口 | 常用参数 |
|---|------|----------|----------|
| 01 | `01_robot_state.py` | 关节状态订阅 / `get_state(s)` / `on_state` | |
| 02 | `02_imu_demo.py` | IMU 姿态（xsens/livox 双源） | `--source xsens\|livox` |
| 03 | `03_head_control_demo.py` | 头部 `move_to` / `impedance` + 回零 | `--mode pos\|imp` |
| 04 | `04_arm_control_demo.py` | 手臂控制 + 回零（homing） | `--mode pos\|imp` |
| 05 | `05_waist_control_demo.py` | 腰部控制 + 回零 | `--mode pos\|imp` |
| 06 | `06_leg_control_demo.py` | 腿部控制 + 回零 ⚠️ 需安全支架 | `--mode pos\|imp` |
| 07 | `07_hand_control_demo.py` | 手部 `set_gesture`/`set_positions`/`set_force`/`set_speed`/`clear_error` | `--hand-vendor inspire\|brainco`、`--side`、`--gesture`、`--pos/--tor/--spd`、`--clear-error` |
| 08 | `08_power_status_demo.py` | 电源（电压/电流/功率/急停键） | |
| 09 | `09_sbus_demo.py` | SBUS 遥控器（axes + A-F 按键） | |
| 10 | `10_lidar_demo.py` | livox 点云透传 | |
| 11 | `11_audio_ai_demo.py` | TTS `speak`/`play_file` + 语音事件 | `--play-file PATH` |
| 12 | `12_camera_demo.py` | 4 路 RGB-D 相机 + 6 路全景 | `--panorama` |
| 13 | `13_gps_demo.py` | GPS 定位（可选硬件） | |
| 14 | `14_light_demo.py` | 灯带 `set_mode`/`set_cmd` | `--mode NAME`、`--cmd INT` |
| 15 | `15_hand_state_demo.py` | 手部状态 `get_status` + 触觉 `on_touch` | `--hand-vendor`、`--side` |
| 16 | `16_serial_number_demo.py` | 序列号服务 | |
| 17 | `17_safety_demo.py` | **项目特有**：急停拦截（`EstopActiveError`）+ 边沿回调；real 纯观测 | |
| 18 | `18_audio_record_demo.py` | **项目特有**：录制 + 环形缓冲快照 + wave 写盘 | `--seconds N`、`--out PATH` |
| 19 | `19_joint_modes_demo.py` | **项目特有**：`ControlMode` 0-5 全集 + `ZERO_CALIB` 解锁（**仅 mock**） | |

## 用法示例

```bash
cd tienkung_dex/python

# mock（默认，任意开发机）
python3 examples/01_robot_state.py
python3 examples/07_hand_control_demo.py --gesture rock --pos 200
python3 examples/12_camera_demo.py --panorama
python3 examples/18_audio_record_demo.py --out /tmp/record.wav

# 真机（机器人主机，先按 README 顶层顺序加载 xos 环境链）
python3 examples/01_robot_state.py --backend real
python3 examples/03_head_control_demo.py --backend real --mode imp
python3 examples/07_hand_control_demo.py --backend real --hand-vendor inspire --pos 500
```

## 真机运行前提

1. 按仓库 README 的环境链顺序 source（xos 消息包须优先于
   `/opt/humanoid/install` 旧版本）；
2. 07/15 inspire 手：先启动 `inspire_hand` 驱动（can0/can1）；
3. **06 腿部**：机器人必须固定在安全支架上；
4. **17 安全**：real 模式纯观测（不发关节指令），按/松急停按钮观察边沿事件；
5. **19**：拒绝 `--backend real`（低层模式直接下发有硬件风险，退出码 2）。

## 与 vendor demo（tiangong_dex_sdk_demo_orin41_2）对照

01~16 与 vendor 脚本 01~16 一一对应（vendor 直接 `import *_msgs` 发布/订阅；
本集全部经由 tienkung_dex 公共 API）。裁剪说明：

- **六维力**（vendor 无对应 demo，HWI 未验证）→ 由 `tests/test_mock_backend.py`
  单测覆盖，不设示例；
- **整库健康自检** → `ros2 run tienkung_dex tienkung_dex_demo`（`once` 模式
  自带终态退出码），不重复设示例；
- **wait_ready 类阻塞等待** → 各控制类 demo 已内置 `wait_joint()` 到位判定。

## 自动化测试

`tests/test_examples.py`：19 个 demo 的类结构检查（继承 TienkungDex、`main`
可调用）+ mock 全量运行断言（退出码 0）+ 19 拒绝 real 的守卫。

数字开头的模块名不能 `from examples.01_... import`（语法非法），测试与跨模块
引用一律 `importlib.import_module('examples.01_robot_state')`。
