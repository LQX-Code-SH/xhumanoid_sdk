# 具身天工 DEX 矢量行走接口

> **参考文档（外部资料）**
>
> - 来源：飞书文档（https://zitd5je6f7j.feishu.cn/wiki/ZseRweRBVitwz9kbHTmcOsSQnZe）
> - 提取日期：2026-09-03
> - 说明：本文档为 TBD-Mani 项目的**外部参考资料**，由飞书文档提取整理。内容涵盖**矢量行走控制接口**（cmd_vel 速度控制/运控状态发布）、**解除半身速度限制**、**开发流程**。代码块与表格为尽力还原；**凡与官方原文不一致之处以官方原文为准**。文内图片请以原文为准（本文仅注明图片位置与标题）。

## 1. 话题控制接口

> 使用话题控制时，遥控器 **e 键上拨**；此模式下遥控器只有 **c 键**有效。

### 1.1 速度控制接口：`/hric/robot/cmd_vel`

- 在**半身行走 / 全身行走 / 站走跑一体策略**下有效。
- `linear.x`：控制机器人的**前进速度**。
- `linear.y`：控制机器人的**侧移速度**。
- `angular.z`：控制机器人的**转向速度**。
- 其他值无含义，发布时保持默认 0 值。
- 三个速度值的**范数小于 0.05** 时，机器人将保持站立状态。

### 1.2 取值范围

| 控制量 | 全身行走 | 半身行走 | 站走跑 |
| --- | --- | --- | --- |
| linear.x (m/s) | [-0.8, 1.0] | [-0.8, 1.0] | [-0.5, 2.2] |
| linear.y (m/s) | [-0.5, 0.5] | [-0.5, 0.5] | [-0.5, 0.5] |
| angular.z (rad/s) | [-0.8, 0.8] | [-0.8, 0.8] | [-1.0, 1.0] |

> 注：原文表格为 4 行 4 列，首行首格为空，首列为控制量名称，列头依次为"全身行走 / 半身行走 / 站走跑"。

### 1.3 示例：发布速度指令（20Hz）

```bash
ros2 topic pub -r 20 /hric/robot/cmd_vel geometry_msgs/msg/TwistStamped "header:
  stamp: now
  frame_id: 'pelvis'
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
```

### 1.4 速度指令发布话题：`/hric/robot/cmd_vel_status`

- 消息类型：`geometry_msgs/msg/TwistStamped`（**行走等策略下有效**）。
- 遥控器的控制指令会被转换成上述控制指令并通过该话题发布出来。

## 2. 运控状态发布接口

- 话题名：`/hric/robot/rl_state`
- 消息类型：`diagnostic_msgs/msg/DiagnosticStatus`（标准 ROS 消息）

### 2.1 示例（系统版本 26.4.x 及其之前）

```yaml
level: "\0"
name: rl_state
message: ''
hardware_id: ''
values:
- key: current_state
  value: STOP
- key: control_tool
  value: joystick
```

### 2.2 示例（系统版本 26.5.x 及其之后）

```yaml
level: "\0"
name: rl_state
message: ''
hardware_id: ''
values:
- key: current_state
  value: STOP
- key: child_state #(26.7.1开始才有，发表复合状态的具体子状态)
  value: STOP
- key: status
  value: start(running,finish)
```

### 2.3 字段说明

- `current_state`：当前状态。取值范围为所有注册状态，如 `HBWALK`、`SWR` 等；完整状态查看**键位映射表里的状态关键字**。
- `control_tool`：控制方式。取值范围：`joystick`（云卓手柄）、`topic`（话题）。
- `child_state`：当前状态的子状态。若当前状态有子状态，则**子状态才是真实的状态**；若无，则子状态与当前状态为同一状态。

## 3. 解除半身速度限制

### 3.1 找到运控包的安装位置

执行 `pip show xmigcs`，安装位置在输出中的 **Location** 字段后面（原文含示意图，图注：*"Location后面的即为安装位置"*）。

```bash
pip show xmigcs
```

### 3.2 修改安装包下的配置文件

进入安装包 config 目录并编辑配置文件（以 xmigcs 为例）：

```bash
cd /home/ubuntu/.local/lib/python3.12/site-packages/xmigcs/config
vim dex_config.yaml
```

> 在配置文件**后半部分**，把 `HBWALK` 的速度参数改成和 `Navigate` 一致的即可；保存退出后**重启运控**，即可通过**话题模式**控制半身模式下的行走。

## 4. 开发流程

可在遥控器执行完状态切换后，**e 键上拨**进入话题控制，再发送速度指令。

### 4.1 示例 1（全身行走使用速度话题控制）

1. 遥控器先进全身；
2. e 键上拨；
3. 发布 topic 控制。

### 4.2 示例 2（半身行走使用速度话题控制）

1. 遥控器先进全身；
2. 再切到半身；
3. 然后 e 键上拨，使用话题控制。

## 附录：原文图片

| 位置 | 图片信息 | 说明 |
| --- | --- | --- |
| §3.1 安装位置示意 | JPEG 881×279（`20260625-131632.jpg`） | 图注："Location 后面的即为安装位置"（`pip show xmigcs` 输出示意图） |
| §3.2 配置文件修改示意 | JPEG 362×305 | 配置文件修改示例图，内容见原文 |

> 图片正文以原文为准；本文档仅记录图片在图注/结构中的位置说明。
