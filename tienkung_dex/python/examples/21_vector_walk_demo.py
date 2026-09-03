#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""21 矢量行走接口测试：HRIC cmd_vel 速度矢量流（前进/侧移/原地旋转 → 归零站立）。

背景：双腿关节（leg 组 54/64 等 12 轴）由独立行走运控 run_patrol /
RL 全身策略独占，SDK 不允许也不应直控腿（会与平衡控制冲突）——leg 关节
"行为测试"改走官方矢量行走接口：给运控发机体坐标系速度矢量让它自己走。

接口语义（详见 core/topics.py，来源文档《具身天工DEX-矢量行走接口》）：
    - 话题 /hric/robot/cmd_vel，消息 geometry_msgs/TwistStamped；
      linear.x = 前进 vx、linear.y = 侧移 vy、angular.z = 转向 wz（rad/s），
      其余字段恒为 0；
    - 运控按"流式"理解指令：需以 ~20 Hz 持续发布当前目标速度，机器人才会
      持续走。SDK walk 子系统启动后由内部 20 Hz timer 泵出最近一次 setpoint，
      stop() 会把目标归零并继续泵零流 → 机器人站立；
    - 速度越界被 clamp 到取值范围表；||(vx,vy,wz)|| < 0.05 视为站立 → 全零；
    - 运动过程与关节指令同级受 e-stop guard 拦截，急停触发会强制归零泵零。

⚠️  真机前置（话题控制模式，文档 §4；SDK 不代做状态切换）：
    1. 遥控器切到"半身行走"/"全身行走"/"站走跑"策略，机器人稳定站立；
    2. 遥控器 e 键上拨进入话题控制（此后遥控只剩 c 键/急停有效）；
    3. 前进/侧移方向各留 1 m+（涉及转向时 360° 需留出）无障碍空间，
       周围无人员，随时可按遥控急停。
    缺任一步骤机器人不会走动（或行为未定义），因此真机默认严格交互确认。

动作（默认极保守，只验通路）：vx=0.06 m/s 前进 ~1.5 s（≈9 cm）→ 归零站立
0.6 s。位移极小；vx/vy/wz 与时长全部可由 CLI 调整。

CLI 组合语义：未显式给 --vx 时，只要给了 --vy/--wz 则 vx 自动取 0（组合
场景默认不带前向分量，避免无意的前向漂移）：
    --wz 0.3            → 原地旋转指令（vx=vy=0，wz≠0：绕机体竖直轴自转）
    --vy 0.05 --wz 0.2  → 侧移 + 原地转（不前进，侧向漂移同时绕 z 转向）
    --vx 0.1 --wz 0.3   → 前进 + 转向（带前向分量的转向弧线）
仅走直线前进时给 --vx（或全默认）。约定 wz>0 为逆时针（从机器人上方看）。

说明：cmd_vel 一侧当前无经运控返回的闭环反馈话题（实测 /hric/motion/status
无发布者），本 demo 的"通过"判定是发布面自检（指令已流式下发 / 已归零；
mock 另有自积分位移/转角核对），机器人实际运动请目测确认。

用法:
    python3 examples/21_vector_walk_demo.py                  # real（交互确认）
    python3 examples/21_vector_walk_demo.py --vx 0.1 --forward 2.0
    python3 examples/21_vector_walk_demo.py --wz 0.3        # 原地旋转指令
    python3 examples/21_vector_walk_demo.py --vy 0.05 --wz 0.2   # 侧移+原地转
    python3 examples/21_vector_walk_demo.py --backend mock   # headless
"""

import math
import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

from tienkung_dex.core import topics as t
from tienkung_dex.core.errors import BackendUnavailableError, EstopActiveError


class VectorWalkDemo(DemoBase):
    DT = 0.05                        # 20 Hz 流式帧（与运控指令节奏一致）

    def __init__(self, backend='real', vx=0.06, vy=0.0, wz=0.0,
                 forward=1.5, hold=0.6):
        super().__init__(backend, enable={'walk'})
        self.vx = vx                  # 前进速度 m/s
        self.vy = vy                  # 侧移速度 m/s
        self.wz = wz                  # 转向速度 rad/s
        self.forward = forward        # 保持非零速度的时长 s
        self.hold = hold              # 归零站立保持时长 s

    def _demo(self):
        walk = self.walk
        if walk is None:
            raise BackendUnavailableError(
                '当前后端没有 walk 子系统（sim 后端只仿真关节模型，矢量行走'
                '话题不可用）；请用 --backend real（真机）或 mock（headless）')
        lim = walk.limits
        print('  矢量行走（HRIC cmd_vel 流式速度）:')
        print(f'    clamp 范围 vx∈[{lim["vx_min"]:+.2f},{lim["vx_max"]:+.2f}] '
              f'vy∈[{lim["vy_min"]:+.2f},{lim["vy_max"]:+.2f}] '
              f'wz∈[{lim["wz_min"]:+.2f},{lim["wz_max"]:+.2f}] rad/s')
        print(f'    ||速度||<{walk.stop_norm:.2f} → 站立；'
              f'发布节奏 {t.WALK_DEFAULT_RATE_HZ:g} Hz; '
              f'topic={t.WALK_CMD_TOPIC}')
        req = f'vx={self.vx:+.3f}, vy={self.vy:+.3f}, wz={self.wz:+.3f}'
        req_norm = math.hypot(math.hypot(self.vx, self.vy), self.wz)
        label, expect = self._plan()
        if self.backend == 'real':
            print('\n  ⚠ 真机前置（话题控制模式，遥控器操作）：')
            print('    1. 遥控器切到半身/全身行走策略，机器人稳定站立;')
            print('    2. 遥控器 e 键上拨进入话题控制（遥控只剩 c 键）;')
            print('    3. 前进/侧移方向各留 1 m+（涉及转向时 360° 需留出）'
                  '无障碍空间，周围无人员，随时可按急停。')
            self._confirm_real_motion(
                f'{label}（{req}，约 {self.forward:g} s，{expect}）')
            print('  先泵 1 s 零速度流（确认话题控制链路已建立）')
            self._drive(walk, 1.0)
        print(f'  下发矢量速度目标: {req} (forward={self.forward:g} s)')
        if 0.0 < req_norm < walk.stop_norm:
            print(f'  [提示] 目标速度范数 {req_norm:.3f} < 站立阈值 '
                  f'{walk.stop_norm:.2f}，按接口语义将下发为归零站立指令；'
                  f'如需移动请加大幅值')
        streamed = False
        sim = 0.0
        set_v = (0.0, 0.0, 0.0)
        try:
            walk.set_velocity(self.vx, self.vy, self.wz)
            streamed = True
            set_v = (walk.velocity.vx, walk.velocity.vy, walk.velocity.wz)
            print(f'  已接受 setpoint: vx={set_v[0]:+.3f} vy={set_v[1]:+.3f} '
                  f'wz={set_v[2]:+.3f} (norm={walk.velocity.norm:.3f})')
            sim = self._drive(walk, self.forward) or 0.0  # mock 返回模拟时长
            if self._node is None:    # mock 才有自积分位姿
                print(f'  mock 自积分位姿: x={walk.pose_x:.4f} m, '
                      f'y={walk.pose_y:.4f} m, yaw={walk.pose_yaw:.4f} rad')
        finally:
            if streamed:              # 任何中断/异常路径都先归零站立再退出
                try:
                    walk.stop()
                except EstopActiveError:   # 急停期间归零被安全层拦截
                    print('  [急停] 运动指令被 e-stop 拦截，跳过归零下发')
                    self.hold = min(self.hold, 0.3)  # 尽快结束
                self._drive(walk, self.hold)
                print(f'  已下发归零站立并保持 {self.hold:g} s')

        self.check('矢量速度指令流已发布(帧数>0)',
                   walk.publish_count > 0,
                   f'publish_count={walk.publish_count}')
        self.check('归零后目标速度为 0',
                   walk.velocity.norm < 1e-9,
                   f'norm={walk.velocity.norm:.4f}')
        if self._node is None:
            # mock：按"已接受的 setpoint（clamp/置零后）"对自积分做逐轴核对，
            # 兼容纯前进 / 侧移 / 原地旋转（仅 wz 非零）任意组合向量。
            exp = (set_v[0] * sim, set_v[1] * sim, set_v[2] * sim)
            moved = (abs(walk.pose_x) > 1e-6 or abs(walk.pose_y) > 1e-6
                     or abs(walk.pose_yaw) > 1e-6)
            if exp == (0.0, 0.0, 0.0):  # 站立/急停/被拦截场景
                print('  [mock] 无有效运动 setpoint，跳过位姿核对')
            else:
                tol = 1e-3 + 1e-2 * sim
                ok = (abs(walk.pose_x - exp[0]) <= tol
                      and abs(walk.pose_y - exp[1]) <= tol
                      and abs(walk.pose_yaw - exp[2]) <= tol)
                self.check('mock 模型确有运动（位姿非零）', moved,
                           f'x={walk.pose_x:.4f}, y={walk.pose_y:.4f}, '
                           f'yaw={walk.pose_yaw:.4f} rad')
                self.check(f'mock 自积分≈指令积分（模拟 {sim:g} s）', ok,
                           f'期望 x={exp[0]:.4f}, y={exp[1]:.4f}, '
                           f'yaw={exp[2]:.4f} rad')
        else:
            print(f'  [目测] 真机请确认 {expect} 后已停稳站立')

    def _plan(self):
        """把 (vx, vy, wz, forward) 翻译成人类可读的预期动作，用于
        真机确认/目测提示。组合语义：仅 wz 非零（vx=vy=0）即"原地旋转指令"。"""
        s = self.forward
        parts = []
        if self.vx:
            seg = f'前进≈{abs(self.vx) * s * 100:.0f} cm'
            if self.vx < 0:
                seg += '（后退）'
            parts.append(seg)
        if self.vy:
            seg = f'侧移≈{abs(self.vy) * s * 100:.0f} cm'
            seg += '（左）' if self.vy > 0 else '（右）'
            parts.append(seg)
        if self.wz:
            deg = math.degrees(abs(self.wz) * s)
            seg = f'原地转向≈{deg:.0f}°'
            seg += '（逆时针）' if self.wz > 0 else '（顺时针）'
            parts.append(seg)
        if not parts:
            return '站立保持', '不移动'
        if self.vx == 0.0 and self.vy == 0.0:
            return '原地旋转指令', '、'.join(parts)
        return '矢量行走速度流', '、'.join(parts)

    def _drive(self, walk, seconds: float):
        """real/sim：保持 executor spin 让子系统 20 Hz timer 持续泵流；
        mock：按帧推进自积分模型（无时钟，headless 快跑）并返回模拟时长 s。"""
        if self._node is not None:
            self.spin_for(seconds)
            return None
        n = max(1, int(round(seconds / self.DT)))
        for _ in range(n):
            walk.step(self.DT)
        return n * self.DT


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--vx', type=float, default=None,
                        help='前进速度 m/s（默认：纯前进时 0.06；只要给了 '
                             '--vy/--wz 就自动为 0，组合场景不带前向分量）')
    parser.add_argument('--vy', type=float, default=0.0,
                        help='侧移速度 m/s')
    parser.add_argument('--wz', type=float, default=0.0,
                        help='转向速度 rad/s')
    parser.add_argument('--forward', type=float, default=1.5,
                        help='保持非零速度的时长 s（默认 1.5 ≈ 前进 9 cm）')
    parser.add_argument('--hold', type=float, default=0.6,
                        help='归零站立保持时长 s')
    args = parser.parse_args(argv)
    vx = args.vx if args.vx is not None else (
        0.0 if (args.vy or args.wz) else 0.06)
    return VectorWalkDemo(backend=args.backend, vx=vx, vy=args.vy,
                          wz=args.wz, forward=args.forward,
                          hold=args.hold).run()


if __name__ == '__main__':
    sys.exit(main())
