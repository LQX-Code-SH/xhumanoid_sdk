#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""21 腿部关节控制：左右膝相对站立位平滑微屈 + 回初始位（pos / imp 两种模式）。

⚠️  真机运行前必须将机器人固定在安全支架上——腿部失稳会直接摔倒。
    home 基准：真机站立默认位并非 0（03_robot_state 实测：54≈+0.368 rad、
    64≈+0.341 rad，膝本就微屈），故先把动作前读数记为 home，目标 = home + Δ
    （Δ = +0.15 rad ≈ 8.6°，朝更屈膝方向多折一小角度），结束后回 home。
    不做"回 0"——0 不是站立自然位，若回 0 等于向伸直方向多走 ~20°。

腿部关节 ID 语义（config/joints.yaml；站立 home 参考见该文件底部
home_standing 模块，与关节参数表分开维护）:
    joint 54 = left_knee_pitch   左膝屈伸，limits [-0.0873, 2.5307]
    joint 64 = right_knee_pitch  右膝屈伸，limits [-0.0873, 2.5307]
    ⚠ 方向语义（Δ 正值=更屈膝小腿后折还是反而伸直？）尚未真机实测——leg 段
    12 轴均仍待微动核对（joints.yaml 顶部"方向验证进度"）。真机跑时请目测
    54/64 位移方向并记录：若读数朝正值方向增加时小腿确实向后折即为正方向一致，
    作为 leg 段方向定标的第一手证据。

演示动作：记录两膝当前读数为 home → 同帧 20 Hz 线性插值走到 home+Δ
（真机目标 ≈0.52/0.49 rad，仅多屈 ~8.6°，极小动作，只验证通路与方向）
→ 判停 → 同帧插值回 home。单段插值时长 --ramp 控制（默认 3 s，加大更平缓），
不走一次到位的高速跳变。总时长约 2 段 × 3 s + 判停 ≈ 10 s。

驱动力说明（沿用 18/19 真机结论）：move_to 默认 spd=0.3/cur=10 扭矩不足、动作
过小甚至判 ❌；膝 pitch 只带动小腿段（负载远小于髋/腰），但保险起见本 demo
pos 直发 JointCommand(spd=0.5, cur=40)，可用 --spd/--cur 现场调整。imp 模式
全程阻抗（含回 home），不中途切回 pos。

用法:
    python3 examples/21_leg_control_demo.py                       # real 默认（支架）
    python3 examples/21_leg_control_demo.py --mode imp            # 阻抗模式
    python3 examples/21_leg_control_demo.py --ramp 5              # 每段更慢
    python3 examples/21_leg_control_demo.py --cur 60              # 提高电流
    python3 examples/21_leg_control_demo.py --backend mock        # headless
"""

import sys
import time

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

from tienkung_dex.core.types import ControlMode, JointCommand


class LegControlDemo(DemoBase):
    GROUP = 'leg'
    DT = 0.05                        # 20 Hz 插值帧
    # 本 demo 的左右膝两轴：(joint_id, 关节名, 相对站立位增量 Δ rad)
    # Δ=+0.15（≈8.6°）：真机站立膝默认位非 0（54≈+0.368/64≈+0.341 rad），
    # 目标 = 动作前读数 + Δ（朝更屈方向），结束后回动作前读数，不回 0。
    AXES = ((54, 'left_knee_pitch', 0.15),
            (64, 'right_knee_pitch', 0.15))

    def __init__(self, backend='real', mode='pos',
                 ramp=3.0, spd=0.5, cur=40.0):
        super().__init__(backend, enable={'joint'})
        self.mode = mode
        self.ramp = ramp              # 单段（到位或回 home）插值时长 s
        self.spd = spd
        self.cur = cur

    def _demo(self):
        if self.backend != 'mock':
            print('  ⚠️  请确认机器人已固定在安全支架上（5 秒后开始）')
            time.sleep(5.0)
        group = getattr(self, self.GROUP)
        # home = 动作前读数：真机站立膝默认位非 0（54≈+0.368/64≈+0.341 rad），
        # 目标 = home + Δ，结束后回 home，勿回 0（0 不是站立自然位）。
        home = {jid: (group.get_state(jid).pos
                      if group.get_state(jid) is not None else 0.0)
                for jid, _, _ in self.AXES}
        self._confirm_real_motion(
            'leg 关节运动指令（左右膝 54/64 相对当前站立位 +0.15 rad 微屈膝，'
            f'单段 {self.ramp:g} s × 2 段 ≈ {2 * self.ramp:g} s）')
        drive = (f'impedance kp=50/kd=2' if self.mode == 'imp'
                 else f'spd={self.spd}, cur={self.cur}')
        print('  两轴同动: ' +
              ', '.join(f'{name}({jid})' for jid, name, _ in self.AXES) +
              f' (mode={self.mode}, {drive}, 单段 {self.ramp:g} s)')
        targets = {jid: home[jid] + d for jid, _, d in self.AXES}
        print(f'  leg 站立位(home) ' +
              ', '.join(f'{jid}={home[jid]:+.3f}' for jid in home))
        print(f'  leg 目标(home+Δ) {targets} (mode={self.mode})')
        self._ramp(group, targets)     # 去程：两膝同帧平滑多屈 Δ 小角度
        for jid, name, _ in self.AXES:
            t = targets[jid]
            ok = self.wait_joint(self.GROUP, jid, t)
            reading = group.get_state(jid)
            detail = (f'target={t:.3f} rad, '
                      f'actual={reading.pos:.3f} rad' if reading else '无读数')
            self.check(f'leg 关节 {jid} ({name}) 到位', ok, detail)
        print('  leg 回到动作前站立位')
        self._ramp(group, home)        # 同帧回 home（非 0）
        for jid, name, _ in self.AXES:
            h = home[jid]
            self.check(f'leg 关节 {jid} ({name}) 回位',
                       self.wait_joint(self.GROUP, jid, h))

    def _ramp(self, group, goals: dict[int, float]) -> None:
        """20 Hz 线性插值：各轴从当前读数同时平滑走到 goal，总时长 ramp 秒。"""
        starts = {}
        for jid in goals:
            reading = group.get_state(jid)
            starts[jid] = reading.pos if reading is not None else 0.0
        n = max(1, int(round(self.ramp / self.DT)))
        for i in range(1, n + 1):
            frac = i / n
            self._command(group, {jid: starts[jid] + (goal - starts[jid]) * frac
                                  for jid, goal in goals.items()})
            self._frame(group)

    def _frame(self, group) -> None:
        """real/sim：同步等一帧 ~50ms；mock：手动推进模型（无时钟）。"""
        if self._node is not None:
            self.spin_for(self.DT)
        else:
            group.step(self.DT)

    def _command(self, group, targets) -> None:
        if self.mode == 'imp':
            group.impedance(targets)      # 全程阻抗（含回 home 段）
            return
        # pos 直发：提高 spd/cur，避免默认 cur=10 扭矩不足动作过小
        cmds = [JointCommand(joint_id=j, pos=p, spd=self.spd, cur=self.cur)
                for j, p in targets.items()]
        group.command(cmds, ControlMode.POSITION)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--mode', default='pos', choices=['pos', 'imp'],
                        help='pos=位置模式 / imp=阻抗模式')
    parser.add_argument('--ramp', type=float, default=3.0,
                        help='单段插值时长 s（到位/回 home 各一段，加大更平缓）')
    parser.add_argument('--spd', type=float, default=0.5,
                        help='pos 模式速度上限 rad/s')
    parser.add_argument('--cur', type=float, default=40.0,
                        help='pos 模式电流上限（默认 10 实测扭矩不足动作过小）')
    args = parser.parse_args(argv)
    return LegControlDemo(backend=args.backend, mode=args.mode,
                          ramp=args.ramp, spd=args.spd,
                          cur=args.cur).run()


if __name__ == '__main__':
    sys.exit(main())
