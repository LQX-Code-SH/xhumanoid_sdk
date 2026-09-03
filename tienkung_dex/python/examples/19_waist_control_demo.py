#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""19 腰部关节控制：三轴依次插值小角度动作 + 回零（pos / imp 两种模式）。

腰部三轴（config/joints.yaml）依次演示，每轴"平滑插值到位 → 判停 → 回零"。
关节方向语义（基准 = 机器人自身视角，2026-09-03 真机实测，与 joints.yaml 一致）:
    joint 31 = waist_pitch  限位 [-0.5236, 0.8727]（-30°~+50°） → +0.20 rad
                        正值 → 前倾（腹侧下弯），负值 → 后仰
    joint 32 = waist_roll   限位 [-0.4363, 0.4363]（±25° 侧倾）  → +0.15 rad
                        正值 → 向自身右侧倾，负值 → 左侧倾
    joint 33 = waist_yaw    限位 [-2.6180, 3.2289]（大幅转身）    → +0.25 rad
                        正值 → 上半身向自身左侧转，负值 → 右侧转

运动方式：单段目标（0→目标 / 目标→0）用 20 Hz 逐帧线性插值发布（--ramp 控制
单段时长，默认 3 s，加大则更平缓），不走一次到位的高速跳变。三轴共 6 段插值
+ 判停，总时长约 20 s。

驱动力说明（2026-09-03 真机实测）：pos 默认 move_to spd=0.3/cur=10 三轴全部
扭矩不足——pitch 目标 0.20 只到 0.121、roll 0.15 只到 0.070、yaw 0.25 几乎
不动（0.002，判 ❌）；负载越大越不够（yaw 要转整个上半身）。与 18 头部同症状。
因此 pos 模式直发 JointCommand(spd=0.5, cur=40)，可用 --spd/--cur 现场调整。

实测证据（2026-09-03 真机 19 记录）：+0.20 pitch → actual 0.177，目测躯干
前倾；+0.15 roll → actual 0.134，目测躯干向自身右侧倾；+0.25 yaw → actual
0.248，目测上半身向自身左侧转。三轴方向均与 joints.yaml 注释及限位分布自洽。

用法:
    python3 examples/19_waist_control_demo.py                        # real 默认
    python3 examples/19_waist_control_demo.py --mode imp             # 阻抗模式
    python3 examples/19_waist_control_demo.py --ramp 5               # 每段更慢
    python3 examples/19_waist_control_demo.py --cur 60               # 提高电流
    python3 examples/19_waist_control_demo.py --backend mock         # headless
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

from tienkung_dex.core.types import ControlMode, JointCommand


class WaistControlDemo(DemoBase):
    GROUP = 'waist'
    DT = 0.05                        # 20 Hz 插值帧
    # 三轴依次演示：(joint_id, 关节名, 相对站立位增量 Δ rad)
    # 目标 = 动作前读数 home + Δ（如 pitch 真机 ≈0.033+0.20≈0.23 rad）。
    AXES = ((31, 'waist_pitch', 0.20),
            (32, 'waist_roll', 0.15),
            (33, 'waist_yaw', 0.25))

    def __init__(self, backend='real', mode='pos',
                 ramp=3.0, spd=0.5, cur=40.0):
        super().__init__(backend, enable={'joint'})
        self.mode = mode
        self.ramp = ramp              # 单段（到位或回 home）插值时长 s
        self.spd = spd
        self.cur = cur

    def _demo(self):
        self._confirm_real_motion(
            'waist 关节运动指令（31/32/33 三轴依次相对站立位小角度动作，'
            f'单段 {self.ramp:g} s × 6 段 ≈ {6 * self.ramp:g} s）')
        drive = (f'impedance kp=50/kd=2' if self.mode == 'imp'
                 else f'spd={self.spd}, cur={self.cur}')
        print(f'  三轴依次: pitch({self.AXES[0][0]}) → '
              f'roll({self.AXES[1][0]}) → yaw({self.AXES[2][0]}) '
              f'(mode={self.mode}, {drive}, 单段 {self.ramp:g} s)')
        group = getattr(self, self.GROUP)
        for jid, name, delta in self.AXES:
            # home = 该轴动作前读数：真机站立非 0（如 31≈+0.033 rad）。
            reading = group.get_state(jid)
            home = reading.pos if reading is not None else 0.0
            target = home + delta
            print(f'  waist {name}({jid}) 站立位(home) {home:+.3f} rad, '
                  f'目标(home+Δ) {target:+.3f} rad (mode={self.mode})')
            self._ramp(group, jid, target)
            ok = self.wait_joint(self.GROUP, jid, target)
            reading = group.get_state(jid)
            detail = (f'target={target:.3f} rad, '
                      f'actual={reading.pos:.3f} rad' if reading else '无读数')
            self.check(f'waist 关节 {jid} ({name}) 到位', ok, detail)
            print(f'  waist {name}({jid}) 回到动作前站立位')
            self._ramp(group, jid, home)
            self.check(f'waist 关节 {jid} ({name}) 回位',
                       self.wait_joint(self.GROUP, jid, home))

    def _ramp(self, group, jid: int, goal: float) -> None:
        """20 Hz 线性插值：从当前读数平滑走到 goal，总时长 self.ramp 秒。"""
        reading = group.get_state(jid)
        start = reading.pos if reading is not None else 0.0
        n = max(1, int(round(self.ramp / self.DT)))
        for i in range(1, n + 1):
            value = start + (goal - start) * i / n
            self._command(group, {jid: value})
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
    return WaistControlDemo(backend=args.backend, mode=args.mode,
                            ramp=args.ramp, spd=args.spd,
                            cur=args.cur).run()


if __name__ == '__main__':
    sys.exit(main())
