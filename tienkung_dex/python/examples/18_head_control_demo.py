#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""18 头部关节控制：大幅周期性摆头 / 点头（pos / imp 两种模式）。

头部关节 ID 语义（config/joints.yaml；方向基准 = 机器人自身视角，
即"左/右"以机器人本体为准而非观察者，2026-09-03 真机实测确认）:
    joint 1 = head_yaw   左右摆头，limits ±1.0472 rad（±60°）
                        正值 → 转向自身左侧，负值 → 自身右侧
                        （实测：+0.6 起摆首半圈先朝自身左侧）
    joint 2 = head_pitch 上下看头，limits [-0.2618, 1.0123]（-15°~+58°）
                        正值 → 低头（下压），负值 → 抬头（机械挡约 -15°）
                        （限位不对称 + 结构 + 实测三向印证）

默认演示（肉眼可见的大动作）:
    · 左右摆头 ±0.6 rad（≈±34°），正弦往返 3 圈；
    · 大幅点头：0 → 0.7 rad（≈40° 向下）→ 回 0，循环 3 次（只朝正向下探，
      永不触及 pitch 负向限位 -0.2618）；
    · 每圈 4 s（freq=0.25 Hz），单轴约 12 s，两轴共约 24 s 后回中。

驱动力说明：pos 模式直发 JointCommand(spd=0.5, cur=40)（而非 move_to 默认
spd=0.3/cur=10）。实测 cur=10 下 head 伺服扭矩不足：目标 0.3 rad 只爬到
0.22（读数有位移但视觉极小）；提高电流限幅后摆动才会肉眼可见。轨迹仍为
连续正弦/余弦平滑曲线，20 Hz 逐点发布，无大角度突变。real 需终端回车确认。

用法:
    python3 examples/18_head_control_demo.py                            # 真机默认
    python3 examples/18_head_control_demo.py --amp-yaw 0.8 --freq 0.2   # 更大更慢
    python3 examples/18_head_control_demo.py --backend mock             # headless
"""

import math
import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

from tienkung_dex.core.types import ControlMode, JointCommand

# 限位内安全幅度（joints.yaml）：yaw ±1.047、pitch +1.0123（负向不涉及）
_MAX_AMP = {'head_yaw': 0.95, 'head_pitch': 0.95}


class HeadControlDemo(DemoBase):
    GROUP = 'head'
    DT = 0.05                        # 20 Hz 插值点

    def __init__(self, backend='real', mode='pos',
                 amp_yaw=0.6, amp_pitch=0.7, freq=0.25, cycles=3.0,
                 spd=0.5, cur=40.0):
        super().__init__(backend, enable={'joint'})
        self.mode = mode
        self.amp_yaw = amp_yaw
        self.amp_pitch = amp_pitch    # 点头深度（0→+amp→0）
        self.freq = freq
        self.cycles = cycles
        self.spd = spd
        self.cur = cur

    def _demo(self):
        for name, amp in (('head_yaw', self.amp_yaw),
                          ('head_pitch', self.amp_pitch)):
            if amp > _MAX_AMP[name]:
                raise ValueError(f'{name} 幅度 {amp} rad 超过安全上限 '
                                 f'{_MAX_AMP[name]} rad（joints.yaml 限位）')
        if self.mode == 'pos':
            drive = f'spd={self.spd}, cur={self.cur}'
        else:
            drive = 'impedance kp=50/kd=2'
        self._confirm_real_motion('头部大幅摆头/点头（限位内，约 24 s）')
        print(f'  参数: yaw ±{self.amp_yaw} rad / pitch 低头深度 '
              f'{self.amp_pitch} rad, {self.freq} Hz, 各 {self.cycles:g} '
              f'圈 (mode={self.mode}, {drive})')
        # 分轴执行；另一轴每步钉在 0，避免漂移
        self._swing_axis(1, self.amp_yaw, '左右摆头', nod=False)
        self._swing_axis(2, self.amp_pitch, '大幅点头', nod=True)

    def _swing_axis(self, jid: int, amp: float, label: str,
                    nod: bool) -> None:
        """单轴周期轨迹：sin 对称摆动，或 (1-cos)/2 单向点头（0→amp→0）。"""
        group = getattr(self, self.GROUP)
        w = 2.0 * math.pi * self.freq
        n = int(self.cycles / self.freq / self.DT)
        lo, hi = float('inf'), float('-inf')
        for i in range(n):
            t = i * self.DT
            theta = w * t
            value = (amp * (1.0 - math.cos(theta)) / 2.0 if nod
                     else amp * math.sin(theta))
            targets = {1: 0.0, 2: 0.0}
            targets[jid] = value
            self._command(targets)
            if self._node is not None:      # real/sim：同步等一帧 ~50ms
                self.spin_for(self.DT)
            else:                           # mock：无时钟，手动推进模型
                group.step(self.DT)
            state = group.get_state(jid)
            if state is not None:
                lo = min(lo, state.pos)
                hi = max(hi, state.pos)
        ok = self.wait_joint(self.GROUP, jid, 0.0)
        span = (f'实测 min={lo:.3f}..max={hi:.3f} rad' if lo < hi
                else '无读数')
        self.check(f'{label} {self.cycles:g} 圈完成并回中', ok, span)

    def _command(self, targets) -> None:
        group = getattr(self, self.GROUP)
        if self.mode == 'imp':
            group.impedance(targets)
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
    parser.add_argument('--amp-yaw', type=float, default=0.6,
                        help='摆头幅度 rad（≤0.95）')
    parser.add_argument('--amp-pitch', type=float, default=0.7,
                        help='低头深度 rad，0→+amp→0（≤0.95，正向下压）')
    parser.add_argument('--freq', type=float, default=0.25,
                        help='摆动频率 Hz（默认每圈 4 s）')
    parser.add_argument('--cycles', type=float, default=3.0,
                        help='每轴循环次数（结束后回中）')
    parser.add_argument('--spd', type=float, default=0.5,
                        help='pos 模式速度上限 rad/s')
    parser.add_argument('--cur', type=float, default=40.0,
                        help='pos 模式电流上限（默认 10 扭矩不足动作过小）')
    args = parser.parse_args(argv)
    return HeadControlDemo(backend=args.backend, mode=args.mode,
                           amp_yaw=args.amp_yaw, amp_pitch=args.amp_pitch,
                           freq=args.freq, cycles=args.cycles,
                           spd=args.spd, cur=args.cur).run()


if __name__ == '__main__':
    sys.exit(main())
