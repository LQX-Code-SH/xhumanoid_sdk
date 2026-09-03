#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""20 手臂关节控制：左右肘相对站立位平滑屈肘 + 回初始位（pos / imp 两种模式）。

手臂关节 ID 语义（config/joints.yaml；站立 home 参考见该文件底部 home_standing
模块）:
    joint 14 = left_elbow_pitch   左肘俯仰（屈伸），limits [-2.5482, 0.1920]
    joint 24 = right_elbow_pitch  右肘俯仰（屈伸），limits [-2.5482, 0.1920]
    home 基准：真机站立肘读数并非 0（03_robot_state 实测 14≈-0.496、
    24≈-0.361 rad，双臂自然下垂本就微曲），故先把动作前读数记为 home，
    目标 = home + Δ（Δ = -0.4 rad），结束后回 home，不做"回 0"。
    ⚠ 方向语义（负值=屈肘还是伸肘？）尚未真机实测——arm 段仍待微动核对
    （joints.yaml 顶部"方向验证进度"）。注意限位高度不对称：负向约 -146°
    为主行程、正向仅 +11°，负向大行程方向很可能即"屈肘"机械位，待实测印证。
    真机跑时请目测 14/24 位移方向并记录，作为 arm 段方向定标的第一手证据。

演示动作：先自旋等待 /robot_state 首帧入库（demo 启动时订阅回调尚未经
executor spin 执行、读数缓存为空，此刻 get_state 返回 None；若静默回退 0
作 home，真机会表现为"先突回 0、以 0 为基准动作、最后停 0"），再取两肘
真实读数为 home → 两轴同帧 20 Hz 线性插值走到 home+Δ
（真机目标 ≈-0.90/-0.76 rad，即相对站立位再向负向屈 ~23°，约为负限位
-2.5482 的 30~35%，不触边界）→ 判停 → 同帧插值回 home。单段插值时长
--ramp 控制（默认 3 s，加大更平缓），不走一次到位的高速跳变。总时长约
2 段 × 3 s + 判停 ≈ 10 s。

驱动力说明（沿用 18/19 真机结论）：move_to 默认 spd=0.3/cur=10 在头/腰实测
扭矩不足、动作过小甚至判 ❌；本 demo pos 直发 JointCommand(spd=0.5, cur=40)，
可用 --spd/--cur 现场调整。imp 模式全程阻抗（含回 home），不中途切回 pos。

用法:
    python3 examples/20_arm_control_demo.py                       # real 默认
    python3 examples/20_arm_control_demo.py --mode imp            # 阻抗模式
    python3 examples/20_arm_control_demo.py --ramp 5              # 每段更慢
    python3 examples/20_arm_control_demo.py --cur 60              # 提高电流
    python3 examples/20_arm_control_demo.py --backend mock        # headless
"""

import sys
import time

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

from tienkung_dex.core.types import ControlMode, JointCommand


class ArmControlDemo(DemoBase):
    GROUP = 'arm'
    DT = 0.05                        # 20 Hz 插值帧
    # 左右肘两轴：(joint_id, 关节名, 相对站立位增量 Δ rad)
    # Δ=-0.4：真机站立肘 14≈-0.496/24≈-0.361（非 0），目标 = 动作前读数 + Δ
    # （≈-0.90/-0.76，更负=更屈）。限位仅负向行程充足（约 -146°），正向仅
    # +0.192 rad（~11°）——真机动作默认向负向屈肘，正值方向太小不做。
    AXES = ((14, 'left_elbow_pitch', -0.4),
            (24, 'right_elbow_pitch', -0.4))

    def __init__(self, backend='real', mode='pos',
                 ramp=3.0, spd=0.5, cur=40.0):
        super().__init__(backend, enable={'joint'})
        self.mode = mode
        self.ramp = ramp              # 单段（到位或回 home）插值时长 s
        self.spd = spd
        self.cur = cur

    def _demo(self):
        group = getattr(self, self.GROUP)
        # home = 动作前真实读数（真机站立肘 14≈-0.496/24≈-0.361 rad，非 0）。
        # demo 刚启动时 /robot_state 首帧可能尚未入库（订阅回调须经 executor
        # spin 才执行），此刻 get_state 返回 None——静默回退 0.0 会把整个流程
        # 基准变成 0，真机表现为"先突回 0 → 以 0 为基准动作 → 最后停 0"。
        # 因此先自旋等到两轴首个真实读数再开跑；拿不到就中断，绝不发 0 基准。
        home = self._read_home(group)
        if home is None:
            self.check('读取 arm 动作前站立位(home)', False,
                       '/robot_state 首帧等待超时，拒绝以 0 为基准下发')
            return
        targets = {jid: home[jid] + d for jid, _, d in self.AXES}
        print(f'  arm 站立位(home) ' +
              ', '.join(f'{jid}={home[jid]:+.3f}' for jid in home))
        print(f'  arm 目标(home+Δ) {targets} (mode={self.mode})')
        self._confirm_real_motion(
            'arm 关节运动指令（左右肘 14/24 两轴同动相对站立位屈肘，'
            f'单段 {self.ramp:g} s × 2 段 ≈ {2 * self.ramp:g} s）')
        drive = (f'impedance kp=50/kd=2' if self.mode == 'imp'
                 else f'spd={self.spd}, cur={self.cur}')
        print('  两轴同动: ' +
              ', '.join(f'{name}({jid})' for jid, name, _ in self.AXES) +
              f' (mode={self.mode}, {drive}, 单段 {self.ramp:g} s)')
        # 预热锁定：真机对"接管关节 / 首条 POSITION 指令"存在一次性突变的
        # 现场表现（肘对抗重力最明显，表现 = 启动动作的瞬间弹跳/沉降）。正式
        # ramp 前先以 pos=home（==当前站立位，无位移需求）连续伺服 1 s，让
        # 驱动完成接管并稳定在当前位。锁定前后读数若已偏移，说明真机未锁定，
        # 后续动作将从该基准继续（现场需要确认）。
        self._prime(group, home)
        self._ramp(group, targets)     # 去程：两轴同帧平滑走到目标
        for jid, name, _ in self.AXES:
            t = targets[jid]
            ok = self.wait_joint(self.GROUP, jid, t)
            reading = group.get_state(jid)
            detail = (f'target={t:.3f} rad, '
                      f'actual={reading.pos:.3f} rad' if reading else '无读数')
            self.check(f'arm 关节 {jid} ({name}) 到位', ok, detail)
        print('  arm 回到动作前站立位')
        self._ramp(group, home)        # 同帧回 home（非 0）
        for jid, name, _ in self.AXES:
            h = home[jid]
            self.check(f'arm 关节 {jid} ({name}) 回位',
                       self.wait_joint(self.GROUP, jid, h))

    def _read_home(self, group, timeout: float = 3.0):
        """返回本 demo 两轴的动作前读数（真机真实站立位，非 0 基准）。

        real/sim：订阅建好后回调只随 executor spin 执行，demo 刚启动时
        /robot_state 首帧常未入库，get_state 返回 None——若回退 0 会把整个
        流程基准变成 0（真机表现为"先突回 0 → 以 0 为基准动作 → 最后停 0"）。
        故自旋等到 14/24 都有真实读数（~12 Hz，几十 ms 内到齐），超时返回
        None 让 _demo 中断，绝不发 0 基准。
        mock：无 /robot_state 时序概念，模型从零位起步、首次命令才建读数，
        直接以 0 为站立位（headless 只验命令通路，与原 fallback 行为一致）。
        """
        if self.backend == 'mock':
            return {jid: 0.0 for jid, _, _ in self.AXES}
        deadline = time.monotonic() + timeout
        home: dict[int, float] = {}
        while time.monotonic() < deadline:
            for jid, _, _ in self.AXES:
                if jid not in home:
                    reading = group.get_state(jid)
                    if reading is not None:
                        home[jid] = reading.pos
            if len(home) == len(self.AXES):
                return home
            if self._node is not None:
                self.spin_for(0.05)        # real/sim：喂回调收首帧
            else:
                group.step(0.02)           # mock：无时钟，手动推进模型
        return None

    def _prime(self, group, home: dict[int, float],
               seconds: float = 1.0) -> None:
        """去程前预热锁定：以 pos=home 连续伺服 seconds 秒（pos/imp 一致）。

        目标恒等于当前站立位（无位移需求），用于规避真机驱动"接管关节/首条
        POSITION 指令"瞬间的一次性突变——若控制端接管是平滑的应保持不动，
        1 s 后读数仍回 home 才进入正式 ramp。锁定前/后读数打印供现场核对。
        """
        before = group.get_state(next(iter(home)))
        n = max(1, int(round(seconds / self.DT)))
        for _ in range(n):
            if self.mode == 'imp':
                group.impedance(home)
            else:
                cmds = [JointCommand(joint_id=j, pos=p, spd=self.spd,
                                     cur=self.cur)
                        for j, p in home.items()]
                group.command(cmds, ControlMode.POSITION)
            self._frame(group)
        after = group.get_state(next(iter(home)))
        pre = f'{before.pos:.3f}' if before is not None else '无读数'
        post = f'{after.pos:.3f}' if after is not None else '无读数'
        print(f'  锁定保持 @ home 14={pre} -> 14={post} rad '
              f'({seconds:g} s, 期望稳定无位移)')
        del before, after

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
    return ArmControlDemo(backend=args.backend, mode=args.mode,
                          ramp=args.ramp, spd=args.spd,
                          cur=args.cur).run()


if __name__ == '__main__':
    sys.exit(main())
