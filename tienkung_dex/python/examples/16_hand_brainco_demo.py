#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16 强脑（brainco）6 电机手控制。

mock:  6 电机内存桩（set_gesture / set_positions 均为真实操作）。
real:  默认厂商；SetMotorMulti 位置指令 + MotorStatus 反馈。
       本厂商无 force / speed / clear_error 通道（占位接口仅告警忽略），
       因时手的力/速度/清错测试见 17。

用法:
    python3 examples/16_hand_brainco_demo.py   # 默认双手依次执行剪刀→布→石头→布→OK→布，间隔 1s
    python3 examples/16_hand_brainco_demo.py --gesture rock
    python3 examples/16_hand_brainco_demo.py --pos 200
    python3 examples/16_hand_brainco_demo.py --side left --gesture ok   # 单侧
"""

# 无参数时默认依次演示的手势（每动作间隔 GESTURE_PAUSE 秒，便于观察）。
DEFAULT_GESTURES = ('scissors', 'paper', 'rock', 'paper', 'ok', 'paper')
GESTURE_PAUSE = 1.0

import sys
import time

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

from tienkung_dex.core.presets import GESTURE_POSITIONS

MOTOR_COUNT = 6                           # brainco 6 电机（1=伸直 .. 1000=弯曲）


class BraincoHandDemo(DemoBase):

    def __init__(self, backend='real', side='both'):
        super().__init__(backend, enable={'hand'}, hand_vendor='brainco')
        self.side = side
        self._args: dict = {}     # main() 注入 CLI 参数（实例级，避免类级共享）

    def _confirm_hand_motion(self, hand, goal, tol: float = 30.0,
                             timeout: float = 10.0) -> None:
        """等待位置插值收敛，并确认手真的发生了运动（real 后端）。

        真机从当前姿态到目标按速度做位置插值，需数百 ms~数秒（实测约
        0.6s 走 1000 单位），且机械限位/堵转可能让个别电机不完全到位。
        因此不按"瞬间等于目标"判定，而是：
          - 轮询直到每电机与目标偏差 ≤ tol；
          - 若到位趋势停滞（1.0s 内最佳偏差无改善）视为运动结束；
          结束时若手相对起点移动 ≥ 100（真的动了）即通过，并把实际/目标
          位置与偏差一并打印，供观察物理到位情况。
        """
        start = last = None
        best = None
        best_at = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            st = hand.get_status()
            if st is not None and len(st.positions) >= len(goal):
                cur = tuple(int(v) for v in st.positions[:len(goal)])
                if start is None:
                    start = cur
                last = cur
                d = max(abs(c - g) for c, g in zip(cur, goal))
                if best is None or d < best - 1.0:
                    best = d
                    best_at = time.monotonic()
                if d <= tol:
                    self.check('手按插值运动到位', True,
                               f'实际 {list(cur)}，目标 {list(goal)}')
                    return
                if best_at is not None and time.monotonic() - best_at > 1.0:
                    break                  # 到位趋势停滞（到位/堵转/限位）
            self.spin_for(0.1)
        if last is None or start is None:
            self.check('手发生运动', False, '未收到 MotorStatus 反馈')
            return
        moved = max(abs(a - b) for a, b in zip(last, start))
        delta = max(abs(c - g) for c, g in zip(last, goal))
        detail = (f'起点 {list(start)} → 终点 {list(last)}，'
                  f'移动量 {moved:.0f}，距目标 Δ{delta:.0f}'
                  f'（目标 {list(goal)}）')
        self.check('手发生运动', moved >= 100.0, detail)

    def _demo(self):
        sides = ('left', 'right') if self.side == 'both' else (self.side,)
        hands = {s: getattr(self, f'hand_{s}') for s in sides}
        print('  手: ' + ', '.join(f'{s}={h.vendor}'
                                   for s, h in hands.items()))

        gestures = self._args.get('gesture')
        pos = self._args.get('pos')
        if gestures is None and pos is None:
            gestures = DEFAULT_GESTURES       # 无参数默认依次演示全部手势
        if isinstance(gestures, str):
            gestures = (gestures,)

        if gestures:
            total = len(gestures)
            for i, g in enumerate(gestures):
                for s in sides:
                    h = hands[s]
                    print(f'  ── 手势 {i + 1}/{total}: {g}（{s}）──')
                    self.check(f"set_gesture('{g}', side={s})",
                               h.set_gesture(g))
                    if self.backend == 'real':
                        self._confirm_hand_motion(h, GESTURE_POSITIONS[g])
                if i < total - 1:
                    print(f'  （间隔 {GESTURE_PAUSE:.0f}s，观察动作…）')
                    self.spin_for(GESTURE_PAUSE)

        if pos is not None:
            for s in sides:
                h = hands[s]
                h.set_positions([pos] * MOTOR_COUNT)
                if self.backend == 'real':
                    self._confirm_hand_motion(h, (pos,) * MOTOR_COUNT)
            self.spin_for(self._observe)   # real: 等 MotorStatus 回读

        if self.backend == 'real':
            # rclpy 订阅建立是异步的：create_subscription 后要等 discovery
            # 完成才能收到 MotorStatus 首帧；刚发完指令立刻读缓存必然为空
            #（表现为手状态"无数据"）。先轮询等待首帧（真机 30Hz，正常数
            # 百 ms 内到位），超时再判失败。两侧同时等：只等目标侧会让对侧
            # discovery 在收尾健康快照时才姗姗来迟，误报 inactive（左右手
            # publisher 的匹配顺序是随机的）。
            pending = [h for h in hands.values() if h.get_status() is None]
            if pending:
                print('  （等待 MotorStatus 回读…）')
                deadline = time.monotonic() + 5.0
                while pending and time.monotonic() < deadline:
                    for h in tuple(pending):
                        if h.get_status() is not None:
                            pending.remove(h)
                    if pending:
                        self.spin_for(0.2)

        for s in sides:
            status = hands[s].get_status()
            ok = status is not None
            if ok and self.backend == 'real':
                ok = len(status.positions) == MOTOR_COUNT  # motor_status 反读
            self.check(f'get_status() 反读 ({s})', ok,
                       f'positions={status.positions}' if status else '无状态')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--side', default='both',
                        choices=['left', 'right', 'both'],
                        help='both(默认，双手)/right/left 单侧（左右逐一执行）')
    parser.add_argument('--gesture', choices=['ok', 'rock', 'scissors', 'paper'])
    parser.add_argument('--pos', type=int,
                        help='6 电机位置设定值（1=伸直 .. 1000=弯曲）')
    args = parser.parse_args(argv)
    demo = BraincoHandDemo(backend=args.backend, side=args.side)
    demo._args = vars(args)
    return demo.run()


if __name__ == '__main__':
    sys.exit(main())
