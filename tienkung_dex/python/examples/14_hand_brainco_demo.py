#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""14 强脑（brainco）6 电机手控制。

mock:  6 电机内存桩（set_gesture / set_positions 均为真实操作）。
real:  默认厂商；SetMotorMulti 位置指令 + MotorStatus 反馈。
       本厂商无 force / speed / clear_error 通道（占位接口仅告警忽略），
       因时手的力/速度/清错测试见 15。

用法:
    python3 examples/14_hand_brainco_demo.py                     # 默认 set_gesture('ok')
    python3 examples/14_hand_brainco_demo.py --gesture rock
    python3 examples/14_hand_brainco_demo.py --pos 200
    python3 examples/14_hand_brainco_demo.py --backend real --side left
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

MOTOR_COUNT = 6                           # brainco 6 电机（1=伸直 .. 1000=弯曲）


class BraincoHandDemo(DemoBase):

    def __init__(self, backend='mock', side='right'):
        super().__init__(backend, enable={'hand'}, hand_vendor='brainco')
        self.side = side
        self._args: dict = {}     # main() 注入 CLI 参数（实例级，避免类级共享）

    def _demo(self):
        hand = getattr(self, f'hand_{self.side}')
        print(f'  手: {self.side} / vendor={hand.vendor}')

        gesture = self._args.get('gesture')
        pos = self._args.get('pos')
        if gesture is None and pos is None:
            gesture = 'ok'                 # 无参数默认手势

        if gesture is not None:
            self.check(f"set_gesture('{gesture}')",
                       hand.set_gesture(gesture))

        if pos is not None:
            hand.set_positions([pos] * MOTOR_COUNT)
            self.spin_for(self._observe)   # real: 等 MotorStatus 回读

        status = hand.get_status()
        ok = status is not None
        if ok and self.backend == 'real':
            ok = len(status.positions) == MOTOR_COUNT   # motor_status 反读
        self.check('get_status() 反读', ok,
                   f'positions={status.positions}' if status else '无状态')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--side', default='right', choices=['left', 'right'])
    parser.add_argument('--gesture', choices=['ok', 'rock', 'scissors', 'paper'])
    parser.add_argument('--pos', type=int,
                        help='6 电机位置设定值（1=伸直 .. 1000=弯曲）')
    args = parser.parse_args(argv)
    demo = BraincoHandDemo(backend=args.backend, side=args.side)
    demo._args = vars(args)
    return demo.run()


if __name__ == '__main__':
    sys.exit(main())
