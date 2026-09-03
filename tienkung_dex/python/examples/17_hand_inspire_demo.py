#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""17 因时（inspire）13 关节手控制。

mock:  内存桩为 6 电机模型（mock 工厂不区分厂商，set_positions 截前 6；
       force/speed 通道是空操作——对应检查显式跳过，不报假通过）。
real:  需先启动 inspire_hand 驱动（can0/can1）；angle/force/speed 三路指令
       + angle_actual 反馈 + SetClearError 清错服务。本厂商无手势预设
       （brainco 的 6 电机预设表到 13 关节无映射），手势测试见 16。
sim:   工厂拒绝 inspire（两指手模型），退出码 2。

用法:
    python3 examples/17_hand_inspire_demo.py                                # 默认半握 500
    python3 examples/17_hand_inspire_demo.py --pos 500 --force 300 --speed 500
    python3 examples/17_hand_inspire_demo.py --backend real --side left
    python3 examples/17_hand_inspire_demo.py --backend real --clear-error
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

JOINT_COUNT = 13                          # 因时 13 关节（0=伸直 .. 1000=弯曲）


class InspireHandDemo(DemoBase):

    def __init__(self, backend='real', side='right'):
        super().__init__(backend, enable={'hand'}, hand_vendor='inspire')
        self.side = side
        self._args: dict = {}     # main() 注入 CLI 参数（实例级，避免类级共享）

    def _demo(self):
        hand = getattr(self, f'hand_{self.side}')
        print(f'  手: {self.side} / vendor={hand.vendor}')

        pos = self._args.get('pos')
        if pos is None:
            pos = 500                        # 无参数默认半握
        force = self._args.get('force')
        speed = self._args.get('speed')

        hand.set_positions([pos] * JOINT_COUNT)
        self.spin_for(self._observe)         # real: 等 angle_actual 回读
        status = hand.get_status()
        ok = status is not None
        if ok and self.backend == 'real':
            ok = len(status.positions) == JOINT_COUNT   # 13 关节反读
        self.check(f'set_positions([{pos}]×{JOINT_COUNT})', ok,
                   f'positions={status.positions}' if status else '无状态')

        # force/speed：real 上真实发布（force/force_actual 话题可 echo 核对）；
        # mock 上是空操作——跳过检查，不报假通过。
        if force is not None:
            if self.backend == 'mock':
                print(f'  ⏭ set_force：mock 无 force 通道，跳过检查')
            else:
                hand.set_force([force] * JOINT_COUNT)
                self.check(f'set_force([{force}]×{JOINT_COUNT}) 指令已发布', True)
        if speed is not None:
            if self.backend == 'mock':
                print(f'  ⏭ set_speed：mock 无 speed 通道，跳过检查')
            else:
                hand.set_speed([speed] * JOINT_COUNT)
                self.check(f'set_speed([{speed}]×{JOINT_COUNT}) 指令已发布', True)

        if self._args.get('clear_error'):
            self.check('clear_error()', hand.clear_error())

        hand.set_positions([0] * JOINT_COUNT)   # 回直
        self.spin_for(self._observe)
        status = hand.get_status()
        self.check('回直 set_positions([0]×13)', status is not None,
                   f'positions={status.positions}' if status else '无状态')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--side', default='right', choices=['left', 'right'])
    parser.add_argument('--pos', type=int,
                        help='13 关节位置设定值（0=伸直 .. 1000=弯曲）')
    parser.add_argument('--force', type=int, help='所有关节力设定值')
    parser.add_argument('--speed', type=int, help='所有关节速度设定值')
    parser.add_argument('--clear-error', action='store_true',
                        help='调用 SetClearError 清错服务')
    args = parser.parse_args(argv)
    demo = InspireHandDemo(backend=args.backend, side=args.side)
    demo._args = vars(args)
    return demo.run()


if __name__ == '__main__':
    sys.exit(main())
