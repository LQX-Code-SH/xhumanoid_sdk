#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""07 灵巧手控制（vendor demo 07 对应：brainco / inspire 双厂商）。

mock:  brainco 6 电机模型（mock 工厂不区分厂商）。
real:  --hand-vendor inspire 需先启动 inspire_hand 驱动（can0/can1）；
       brainco 为默认。

用法:
    python3 examples/07_hand_control_demo.py                        # 默认 set_gesture('ok')
    python3 examples/07_hand_control_demo.py --gesture rock
    python3 examples/07_hand_control_demo.py --pos 200 --tor 300 --spd 500
    python3 examples/07_hand_control_demo.py --backend real --hand-vendor inspire --pos 500
    python3 examples/07_hand_control_demo.py --clear-error          # inspire 清错服务
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

INSPIRE_JOINTS = 13                        # 因时手 13 关节；brainco 内部截 6


class HandControlDemo(DemoBase):

    def __init__(self, backend='mock', hand_vendor='brainco', side='right'):
        super().__init__(backend, enable={'hand'}, hand_vendor=hand_vendor)
        self.side = side

    def _demo(self):
        hand = getattr(self, f'hand_{self.side}')
        print(f'  手: {self.side} / vendor={hand.vendor}')

        if self._args.get('clear_error'):
            self.check('clear_error()', hand.clear_error())

        gesture = self._args.get('gesture')
        if gesture is None and not any(
                self._args.get(k) is not None
                for k in ('pos', 'tor', 'spd', 'clear_error')):
            gesture = 'ok'                 # 无参数默认手势
        if gesture is not None:
            self.check(f"set_gesture('{gesture}')",
                       hand.set_gesture(gesture))

        pos = self._args.get('pos')
        if pos is not None:
            hand.set_positions([pos] * INSPIRE_JOINTS)
            self.check(f'set_positions([{pos}]×{INSPIRE_JOINTS})', True)
        tor = self._args.get('tor')
        if tor is not None:
            hand.set_force([tor] * INSPIRE_JOINTS)
            self.check(f'set_force([{tor}]×{INSPIRE_JOINTS})', True)
        spd = self._args.get('spd')
        if spd is not None:
            hand.set_speed([spd] * INSPIRE_JOINTS)
            self.check(f'set_speed([{spd}]×{INSPIRE_JOINTS})', True)

        status = hand.get_status()
        self.check('get_status()', status is not None,
                   f'positions={status.positions}' if status else '无状态')

    # main() 把 CLI 参数放进这里（比一长串构造参数更贴近 vendor demo 语义）
    _args: dict = {}


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--hand-vendor', default='brainco',
                        choices=['brainco', 'inspire'])
    parser.add_argument('--side', default='right', choices=['left', 'right'])
    parser.add_argument('--gesture', choices=['ok', 'rock', 'scissors', 'paper'])
    parser.add_argument('--pos', type=int, help='所有关节位置设定值')
    parser.add_argument('--tor', type=int, help='所有关节力设定值')
    parser.add_argument('--spd', type=int, help='所有关节速度设定值')
    parser.add_argument('--clear-error', action='store_true',
                        help='调用清错服务（inspire）')
    args = parser.parse_args(argv)
    demo = HandControlDemo(backend=args.backend, hand_vendor=args.hand_vendor,
                           side=args.side)
    demo._args = vars(args)
    return demo.run()


if __name__ == '__main__':
    sys.exit(main())
