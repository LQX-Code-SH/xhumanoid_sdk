#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""12 灵巧手状态与触觉（get_status + on_touch，双厂商共用）。

get_status / on_touch 是两厂商的真实接口交集，故共用一个脚本：
- brainco: MotorStatus positions[6]；TouchStatus 每指五元组
  （法向/切向力、切向方向、接近、状态）。
- inspire: angle_actual joint_values[13]；TouchData 布局未文档化
  （仅透传数值）。

mock:  inject_touch 验证 on_touch 回调；set_positions 后核对 get_status。
real:  spin 5 秒订阅手部状态话题。

用法:
    python3 examples/12_hand_state_demo.py
    python3 examples/12_hand_state_demo.py --backend real --hand-vendor inspire
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class HandStateDemo(DemoBase):

    def __init__(self, backend='mock', hand_vendor='brainco', side='right'):
        super().__init__(backend, enable={'hand'}, hand_vendor=hand_vendor)
        self.side = side

    def _demo(self):
        hand = getattr(self, f'hand_{self.side}')
        print(f'  手: {self.side} / vendor={hand.vendor}')

        touches = []
        hand.on_touch(touches.append)

        if self.backend == 'mock':
            from tienkung_dex import TouchReading
            hand.inject_touch(TouchReading(values=((100, 200),)))
            hand.set_positions([500] * 13)   # brainco 内部截前 6
        else:
            self.spin_for(self._observe)

        status = hand.get_status()
        self.check('get_status()', status is not None,
                   f'positions={status.positions}' if status else '无状态')
        self.check('on_touch 回调', len(touches) >= 1, f'{len(touches)} 条')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--hand-vendor', default='brainco',
                        choices=['brainco', 'inspire'])
    parser.add_argument('--side', default='right', choices=['left', 'right'])
    args = parser.parse_args(argv)
    return HandStateDemo(backend=args.backend, hand_vendor=args.hand_vendor,
                         side=args.side).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
