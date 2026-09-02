#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""19 腿部关节控制。

⚠️  真机运行前必须将机器人固定在安全支架上——腿部失稳会直接摔倒。
    仅下发小角度目标并自动回零。

用法:
    python3 examples/19_leg_control_demo.py                   # mock
    python3 examples/19_leg_control_demo.py --backend real    # 真机（需支架）
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class LegControlDemo(DemoBase):
    TARGETS = {51: 0.15, 61: 0.15}        # 左/右髋，小角度
    GROUP = 'leg'

    def __init__(self, backend='mock', mode='pos'):
        super().__init__(backend, enable={'joint'})
        self.mode = mode

    def _demo(self):
        if self.backend != 'mock':
            print('  ⚠️  请确认机器人已固定在安全支架上（5 秒后开始）')
            import time
            time.sleep(5.0)
        self.joint_move_demo(self.GROUP, self.TARGETS, mode=self.mode)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--mode', default='pos', choices=['pos', 'imp'],
                        help='pos=位置模式 / imp=阻抗模式')
    args = parser.parse_args(argv)
    return LegControlDemo(backend=args.backend, mode=args.mode).run()


if __name__ == '__main__':
    sys.exit(main())
