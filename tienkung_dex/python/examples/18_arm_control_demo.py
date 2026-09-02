#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""18 手臂关节控制（左/右臂 + 回零 homing）。

用法:
    python3 examples/18_arm_control_demo.py --mode imp        # mock
    python3 examples/18_arm_control_demo.py --backend real    # 真机
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class ArmControlDemo(DemoBase):
    TARGETS = {21: -0.5, 27: 0.3}         # 右肩/右腕，小角度
    GROUP = 'arm'

    def __init__(self, backend='mock', mode='pos'):
        super().__init__(backend, enable={'joint'})
        self.mode = mode

    def _demo(self):
        self.joint_move_demo(self.GROUP, self.TARGETS, mode=self.mode)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--mode', default='pos', choices=['pos', 'imp'],
                        help='pos=位置模式 / imp=阻抗模式')
    args = parser.parse_args(argv)
    return ArmControlDemo(backend=args.backend, mode=args.mode).run()


if __name__ == '__main__':
    sys.exit(main())
