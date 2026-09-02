#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""05 腰部关节控制（vendor demo 05 对应：pos / imp + 回零）。

用法:
    python3 examples/05_waist_control_demo.py                 # mock
    python3 examples/05_waist_control_demo.py --backend real --mode imp
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class WaistControlDemo(DemoBase):
    TARGETS = {31: 0.2}                   # 腰部，小角度
    GROUP = 'waist'

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
    return WaistControlDemo(backend=args.backend, mode=args.mode).run()


if __name__ == '__main__':
    sys.exit(main())
