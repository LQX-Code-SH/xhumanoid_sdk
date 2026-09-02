#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""09 SBUS 遥控器（vendor demo 09 对应：Joy 摇杆轴 + A-F 按键事件）。

mock:  inject 一条 SbusReading（axes + buttons）。
real:  spin 5 秒订阅 /sbus_data 与 /sbus_data/event。

用法:
    python3 examples/09_sbus_demo.py
    python3 examples/09_sbus_demo.py --backend real
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class SbusDemo(DemoBase):

    def __init__(self, backend='mock'):
        super().__init__(backend, enable={'sbus'})

    def _demo(self):
        updates = []
        self.sbus.on_update(updates.append)

        if self.backend == 'mock':
            from tienkung_dex import SbusReading
            self.sbus.inject(SbusReading(axes=(0.5, -0.2, 0.0, 0.0),
                                         buttons=(1, 0, 0, 0, 0, 0)))
        else:
            self.spin_for(self._observe)

        latest = self.sbus.latest()
        self.check('latest() 非空', latest is not None)
        if latest is not None:
            self.check('遥控器读数', True,
                       f'axes={latest.axes} buttons(A-F)={latest.buttons}')
        self.check('on_update 回调', len(updates) >= 1, f'{len(updates)} 次')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return SbusDemo(backend=args.backend).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
