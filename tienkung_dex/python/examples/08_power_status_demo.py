#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""08 电源状态（vendor demo 08 对应：电压/电流/功率/急停键）。

mock:  inject 一条 PowerReading，验证回调与 latest()。
real:  spin 5 秒订阅 /power/battery|board/status。

用法:
    python3 examples/08_power_status_demo.py
    python3 examples/08_power_status_demo.py --backend real
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class PowerStatusDemo(DemoBase):

    def __init__(self, backend='mock'):
        super().__init__(backend, enable={'power'})

    def _demo(self):
        updates = []
        self.power.on_update(updates.append)

        if self.backend == 'mock':
            from tienkung_dex import PowerReading
            self.power.inject(PowerReading(
                voltage=48.0, current=2.0, power_w=96.0,
                is_estop=False, is_power_on=True))
        else:
            self.spin_for(self._observe)

        latest = self.power.latest()
        self.check('latest() 非空', latest is not None)
        if latest is not None:
            self.check('电池读数', True,
                       f'{latest.voltage:.1f} V / {latest.current:.2f} A / '
                       f'{latest.power_w:.1f} W / estop={latest.is_estop} / '
                       f'power_on={latest.is_power_on}')
        self.check('on_update 回调', len(updates) >= 1,
                   f'{len(updates)} 次')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return PowerStatusDemo(backend=args.backend).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
