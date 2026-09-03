#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""08 GPS 定位（可选硬件）。

mock:  inject 一条有效 GpsFixReading，验证 is_valid / 回调。
real:  spin 5 秒订阅 /gps/fix（无 GPS 硬件时 latest() 为 None 属预期）。

用法:
    python3 examples/08_gps_demo.py
    python3 examples/08_gps_demo.py --backend real
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class GpsDemo(DemoBase):

    def __init__(self, backend='real'):
        super().__init__(backend, enable={'gps'})

    def _demo(self):
        fixes = []
        self.gps.on_fix(fixes.append)

        if self.backend == 'mock':
            from tienkung_dex import GpsFixReading
            self.gps.inject(GpsFixReading(latitude=39.9, longitude=116.4,
                                          altitude=50.0, status=2, num_sats=12))
        else:
            self.spin_for(self._observe)

        latest = self.gps.latest()
        self.check('latest() 非空', latest is not None)
        if latest is not None:
            self.check('定位有效 (is_valid)', latest.is_valid,
                       f'lat={latest.latitude:.5f} lon={latest.longitude:.5f} '
                       f'sats={latest.num_sats}')
        self.check('on_fix 回调', len(fixes) >= 1, f'{len(fixes)} 条')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return GpsDemo(backend=args.backend).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
