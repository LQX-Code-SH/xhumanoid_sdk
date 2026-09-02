#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""04 IMU 姿态订阅（支持 xsens / livox 双源）。

mock:  inject 一条 ImuReading，验证 latest() / on_reading。
real:  imu_source 选择数据源，spin 5 秒计帧。

用法:
    python3 examples/04_imu_demo.py                             # mock
    python3 examples/04_imu_demo.py --backend real --source xsens
    python3 examples/04_imu_demo.py --backend real --source livox
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class ImuDemo(DemoBase):

    def __init__(self, backend='mock', source='xsens'):
        super().__init__(backend, enable={'imu'}, imu_source=source)

    def _demo(self):
        readings = []
        self.imu.on_reading(readings.append)

        if self.backend == 'mock':
            from tienkung_dex import ImuReading
            self.imu.inject(ImuReading(roll=0.1, pitch=-0.05, yaw=1.2,
                                       source='mock'))
        else:
            self.spin_for(self._observe)

        latest = self.imu.latest()
        self.check('latest() 非空', latest is not None)
        if latest is not None:
            self.check('姿态读数', True,
                       f'roll={latest.roll:.3f} pitch={latest.pitch:.3f} '
                       f'yaw={latest.yaw:.3f} source={latest.source}')
        self.check('on_reading 回调', len(readings) >= 1,
                   f'{len(readings)} 条')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--source', default='xsens',
                        choices=['xsens', 'livox'],
                        help='IMU 数据源（仅 real 后端生效）')
    args = parser.parse_args(argv)
    return ImuDemo(backend=args.backend, source=args.source).run(
        observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
