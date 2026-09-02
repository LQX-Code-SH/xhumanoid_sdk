#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""07 激光雷达点云（livox PointCloud2 透传）。

mock:  inject 一个占位对象，验证回调与 latest()。
real:  spin 5 秒订阅 /livox/lidar。

用法:
    python3 examples/07_lidar_demo.py
    python3 examples/07_lidar_demo.py --backend real
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class LidarDemo(DemoBase):

    def __init__(self, backend='mock'):
        super().__init__(backend, enable={'lidar'})

    def _demo(self):
        clouds = []
        self.lidar.on_cloud(clouds.append)

        if self.backend == 'mock':
            self.lidar.inject(object())    # 点云消息对 SDK 是不透明的
        else:
            self.spin_for(self._observe)

        latest = self.lidar.latest()
        self.check('latest() 非空', latest is not None)
        self.check('on_cloud 回调', len(clouds) >= 1, f'{len(clouds)} 帧')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return LidarDemo(backend=args.backend).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
