#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""09 相机图像（4 路 RGB-D + 可选 6 路全景）。

mock:  对每路相机 publish_frame ×2，验证 latest() / frame_rate()。
real:  spin 5 秒统计各路帧数；--panorama 附加 6 路全景相机。

用法:
    python3 examples/09_camera_demo.py
    python3 examples/09_camera_demo.py --panorama
    python3 examples/09_camera_demo.py --backend real --panorama
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class CameraDemo(DemoBase):

    def __init__(self, backend='mock', panorama=False):
        enable = {'camera', 'panorama'} if panorama else {'camera'}
        super().__init__(backend, enable=enable)

    def _demo(self):
        import numpy as np
        from tienkung_dex import CameraFrame

        def make_frame(ns):
            return CameraFrame(color=np.zeros((4, 4, 3), dtype=np.uint8),
                               depth=None, frame_id=ns)

        if self.backend == 'mock':
            for ns, camera in self.cameras.items():
                for _ in range(2):        # 两帧才能算出 frame_rate
                    camera.publish_frame(make_frame(ns))
                frame = camera.latest()
                ok = frame is not None and frame.frame_id == ns
                self.check(f'{ns} latest()', ok)
                self.check(f'{ns} frame_rate()', camera.frame_rate is not None,
                           f'{camera.frame_rate:.1f} fps'
                           if camera.frame_rate else '')
            if self.panorama is not None:
                for idx in self.panorama.indices:
                    self.panorama.publish_frame(idx, make_frame(f'cam{idx}'))
                    self.check(f'panorama[{idx}] latest()',
                               self.panorama.latest(idx) is not None)
        else:
            self.spin_for(self._observe)
            for ns, camera in self.cameras.items():
                frame = camera.latest()
                rate = camera.frame_rate
                self.check(f'{ns} 收到图像', frame is not None,
                           f'{rate:.1f} fps' if rate else '无帧')
            if self.panorama is not None:
                for idx in self.panorama.indices:
                    self.check(f'panorama[{idx}] latest()',
                               self.panorama.latest(idx) is not None)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--panorama', action='store_true',
                        help='附加 6 路全景相机（可选硬件）')
    args = parser.parse_args(argv)
    return CameraDemo(backend=args.backend,
                      panorama=args.panorama).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
