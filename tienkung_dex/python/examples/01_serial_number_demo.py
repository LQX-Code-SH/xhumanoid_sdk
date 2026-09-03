#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""01 序列号查询（/xsys/get_serial_number 服务）。

mock:  返回固定 'MOCK-SN-0000'。
real:  调用服务返回机器人序列号。

用法:
    python3 examples/01_serial_number_demo.py
    python3 examples/01_serial_number_demo.py --backend real
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class SerialNumberDemo(DemoBase):

    def __init__(self, backend='real'):
        super().__init__(backend, enable={'serial'})

    def _demo(self):
        serial = self.serial.get_serial_number()
        self.check('get_serial_number() 返回非空', bool(serial),
                   f'serial={serial!r}')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return SerialNumberDemo(backend=args.backend).run()


if __name__ == '__main__':
    sys.exit(main())
