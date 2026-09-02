#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""13 灯带控制（xsys 灯光指令）。

mock:  set_mode/set_cmd 并核对 MockLightControl.commands 记录。
real:  发布灯带指令（观察机器人头部/胸部灯带变化），check is_active。

用法:
    python3 examples/13_light_demo.py
    python3 examples/13_light_demo.py --mode wakeup --cmd 301
    python3 examples/13_light_demo.py --backend real --mode listening
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

MODES = ('off', 'battery_normal', 'battery_low', 'battery_critical',
         'charging', 'wakeup', 'listening', 'thinking', 'running')


class LightDemo(DemoBase):

    def __init__(self, backend='mock', mode='battery_normal', cmd=None):
        super().__init__(backend, enable={'light'})
        self.mode = mode
        self.cmd = cmd

    def _demo(self):
        from tienkung_dex.core import topics as t

        ok = self.light.set_mode(self.mode)
        self.check(f"set_mode('{self.mode}')", ok,
                   f'cmd={t.LIGHT_CMDS.get(self.mode)}')
        if self.cmd is not None:
            self.light.set_cmd(self.cmd)
            self.check(f'set_cmd({self.cmd})', True)

        if self.backend == 'mock':
            recorded = self.light.commands
            expected = (t.LIGHT_CMDS[self.mode], ())
            self.check('指令已记录', recorded[:1] == [expected],
                       f'{recorded}')
        else:
            self.check('灯带发布者就绪 (is_active)', self.light.is_active)
            print('  （观察机器人灯带变化 3 秒）')
            self.spin_for(3.0)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--mode', default='battery_normal', choices=MODES)
    parser.add_argument('--cmd', type=int,
                        help='原始 cmd 值（可选，覆盖 mode 之后发送）')
    args = parser.parse_args(argv)
    return LightDemo(backend=args.backend, mode=args.mode,
                     cmd=args.cmd).run()


if __name__ == '__main__':
    sys.exit(main())
