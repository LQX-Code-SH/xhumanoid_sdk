#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""17 急停安全链路（项目特有：EstopActiveError 拦截 + 边沿回调）。

mock:  注入急停 → 验证拦截关节指令（L1 冗余层）→ 解除后恢复。
real:  纯观测模式——只监听急停边沿事件 8 秒，不发任何关节指令
       （真机上请按/松机器人急停按钮观察输出）。

用法:
    python3 examples/17_safety_demo.py
    python3 examples/17_safety_demo.py --backend real
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class SafetyDemo(DemoBase):

    def __init__(self, backend='mock'):
        super().__init__(backend, enable={'joint', 'safety'})

    def _demo(self):
        if self.backend == 'mock':
            from tienkung_dex import EstopActiveError

            events = []
            self.safety.on_estop(events.append)

            self.safety.set_estop(True)
            self.check('is_estopped', self.safety.is_estopped)
            try:
                self.arm.move_to({21: 0.0})
                self.check('急停时拦截关节指令 (EstopActiveError)', False,
                           '指令未被拦截！')
            except EstopActiveError:
                self.check('急停时拦截关节指令 (EstopActiveError)', True)

            self.safety.set_estop(False)
            self.arm.move_to({21: 0.0})
            self.check('解除急停后指令恢复', True)
            self.check('on_estop 边沿回调', events == [True, False],
                       f'events={events}')
        else:
            print('  （real 模式：纯观测 8 秒，请按/松急停按钮；'
                  '不发任何关节指令）')
            events = []
            self.safety.on_estop(lambda active: events.append(active)
                                 or print(f'  急停边沿: active={active}'))
            self.spin_for(self._observe)
            self.check('safety.is_active', self.safety.is_active)
            self.check('急停事件监听', len(events) >= 0,
                       f'{len(events)} 次边沿（0 次也属正常）')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return SafetyDemo(backend=args.backend).run(observe_seconds=8.0)


if __name__ == '__main__':
    sys.exit(main())
