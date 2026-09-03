#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""02 电源状态（电压/电流/功率/急停键）。

mock:  inject 一条 PowerReading，验证回调与 latest()。
real:  spin 5 秒订阅 /power/battery|board/status（battery ~1Hz）与
       /power/board/key_status（~12Hz）：急停键状态随 key_status
       高频刷新，电池电压/电流/功率随 battery 话题 ~1Hz 更新。

用法:
    python3 examples/02_power_status_demo.py
    python3 examples/02_power_status_demo.py --backend real
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class PowerStatusDemo(DemoBase):

    def __init__(self, backend='real'):
        super().__init__(backend, enable={'power'})

    def _demo(self):
        # on_update 的驱动源不止 battery(~1Hz)：power 子系统还订阅
        # key_status(~12Hz)，急停状态帧到达时同样会回调。因此"回调次数"
        # 与"电池读数更新次数"并不相同。这里分开统计：updates=回调总数，
        # battery_ticks=电压/电流值实际变化的电池读数刻度（值去抖）。
        updates = []
        battery_ticks = []
        prev = None

        def _collect(reading):
            nonlocal prev
            updates.append(reading)
            if prev is None or (reading.voltage, reading.current) != (
                    prev.voltage, prev.current):
                battery_ticks.append(reading)
            prev = reading

        self.power.on_update(_collect)

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
            # voltage>0 作为读数有效判据：若为 0 通常表示消息字段名与
            # master_battery_* 不匹配（解析命中了默认值），此时应 ❌ 并
            # 给出提示，而不是静默通过、让人工去盯读数行。
            ok = latest.voltage > 0.0
            detail = (f'{latest.voltage:.1f} V / {latest.current:.2f} A / '
                      f'{latest.power_w:.1f} W / estop={latest.is_estop} / '
                      f'power_on={latest.is_power_on}')
            self.check('电池读数', ok, detail if ok else
                       detail + '；voltage=0，请核对 master_battery_voltage 字段名')
        self.check('on_update 回调', len(updates) >= 1,
                   f'{len(updates)} 次（含 key_status ~12Hz 刷新）')
        self.check('电池值刷新（值去抖）', len(battery_ticks) >= 1,
                   f'{len(battery_ticks)} 次（battery 话题 ~1Hz）')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return PowerStatusDemo(backend=args.backend).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
