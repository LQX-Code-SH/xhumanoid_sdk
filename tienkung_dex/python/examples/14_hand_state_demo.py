#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""14 双手状态与触觉（get_status + on_touch，双厂商共用）。

参考 03_robot_state：观察窗口后按关节名称打印左右手最近一帧快照。
- brainco: MotorStatus positions[6]（1=伸直 .. 1000=弯曲）；
  TouchStatus 每指五元组（法向/切向力、切向方向、接近、状态）。
- inspire: angle_actual joint_values[13]；TouchData 布局未文档化
  （仅透传数值）。

mock:  对左右手注入 touch/positions，核对回调与状态快照。
real:  spin 5 秒订阅左右手状态/触觉话题，逐手打印快照。

用法:
    python3 examples/14_hand_state_demo.py
    python3 examples/14_hand_state_demo.py --backend real --hand-vendor inspire
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

# brainco 6 电机的关节顺序与量纲（1 = 完全伸直 .. 1000 = 完全弯曲）
BRAINCO_JOINTS = ('thumb_flex', 'thumb_rot', 'index', 'middle', 'ring', 'pinky')
SIDES = ('left', 'right')


class HandStateDemo(DemoBase):

    def __init__(self, backend='real', hand_vendor='brainco'):
        super().__init__(backend, enable={'hand'}, hand_vendor=hand_vendor)

    @staticmethod
    def _joint_label(vendor: str, i: int) -> str:
        """brainco 用电机名；inspire 关节布局未文档化，回退 joint {i}。"""
        if 'brainco' in vendor and i < len(BRAINCO_JOINTS):
            return BRAINCO_JOINTS[i]
        return f'joint {i}'

    def _show_snapshot(self, side: str, st) -> None:
        """参考 03_robot_state：按关节名称打印最近一帧快照。"""
        hand = getattr(self, f'hand_{side}')
        if st is None:
            print(f'  -- hand_{side}（{hand.vendor}）手部快照：'
                  '未收到状态 --')
            return
        print(f'  -- hand_{side}（{hand.vendor}）手部快照'
              f'（最近一帧，按关节名称）--')
        for i, v in enumerate(st.positions):
            label = self._joint_label(hand.vendor, i)
            print(f'  {label:<14} {v:>5}')
        if len(st.positions) > len(BRAINCO_JOINTS):
            print(f'  共 {len(st.positions)} 个关节')

    def _demo(self):
        counts = {side: 0 for side in SIDES}

        def make_cb(side):
            def cb(_reading):
                counts[side] += 1
            return cb

        for side in SIDES:
            getattr(self, f'hand_{side}').on_touch(make_cb(side))

        if self.backend == 'mock':
            from tienkung_dex import TouchReading
            for side in SIDES:
                hand = getattr(self, f'hand_{side}')
                hand.inject_touch(TouchReading(values=((100, 200),)))
                hand.set_positions([500] * 13)   # brainco 内部截前 6
        else:
            print(f'  （{self.backend} 模式：观察 {self._observe:.0f} 秒'
                  '左右手状态/触觉话题…）')
            self.spin_for(self._observe)

        for side in SIDES:
            hand = getattr(self, f'hand_{side}')
            st = hand.get_status()
            self._show_snapshot(side, st)
            self.check(f'hand_{side} get_status()', st is not None,
                       f'positions={st.positions}' if st else '无状态')
            self.check(f'hand_{side} on_touch 回调', counts[side] >= 1,
                       f'{counts[side]} 条')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--hand-vendor', default='brainco',
                        choices=['brainco', 'inspire'])
    args = parser.parse_args(argv)
    return HandStateDemo(backend=args.backend,
                         hand_vendor=args.hand_vendor).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
