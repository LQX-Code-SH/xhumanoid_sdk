#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""03 关节状态订阅。

mock:  move_to 产生状态快照，验证 get_state/get_states/on_state。
real:  spin 5 秒统计 arm/head/waist/leg 四组的 /robot_state 快照。
两类后端在检查后都会按关节名称打印最近一帧快照（pos/vel）。

用法:
    python3 examples/03_robot_state.py                    # mock（默认）
    python3 examples/03_robot_state.py --backend real     # 真机
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class RobotStateDemo(DemoBase):

    def __init__(self, backend='real'):
        super().__init__(backend, enable={'joint'})
        try:                    # joints.yaml: group -> id -> name（可按名称打印）
            from tienkung_dex.core.joints import load_joints_table
            self._joint_table = load_joints_table()
        except Exception:
            self._joint_table = None

    def _joint_label(self, group: str, joint_id: int) -> str:
        """关节名称（缺失映射时回退为数字 ID）。"""
        meta = None
        if self._joint_table is not None and not self._joint_table.is_empty:
            meta = self._joint_table.meta(group, joint_id)
        if meta is not None and meta.name:
            return f'{meta.name} (id {joint_id})'
        return f'joint {joint_id}'

    def _show_snapshot(self, group: str, states: dict) -> None:
        """按关节名称打印最近一帧快照（pos/vel）。"""
        if not states:
            return
        print(f'  -- {group} 关节快照（最近一帧，按名称）--')
        for jid in sorted(states):
            st = states[jid]
            pos = getattr(st, 'pos', 0.0)
            vel = getattr(st, 'vel', 0.0)
            print(f'  {self._joint_label(group, jid):<28} '
                  f'pos={pos:+9.4f} rad  vel={vel:+9.4f} rad/s')

    def _demo(self):
        counts = {group: 0 for group in ('arm', 'head', 'waist', 'leg')}

        def make_cb(group):
            def cb(_states):
                counts[group] += 1
            return cb

        for group in counts:
            getattr(self, group).on_state(make_cb(group))

        if self.backend == 'mock':
            self.arm.move_to({21: -0.5})
            states = self.arm.get_states()
            self._show_snapshot('arm', states)
            self.check('arm.get_states() 含关节 21', 21 in states,
                       f'{sorted(states)} 关节')
            reading = self.arm.get_state(21)
            self.check('arm.get_state(21) 到位',
                       reading is not None and abs(reading.pos + 0.5) < 1e-6,
                       f'pos={reading.pos:.3f}' if reading else '无读数')
            self.check('on_state 回调触发', counts['arm'] >= 1,
                       f'{counts["arm"]} 次')
        else:
            self.spin_for(self._observe)
            for group in counts:
                states = getattr(self, group).get_states()
                self._show_snapshot(group, states)
                self.check(f'{group} 状态快照', bool(states),
                           f'{len(states)} 关节 / {counts[group]} 次回调')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return RobotStateDemo(backend=args.backend).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
