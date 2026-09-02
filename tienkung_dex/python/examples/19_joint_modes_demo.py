#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""19 关节控制模式全集（项目特有：ControlMode 0-5 + ZERO_CALIB 解锁）。

仅 mock：逐个模式发送低层 command()，验证 ZERO_CALIB 默认锁定、
unlock_calibration_mode() 后放行。真机上这些模式直接下发有硬件风险，
故拒绝 --backend real（真机模式测试请用 03~06 的 move_to/impedance）。

用法:
    python3 examples/19_joint_modes_demo.py
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class JointModesDemo(DemoBase):

    def __init__(self, backend='mock'):
        if backend != 'mock':
            print('本 demo 仅支持 mock 后端：低层模式直接下发对真机有'
                  '硬件风险（真机请用 03~06 的 move_to/impedance）。')
            sys.exit(2)
        super().__init__(backend, enable={'joint'})

    def _demo(self):
        from tienkung_dex import ControlMode, JointCommand, UnsafeModeError

        print('  ControlMode 枚举:')
        for mode in ControlMode:
            print(f'    {mode.value} = {mode.name}')

        cases = [
            (ControlMode.POSITION, 'pos=0.2'),
            (ControlMode.IMPEDANCE, 'kp=50 kd=2 pos=0.2'),
            (ControlMode.VELOCITY, 'spd=0.5'),
            (ControlMode.DISTANCE, 'pos=0.2'),
            (ControlMode.CURRENT, 'cur=5'),
        ]
        for mode, desc in cases:
            self.arm.command([JointCommand(joint_id=21, pos=0.2, spd=0.5,
                                           cur=5.0, kp=50.0, kd=2.0)], mode)
            self.check(f'command({mode.name}) 无异常', True, desc)

        try:
            self.arm.command([JointCommand(joint_id=21)],
                             ControlMode.ZERO_CALIB)
            self.check('ZERO_CALIB 默认锁定 (UnsafeModeError)', False,
                       '未被锁定！')
        except UnsafeModeError:
            self.check('ZERO_CALIB 默认锁定 (UnsafeModeError)', True)

        self.arm.unlock_calibration_mode()
        self.arm.command([JointCommand(joint_id=21)],
                         ControlMode.ZERO_CALIB)
        self.check('unlock_calibration_mode() 后 ZERO_CALIB 放行', True)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return JointModesDemo(backend=args.backend).run()


if __name__ == '__main__':
    sys.exit(main())
