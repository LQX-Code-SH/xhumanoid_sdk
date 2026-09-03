#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""06 SBUS 遥控器（摇杆轴 + A-H 按键 + key_event 事件）读数与活动报告。

真机 12ch 通道定标（real，2026-09-02 逐件实测；axis 序号与字母无对应，勿想当然）：

    摇杆轴    axis[0]=右杆X   右推 +1 / 左推 −1     axis[1]=右杆Y   上推 −1 / 下推 +1
              axis[2]=左杆Y   上推 +1 / 下推 −1     axis[3]=左杆X   右推 +1 / 左推 −1
              左右杆 Y 极性镜像（同一方向拨动符号相反），作速度合成需符号对齐。
    拨杆档位  axis[4]=E档  axis[5]=G档  axis[6]=H档  axis[7]=F档
              （轴序交错：4=E, 5=G, 6=H, 7=F，勿按字母对号入座）
              三态档位镜像 −1/0/+1，与按键电平同源；
              个别拨杆模拟抽头接触偶发（F 首测未检出、复测即现），以活动窗口为准。
    按键伴生  axis[8]=A伴生  axis[9]=B伴生  axis[10]=C伴生  axis[11]=D伴生
              按键按下时模拟镜像 −1/+1 摆动；SDK 走 buttons + key_event，无需消费。

按键两路信号（与 bodyctrl_msgs/SbusData 一致）：
    buttons(A-H)   档位电平：-1 松/复位, 0 中位, 1 一端, 2 另一端；
                   两态键 A-D 一般只出现 -1 / 1。
    key_event      边沿事件：event_new=变化后档位 / event_old=变化前档位，
                   键码 1..20（0=NONE，SbusReading.key_name() 读作如 G_RIGHT）：
                     A-D 两态   1:A_UP   2:A_DOWN   3:B_UP   4:B_DOWN
                                5:C_UP   6:C_DOWN   7:D_UP   8:D_DOWN
                     E-F 三档   9:E_UP  10:E_MID  11:E_DOWN  12:F_UP
                               13:F_MID 14:F_DOWN
                     G-H 左右  15:G_LEFT 16:G_MID 17:G_RIGHT
                               18:H_LEFT 19:H_MID 20:H_RIGHT

mock:  inject 一条 SbusReading（axes + buttons + key_event）。
real:  spin 5 秒订阅 /sbus_data 与 /sbus_data/event。

用法:
    python3 examples/06_sbus_demo.py
    python3 examples/06_sbus_demo.py --backend real
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


# 真机通道定标名（见模块 docstring；轴序与字母序交错，勿想当然）。
AXIS_NAME = {
    0: '右杆X', 1: '右杆Y', 2: '左杆Y', 3: '左杆X',
    4: 'E档', 5: 'G档', 6: 'H档', 7: 'F档',
    8: 'A伴生', 9: 'B伴生', 10: 'C伴生', 11: 'D伴生',
}


def _axis_label(i):
    return AXIS_NAME.get(i, f'axis[{i}]')


class SbusDemo(DemoBase):

    def __init__(self, backend='real'):
        super().__init__(backend, enable={'sbus'})

    def _demo(self):
        from tienkung_dex import SbusReading
        updates = []
        self.sbus.on_update(updates.append)

        if self.backend == 'mock':
            # 两帧轨迹演示（左摇杆摆动 + A 松开 / G 拨到右的事件链），
            # 末尾活动报告据此展示"变化过的轴/键/事件"。
            self.sbus.inject(SbusReading(axes=(0.5, -0.2, 0.0, 0.0),
                                         buttons=(1, 0, 0, 0, 0, 0, 1, 1),
                                         event_new=17,   # G_RIGHT
                                         event_old=16))  # 上一事件 G_MID
            self.sbus.inject(SbusReading(axes=(-1.0, 0.3, 0.0, 0.0),
                                         buttons=(-1, 0, 0, 0, 0, 0, 1, 1),
                                         event_new=1,    # A 松开
                                         event_old=17))
        else:
            self.spin_for(self._observe)

        latest = self.sbus.latest()
        self.check('latest() 非空', latest is not None)
        if latest is not None:
            new, old = latest.event_new, latest.event_old
            axis_vals = '  '.join(
                f'{_axis_label(i)}={v:+.2f}'
                for i, v in enumerate(latest.axes))
            self.check(
                '遥控器读数', True,
                f'axes({len(latest.axes)}ch)  {axis_vals}\n'
                f'          buttons(A-H)={latest.buttons}\n'
                f'          key_event: new={new} {SbusReading.key_name(new)}'
                f'   old={old} {SbusReading.key_name(old)}')
        self.check('on_update 回调', len(updates) >= 1, f'{len(updates)} 次')
        self._report_activity(updates)

    @staticmethod
    def _report_activity(readings):
        """采样窗口内有变化的轴/按键/事件（定标用，一次拨动全部即可）。"""
        if len(readings) < 2:
            return
        act_axes, act_btn, events = [], [], set()
        n_ax = len(readings[0].axes)
        for i in range(n_ax):
            vals = [r.axes[i] for r in readings if i < len(r.axes)]
            if max(vals) - min(vals) > 0.01:
                act_axes.append(f'{_axis_label(i)} {min(vals):+.2f}..{max(vals):+.2f}')
        for i, letter in enumerate('ABCDEFGH'):
            vals = [r.buttons[i] for r in readings if i < len(r.buttons)]
            if len(set(vals)) > 1:
                act_btn.append(f'{letter}={sorted(set(vals))}')
        events = sorted({r.event_new for r in readings if r.event_new})
        if not (act_axes or act_btn or events):
            return
        lines = ['-- 遥控器活动（采样窗口）--']
        if act_axes:
            lines.append('  活动轴: ' + '  '.join(act_axes))
        if act_btn:
            lines.append('  活动键: ' + '  '.join(act_btn))
        if events:
            from tienkung_dex import SbusReading
            lines.append('  key_event: ' + ' '.join(
                f'{SbusReading.key_name(c)}({c})' for c in events))
        print('\n'.join(lines))


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    args = parser.parse_args(argv)
    return SbusDemo(backend=args.backend).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
