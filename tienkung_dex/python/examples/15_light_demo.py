#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""15 灯带控制（xsys 灯光指令）。

mock:  set_mode/set_cmd 并核对 MockLightControl.commands 记录。
real:  发布灯带指令（观察机器人头部/胸部灯带变化），check is_active。

默认 = 跑马灯测试：灯带只有命名灯效、无像素/位置回读，所以"跑马灯"
在这里 = 把各档颜色灯效依次轮流点亮（位置与顺序无约束），持续
CYCLE_SECONDS 秒后自动恢复为原状灯态（RESTORE_MODE = battery_normal），
期间肉眼逐档确认颜色正常即可顺带排查坏灯/颜色错乱。

用法:
    python3 examples/15_light_demo.py                     # 跑马灯轮播 10s 后恢复
    python3 examples/15_light_demo.py --cycle 5           # 轮播 5 秒后恢复
    python3 examples/15_light_demo.py --mode wakeup --cmd 301   # 单发（不轮播、不恢复）
    python3 examples/15_light_demo.py --backend real --mode listening
"""

import sys
import time

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase

# -- LightCtrl.cmd 完整命令码表 -------------------------------------------
# 消息发布到 /xsys/light/ctrl，cmd 是整型"灯效定义 ID"（不是 RGB/像素），
# 控制器收到后翻译成固定颜色/动画。★ = SDK 命名别名（见 core/topics.py
# LIGHT_CMDS，共 9 个，set_mode 只认这些名字）。
# 系统:
#     0   关机中(★off)        10  故障慢闪         11  故障消除
#     12  告警                13  告警消除         14  故障快闪
#     20  服务等待中          21  服务启动中       22  服务就绪
#     23  服务启动失败        99  系统待机
# OTA:
#     100 升级退出            101 升级开始
# 电源:
#     200 电量状态退出        201 放电·电量充足(★battery_normal)
#     202 放电·低电量(★battery_low)     203 放电·电量危急(★battery_critical)
#     210 充电中(★charging)  211 充电·已充满     220 备份电池放电中
# 交互:
#     300 对话结束·休眠      301 对话唤醒(★wakeup)    310 拾音中(★listening)
#     311 推理中(★thinking)  312 TTS 合成中      313 应答播报中
#     320 断网               321 网络恢复
# 运动:
#     400 运动退出           401 奔跑中(★running)
# 无 ★ 的码（如 211 充满、312/313 播报、320/321 网络、99 待机等）只能走
# --cmd 直接发送（跳过查表）；其余未列出整数亦可强发，真机自行忽略/处理。
# 权威定义（含 CMD_* 常量名）见真机安装目录:
#     /home/nvidia/xos/bodyctrl_msgs/share/bodyctrl_msgs/msg/LightCtrl.msg

MODES = ('off', 'battery_normal', 'battery_low', 'battery_critical',
         'charging', 'wakeup', 'listening', 'thinking', 'running')

# 跑马灯遍历的灯效/颜色序列（跳过 off，避免测试中途灭灯）。
# 目的只是把每档颜色点亮一次供肉眼观察，顺序与"灯珠位置"无约束。
CYCLE_MODES = ('battery_normal', 'battery_low', 'battery_critical',
               'charging', 'wakeup', 'listening', 'thinking', 'running')
CYCLE_SECONDS = 10.0          # 跑马灯总时长（秒）
CYCLE_INTERVAL = 0.8          # 相邻灯效切换间隔（秒）
RESTORE_MODE = 'battery_normal'   # 测试结束恢复的灯态（协议无回读，"原状"
                                  # 按约定取机器人正常态，即 cmd=201）


class LightDemo(DemoBase):

    def __init__(self, backend='real', mode=None, cmd=None,
                 cycle_seconds=CYCLE_SECONDS):
        super().__init__(backend, enable={'light'})
        self.mode = mode
        self.cmd = cmd
        self.cycle_seconds = cycle_seconds

    def _demo(self):
        from tienkung_dex.core import topics as t

        if not self.light.is_active:
            self.check('灯带发布者就绪 (is_active)', False,
                       '消息包缺失/未启动，无法下发')
            return
        self.check('灯带发布者就绪 (is_active)', True)

        if self.mode is None and self.cmd is None:
            self._cycle_demo(t)          # 默认：跑马灯轮播 + 恢复原状
        else:
            self._single_demo(t)         # 显式 --mode/--cmd：单发指令

    # -- 跑马灯测试 -------------------------------------------------------
    def _cycle_demo(self, topics):
        """按 CYCLE_MODES 顺序循环点亮各灯效共 cycle_seconds，结束后恢复。

        mock 无 ROS 图：以 0 间隔回放同一命令序列，纯校验记录结构
        （覆盖全部灯效 + 末条为恢复指令），不拖慢 headless 回归。
        """
        interval = CYCLE_INTERVAL if self._node is not None else 0.0
        steps = max(1, int(self.cycle_seconds / CYCLE_INTERVAL))
        restore_cmd = topics.LIGHT_CMDS[RESTORE_MODE]
        print(f'  （跑马灯测试：遍历 {len(CYCLE_MODES)} 种灯效，'
              f'{self.cycle_seconds:g}s / {steps} 次切换，结束后恢复 '
              f'{RESTORE_MODE} cmd={restore_cmd}）')

        deadline = time.monotonic() + self.cycle_seconds
        for k in range(steps):
            mode = CYCLE_MODES[k % len(CYCLE_MODES)]
            self.light.set_mode(mode)      # 表内模式，必然返回 True
            self.spin_for(interval)
        self.light.set_cmd(restore_cmd)    # 恢复原状灯态

        if self.backend == 'mock':
            cmds = [c for c, _ in self.light.commands]
            sent = set(cmds[:-1])
            covered = {topics.LIGHT_CMDS[m] for m in CYCLE_MODES} <= sent
            self.check(f'轮播覆盖全部 {len(CYCLE_MODES)} 种灯效', covered,
                       f'共 {len(cmds) - 1} 条 / 去重 {sorted(sent)}')
            self.check('结束恢复原状灯态', bool(cmds) and cmds[-1] == restore_cmd,
                       f'{RESTORE_MODE} cmd={restore_cmd}')
        else:
            print('  （观察灯带逐档变色；结束后自动恢复原状灯态…）')
            left = deadline - time.monotonic()
            if left > 0:
                self.spin_for(left)

    # -- 单发指令（保留原行为） ------------------------------------------
    def _single_demo(self, topics):
        if self.mode is not None:
            ok = self.light.set_mode(self.mode)
            self.check(f"set_mode('{self.mode}')", ok,
                       f'cmd={topics.LIGHT_CMDS.get(self.mode)}')
        if self.cmd is not None:
            self.light.set_cmd(self.cmd)
            self.check(f'set_cmd({self.cmd})', True)

        if self.backend == 'mock':
            recorded = self.light.commands
            expected_cmd = (topics.LIGHT_CMDS[self.mode]
                            if self.mode is not None else self.cmd)
            self.check('指令已记录', bool(recorded) and
                       recorded[0] == (expected_cmd, ()), f'{recorded}')
        else:
            print('  （观察机器人灯带变化 3 秒）')
            self.spin_for(3.0)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--mode', default=None, choices=MODES,
                        help='命名灯效（默认 None：走跑马灯轮播；给出则单发）')
    parser.add_argument('--cmd', type=int,
                        help='原始 cmd 值（可发表外值；与 --mode 同时给出则'
                             '先 set_mode 再 set_cmd 两条都发）')
    parser.add_argument('--cycle', type=float, default=CYCLE_SECONDS,
                        help=f'跑马灯总时长秒数（默认 {CYCLE_SECONDS:g}，'
                             '仅未指定 --mode/--cmd 时生效）')
    args = parser.parse_args(argv)
    return LightDemo(backend=args.backend, mode=args.mode, cmd=args.cmd,
                     cycle_seconds=args.cycle).run()


if __name__ == '__main__':
    sys.exit(main())
