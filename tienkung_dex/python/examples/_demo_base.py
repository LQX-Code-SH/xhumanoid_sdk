#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DemoBase：所有 examples demo 的公共基类（继承 TienkungDex facade）。

设计要点（实施计划 §二）：
- mock 后端完全 headless（node=None，不导入 rclpy），任意开发机可跑；
  real/sim 后端才延迟导入 rclpy 并创建节点（SDK 依赖注入设计的要求）。
- 装配复用 create_robot()：子类只需传 backend 与 overrides（enable /
  hand_vendor / imu_source / topic overrides 全部透传）。
- run() 是模板方法：start → _demo() → 报告；退出码 0=全部检查通过，
  1=有失败项，130=用户中断（CI / test_demos.sh 友好）。
"""

from __future__ import annotations

import math
import os
import sys
import time

# 直接运行 `python3 examples/xx.py` 时脚本目录在 sys.path，包根不在——
# 补上，使 `import tienkung_dex` 在源码树任意位置可用。
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from tienkung_dex import TienkungDex, create_robot


class DemoBase(TienkungDex):
    """一个 demo = 一次完整生命周期（create → start → 检查 → shutdown）。"""

    def __init__(self, backend: str = 'mock', **overrides):
        self._rclpy = None
        self._node = None
        if backend != 'mock':
            import rclpy
            if not rclpy.ok():
                rclpy.init()
            self._rclpy = rclpy
            self._node = rclpy.create_node('tienkung_dex_example')
        robot = create_robot(self._node, backend=backend, **overrides)
        super().__init__(robot._subsystems, backend=backend,
                         logger=robot._log)
        self._exit_code = 0
        self._failures: list[str] = []
        self._observe = 5.0            # run() 注入，real/sim 分支自旋用
        self._health: dict[str, bool] = {}
        print(f'== {type(self).__name__} (backend={backend}) ==')

    # -- 模板方法 -----------------------------------------------------------
    def run(self, observe_seconds: float = 5.0) -> int:
        self._observe = observe_seconds
        try:
            self.start()
            self._demo()
            self._health = self.health()   # 关机前快照（mock on_stop 会清状态）
        except KeyboardInterrupt:
            print('\n[中断] 用户退出')
            self._exit_code = 130
        except Exception as exc:
            print(f'\n[错误] {type(exc).__name__}: {exc}')
            self._exit_code = 1
        finally:
            try:
                self.shutdown()
            finally:
                self._teardown_node()
        self._print_report()
        return self._exit_code

    def _demo(self):
        raise NotImplementedError

    # -- 子类工具 -----------------------------------------------------------
    def spin_for(self, seconds: float) -> None:
        """real/sim：spin 一段时间喂订阅回调；mock：sleep（无 ROS 图）。"""
        if seconds <= 0:
            return
        if self._node is not None:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                self._rclpy.spin_once(self._node, timeout_sec=0.05)
        else:
            time.sleep(seconds)

    def wait_joint(self, group_name: str, joint_id: int, target: float,
                   tol_deg: float = 5.0, timeout: float = 8.0) -> bool:
        """边 spin 边等关节到位（mock 立即收敛，无需 spin）。"""
        group = getattr(self, group_name)
        deadline = time.monotonic() + timeout
        tol = math.radians(tol_deg)
        while time.monotonic() < deadline:
            if self._node is not None:
                self._rclpy.spin_once(self._node, timeout_sec=0.05)
            elif hasattr(group, 'step'):       # headless mock：手动推进模型
                group.step(0.02)
            reading = group.get_state(joint_id)
            if reading is not None and abs(reading.pos - target) < tol:
                return True
            time.sleep(0.02)
        return False

    def joint_move_demo(self, group_name: str, targets: dict[int, float],
                        mode: str = 'pos') -> None:
        """03~06 共用流程：到位检查 → 回零（vendor demo 的 homing）。"""
        group = getattr(self, group_name)
        print(f'  {group_name} 目标 {targets} (mode={mode})')
        if mode == 'imp':
            group.impedance(targets)
        else:
            group.move_to(targets)
        for jid, target in sorted(targets.items()):
            ok = self.wait_joint(group_name, jid, target)
            reading = group.get_state(jid)
            detail = (f'target={target:.3f} rad, '
                      f'actual={reading.pos:.3f} rad' if reading else '无读数')
            self.check(f'{group_name} 关节 {jid} 到位', ok, detail)
        print(f'  {group_name} 回零 {sorted(targets)}')
        group.move_to({jid: 0.0 for jid in targets})
        for jid in sorted(targets):
            self.check(f'{group_name} 关节 {jid} 回零',
                       self.wait_joint(group_name, jid, 0.0))

    def check(self, name: str, ok: bool, detail: str = '') -> bool:
        mark = '✅' if ok else '❌'
        suffix = f'  ({detail})' if detail else ''
        print(f'  {mark} {name}{suffix}')
        if not ok:
            self._exit_code = 1
            self._failures.append(name)
        return ok

    # -- 内部 ---------------------------------------------------------------
    def _print_report(self) -> None:
        print(f'-- 子系统健康 (backend={self.backend}) --')
        health = self._health or self.health()
        for name, active in sorted(health.items()):
            print(f'  {"✅" if active else "❌"} {name:<28} active={active}')
        if self._failures:
            print(f'结果: 失败 {len(self._failures)} 项: '
                  f'{", ".join(self._failures)}')
        else:
            print('结果: 全部通过')

    def _teardown_node(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()
        self._rclpy = None
