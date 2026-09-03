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

    def __init__(self, backend: str = 'real', **overrides):
        self._rclpy = None
        self._node = None
        self._ex = None
        if backend != 'mock':
            import rclpy
            if not rclpy.ok():
                rclpy.init()
            self._rclpy = rclpy
            self._node = rclpy.create_node('tienkung_dex_example')
            # 单一持久 executor：spin_for / wait_joint / SDK 底层 blocking
            # 调用（audio.speak / stop_playback 等 service 等待）全部复用它。
            # 反复 rclpy.spin_once() 会临时绑定/解绑 node，实测会使节点
            # 上随后的 service 响应收不到（11 demo stop_playback 超时）。
            from rclpy.executors import SingleThreadedExecutor
            self._ex = SingleThreadedExecutor()
            self._ex.add_node(self._node)
            # 暴露给 SDK：backends/real/_utils 等 getattr(node, 'executor')
            # 找到它后直接复用，避免另建 executor 再 add_node。
            self._node.executor = self._ex
        robot = create_robot(self._node, backend=backend, **overrides)
        super().__init__(robot._subsystems, backend=backend,
                         logger=robot._log)
        self._exit_code = 0
        self._failures: list[str] = []
        self._observe = 5.0            # run() 注入，real/sim 分支自旋用
        self._health: dict[str, bool] = {}
        # Health 快照前的"稳定观察窗"。瞬时探针类 demo（如 01 serial）从建
        # 订阅到采集快照往往只有几十 ms，会随机撞进 staleness 空窗而误报
        # inactive（real 真机 /power/board/key_status ~12Hz=83ms/帧，volatile
        # QoS 不补发）。快照前 spin 超过 stale_timeout(0.5s)+一帧余量，让
        # 数据流先开口再问活性。mock 无 ROS 图，该窗自动跳过（零开销）。
        self._health_settle = 0.7
        print(f'== {type(self).__name__} (backend={backend}) ==')

    # -- 模板方法 -----------------------------------------------------------
    def run(self, observe_seconds: float = 5.0) -> int:
        self._observe = observe_seconds
        try:
            self.start()
            self._demo()
            self._settle_health()   # real/sim: 喂数据后再采健康快照
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
                if self._ex is not None:
                    self._ex.spin_once(timeout_sec=0.05)
                else:
                    self._rclpy.spin_once(self._node, timeout_sec=0.05)
        else:
            time.sleep(seconds)

    def _settle_health(self) -> None:
        """健康快照前让数据流开口（修复瞬时探针误报 inactive）。

        仅 real/sim 需要真实 spin 等首帧；mock 无数据源概念且 is_active
        恒定，直接跳过，避免拖慢 headless 回归。
        """
        if self._node is not None and self._health_settle > 0:
            self.spin_for(self._health_settle)

    def wait_joint(self, group_name: str, joint_id: int, target: float,
                   tol_deg: float = 5.0, timeout: float = 8.0) -> bool:
        """边 spin 边等关节到位（mock 立即收敛，无需 spin）。"""
        group = getattr(self, group_name)
        deadline = time.monotonic() + timeout
        tol = math.radians(tol_deg)
        while time.monotonic() < deadline:
            if self._node is not None:
                if self._ex is not None:
                    self._ex.spin_once(timeout_sec=0.05)
                else:
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
        """18~21 共用流程：到位检查 → 回零（homing）。"""
        self._confirm_real_motion(f'{group_name} 关节运动指令')
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

    def _confirm_real_motion(self, what: str) -> None:
        """真机下发运动指令前的交互安全确认。

        - mock / sim：直接放行（仿真无硬件风险，不打断自动化验证）。
        - real + stdin 有 TTY：打印风险提示并要求回车确认，
          Ctrl-C（或 EOF）取消。
        - real + 非交互调用（AI / CI / 管道重定向，无 TTY）：直接拒绝
          并以退出码 2 终止——真机运动必须由人在终端确认后触发，
          从机制上杜绝无人值守/自动脚本意外下发。

        run() 不捕获 SystemExit：拒绝时先执行 shutdown 清理，再以
        退出码 2 退出，不打印健康报告。
        """
        if self.backend != 'real':
            return
        print(f'\n  ⚠ 即将向真机下发：{what}')
        print('  请确认机器人处于安全状态（支架/急停可用），'
              '人员与障碍物已远离。')
        if not sys.stdin.isatty():
            print('  [拒绝] 检测到非交互调用（stdin 无 TTY）。真机运动必须'
                  '由人在终端回车确认，AI/脚本/CI 无法自动触发。')
            print('  如需无机器人验证流程，请改用 --backend mock 或 sim。')
            raise SystemExit(2)
        try:
            input('  回车 = 确认执行；Ctrl-C = 取消: ')
        except EOFError:
            print('\n  [拒绝] 未收到确认输入（EOF）。')
            raise SystemExit(2)

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
            if active:
                print(f'  ✅ {name:<28} active={active}')
            else:
                # An inactive row must say *why*, otherwise a functional demo
                # whose target subsystem passed is misread as failing. On the
                # headless mock backend an undriven subsystem has no data
                # source (is_active = staleness window), which is not a fault.
                if self.backend == 'mock':
                    hint = 'mock: 本 demo 未驱动/无数据源'
                else:
                    hint = '数据超过时限/无数据'
                print(f'  ❌ {name:<28} active={active}  ({hint})')
        if self._failures:
            print(f'结果: 失败 {len(self._failures)} 项: '
                  f'{", ".join(self._failures)}')
        elif self._exit_code == 1:
            # Exceptions bypass check(); a report saying "all passed" while
            # the run errored is actively misleading.
            print('结果: 运行出错（见上方 [错误]，退出码 1）')
        else:
            print('结果: 全部通过')

    def _teardown_node(self) -> None:
        if self._ex is not None:
            self._ex.shutdown()
            self._ex = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()
        self._rclpy = None
