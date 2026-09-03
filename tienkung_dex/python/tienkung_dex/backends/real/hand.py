#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real dexterous hand (brainco, 6 motors) + optional touch (design §4.3).

SetMotorMulti fields (verified against the gesture demo):
    mode, positions[6], speeds[6], currents[6], pwms[6], durations[6]
Position scale: 1 = fully straight .. 1000 = fully bent.
TouchStatus: data[] items with normal_force1 (0.01 N), tangential_force1,
tangential_direction1 (65535 = N/A), self_proximity1, status.
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Sequence

from tienkung_dex.core.base import DexterousHandBase
from tienkung_dex.core.types import HandStatus, TouchReading

from . import _msgs

# Gesture preset table lives in core (shared with sim/mock); re-exported
# here for existing importers.
from tienkung_dex.core.presets import GESTURE_POSITIONS  # noqa: E402

MOTOR_COUNT = 6
POS_MIN, POS_MAX = 1, 1000

# 部分手势在真机上的分段执行序列（避免关节机械干涉）。
# 'rock' 若让拇指 flex 与四指同时朝掌心弯曲会发生干涉（实测食指/中指被
# 卡在 ~630/600）；四指全收后拇指 flex 还会被食指/中指干涉，上限约 ~690
# （仅该干涉状态成立：无干涉时如 scissors，thumb flex 仍可达 1000，见
# core presets 注），preset 已校准为 685 这一可达形位。故拆两段执行
# （终态=preset）：
#   1) thumb flex 伸直、rotate 至最终外展位、四指收拢到 1000；
#   2) 四指保持，再弯拇指至 preset 终态。
# 阶段间轮询 MotorStatus 等待到位/停滞，避免相位叠加（见 _run_sequence）。
_GESTURE_SEQUENCES = {
    'rock': (
        (POS_MIN, 700, 1000, 1000, 1000, 1000),   # 拇指外展 + 四指弯曲
        GESTURE_POSITIONS['rock'],                # 再弯拇指（rock 可达终态）
    ),
}


def _wait_pub_matched(node, log, label: str, pub, timeout: float = 3.0) -> bool:
    """Publish 前等待至少一个匹配的订阅者（Fast-DDS discovery 滞后）。

    短命节点 create_publisher 后立即 publish 时，首帧常在 discovery 完成前
    发出而被系统端静默丢弃（实测：16 demo set_gesture('ok') 返回成功但手不
    动；同一消息用常驻 1Hz 重发，~1.7s discovery 稳定后手正常执行）。判断
    依据是发布端实际匹配数（get_subscription_count），订阅侧收到首帧不代
    表本 publisher 已就绪。边 spin 边轮询：复用 node.executor（若已挂载，
    见 DemoBase），否则 rclpy.spin_once。
    """
    if pub is None:
        return False
    import rclpy
    executor = getattr(node, 'executor', None)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if pub.get_subscription_count() > 0:
                return True
        except Exception:
            pass
        try:
            if executor is not None:
                executor.spin_once(timeout_sec=0.05)
            else:
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            time.sleep(0.05)
    if log is not None:
        log.warn(f'{label}: {timeout:.0f}s 内未发现匹配订阅者'
                 '（系统端 hand 节点未运行/被占用？），仍尝试发送')
    return False


class RealDexterousHand(DexterousHandBase):
    """Brainco 6-motor hand via set_motor_multi / motor_status."""

    def __init__(self, node, side: str, topics: dict, logger,
                 control_mode: int = 1):
        super().__init__(node, side, vendor='brainco')
        self._topics = topics
        self._log = logger
        self._control_mode = control_mode
        self._pub = None
        self._sub = None
        self._touch_sub = None
        self._msg_cls = None
        self._status_cls = None
        self._touch_cls = None
        self._status = None
        self._touch_cbs: list[Callable[[TouchReading], None]] = []

    def on_start(self) -> None:
        self._msg_cls, self._status_cls, _touch_cls = (None, None, None)
        module = None
        try:
            import importlib
            module = importlib.import_module('brainco_hand_msgs.msg')
        except Exception as exc:
            raise RuntimeError(
                f'{self.name}: brainco_hand_msgs not importable ({exc})')
        self._msg_cls = getattr(module, 'SetMotorMulti')
        self._status_cls = getattr(module, 'MotorStatus')
        self._touch_cls = getattr(module, 'TouchStatus', None)

        self._pub = self._node.create_publisher(
            self._msg_cls, self._topics['cmd'], 10)
        self._sub = self._node.create_subscription(
            self._status_cls, self._topics['status'],
            self._on_status, 10)
        if self._touch_cls is not None:
            self._touch_sub = self._node.create_subscription(
                self._touch_cls, self._topics['touch'],
                self._on_touch, 10)
        if self._log is not None:
            self._log.info(
                f'{self.name} (brainco): pub {self._topics["cmd"]}, '
                f'sub {self._topics["status"]} (+touch)')

    def on_stop(self) -> None:
        self._pub = None
        self._sub = None
        self._touch_sub = None
        self._status = None

    @property
    def is_active(self) -> bool:
        return self._status is not None

    def _on_status(self, msg) -> None:
        positions = tuple(int(getattr(msg, 'positions', ())[i]) if i < len(
            getattr(msg, 'positions', ())) else 0 for i in range(MOTOR_COUNT))
        self._status = HandStatus(positions=positions, raw=msg)

    def _on_touch(self, msg) -> None:
        items = []
        for item in getattr(msg, 'data', ()) or ():
            items.append((
                int(getattr(item, 'normal_force1', 0)),
                int(getattr(item, 'tangential_force1', 0)),
                int(getattr(item, 'tangential_direction1', 65535)),
                int(getattr(item, 'self_proximity1', 0)),
                int(getattr(item, 'status', 0)),
            ))
        reading = TouchReading(values=tuple(items))
        for cb in tuple(self._touch_cbs):
            cb(reading)

    # -- control ----------------------------------------------------------
    def _publish(self, positions, speeds=None, currents=None) -> None:
        if self._pub is None:
            raise RuntimeError(f'{self.name} not started')
        _wait_pub_matched(self._node, self._log, self.name, self._pub)
        msg = self._msg_cls()
        msg.mode = self._control_mode
        positions = tuple(positions)[:MOTOR_COUNT]
        speeds = tuple(speeds or ())[:MOTOR_COUNT] or (0,) * MOTOR_COUNT
        currents = tuple(currents or ())[:MOTOR_COUNT] or (0,) * MOTOR_COUNT
        for i in range(MOTOR_COUNT):
            msg.positions[i] = int(positions[i]) if i < len(positions) else POS_MIN
            msg.speeds[i] = int(speeds[i]) if i < len(speeds) else 0
            msg.currents[i] = int(currents[i]) if i < len(currents) else 0
            msg.pwms[i] = 0
            msg.durations[i] = 0
        self._pub.publish(msg)

    def set_positions(self, positions: Sequence[int]) -> None:
        clipped = tuple(max(POS_MIN, min(POS_MAX, int(p))) for p in positions)
        self._publish(clipped)

    # -- sequenced control --------------------------------------------------
    def _spin_once(self, timeout_sec: float = 0.05) -> None:
        """复用 node.executor（DemoBase 已挂载）spin，否则 rclpy.spin_once。"""
        import rclpy
        executor = getattr(self._node, 'executor', None)
        try:
            if executor is not None:
                executor.spin_once(timeout_sec=timeout_sec)
            else:
                rclpy.spin_once(self._node, timeout_sec=timeout_sec)
        except Exception:
            time.sleep(timeout_sec)

    def _spin_until(self, target: Sequence[int], tol: float = 60.0,
                    stall: float = 1.0, timeout: float = 8.0) -> None:
        """轮询 MotorStatus 直到与 target 偏差 ≤ tol（或到位趋势停滞），
        保证序列下一相位不在真机位置插值完成前叠加。真机插值约 0.6s /
        1000 单位，机械堵转时以"最佳偏差 1s 无改善"结束等待而不死等。"""
        deadline = time.monotonic() + timeout
        best: Optional[float] = None
        best_at: Optional[float] = None
        while time.monotonic() < deadline:
            st = self.get_status()
            if st is not None and len(st.positions) >= len(target):
                cur = tuple(int(v) for v in st.positions[:len(target)])
                d = max(abs(a - b) for a, b in zip(cur, target))
                if best is None or d < best - 1.0:
                    best, best_at = d, time.monotonic()
                if d <= tol:
                    return
                if best_at is not None and time.monotonic() - best_at > stall:
                    return                     # 停滞（堵转/限位），不阻塞序列
            self._spin_once(0.1)
        if self._log is not None:
            self._log.warn(f'{self.name}: 序列相位等待超时（{timeout:.0f}s）')

    def _run_sequence(self, phases) -> None:
        for step, target in enumerate(phases, 1):
            self._publish(target)
            if self._log is not None:
                self._log.info(f'{self.name}: 手势分段 {step}/{len(phases)} '
                               f'→ {list(target)}')
            self._spin_until(target)

    def set_gesture(self, gesture: str) -> bool:
        preset = GESTURE_POSITIONS.get(gesture)
        if preset is None:
            if self._log is not None:
                self._log.warn(
                    f'{self.name}: unknown gesture {gesture!r} '
                    f'(known: {sorted(GESTURE_POSITIONS)})')
            return False
        seq = _GESTURE_SEQUENCES.get(gesture)
        if seq is not None:
            self._run_sequence(seq)          # 分段执行（防关节干涉）
        else:
            self._publish(preset)
        return True

    def set_force(self, forces: Sequence[int]) -> None:
        # Brainco demo controls position only; force setpoints are not
        # demonstrated - accept and ignore with a warning (inspire-style
        # interface kept for symmetry).
        if self._log is not None:
            self._log.warn(f'{self.name}: set_force not supported by the '
                           'brainco demo interface (ignored)')

    def set_speed(self, speeds: Sequence[int]) -> None:
        # Brainco demo controls position only; speed setpoints are not
        # demonstrated - accept and ignore with a warning (publishing
        # POS_MIN positions alongside would physically straighten the hand).
        if self._log is not None:
            self._log.warn(f'{self.name}: set_speed not supported by the '
                           'brainco demo interface (ignored)')

    def get_status(self) -> Optional[HandStatus]:
        return self._status

    def on_touch(self, cb: Callable[[TouchReading], None]) -> None:
        self._touch_cbs.append(cb)


INSPIRE_JOINT_COUNT = 13


class RealInspireHand(DexterousHandBase):
    """Inspire 13-joint hand: angle/force/speed_set
    commands, angle_actual/force_actual/touch_data feedback and the
    SetClearError service. joint_values are broadcast to all 13 joints
    like the demo does."""

    def __init__(self, node, side: str, topics: dict, logger):
        super().__init__(node, side, vendor='inspire')
        self._topics = topics
        self._log = logger
        self._pubs = {}          # 'angle' | 'force' | 'speed' -> publisher
        self._msg_cls = {}       # same keys -> message classes
        self._sub_angle = None
        self._sub_touch = None
        self._status = None
        self._clear_cli = None
        self._clear_srv_cls = None
        self._touch_cbs: list[Callable[[TouchReading], None]] = []

    def on_start(self) -> None:
        classes, err = _msgs.inspire_hand_msgs()
        (angle_cls, force_cls, speed_cls,
         angle_act_cls, _force_act_cls, touch_cls) = classes
        if angle_cls is None:
            raise RuntimeError(f'{self.name}: {err}')

        self._msg_cls = {'angle': angle_cls, 'force': force_cls,
                         'speed': speed_cls}
        self._pubs = {
            'angle': self._node.create_publisher(
                angle_cls, self._topics['angle_cmd'], 10),
            'force': self._node.create_publisher(
                force_cls, self._topics['force_cmd'], 10),
            'speed': self._node.create_publisher(
                speed_cls, self._topics['speed_cmd'], 10),
        }
        self._sub_angle = self._node.create_subscription(
            angle_act_cls, self._topics['angle_actual'],
            self._on_angle_actual, 10)
        if touch_cls is not None:
            self._sub_touch = self._node.create_subscription(
                touch_cls, self._topics['touch'], self._on_touch_msg, 10)

        self._clear_srv_cls, err = _msgs.clear_error_service()
        if self._clear_srv_cls is not None:
            self._clear_cli = self._node.create_client(
                self._clear_srv_cls, self._topics['clear_error'])
        elif self._log is not None:
            self._log.warn(f'{self.name}: {err}; clear_error unavailable')

        if self._log is not None:
            self._log.info(
                f'{self.name} (inspire): pub {self._topics["angle_cmd"]} '
                f'+ force/speed, sub {self._topics["angle_actual"]} '
                f'+ touch')

    def on_stop(self) -> None:
        self._pubs = {}
        self._sub_angle = None
        self._sub_touch = None
        self._status = None

    @property
    def is_active(self) -> bool:
        return self._status is not None

    def _on_angle_actual(self, msg) -> None:
        values = (getattr(msg, 'joint_values', None)
                  or getattr(msg, 'angles', None)
                  or getattr(msg, 'angle', None) or ())
        positions = tuple(int(v) for v in values)
        self._status = HandStatus(positions=positions, raw=msg)

    def _on_touch_msg(self, msg) -> None:
        # TouchData layout is not demo-documented (demo 15 only counts
        # frames); pass through per-item values when present.
        items = []
        for item in getattr(msg, 'data', ()) or ():
            if hasattr(item, 'value'):
                items.append((int(item.value),))
            else:
                items.append(())
        for cb in tuple(self._touch_cbs):
            cb(TouchReading(values=tuple(items)))

    # -- control ----------------------------------------------------------
    def _joint_values(self, values: Sequence[int], default: int = 0) -> list:
        padded = list(values) + [default] * (INSPIRE_JOINT_COUNT - len(values))
        return [int(v) for v in padded[:INSPIRE_JOINT_COUNT]]

    def _publish(self, kind: str, values: Sequence[int]) -> None:
        if kind not in self._pubs:
            raise RuntimeError(f'{self.name} not started')
        _wait_pub_matched(self._node, self._log, self.name,
                          self._pubs[kind])
        msg = self._msg_cls[kind]()
        msg.hand_id = 1 if self.side == 'left' else 2
        msg.joint_values = self._joint_values(values)
        self._pubs[kind].publish(msg)

    def set_positions(self, positions: Sequence[int]) -> None:
        clipped = tuple(max(0, min(1000, int(p))) for p in positions)
        self._publish('angle', clipped)

    def set_force(self, forces: Sequence[int]) -> None:
        self._publish('force', tuple(int(f) for f in forces))

    def set_speed(self, speeds: Sequence[int]) -> None:
        self._publish('speed', tuple(int(s) for s in speeds))

    def clear_error(self) -> bool:
        if self._clear_cli is None:
            return False
        if not self._clear_cli.wait_for_service(timeout_sec=2.0):
            if self._log is not None:
                self._log.error(f'{self.name}: clear_error service '
                                'not reachable')
            return False
        self._clear_cli.call_async(self._clear_srv_cls.Request())
        return True

    def get_status(self) -> Optional[HandStatus]:
        return self._status

    def on_touch(self, cb: Callable[[TouchReading], None]) -> None:
        self._touch_cbs.append(cb)
