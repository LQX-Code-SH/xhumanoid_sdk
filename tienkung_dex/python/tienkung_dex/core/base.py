#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Subsystem abstract bases (design doc §4).

Every subsystem follows the same Template Method lifecycle:
    start()     -> on_start()     (idempotent, creates subscriptions/services)
    shutdown()  -> on_stop()      (idempotent, reverse of on_start)

Observer contract (design doc §7.1): callbacks run synchronously on the
executor thread and MUST be lightweight and reentrant; under a
MultiThreadedExecutor different subsystem callbacks may run concurrently.
For heavy consumers prefer latest() polling.

The 'node' argument is duck-typed: anything exposing the rclpy.Node API
works, which lets the headless mock backend use nodes without rclpy.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Mapping, Sequence
from typing import Optional

import numpy as np

from . import topics as t
from .types import (
    AudioChunk,
    CameraFrame,
    ControlMode,
    GpsFixReading,
    HandStatus,
    ImuReading,
    JointCommand,
    JointReading,
    PowerReading,
    SbusReading,
    TouchReading,
    VelocityCommand,
    WrenchReading,
)


class SubsystemBase(ABC):
    """Template Method lifecycle shared by every subsystem.

    Lifecycle is ONE-SHOT: on_stop drops references without destroying the
    underlying rclpy entities (publishers/subscriptions stay registered on
    the node), so a stop -> start cycle would create DUPLICATE handles and
    observer registrations. For a fresh lifecycle rebuild the facade via
    create_robot(); node destruction is the resource cleanup point.
    """

    def __init__(self, node, name: str):
        self._node = node
        self.name = name
        self._started = False

    @property
    def node(self):
        return self._node

    @property
    def is_started(self) -> bool:
        return self._started

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        try:
            self.on_start()
        except Exception:
            self._started = False
            raise

    def shutdown(self) -> None:
        if not self._started:
            return
        try:
            self.on_stop()
        finally:
            self._started = False

    @abstractmethod
    def on_start(self) -> None:
        """Create subscriptions/services/clients; called exactly once."""

    @abstractmethod
    def on_stop(self) -> None:
        """Release resources; called exactly once (idempotent)."""

    @property
    @abstractmethod
    def is_active(self) -> bool:
        """True while the data source has spoken within the staleness limit."""


class _Snapshot(ABC):
    """Shared Proxy-cache helper: lock-protected latest snapshot."""

    def __init__(self):
        import threading
        self._lock = threading.Lock()
        self._value = None
        self._stamp_sec = None

    def update(self, value) -> None:
        with self._lock:
            self._value = value
            self._stamp_sec = _monotonic()

    def latest(self):
        with self._lock:
            return self._value


def _monotonic() -> float:
    import time
    return time.monotonic()


class _Observable(ABC):
    """Shared observer registry helper."""

    def __init__(self):
        self._callbacks: list[Callable] = []

    def on(self, cb: Callable) -> None:
        """Register an observer; duplicates are allowed (caller's choice)."""
        self._callbacks.append(cb)

    def _emit(self, *args) -> None:
        for cb in tuple(self._callbacks):
            cb(*args)


class JointGroupBase(SubsystemBase, _Observable):
    """One joint group (arm / head / waist / leg), design doc §4.2.

    Command path: publish from the caller thread (no internal timer); the
    50 Hz cadence belongs to the caller's control loop. The e-stop
    pre-check lives in the shared _guard hook so every command path is
    intercepted even when the facade is bypassed.
    """

    def __init__(self, node, group: str, name: str = None):
        SubsystemBase.__init__(self, node, name or f'joint_{group}')
        _Observable.__init__(self)
        if group not in ('arm', 'head', 'waist', 'leg'):
            raise ValueError(f'unknown joint group: {group!r}')
        self.group = group
        self._guard = None              # callable raising EstopActiveError
        self._calibration_unlocked = False

    def attach_guard(self, guard) -> None:
        """Install the e-stop guard invoked before every publish."""
        self._guard = guard

    # -- command ----------------------------------------------------------
    def command(self, cmds: Sequence[JointCommand],
                mode: ControlMode = ControlMode.POSITION) -> None:
        self._precheck(mode)
        self.publish_command(tuple(cmds), mode)

    def move_to(self, positions: Mapping[int, float],
                spd: float = 0.3, cur: float = 10.0) -> None:
        """Position-mode convenience (SDK demo POS_SPD / POS_CUR defaults)."""
        cmds = [JointCommand(joint_id=j, pos=p, spd=spd, cur=cur)
                for j, p in positions.items()]
        self.command(cmds, ControlMode.POSITION)

    def impedance(self, positions: Mapping[int, float],
                  kp: float = 50.0, kd: float = 2.0) -> None:
        """Force-position hybrid convenience (SDK demo HYBRID_KP/KD)."""
        cmds = [JointCommand(joint_id=j, pos=p, kp=kp, kd=kd)
                for j, p in positions.items()]
        self.command(cmds, ControlMode.IMPEDANCE)

    def unlock_calibration_mode(self) -> None:
        """One-shot unlock for ZERO_CALIB (design doc §4.2, explicit opt-in)."""
        self._calibration_unlocked = True

    def _precheck(self, mode: ControlMode) -> None:
        if mode == ControlMode.ZERO_CALIB and not self._calibration_unlocked:
            from .errors import UnsafeModeError
            raise UnsafeModeError(
                f'{self.group}: ZERO_CALIB mode requires '
                'unlock_calibration_mode() first')
        if self._guard is not None:
            self._guard()

    # -- state ------------------------------------------------------------
    @abstractmethod
    def get_state(self, joint_id: int) -> Optional[JointReading]:
        """Cached latest reading (Proxy pattern; never blocks long)."""

    def get_states(self) -> dict[int, JointReading]:
        return {}

    def on_state(self, cb: Callable[[dict[int, JointReading]], None]) -> None:
        """Observer: invoked with the full snapshot on every /robot_state."""
        self.on(cb)

    def wait_until(self, joint_id: int, target: float,
                   tol_deg: float = 5.0,
                   timeout: float = 10.0) -> bool:
        """Block until |pos - target| < tol_deg, polled in the caller's
        control loop (design doc §4.2, SDK demo POS_THRESHOLD_DEG=5.0).

        Requires the node to keep spinning (callbacks deliver fresh state).
        """
        import math
        import time

        deadline = time.monotonic() + timeout
        tol_rad = math.radians(tol_deg)
        while time.monotonic() < deadline:
            reading = self.get_state(joint_id)
            if reading is not None and abs(reading.pos - target) < tol_rad:
                return True
            time.sleep(0.02)
        return False

    @property
    def last_update_age(self) -> Optional[float]:
        """Seconds since the freshest state snapshot (None = never seen)."""
        return None

    @abstractmethod
    def publish_command(self, cmds: Sequence[JointCommand],
                        mode: ControlMode) -> None:
        """Backend hook invoked by command() after the guards."""


class DexterousHandBase(SubsystemBase):
    """Dexterous hand abstraction (design doc §4.3)."""

    def __init__(self, node, side: str, vendor: str, name: str = None):
        super().__init__(node, name or f'hand_{side}')
        if side not in ('left', 'right'):
            raise ValueError(f"side must be 'left'/'right', got {side!r}")
        self.side = side
        self.vendor = vendor

    @abstractmethod
    def set_positions(self, positions: Sequence[int]) -> None:
        """Brainco: 1=straight .. 1000=fully bent (6 motors)."""

    def set_gesture(self, gesture: str) -> bool:
        """Preset gesture ('ok'/'rock'/'scissors'/'paper'); False if unknown."""
        return False

    @abstractmethod
    def set_force(self, forces: Sequence[int]) -> None:
        """Vendor force setpoints (inspire-style force_set)."""

    @abstractmethod
    def set_speed(self, speeds: Sequence[int]) -> None:
        """Vendor speed setpoints."""

    def clear_error(self) -> bool:
        """Vendor clear-error service (inspire hands); False when
        unsupported by the backend/vendor."""
        return False

    @abstractmethod
    def get_status(self) -> Optional[HandStatus]:
        """Cached latest motor status."""

    def on_touch(self, cb: Callable[[TouchReading], None]) -> None:
        """Observer for optional touch hardware (silent when absent)."""


class CameraStreamBase(SubsystemBase, _Observable):
    """One RGB-D camera stream (design doc §4.4)."""

    def __init__(self, node, namespace: str, name: str = None):
        SubsystemBase.__init__(self, node, name or namespace)
        _Observable.__init__(self)
        self.namespace = namespace

    def on_frame(self, cb: Callable[[CameraFrame], None]) -> None:
        self.on(cb)

    @abstractmethod
    def latest(self) -> Optional[CameraFrame]:
        """Lock-protected latest paired frame."""

    @property
    @abstractmethod
    def frame_rate(self) -> Optional[float]:
        """Measured frame rate (sliding window); None before first frames."""


class MultiCameraGroupBase(SubsystemBase):
    """Panoramic 6-camera group, RGB only (design doc §4.4, optional)."""

    indices: Sequence[int] = ()

    @abstractmethod
    def on_frame(self, idx: int, cb: Callable[[CameraFrame], None]) -> None: ...

    @abstractmethod
    def latest(self, idx: int) -> Optional[CameraFrame]: ...


class AudioSystemBase(SubsystemBase):
    """TTS / recording / ASR (design doc §4.5)."""

    def speak(self, text: str, blocking: bool = False,
              timeout: float = 3.0) -> bool:
        """TTS text playback; False on any failure (no retry, no raise).

        With blocking=True the implementation spins the node internally -
        MUST be called from outside the executor spin thread (calling it
        from inside a subscription/service callback deadlocks or raises).
        """
        raise NotImplementedError

    def play_file(self, path: str, blocking: bool = False,
                  timeout: float = 3.0) -> bool:
        """Play a local audio file through the TTS service.

        Same blocking constraint as speak(blocking=True)."""
        raise NotImplementedError

    def stop_playback(self, timeout: float = 5.0) -> bool:
        """TTS cmd='stop'."""
        raise NotImplementedError

    def start_recording(self) -> bool:
        """Enable /lyre/audio_stream (AudioControl enable=True)."""
        raise NotImplementedError

    def stop_recording(self) -> list[AudioChunk]:
        """Stop recording and return the ring-buffered chunks."""
        raise NotImplementedError

    def on_audio_frame(self, cb: Callable[[AudioChunk], None]) -> None:
        """Observer: every audio chunk (streaming path for long recordings)."""

    def on_voice_event(self, cb: Callable[[dict], None],
                       event_types: Optional[set[int]] = None) -> None:
        """Observer: structured ASR/voice events (design doc §4.5)."""


class SafetyMonitorBase(SubsystemBase, _Observable):
    """E-stop monitor (design doc §4.6)."""

    def __init__(self, node, name: str = 'safety'):
        SubsystemBase.__init__(self, node, name)
        _Observable.__init__(self)

    @property
    @abstractmethod
    def is_estopped(self) -> bool:
        """Robot e-stop or wireless e-stop active."""

    @property
    def is_remote_estopped(self) -> bool:
        return False

    def on_estop(self, cb: Callable[[bool], None]) -> None:
        """Edge-triggered observer: cb(active) on every transition."""
        self.on(cb)


class VectorWalkBase(SubsystemBase):
    """Vector-velocity walking subsystem (矢量行走, HRIC cmd_vel).

    Leg joints are owned by the locomotion policy (run_patrol / RL full-body
    control) and must NOT be driven through the joint path; walking is only
    requested as a robot-body-frame velocity stream on
    /hric/robot/cmd_vel (geometry_msgs/TwistStamped). Reference:
    具身天工DEX-矢量行走接口.md.

    Command semantics:
      - vx = forward (m/s), vy = lateral (m/s), wz = turn (rad/s); all other
        Twist fields stay 0;
      - the backend keeps re-publishing the latest setpoint at ~20 Hz
        (streaming cadence the locomotion expects), so the robot keeps
        walking until stop() re-publishes a zero setpoint;
      - out-of-range setpoints are clamped to the WALK_LIMITS box;
      - ||(vx, vy, wz)|| < stop_norm (0.05) is treated as standing and
        sent as a full-zero stop setpoint.
    The shared e-stop guard intercepts every setpoint change like the joint
    groups (design doc §4.6); a running stream is force-zeroed on e-stop.
    """

    def __init__(self, node, name: str = 'walk', logger=None,
                 limits: Mapping | None = None,
                 stop_norm: float | None = None):
        SubsystemBase.__init__(self, node, name)
        self._log = logger
        self._guard = None
        self._limits = dict(limits if limits is not None else t.WALK_LIMITS)
        self._stop_norm = t.WALK_STOP_NORM if stop_norm is None \
            else float(stop_norm)
        self._velocity = VelocityCommand()
        self._published_count = 0

    def attach_guard(self, guard) -> None:
        """Install the e-stop guard invoked before every setpoint change."""
        self._guard = guard

    # -- command ----------------------------------------------------------
    def set_velocity(self, vx: float = 0.0, vy: float = 0.0,
                     wz: float = 0.0) -> None:
        """Request one velocity setpoint (streamed by the backend until the
        next request; zero/near-zero -> standing)."""
        self._precheck()
        cmd = self._sanitize(vx, vy, wz)
        self._velocity = cmd
        self.publish_command(cmd)

    def stop(self) -> None:
        """Request standing: re-publish a zero setpoint immediately."""
        self.set_velocity(0.0, 0.0, 0.0)

    def stand(self) -> None:
        """Alias of stop(): request standing."""
        self.stop()

    # -- state ------------------------------------------------------------
    @property
    def velocity(self) -> VelocityCommand:
        """Latest accepted setpoint (already clamped / zeroed)."""
        return self._velocity

    @property
    def publish_count(self) -> int:
        """Setpoint frames emitted so far (streaming cadence included)."""
        return self._published_count

    @property
    def limits(self) -> dict:
        """Active velocity clamp box (copy of the configured limits)."""
        return dict(self._limits)

    @property
    def stop_norm(self) -> float:
        """Setpoints with ||(vx, vy, wz)|| below this are sent as standing."""
        return self._stop_norm

    # -- internal ---------------------------------------------------------
    def _precheck(self) -> None:
        if self._guard is not None:
            self._guard()

    def _sanitize(self, vx: float, vy: float, wz: float) -> VelocityCommand:
        lim = self._limits

        def _clip(value: float, lo: float, hi: float) -> float:
            value = float(value)
            return max(lo, min(hi, value))

        cmd = VelocityCommand(
            vx=_clip(vx, lim['vx_min'], lim['vx_max']),
            vy=_clip(vy, lim['vy_min'], lim['vy_max']),
            wz=_clip(wz, lim['wz_min'], lim['wz_max']))
        if cmd.norm < self._stop_norm:
            return VelocityCommand()      # standing (full zero)
        return cmd

    def _note_publish(self) -> None:
        self._published_count += 1

    @abstractmethod
    def publish_command(self, cmd: VelocityCommand) -> None:
        """Backend hook invoked by set_velocity() after guards/clamping:
        emit the setpoint now and (real) keep pumping it on the cadence."""


class ImuStreamBase(SubsystemBase, _Observable):
    """Unified IMU stream (design doc §4.7)."""

    def __init__(self, node, source: str, name: str = 'imu'):
        SubsystemBase.__init__(self, node, name)
        _Observable.__init__(self)
        self.source = source

    def on_reading(self, cb: Callable[[ImuReading], None]) -> None:
        self.on(cb)

    @abstractmethod
    def latest(self) -> Optional[ImuReading]: ...


class LidarStreamBase(SubsystemBase, _Observable):
    """Livox lidar point cloud stream (design doc §4.7)."""

    def __init__(self, node, name: str = 'lidar'):
        SubsystemBase.__init__(self, node, name)
        _Observable.__init__(self)

    def on_cloud(self, cb: Callable[[object], None]) -> None:
        """cb(sensor_msgs.PointCloud2) - standard message, passed through."""
        self.on(cb)

    @abstractmethod
    def latest(self) -> Optional[object]: ...


class GpsStreamBase(SubsystemBase, _Observable):
    """GPS stream, optional hardware (design doc §4.7)."""

    def __init__(self, node, name: str = 'gps'):
        SubsystemBase.__init__(self, node, name)
        _Observable.__init__(self)

    def on_fix(self, cb: Callable[[GpsFixReading], None]) -> None:
        self.on(cb)

    @abstractmethod
    def latest(self) -> Optional[GpsFixReading]: ...


class ForceStreamBase(SubsystemBase, _Observable):
    """Six-axis force stream, optional hardware (design doc §4.7).

    While the hardware is absent/unverified, latest() stays None and
    is_active stays False - never raises.
    """

    def __init__(self, node, name: str = 'force'):
        SubsystemBase.__init__(self, node, name)
        _Observable.__init__(self)

    def on_wrench(self, cb: Callable[[WrenchReading], None]) -> None:
        self.on(cb)

    @abstractmethod
    def latest(self) -> Optional[WrenchReading]: ...


class PowerSystemBase(SubsystemBase, _Observable):
    """Battery / power-board monitoring .

    Degrades like the optional sensor streams: while the vendor message
    package is absent latest() stays None and is_active stays False.
    """

    def __init__(self, node, name: str = 'power'):
        SubsystemBase.__init__(self, node, name)
        _Observable.__init__(self)

    def on_update(self, cb: Callable[[PowerReading], None]) -> None:
        self.on(cb)

    @abstractmethod
    def latest(self) -> Optional[PowerReading]: ...


class LightControlBase(SubsystemBase):
    """Light-strip control over /xsys/light/ctrl .

    set_mode() accepts the named presets (battery_normal, wakeup, ...);
    set_cmd() sends the raw LightCtrl cmd value. Pure publish path -
    is_active reflects whether the publisher was created.
    """

    def __init__(self, node, name: str = 'light'):
        super().__init__(node, name)

    def set_mode(self, mode: str) -> bool:
        """Named preset; False when unknown. Presets live in core/topics."""
        return False

    @abstractmethod
    def set_cmd(self, cmd: int, data: Sequence[int] = ()) -> None: ...


class SbusStreamBase(SubsystemBase, _Observable):
    """RC SBUS receiver : Joy axes + button events."""

    def __init__(self, node, name: str = 'sbus'):
        SubsystemBase.__init__(self, node, name)
        _Observable.__init__(self)

    def on_update(self, cb: Callable[[SbusReading], None]) -> None:
        self.on(cb)

    @abstractmethod
    def latest(self) -> Optional[SbusReading]: ...


class SerialNumberBase(SubsystemBase):
    """Robot serial number service /xsys/get_serial_number .

    is_active reflects whether the service was reachable; get_serial_number
    returns None on any failure instead of raising.
    """

    def __init__(self, node, name: str = 'serial'):
        super().__init__(node, name)

    @abstractmethod
    def get_serial_number(self, timeout: float = 5.0) -> Optional[str]:
        """Blocking service call - MUST be invoked outside the executor
        spin thread (the implementation spins the node internally)."""
        ...


__all__ = [
    'SubsystemBase',
    'JointGroupBase',
    'DexterousHandBase',
    'CameraStreamBase',
    'MultiCameraGroupBase',
    'AudioSystemBase',
    'SafetyMonitorBase',
    'VectorWalkBase',
    'ImuStreamBase',
    'LidarStreamBase',
    'GpsStreamBase',
    'ForceStreamBase',
    'PowerSystemBase',
    'LightControlBase',
    'SbusStreamBase',
    'SerialNumberBase',
]
