#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Headless mock backend (design doc §6, §7): in-process substitutes.

Purpose: unit tests and development without a ROS graph, and failure-path
injection (e-stop, ASR events, frames) that real hardware cannot produce
deterministically. The node argument is optional - every subsystem works
headless, and attaches timers automatically when a node is given.
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Sequence

import numpy as np

from tienkung_dex.core.base import (AudioSystemBase, CameraStreamBase,
                                    DexterousHandBase, ForceStreamBase,
                                    GpsStreamBase, ImuStreamBase,
                                    JointGroupBase, LidarStreamBase,
                                    LightControlBase, MultiCameraGroupBase,
                                    PowerSystemBase, SafetyMonitorBase,
                                    SbusStreamBase, SerialNumberBase,
                                    VectorWalkBase)
from tienkung_dex.core.ring import AudioRingBuffer
from tienkung_dex.core.types import (AudioChunk, CameraFrame, ControlMode,
                                     GpsFixReading, HandStatus, ImuReading,
                                     JointCommand, JointReading,
                                     PowerReading, SbusReading, TouchReading,
                                     WrenchReading)


class MockJointGroup(JointGroupBase):
    """In-memory joints that integrate toward commanded targets.

    With a node: a 50 Hz timer advances the model automatically. Headless
    (node=None): callers drive step(dt) explicitly - used by unit tests
    and wait_until() verification.
    """

    def __init__(self, node, group: str, positions: Optional[dict] = None,
                 logger=None, stale_timeout: float = 0.5):
        super().__init__(node, group)
        self._log = logger
        self._stale_timeout = stale_timeout
        self._positions: dict[int, float] = dict(positions or {})
        self._targets: dict[int, float] = {}
        self._speeds: dict[int, float] = {}
        self._timer = None
        self._last_seen = time.monotonic()

    def on_start(self) -> None:
        if self._node is not None:
            self._timer = self._node.create_timer(0.02, self._tick)

    def on_stop(self) -> None:
        self._timer = None

    @property
    def is_active(self) -> bool:
        return time.monotonic() - self._last_seen < self._stale_timeout

    @property
    def last_update_age(self) -> Optional[float]:
        return time.monotonic() - self._last_seen

    def _tick(self) -> None:
        self.step(0.02)

    def step(self, dt: float) -> None:
        """Advance the model by dt seconds toward the latest targets."""
        for jid, target in self._targets.items():
            current = self._positions.get(jid, target)
            speed = self._speeds.get(jid, 0.0)
            if speed <= 0:
                self._positions[jid] = target
                continue
            delta = target - current
            step_size = speed * dt
            if abs(delta) <= step_size:
                self._positions[jid] = target
            else:
                self._positions[jid] = current + step_size * (1 if delta > 0 else -1)
        self._last_seen = time.monotonic()
        self._emit(self.get_states())

    def publish_command(self, cmds: Sequence[JointCommand],
                        mode: ControlMode) -> None:
        for cmd in cmds:
            self._targets[cmd.joint_id] = float(cmd.pos)
            self._speeds[cmd.joint_id] = float(cmd.spd)
            if cmd.joint_id not in self._positions:
                # No known position yet: assume already arrived (the real
                # robot knows where it is). Integration only happens once a
                # position history exists (set_position / previous step).
                self._positions[cmd.joint_id] = float(cmd.pos)
        # A command also refreshes liveness: the real path's /robot_state
        # echo follows the command within the staleness window.
        self._last_seen = time.monotonic()
        # Mock convenience: a command produces a state snapshot for
        # observers immediately (the real path is the async /robot_state).
        self._emit(self.get_states())

    def get_state(self, joint_id: int) -> Optional[JointReading]:
        # NOTE: dict.get(key, self._targets[joint_id]) would evaluate the
        # default eagerly - use explicit branches.
        if joint_id in self._positions:
            pos = float(self._positions[joint_id])
        elif joint_id in self._targets:
            pos = float(self._targets[joint_id])
        else:
            return None
        return JointReading(joint_id=joint_id, pos=pos)

    def get_states(self) -> dict[int, JointReading]:
        ids = set(self._positions) | set(self._targets)
        return {jid: self.get_state(jid) for jid in ids
                if self.get_state(jid) is not None}

    # -- injection --------------------------------------------------------
    def set_position(self, joint_id: int, pos: float) -> None:
        self._positions[joint_id] = float(pos)
        self._last_seen = time.monotonic()
        self._emit(self.get_states())


class MockSafetyMonitor(SafetyMonitorBase):
    """Programmable e-stop (design doc §6: failure-path injection)."""

    def __init__(self, node, logger=None):
        super().__init__(node)
        self._log = logger
        self._estopped = False
        self._remote = False

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        pass

    @property
    def is_active(self) -> bool:
        return True

    @property
    def is_estopped(self) -> bool:
        return self._estopped or self._remote

    @property
    def is_remote_estopped(self) -> bool:
        return self._remote

    def set_estop(self, active: bool, remote: bool = False) -> None:
        previous = self.is_estopped
        if remote:
            self._remote = active
        else:
            self._estopped = active
        if self.is_estopped != previous:
            self._emit(self.is_estopped)

    def guard(self) -> None:
        from tienkung_dex.core.errors import EstopActiveError
        if self.is_estopped:
            raise EstopActiveError(
                'robot e-stop active: joint commands rejected (L1)')


class MockVectorWalk(VectorWalkBase):
    """Headless vector walk: records setpoints + a simple kinematic model.

    node may be None - callers then drive step(dt) explicitly (no clock,
    like MockJointGroup). With a node a 20 Hz timer advances the model
    automatically. The model integrates the latest setpoint so headless
    demos/tests can assert that a non-zero stream produced displacement and
    that stop() froze it.
    """

    def __init__(self, node, logger=None):
        super().__init__(node, 'walk', logger=logger)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._history: list = []
        self._timer = None
        self._ready = False

    def on_start(self) -> None:
        self._ready = True
        if self._node is not None:
            self._timer = self._node.create_timer(0.05, self._tick)

    def on_stop(self) -> None:
        self._timer = None
        self._ready = False

    @property
    def is_active(self) -> bool:
        return self._ready

    @property
    def pose_x(self) -> float:
        return self._x

    @property
    def pose_y(self) -> float:
        return self._y

    @property
    def pose_yaw(self) -> float:
        return self._yaw

    @property
    def history(self) -> list:
        """Every setpoint frame accepted by publish_command (newest last)."""
        return list(self._history)

    def _tick(self) -> None:
        self.step(0.05)

    def step(self, dt: float) -> None:
        """Advance the kinematic model by dt seconds at the latest setpoint."""
        vel = self._velocity
        if vel.norm > 0.0:
            self._x += vel.vx * dt
            self._y += vel.vy * dt
            self._yaw += vel.wz * dt

    def publish_command(self, cmd) -> None:
        self._history.append(cmd)
        self._note_publish()


class MockCameraStream(CameraStreamBase):
    """Synthetic/injected frames (design doc §6 InMemoryMock)."""

    def __init__(self, node, namespace: str, logger=None,
                 resolution=(320, 240)):
        super().__init__(node, namespace)
        self._log = logger
        self._resolution = resolution
        self._frame = None
        self._stamps = []
        self._timer = None

    def on_start(self) -> None:
        if self._node is not None:
            self._timer = self._node.create_timer(0.033, self._synthesize)

    def on_stop(self) -> None:
        self._timer = None
        self._frame = None

    @property
    def is_active(self) -> bool:
        return self._frame is not None

    def _synthesize(self) -> None:
        h, w = self._resolution
        frame = CameraFrame(
            color=np.full((h, w, 3), 128, dtype=np.uint8),
            depth=np.full((h, w), 1000, dtype=np.uint16),
            frame_id=self.namespace)
        self.publish_frame(frame)

    def publish_frame(self, frame: CameraFrame) -> None:
        """Injection hook: push a CameraFrame through the observer path."""
        self._frame = frame
        self._stamps.append(time.monotonic())
        self._stamps = self._stamps[-100:]
        self._emit(frame)

    def latest(self) -> Optional[CameraFrame]:
        return self._frame

    @property
    def frame_rate(self) -> Optional[float]:
        if len(self._stamps) < 2:
            return None
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0 else None


class MockMultiCameraGroup(MultiCameraGroupBase):
    """Headless panoramic group; inject per-index frames."""

    def __init__(self, node, indices=(0, 1, 2, 4, 5, 6), logger=None):
        super().__init__(node, 'panorama')
        self.indices = tuple(indices)
        self._log = logger
        self._frames = {}
        self._callbacks = {i: [] for i in self.indices}

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        self._frames = {}

    @property
    def is_active(self) -> bool:
        return bool(self._frames)

    def on_frame(self, idx: int, cb: Callable[[CameraFrame], None]) -> None:
        self._callbacks.setdefault(idx, []).append(cb)

    def latest(self, idx: int) -> Optional[CameraFrame]:
        return self._frames.get(idx)

    def publish_frame(self, idx: int, frame: CameraFrame) -> None:
        self._frames[idx] = frame
        for cb in tuple(self._callbacks.get(idx, ())):
            cb(frame)


class MockAudioSystem(AudioSystemBase):
    """Programmable TTS/recording/ASR (failure-path injection)."""

    def __init__(self, node, logger=None, buffer_sec: float = 60.0,
                 tts_timeout: float = 3.0):
        super().__init__(node, 'audio')
        self._log = logger
        self._tts_timeout = tts_timeout
        self._speak_result = True
        self._ring = AudioRingBuffer(max_seconds=buffer_sec)
        self._recording = False
        self._frame_cbs: list[Callable[[AudioChunk], None]] = []
        self._voice_cbs: list[tuple[Callable[[dict], None], Optional[set]]] = []

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        self._recording = False

    @property
    def is_active(self) -> bool:
        return True

    def set_speak_result(self, ok: bool) -> None:
        """Injection: simulate TTS service availability."""
        self._speak_result = ok

    def speak(self, text: str, blocking: bool = False,
              timeout: float = 3.0) -> bool:
        return self._speak_result

    def play_file(self, path: str, blocking: bool = False,
                  timeout: float = 3.0) -> bool:
        return self._speak_result

    def stop_playback(self, timeout: float = 5.0) -> bool:
        return True

    def start_recording(self) -> bool:
        self._recording = True
        self._ring.clear()
        return True

    def stop_recording(self) -> list[AudioChunk]:
        self._recording = False
        chunks = self._ring.snapshot()
        self._ring.clear()          # handed over to the caller, once
        return chunks

    def inject_audio_chunk(self, chunk: AudioChunk) -> None:
        """Injection: a synthetic /lyre/audio_stream chunk."""
        for cb in tuple(self._frame_cbs):
            cb(chunk)
        if self._recording:
            self._ring.push(chunk)

    def on_audio_frame(self, cb: Callable[[AudioChunk], None]) -> None:
        self._frame_cbs.append(cb)

    def inject_voice_event(self, event_type: int, text: str = '',
                           trace_id: str = 'mock', **extra) -> None:
        """Injection: a structured ASR event through the observer path."""
        from tienkung_dex.core.presets import EVENT_TYPE_NAMES
        event = {'event_type': event_type,
                 'name': EVENT_TYPE_NAMES.get(event_type, f'unknown({event_type})'),
                 'text': text, 'angle': -1, 'trace_id': trace_id,
                 'raw': {'mock': True}}
        event.update(extra)
        for cb, event_types in tuple(self._voice_cbs):
            if event_types is not None and event_type not in event_types:
                continue
            cb(event)

    def on_voice_event(self, cb: Callable[[dict], None],
                       event_types: Optional[set] = None) -> None:
        self._voice_cbs.append((cb, event_types))


class MockDexterousHand(DexterousHandBase):
    """Headless hand: mirrors the sim two-finger model."""

    def __init__(self, node, side: str, logger=None):
        super().__init__(node, side, vendor='brainco-mock')
        self._log = logger
        self._positions = (1,) * 6
        self._touch_cbs = []

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        pass

    @property
    def is_active(self) -> bool:
        return True

    def set_positions(self, positions: Sequence[int]) -> None:
        self._positions = tuple(int(p) for p in positions[:6])

    def set_gesture(self, gesture: str) -> bool:
        from tienkung_dex.core.presets import GESTURE_POSITIONS
        preset = GESTURE_POSITIONS.get(gesture)
        if preset is None:
            return False
        self.set_positions(preset)
        return True

    def set_force(self, forces: Sequence[int]) -> None:
        pass

    def set_speed(self, speeds: Sequence[int]) -> None:
        pass

    def clear_error(self) -> bool:
        return True

    def get_status(self) -> Optional[HandStatus]:
        return HandStatus(positions=self._positions)

    def on_touch(self, cb) -> None:
        self._touch_cbs.append(cb)

    def inject_touch(self, reading: TouchReading) -> None:
        for cb in tuple(self._touch_cbs):
            cb(reading)


class MockImuStream(ImuStreamBase):
    """Injection-driven IMU."""

    def __init__(self, node, source: str = 'mock', logger=None):
        super().__init__(node, source)
        self._log = logger
        self._latest = None
        self._last_seen = None

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        self._latest = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < 0.5)

    def inject(self, reading: ImuReading) -> None:
        self._latest = reading
        self._last_seen = time.monotonic()
        self._emit(reading)

    def latest(self) -> Optional[ImuReading]:
        return self._latest


class MockLidarStream(LidarStreamBase):
    """Injection-driven point cloud."""

    def __init__(self, node, logger=None):
        super().__init__(node, 'lidar')
        self._log = logger
        self._latest = None

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        self._latest = None

    @property
    def is_active(self) -> bool:
        return self._latest is not None

    def inject(self, cloud) -> None:
        self._latest = cloud
        self._emit(cloud)

    def latest(self):
        return self._latest


class MockGpsStream(GpsStreamBase):
    """Injection-driven GPS."""

    def __init__(self, node, logger=None):
        super().__init__(node, 'gps')
        self._log = logger
        self._latest = None

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        self._latest = None

    @property
    def is_active(self) -> bool:
        return self._latest is not None

    def inject(self, fix: GpsFixReading) -> None:
        self._latest = fix
        self._emit(fix)

    def latest(self) -> Optional[GpsFixReading]:
        return self._latest


class MockForceStream(ForceStreamBase):
    """Injection-driven wrench."""

    def __init__(self, node, logger=None):
        super().__init__(node, 'force')
        self._log = logger
        self._latest = None
        self._last_seen = None

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        self._latest = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < 0.5)

    def inject(self, wrench: WrenchReading) -> None:
        self._latest = wrench
        self._last_seen = time.monotonic()
        self._emit(wrench)

    def latest(self) -> Optional[WrenchReading]:
        return self._latest


class MockPowerSystem(PowerSystemBase):
    """Injection-driven power readings."""

    def __init__(self, node, logger=None):
        super().__init__(node)
        self._log = logger
        self._latest = None

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        self._latest = None

    @property
    def is_active(self) -> bool:
        return self._latest is not None

    def inject(self, reading: PowerReading) -> None:
        self._latest = reading
        self._emit(reading)

    def latest(self) -> Optional[PowerReading]:
        return self._latest


class MockLightControl(LightControlBase):
    """Command-recording light strip."""

    def __init__(self, node, logger=None):
        super().__init__(node)
        self._log = logger
        self.commands: list[tuple] = []      # history of (cmd, data)
        self._ready = False

    def on_start(self) -> None:
        self._ready = True

    def on_stop(self) -> None:
        self._ready = False

    @property
    def is_active(self) -> bool:
        return self._ready

    def set_cmd(self, cmd: int, data: Sequence[int] = ()) -> None:
        self.commands.append((int(cmd), tuple(int(d) for d in data)))

    def set_mode(self, mode: str) -> bool:
        from tienkung_dex.core import topics as t
        cmd = t.LIGHT_CMDS.get(mode)
        if cmd is None:
            return False
        self.set_cmd(cmd)
        return True


class MockSbusStream(SbusStreamBase):
    """Injection-driven RC receiver."""

    def __init__(self, node, logger=None):
        super().__init__(node)
        self._log = logger
        self._latest = None

    def on_start(self) -> None:
        pass

    def on_stop(self) -> None:
        self._latest = None

    @property
    def is_active(self) -> bool:
        return self._latest is not None

    def inject(self, reading: SbusReading) -> None:
        self._latest = reading
        self._emit(reading)

    def latest(self) -> Optional[SbusReading]:
        return self._latest


class MockSerialNumber(SerialNumberBase):
    """Programmable serial number service stub."""

    def __init__(self, node, logger=None, serial: str = 'MOCK-SN-0000'):
        super().__init__(node)
        self._log = logger
        self._serial = serial
        self._ready = False

    def on_start(self) -> None:
        self._ready = True

    def on_stop(self) -> None:
        self._ready = False

    @property
    def is_active(self) -> bool:
        return self._ready

    def set_serial(self, serial: str) -> None:
        self._serial = serial

    def get_serial_number(self, timeout: float = 5.0) -> Optional[str]:
        return self._serial


class MockBackendFactory:
    """Headless factory: builds the full mock subsystem set.

    node may be None - everything runs in-process. Used by the unit tests
    and by tools that need a robot without any ROS graph (design doc §6).
    """

    def __init__(self, node=None, logger=None, joints_table=None,
                 enable: set[str] | None = None,
                 hand_vendor: str = 'brainco', **params):
        self._node = node
        self._log = logger
        self._joints_table = joints_table
        self._enable = set(enable) if enable else None
        self._params = params

    def _wanted(self, key: str) -> bool:
        return self._enable is None or key in self._enable

    def build(self) -> dict:
        subsystems = {}
        safety = MockSafetyMonitor(self._node, logger=self._log)
        subsystems['safety'] = safety

        if self._wanted('joint'):
            from tienkung_dex.core import topics as t
            for group in t.JOINT_GROUPS:
                joint = MockJointGroup(self._node, group, logger=self._log)
                joint.attach_guard(safety.guard)
                subsystems[f'joint_{group}'] = joint

        if self._wanted('walk'):
            walk = MockVectorWalk(self._node, logger=self._log)
            walk.attach_guard(safety.guard)
            subsystems['walk'] = walk

        if self._wanted('camera'):
            from tienkung_dex.core import topics as t
            for namespace in t.CAMERA_NAMESPACES:
                subsystems[f'camera_{namespace}'] = MockCameraStream(
                    self._node, namespace, logger=self._log)

        if self._wanted('panorama'):
            subsystems['panorama'] = MockMultiCameraGroup(
                self._node, logger=self._log)

        if self._wanted('hand'):
            for side in ('left', 'right'):
                subsystems[f'hand_{side}'] = MockDexterousHand(
                    self._node, side, logger=self._log)

        if self._wanted('audio'):
            subsystems['audio'] = MockAudioSystem(
                self._node, logger=self._log,
                buffer_sec=self._params.get('recording.buffer_sec', 60.0))
        if self._wanted('power'):
            subsystems['power'] = MockPowerSystem(self._node, logger=self._log)
        if self._wanted('light'):
            subsystems['light'] = MockLightControl(self._node, logger=self._log)
        if self._wanted('sbus'):
            subsystems['sbus'] = MockSbusStream(self._node, logger=self._log)
        if self._wanted('serial'):
            subsystems['serial'] = MockSerialNumber(self._node, logger=self._log)
        if self._wanted('imu'):
            subsystems['imu'] = MockImuStream(self._node, logger=self._log)
        if self._wanted('lidar'):
            subsystems['lidar'] = MockLidarStream(self._node, logger=self._log)
        if self._wanted('gps'):
            subsystems['gps'] = MockGpsStream(self._node, logger=self._log)
        if self._wanted('force'):
            subsystems['force'] = MockForceStream(self._node, logger=self._log)
        return subsystems
