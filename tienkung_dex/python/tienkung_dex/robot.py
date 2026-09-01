#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienkungDex facade + create_robot() abstract factory (design doc §4.8).

The facade aggregates subsystems built by exactly one backend factory
(real / sim / mock - never mixed) and manages the shared lifecycle.
No business logic lives here (design doc §8); upper layers (ForceController,
navigation clients, perception) consume the subsystems.
"""

from __future__ import annotations

from typing import Optional

from .core import topics as t
from .core.base import SubsystemBase
from .core.errors import BackendUnavailableError, RobotError
from .core.joints import load_joints_table


class TienkungDex:
    """Facade over one robot's hardware capabilities (design doc §4.8)."""

    def __init__(self, subsystems: dict, backend: str, logger=None):
        self.backend = backend
        self._log = logger
        self._subsystems: dict[str, SubsystemBase] = {}

        def _take(*keys, default=None):
            for key in keys:
                if key in subsystems:
                    self._subsystems[key] = subsystems[key]
                    return subsystems[key]
            return default

        # Joint groups (arm/head/waist/leg) + shared state cache.
        for group in t.JOINT_GROUPS:
            setattr(self, group, _take(f'joint_{group}'))
        self.state_cache = subsystems.get('state_cache')

        # Cameras (head/waist/wrist_left/wrist_right) + panoramic group.
        self.cameras: dict[str, SubsystemBase] = {}
        for namespace in t.CAMERA_NAMESPACES:
            camera = _take(f'camera_{namespace}')
            if camera is not None:
                self.cameras[namespace] = camera
        self.panorama = _take('panorama')

        # Hands (optional hardware: None when the vendor package is absent).
        self.hand_left = _take('hand_left')
        self.hand_right = _take('hand_right')

        # Audio / safety / sensors.
        self.audio = _take('audio')
        self.safety = _take('safety')
        self.imu = _take('imu')
        self.lidar = _take('lidar')
        self.gps = _take('gps')
        self.force = _take('force')

    # -- lifecycle (Template Method over the subsystem set) ---------------
    def start(self) -> None:
        """Start subsystems in dependency order (safety/state first)."""
        if self.safety is not None:
            self.safety.start()
        for key, subsystem in self._subsystems.items():
            if key == 'safety':
                continue
            subsystem.start()

    def shutdown(self) -> None:
        """Stop subsystems in reverse order; idempotent."""
        for subsystem in reversed(list(self._subsystems.values())):
            try:
                subsystem.shutdown()
            except Exception as exc:  # pragma: no cover - defensive
                if self._log is not None:
                    self._log.warn(f'shutdown {subsystem.name}: {exc}')

    def health(self) -> dict[str, bool]:
        """{subsystem_name: is_active} summary for bringup self-checks."""
        return {name: subsystem.is_active
                for name, subsystem in self._subsystems.items()}

    def report(self) -> str:
        """One-line-per-subsystem status text (demo/verification aid)."""
        lines = [f'TienkungDex backend={self.backend}']
        for name, subsystem in sorted(self._subsystems.items()):
            lines.append(f'  {name:<28} active={subsystem.is_active}')
        return '\n'.join(lines)


_BACKEND_FACTORIES = {
    'real': 'tienkung_dex.backends.real.factory:RealBackendFactory',
    'sim': 'tienkung_dex.backends.sim.factory:SimBackendFactory',
    'mock': 'tienkung_dex.backends.mock:MockBackendFactory',
}


def _load_factory(backend: str):
    spec = _BACKEND_FACTORIES.get(backend)
    if spec is None:
        raise BackendUnavailableError(
            f'unknown backend {backend!r}; expected one of '
            f'{sorted(_BACKEND_FACTORIES)}')
    module_name, _, class_name = spec.partition(':')
    import importlib
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:
        raise BackendUnavailableError(
            f'backend {backend!r} could not be imported: {exc}')
    return getattr(module, class_name)


def create_robot(node, backend: str = 'real', *,
                 hand_vendor: str = 'brainco',
                 enable: Optional[set] = None,
                 joints_table_path: Optional[str] = None,
                 strict_joint_ids: bool = False,
                 logger=None,
                 **topic_overrides) -> TienkungDex:
    """Abstract-factory entry (design doc §4.8).

    Returns a fully built (not yet started) facade. Call robot.start()
    to open subscriptions, robot.shutdown() before node destruction.

    backend : 'real' (SDK topics, robot host) | 'sim' (ros_gz bridge
              topics) | 'mock' (headless in-process, unit tests)
    enable  : subset of {'joint','camera','hand','audio','safety','imu',
              'lidar','gps','force','panorama'} - None builds everything
              (panorama excluded by default: optional hardware)
    **topic_overrides : per-robot topic names overriding core/topics.py
    """
    default_enable = {'joint', 'camera', 'hand', 'audio', 'safety',
                      'imu', 'lidar', 'gps', 'force'}
    enable = set(enable) if enable is not None else set(default_enable)

    joints_table = load_joints_table(path=joints_table_path,
                                     strict=strict_joint_ids)

    factory_cls = _load_factory(backend)
    factory = factory_cls(node, logger=logger,
                          joints_table=joints_table,
                          enable=enable,
                          hand_vendor=hand_vendor,
                          **topic_overrides)
    subsystems = factory.build()

    # Enforce the facade's own invariants on whatever the factory produced.
    required = ['safety']
    if 'joint' in enable:
        required += [f'joint_{g}' for g in t.JOINT_GROUPS]
    missing = [name for name in required if name not in subsystems]
    if missing:
        raise RobotError(
            f'backend {backend!r} did not provide required subsystems: '
            f'{missing}')

    robot = TienkungDex(subsystems, backend=backend, logger=logger)
    if logger is not None:
        logger.info(
            f'TienkungDex created (backend={backend}, '
            f'enable={sorted(enable)}, hand_vendor={hand_vendor}, '
            f'joints={joints_table!r})')
    return robot
