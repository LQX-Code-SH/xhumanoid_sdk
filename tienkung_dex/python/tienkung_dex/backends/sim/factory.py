#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SimBackendFactory (design doc §6).

Camera / IMU / lidar topics are identical to the real machine (main-project
06 §4.3), so the real subscription classes are reused verbatim; only the
joint path and the non-physical services (audio, safety, hands) are
replaced. Vendor message packages are never imported by this backend.
"""

from __future__ import annotations

from tienkung_dex.core import topics as t
from tienkung_dex.core.errors import BackendUnavailableError

from ..real.camera import RealCameraStream, RealMultiCameraGroup
from ..real.sensors import ImuStream, LidarStream
from ..mock import MockAudioSystem, MockSafetyMonitor
from .hand import SimDexterousHand
from .joint import SimJointGroup


class SimBackendFactory:
    """Builds the Gazebo-compatible subsystem set (same API as real)."""

    # What this factory can actually build; anything else in `enable` has no
    # simulation equivalent and would silently leave facade attrs as None.
    SUPPORTED = frozenset({'joint', 'camera', 'panorama', 'hand', 'audio',
                           'safety', 'imu', 'lidar'})

    def __init__(self, node, logger, joints_table,
                 enable: set[str] | None = None,
                 hand_vendor: str = 'brainco',
                 **params):
        self._node = node
        self._log = logger
        self._joints_table = joints_table
        self._enable = set(enable) if enable else None
        self._params = params

        if hand_vendor != 'brainco':
            # Design decision (doc §6): the sim only models the brainco
            # 6-motor -> two-finger equivalent; an inspire mapping is not
            # invented until the real interface is verified (§11.2).
            raise BackendUnavailableError(
                f'sim backend supports hand_vendor="brainco" only, '
                f'got {hand_vendor!r}')

        self._camera_namespaces = params.get(
            'camera_namespaces', t.CAMERA_NAMESPACES)
        self._joint_state_topic = params.get(
            'sim_joint_state_topic', t.SIM_JOINT_STATE_TOPIC)
        self._joint_cmd_topic = params.get(
            'sim_joint_cmd_topic', t.SIM_JOINT_CMD_TOPIC)
        self._imu_topic = params.get('sim_imu_topic', t.SIM_IMU_TOPIC)
        self._lidar_topic = params.get('lidar_topic', t.LIDAR_TOPIC)
        self._pair_window = params.get('camera.pair_window', 0.05)
        self._state_timeout = params.get('state.stale_timeout', 0.5)

        unsupported = self._enable - self.SUPPORTED if self._enable else set()
        if unsupported and self._log is not None:
            self._log.warn(
                f'sim backend has no simulation for {sorted(unsupported)}; '
                'the corresponding facade attributes stay None')

    def _wanted(self, key: str) -> bool:
        return self._enable is None or key in self._enable

    def build(self) -> dict:
        subsystems = {}

        if self._wanted('joint'):
            for group in t.JOINT_GROUPS:
                subsystems[f'joint_{group}'] = SimJointGroup(
                    self._node, group,
                    state_topic=self._joint_state_topic,
                    cmd_topic=self._joint_cmd_topic,
                    logger=self._log, stale_timeout=self._state_timeout)

        # Programmable safety (no key_status topic in gazebo).
        subsystems['safety'] = MockSafetyMonitor(self._node, logger=self._log)

        if self._wanted('camera'):
            for namespace in self._camera_namespaces:
                subsystems[f'camera_{namespace}'] = RealCameraStream(
                    self._node, namespace,
                    topics=t.camera_topics(namespace), logger=self._log,
                    pair_window=self._pair_window)

        if self._wanted('panorama'):
            subsystems['panorama'] = RealMultiCameraGroup(
                self._node, indices=self._params.get(
                    'panorama_indices', t.PANORAMA_INDICES),
                prefix=self._params.get('panorama_prefix',
                                        t.PANORAMA_TOPIC_PREFIX),
                logger=self._log)

        if self._wanted('hand'):
            for side in ('left', 'right'):
                subsystems[f'hand_{side}'] = SimDexterousHand(
                    self._node, side, logger=self._log)

        if self._wanted('audio'):
            subsystems['audio'] = MockAudioSystem(self._node, logger=self._log)

        if self._wanted('imu'):
            subsystems['imu'] = ImuStream(
                self._node, source='sim', topic=self._imu_topic,
                logger=self._log, stale_timeout=self._state_timeout)
        if self._wanted('lidar'):
            subsystems['lidar'] = LidarStream(
                self._node, self._lidar_topic, logger=self._log)

        return subsystems
