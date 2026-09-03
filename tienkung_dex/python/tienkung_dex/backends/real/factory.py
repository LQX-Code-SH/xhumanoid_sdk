#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RealBackendFactory: builds the full set of SDK-topic-backed subsystems.

Assembled by create_robot(); never mixed with sim/mock subsystems
(design doc §4.8 - one factory, one source of truth per robot).
"""

from __future__ import annotations

from tienkung_dex.core import topics as t
from tienkung_dex.core.errors import BackendUnavailableError

from .audio import RealAudioSystem
from .camera import RealCameraStream, RealMultiCameraGroup
from .hand import RealDexterousHand, RealInspireHand
from .joint import RealJointGroup, RobotStateCache
from .light import RealLightControl
from .power import RealPowerSystem
from .safety import RealSafetyMonitor
from .sbus import RealSbusStream
from .sensors import ForceStream, GpsStream, ImuStream, LidarStream
from .serial import RealSerialNumber

# Subsystems that degrade gracefully when their vendor message package is
# missing (optional hardware); everything else is core to the real backend.
OPTIONAL_HAND = 'hand'
OPTIONAL_AUDIO = 'audio'
OPTIONAL_GPS = 'gps'
OPTIONAL_PANORAMA = 'panorama'
OPTIONAL_POWER = 'power'
OPTIONAL_LIGHT = 'light'
OPTIONAL_SBUS = 'sbus'
OPTIONAL_SERIAL = 'serial'


class RealBackendFactory:
    """Creates the real subsystem set bound to one ROS node."""

    def __init__(self, node, logger, joints_table,
                 enable: set[str] | None = None,
                 hand_vendor: str = 'brainco',
                 **params):
        self._node = node
        self._log = logger
        self._joints_table = joints_table
        self._enable = set(enable) if enable else None
        self._hand_vendor = hand_vendor
        self._params = params

        if hand_vendor not in ('brainco', 'inspire'):
            raise BackendUnavailableError(
                f'real backend supports hand_vendor="brainco"/"inspire" '
                f'only, got {hand_vendor!r}')

        # Topic overrides (create_robot **topic_overrides)
        self._robot_state_topic = params.get(
            'robot_state_topic', t.ROBOT_STATE_TOPIC)
        self._cmd_topics = params.get('joint_cmd_topics', t.JOINT_CMD_TOPIC)
        self._camera_namespaces = params.get(
            'camera_namespaces', t.CAMERA_NAMESPACES)
        self._hand_topics = params.get('hand_topics', t.HAND_TOPICS)
        self._inspire_hand_topics = params.get(
            'inspire_hand_topics', t.INSPIRE_HAND_TOPICS)
        self._tts_service = params.get('tts_service', t.TTS_SERVICE)
        self._audio_control = params.get(
            'audio_control_service', t.AUDIO_CONTROL_SERVICE)
        self._audio_stream = params.get(
            'audio_stream_topic', t.AUDIO_STREAM_TOPIC)
        self._voice_topic = params.get(
            'voice_activity_topic', t.VOICE_ACTIVITY_TOPIC)
        self._key_status_topic = params.get(
            'key_status_topic', t.KEY_STATUS_TOPIC)
        self._power_topics = params.get('power_topics', t.POWER_TOPICS)
        self._light_topic = params.get('light_topic', t.LIGHT_TOPIC)
        self._sbus_topics = params.get('sbus_topics', {
            'joy': t.SBUS_TOPIC, 'event': t.SBUS_EVENT_TOPIC})
        self._serial_service = params.get('serial_service', t.SERIAL_SERVICE)
        self._lidar_topic = params.get('lidar_topic', t.LIDAR_TOPIC)
        self._imu_source = params.get('imu_source', 'xsens')
        self._imu_topic = params.get('imu_topic', t.IMU_TOPIC_LIVOX)
        self._gps_topic = params.get('gps_topic', t.GPS_TOPIC)
        self._force_topic = params.get('force_topic', t.FORCE_TOPIC)
        self._panorama_prefix = params.get(
            'panorama_prefix', t.PANORAMA_TOPIC_PREFIX)
        self._panorama_indices = params.get(
            'panorama_indices', t.PANORAMA_INDICES)
        self._panorama_compressed = params.get('panorama_compressed', False)

        self._state_timeout = params.get('state.stale_timeout', 0.5)
        self._tts_timeout = params.get('service.tts_timeout', 3.0)
        self._buffer_sec = params.get('recording.buffer_sec', 60.0)
        self._pair_window = params.get('camera.pair_window', 0.05)
        self._power_timeout = params.get('power.stale_timeout', 5.0)

    def _wanted(self, key: str) -> bool:
        return self._enable is None or key in self._enable

    def build(self) -> dict:
        subsystems = {}

        # Robot state cache: joint groups and the xsens IMU source both feed
        # off the one /robot_state subscription, so the cache must exist
        # whenever either is enabled (missing vendor msgs = fatal).
        need_state_cache = (self._wanted('joint')
                            or (self._wanted('imu')
                                and self._imu_source == 'xsens'))
        state_cache = None
        if need_state_cache:
            state_cache = RobotStateCache(
                self._node, self._robot_state_topic, logger=self._log)
            if not state_cache.start():
                raise BackendUnavailableError(
                    'real backend: /robot_state subscription failed - '
                    'ros2_bridge_msgs unavailable (expected on the robot '
                    'host at /opt/humanoid/install)')
            subsystems['state_cache'] = state_cache

        if self._wanted('joint'):
            if state_cache is None:   # defensive: wanted('joint') => cache
                raise RuntimeError(
                    'real backend: joint groups require the /robot_state '
                    'state cache')
            for group in t.JOINT_GROUPS:
                joint = RealJointGroup(
                    self._node, group, state_cache, self._joints_table,
                    self._log, cmd_topic=self._cmd_topics.get(group),
                    stale_timeout=self._state_timeout)
                subsystems[f'joint_{group}'] = joint

        # Safety: degraded (interception off) when the message is unknown.
        safety = RealSafetyMonitor(
            self._node, self._key_status_topic, self._log,
            stale_timeout=self._state_timeout)
        subsystems['safety'] = safety

        # Guard wiring: every joint command path checks the e-stop.
        for key, subsystem in subsystems.items():
            if key.startswith('joint_'):
                subsystem.attach_guard(safety.guard)

        # Cameras (sensor_msgs only - always available).
        if self._wanted('camera'):
            for namespace in self._camera_namespaces:
                subsystems[f'camera_{namespace}'] = RealCameraStream(
                    self._node, namespace,
                    topics=t.camera_topics(namespace), logger=self._log,
                    pair_window=self._pair_window)

        # Panoramic 6-camera group (optional hardware, default disabled).
        if self._wanted(OPTIONAL_PANORAMA):
            subsystems['panorama'] = RealMultiCameraGroup(
                self._node, indices=self._panorama_indices,
                prefix=self._panorama_prefix, logger=self._log,
                use_compressed=self._panorama_compressed)

        # Hands (optional vendor package; vendor routed by hand_vendor).
        if self._wanted(OPTIONAL_HAND):
            try:
                for side in ('left', 'right'):
                    if self._hand_vendor == 'inspire':
                        hand = RealInspireHand(
                            self._node, side,
                            topics=self._inspire_hand_topics[side],
                            logger=self._log)
                    else:
                        hand = RealDexterousHand(
                            self._node, side,
                            topics=self._hand_topics[side], logger=self._log)
                    subsystems[f'hand_{side}'] = hand
            except Exception as exc:
                if self._log is not None:
                    self._log.error(
                        f'hand ({self._hand_vendor}): vendor message package '
                        f'unavailable ({exc}); hands left as None')

        # Audio (interaction_msgs + lyre_msgs; degrades to speak()=False).
        if self._wanted(OPTIONAL_AUDIO):
            subsystems['audio'] = RealAudioSystem(
                self._node, self._log,
                tts_service=self._tts_service,
                audio_control=self._audio_control,
                audio_stream=self._audio_stream,
                voice_activity=self._voice_topic,
                buffer_sec=self._buffer_sec,
                tts_timeout=self._tts_timeout)

        # Power / light / sbus / serial (bodyctrl_msgs / xsys_msgs; all
        # degrade internally to inactive when their package is missing).
        if self._wanted(OPTIONAL_POWER):
            subsystems['power'] = RealPowerSystem(
                self._node, topics=self._power_topics, logger=self._log,
                stale_timeout=self._power_timeout)
        if self._wanted(OPTIONAL_LIGHT):
            subsystems['light'] = RealLightControl(
                self._node, self._light_topic, logger=self._log)
        if self._wanted(OPTIONAL_SBUS):
            subsystems['sbus'] = RealSbusStream(
                self._node, topics=self._sbus_topics, logger=self._log)
        if self._wanted(OPTIONAL_SERIAL):
            subsystems['serial'] = RealSerialNumber(
                self._node, self._serial_service, logger=self._log)

        # Sensors.
        if self._wanted('imu'):
            subsystems['imu'] = ImuStream(
                self._node, source=self._imu_source,
                topic=self._imu_topic, state_cache=state_cache,
                logger=self._log, stale_timeout=self._state_timeout)
        if self._wanted('lidar'):
            subsystems['lidar'] = LidarStream(
                self._node, self._lidar_topic, logger=self._log)
        if self._wanted(OPTIONAL_GPS):
            subsystems['gps'] = GpsStream(
                self._node, self._gps_topic, logger=self._log)
        if self._wanted('force'):
            subsystems['force'] = ForceStream(
                self._node, self._force_topic, logger=self._log)

        return subsystems
