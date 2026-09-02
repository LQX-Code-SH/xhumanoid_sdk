#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real audio system: TTS playback, recording, ASR/voice events (§4.5).

TtsService request: text / type (text|file|url) / cmd (append|stop|query);
response: success / status.
AudioControl request: enable; response: success.
LyreVoiceActivity.content: JSON, aiui_event with eventType
    1=ASR result, 4=keyword wake, 5=exit dialogue, 6=VAD, 20=face wake.
"""

from __future__ import annotations

import json
import time
from typing import Callable, Optional

from tienkung_dex.core.base import AudioSystemBase
from tienkung_dex.core.ring import AudioRingBuffer
from tienkung_dex.core.types import AudioChunk

from . import _msgs

# Event-type names live in core (shared with mock); re-exported here for
# existing importers.
from tienkung_dex.core.presets import EVENT_TYPE_NAMES  # noqa: E402


def parse_voice_activity(content: str) -> Optional[dict]:
    """Pure conversion of the LyreVoiceActivity JSON payload (demo
    voice_activity_callback): returns a structured event dict or None."""
    try:
        data = json.loads(content)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    trace_id = data.get('traceId', '')
    if data.get('type') != 'aiui_event':
        return {'event_type': -1, 'name': data.get('type', ''),
                'text': '', 'trace_id': trace_id, 'raw': data}
    inner = data.get('content', {}) or {}
    event_type = inner.get('eventType', -1)
    text = ''
    angle = -1
    result = inner.get('result', {}) or {}
    if event_type == 1:
        text_data = result.get('text', {}) or {}
        words = []
        for ws in text_data.get('ws', []) or []:
            for cw in ws.get('cw', []) or []:
                words.append(cw.get('w', ''))
        text = ''.join(words)
    elif event_type == 4:
        angle = (result.get('ivw', {}) or {}).get('angle', -1)
    return {'event_type': event_type,
            'name': EVENT_TYPE_NAMES.get(event_type, f'unknown({event_type})'),
            'text': text, 'angle': angle, 'trace_id': trace_id, 'raw': data}


class RealAudioSystem(AudioSystemBase):
    """TTS client + /lyre/audio_stream recording + voice activity events."""

    def __init__(self, node, logger, tts_service: str,
                 audio_control: str, audio_stream: str,
                 voice_activity: str, buffer_sec: float = 60.0,
                 tts_timeout: float = 3.0):
        super().__init__(node, 'audio')
        self._log = logger
        self._tts_service = tts_service
        self._audio_control = audio_control
        self._audio_stream = audio_stream
        self._voice_activity = voice_activity
        self._buffer_sec = buffer_sec
        self._tts_timeout = tts_timeout

        self._tts_cls = None
        self._tts_client = None
        self._ctrl_cls = None
        self._ctrl_client = None
        self._frame_cls = None
        self._voice_cls = None
        self._frame_sub = None
        self._voice_sub = None

        self._recording = False
        self._ring = AudioRingBuffer(max_seconds=buffer_sec)
        self._frame_cbs: list[Callable[[AudioChunk], None]] = []
        self._voice_cbs: list[tuple[Callable[[dict], None], Optional[set]]] = []
        self._last_frame_monotonic = None

    def on_start(self) -> None:
        self._tts_cls, err = _msgs.tts_service()
        if self._tts_cls is None:
            if self._log is not None:
                self._log.error(f'audio: {err}; speak() will return False')
        else:
            try:
                self._tts_client = self._node.create_client(
                    self._tts_cls, self._tts_service)
            except Exception as exc:
                if self._log is not None:
                    self._log.error(f'audio: TTS client failed: {exc}')
                self._tts_client = None

        ctrl_cls, frame_cls, voice_cls = (None, None, None)
        try:
            import importlib
            # Import the submodules explicitly: an older lyre_msgs prefix on
            # PYTHONPATH may lack the .srv submodule entirely (observed on the
            # robot host, /opt/humanoid/install vs ~/xos).
            srv_mod = importlib.import_module('lyre_msgs.srv')
            msg_mod = importlib.import_module('lyre_msgs.msg')
            ctrl_cls = getattr(srv_mod, 'AudioControl', None)
            frame_cls = getattr(msg_mod, 'AudioFrame', None)
            voice_cls = getattr(msg_mod, 'LyreVoiceActivity', None)
        except Exception as exc:
            if self._log is not None:
                self._log.error(f'audio: lyre_msgs not importable ({exc})')
        self._ctrl_cls, self._frame_cls, self._voice_cls = \
            ctrl_cls, frame_cls, voice_cls

        if ctrl_cls is not None:
            try:
                self._ctrl_client = self._node.create_client(
                    ctrl_cls, self._audio_control)
            except Exception as exc:
                if self._log is not None:
                    self._log.error(f'audio: AudioControl client failed: {exc}')
        if frame_cls is not None:
            self._frame_sub = self._node.create_subscription(
                frame_cls, self._audio_stream, self._on_frame, 10)
        if voice_cls is not None:
            self._voice_sub = self._node.create_subscription(
                voice_cls, self._voice_activity, self._on_voice, 10)
        if self._log is not None:
            self._log.info(
                f'audio: TTS={self._tts_service} '
                f'(ready={self._tts_client is not None}), '
                f'stream={self._audio_stream}, '
                f'voice={self._voice_activity}')

    def on_stop(self) -> None:
        self._tts_client = None
        self._ctrl_client = None
        self._frame_sub = None
        self._voice_sub = None
        self._recording = False

    @property
    def is_active(self) -> bool:
        return (self._tts_client is not None
                and self._tts_client.service_is_ready()) \
            or self._frame_sub is not None or self._voice_sub is not None

    # -- TTS --------------------------------------------------------------
    def _call_tts(self, text: str, res_type: str, cmd: str,
                  blocking: bool, timeout: float) -> bool:
        if self._tts_client is None or self._tts_cls is None:
            return False
        if blocking and not self._tts_client.wait_for_service(
                timeout_sec=min(timeout, 1.0)):
            return False
        request = self._tts_cls.Request()
        request.text = text
        request.type = res_type
        request.cmd = cmd
        try:
            future = self._tts_client.call_async(request)
        except Exception as exc:
            if self._log is not None:
                self._log.error(f'audio: TTS call failed: {exc}')
            return False
        if not blocking:
            return True

        import rclpy
        try:
            executor = getattr(self._node, 'executor', None)
            if executor is None:
                from rclpy.executors import SingleThreadedExecutor
                executor = SingleThreadedExecutor()
                executor.add_node(self._node)
            executor.spin_until_future_complete(future, timeout_sec=timeout)
        except Exception as exc:  # pragma: no cover - spin environment
            if self._log is not None:
                self._log.error(f'audio: TTS wait failed: {exc}')
            return False
        if not future.done():
            return False
        try:
            return bool(future.result().success)
        except Exception as exc:
            if self._log is not None:
                self._log.error(f'audio: TTS response failed: {exc}')
            return False

    def speak(self, text: str, blocking: bool = False,
              timeout: float = 3.0) -> bool:
        return self._call_tts(text, 'text', 'append', blocking, timeout)

    def play_file(self, path: str, blocking: bool = False,
                  timeout: float = 3.0) -> bool:
        return self._call_tts(path, 'file', 'append', blocking, timeout)

    def stop_playback(self) -> bool:
        return self._call_tts('', 'text', 'stop', blocking=True, timeout=2.0)

    # -- recording --------------------------------------------------------
    def start_recording(self) -> bool:
        if self._ctrl_client is None or self._ctrl_cls is None:
            return False
        if not self._ctrl_client.wait_for_service(timeout_sec=1.0):
            if self._log is not None:
                self._log.error('audio: AudioControl service not reachable; '
                                'recording not started')
            return False
        request = self._ctrl_cls.Request()
        request.enable = True
        try:
            future = self._ctrl_client.call_async(request)
        except Exception as exc:
            if self._log is not None:
                self._log.error(f'audio: AudioControl failed: {exc}')
            return False
        self._recording = True
        self._ring.clear()
        return True

    def stop_recording(self) -> list[AudioChunk]:
        self._recording = False
        if self._ctrl_client is not None and self._ctrl_cls is not None:
            request = self._ctrl_cls.Request()
            request.enable = False
            try:
                self._ctrl_client.call_async(request)
            except Exception:
                pass
        chunks = self._ring.snapshot()
        self._ring.clear()          # handed over to the caller, once
        return chunks

    def _on_frame(self, msg) -> None:
        chunk = AudioChunk(
            data=bytes(getattr(msg, 'data', b'')),
            sample_rate=int(getattr(msg, 'sample_rate', 0)),
            channels=int(getattr(msg, 'channels', 0)),
            bits_per_sample=int(getattr(msg, 'bits_per_sample', 0)),
        )
        self._last_frame_monotonic = time.monotonic()
        for cb in tuple(self._frame_cbs):
            cb(chunk)
        if self._recording:
            self._ring.push(chunk)

    def on_audio_frame(self, cb: Callable[[AudioChunk], None]) -> None:
        self._frame_cbs.append(cb)

    # -- voice / ASR ------------------------------------------------------
    def _on_voice(self, msg) -> None:
        event = parse_voice_activity(getattr(msg, 'content', ''))
        if event is None:
            return
        for cb, event_types in tuple(self._voice_cbs):
            if event_types is not None and event['event_type'] not in event_types:
                continue
            cb(event)

    def on_voice_event(self, cb: Callable[[dict], None],
                       event_types: Optional[set] = None) -> None:
        self._voice_cbs.append((cb, event_types))
