#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Recording ring-buffer bound (design doc §4.5: OOM prevention)."""

from tienkung_dex.backends.mock import MockAudioSystem
from tienkung_dex.core.ring import AudioRingBuffer
from tienkung_dex.core.types import AudioChunk


def _chunk(seconds: float) -> AudioChunk:
    return AudioChunk(data=b'\x00' * int(16000 * seconds), sample_rate=8000,
                      channels=1, bits_per_sample=16)


def test_ring_buffer_drops_oldest_beyond_limit():
    ring = AudioRingBuffer(max_seconds=2.0)
    for _ in range(5):
        ring.push(_chunk(1.0))
    snapshot = ring.snapshot()
    assert len(snapshot) == 2
    assert abs(ring.duration_seconds - 2.0) < 1e-6


def test_ring_buffer_clear():
    ring = AudioRingBuffer(max_seconds=2.0)
    ring.push(_chunk(1.0))
    ring.clear()
    assert len(ring) == 0


def test_mock_audio_recording_respects_buffer():
    audio = MockAudioSystem(node=None, buffer_sec=3.0)
    assert audio.start_recording()
    for _ in range(6):
        audio.inject_audio_chunk(_chunk(1.0))
    recorded = audio.stop_recording()
    assert len(recorded) == 3  # oldest 3 chunks dropped
    # Chunks arriving while not recording are NOT buffered.
    audio.inject_audio_chunk(_chunk(1.0))
    assert audio.stop_recording() == []
