#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Duration-bounded ring buffer for audio chunks (design doc §4.5).

Recording memory constraint: the library keeps at most `max_seconds` of
audio; older chunks are dropped. Long recordings must be consumed
streaming through on_audio_frame(); stop_recording() only ever returns
the bounded buffer, so it cannot OOM a long session.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Iterable

from .types import AudioChunk


class AudioRingBuffer:
    """Thread-safe ring of AudioChunk bounded by total duration."""

    def __init__(self, max_seconds: float = 60.0):
        self._max_seconds = max_seconds
        self._chunks: deque[AudioChunk] = deque()
        self._lock = threading.Lock()

    def push(self, chunk: AudioChunk) -> None:
        with self._lock:
            self._chunks.append(chunk)
            total = sum(c.duration_seconds for c in self._chunks)
            while total > self._max_seconds and len(self._chunks) > 1:
                dropped = self._chunks.popleft()
                total -= dropped.duration_seconds

    def clear(self) -> None:
        with self._lock:
            self._chunks.clear()

    def snapshot(self) -> list[AudioChunk]:
        with self._lock:
            return list(self._chunks)

    @property
    def duration_seconds(self) -> float:
        with self._lock:
            return sum(c.duration_seconds for c in self._chunks)

    def extend_from(self, chunks: Iterable[AudioChunk]) -> None:
        for chunk in chunks:
            self.push(chunk)

    def __len__(self) -> int:
        with self._lock:
            return len(self._chunks)
