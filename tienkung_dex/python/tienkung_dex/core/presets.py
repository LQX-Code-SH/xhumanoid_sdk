#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Vendor-agnostic preset tables shared by real / sim / mock backends.

Kept in core so the mock backend never imports the real backend (layering:
mock and sim must stay importable headless even if a real module later
gains a module-level vendor import).
"""

from __future__ import annotations

# Brainco 6-motor gesture presets copied from the gesture demo
# (motor order: thumb flex, thumb rotate, index, middle, ring, pinky;
# scale 1 = fully straight .. 1000 = fully bent).
GESTURE_POSITIONS = {
    'ok': (450, 800, 450, 1, 1, 1),
    'rock': (1000, 700, 1000, 1000, 1000, 1000),
    'scissors': (1000, 500, 1, 1, 1000, 1000),
    'paper': (1, 500, 1, 1, 1, 1),
}

# LyreVoiceActivity aiui_event eventType names (demo voice_activity_callback).
EVENT_TYPE_NAMES = {
    1: 'asr_result',
    4: 'keyword_wake',
    5: 'exit_dialogue',
    6: 'vad',
    20: 'face_wake',
}
