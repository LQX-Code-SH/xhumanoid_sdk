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
    # 'rock' flex 取 685：按双手真机实测"扣实拳头"状态固化（左 flex≈689 /
    # 右 flex≈681，rotate≈700，四指≈999-1000）。注意：拇指 flex 的机械上限
    # 是状态相关的——四指全收(~1000)时 thumb 才被食指/中指关节干涉限制在
    # ~690 以下（原值 1000 单步会把食指/中指卡在 ~630/600 堵转）；而无干涉
    # 状态（如 scissors，中指伸直留出空间）flex 仍可正常到 1000（其 preset
    # 即 1000，真机实测到位）。故 685 只适用于"四指全收+拇指扣拳"的 rock
    # 形位，不可推广为拇指全局上限。real 后端对该手势做两段式执行（先伸直
    # 拇指外展、收拢四指，再弯拇指，见 hand.py _GESTURE_SEQUENCES）。
    'rock': (685, 700, 1000, 1000, 1000, 1000),
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
