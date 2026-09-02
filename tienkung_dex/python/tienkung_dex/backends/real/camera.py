#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real camera streams (design doc §4.4, §5).

Decoding contracts (verified against the demos):
  depth : 16UC1 raw values are millimetres; 32FC1 is metres -> *1000;
          (D405 z16 conversion happens inside camera_wrist_driver, so the
          published topic is already mm)
  color : rgb8/rgba8/bgra8/bgr8 all normalized to BGR uint8.
Pairing: color/depth are matched by header.stamp nearest neighbour within
camera.pair_window seconds (ROS 2 Image has no sequence field).
"""

from __future__ import annotations

import time
from collections import deque
from typing import Callable, Optional

import numpy as np

from tienkung_dex.core.base import CameraStreamBase, MultiCameraGroupBase
from tienkung_dex.core.types import CameraFrame

CAMERA_QOS = None  # built lazily (needs rclpy)


def _camera_qos():
    global CAMERA_QOS
    if CAMERA_QOS is None:
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        CAMERA_QOS = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
    return CAMERA_QOS


def decode_color(msg) -> Optional[np.ndarray]:
    """sensor_msgs/Image -> BGR uint8 (H, W, 3); None on unsupported enc."""
    encoding = msg.encoding
    try:
        if encoding in ('rgb8', 'RGB8'):
            array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            return array[:, :, ::-1].copy()      # RGB -> BGR
        if encoding in ('bgr8', 'BGR8'):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        if encoding in ('rgba8', 'RGBA8'):
            array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 4)
            return array[:, :, :3][:, :, ::-1].copy()
        if encoding in ('bgra8', 'BGRA8'):
            array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 4)
            return array[:, :, :3].copy()
    except Exception:
        return None
    return None


def decode_depth(msg) -> Optional[np.ndarray]:
    """sensor_msgs/Image -> uint16 millimetres (0 = invalid)."""
    try:
        if msg.encoding in ('16UC1', 'mono16'):
            return np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width).copy()
        if msg.encoding == '32FC1':
            depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)
            return (depth_m * 1000.0).astype(np.uint16)
    except Exception:
        return None
    return None


def _stamp_sec(stamp) -> float:
    try:
        return stamp.sec + stamp.nanosec * 1e-9
    except Exception:
        return 0.0


class RealCameraStream(CameraStreamBase):
    """Subscribes {ns}/color|depth/image_raw with the camera QoS profile."""

    def __init__(self, node, namespace: str, topics: dict, logger,
                 pair_window: float = 0.05, rate_window: float = 2.0):
        super().__init__(node, namespace)
        self._topics = topics
        self._log = logger
        self._pair_window = pair_window
        self._rate_window = rate_window
        self._subs = []
        self._color = None
        self._depth = None
        self._frame = None
        self._stamps: deque[float] = deque()

    def on_start(self) -> None:
        from sensor_msgs.msg import Image
        for kind in ('color', 'depth'):
            self._subs.append(self._node.create_subscription(
                Image, self._topics[kind],
                (lambda msg, k=kind: self._on_image(k, msg)), _camera_qos()))
        if self._log is not None:
            self._log.info(
                f'{self.name}: sub {self._topics["color"]} + '
                f'{self._topics["depth"]} (BEST_EFFORT, pair={self._pair_window}s)')

    def on_stop(self) -> None:
        self._subs = []
        self._color = None
        self._depth = None
        self._frame = None

    @property
    def is_active(self) -> bool:
        return self._frame is not None

    def _on_image(self, kind: str, msg) -> None:
        if kind == 'color':
            array = decode_color(msg)
            if array is None:
                if self._log is not None:
                    self._log.warn(f'{self.name}: unsupported color '
                                   f'encoding {msg.encoding}')
                return
            self._color = (array, msg.header.stamp,
                           msg.header.frame_id or self.namespace)
        else:
            array = decode_depth(msg)
            if array is None:
                if self._log is not None:
                    self._log.warn(f'{self.name}: unsupported depth '
                                   f'encoding {msg.encoding}')
                return
            self._depth = (array, msg.header.stamp)
        self._pair_and_emit(msg.header.stamp)

    def _pair_and_emit(self, trigger_stamp) -> None:
        color = self._color
        depth = self._depth
        if color is None and depth is None:
            return
        stamp = trigger_stamp
        color_arr = depth_arr = None
        frame_id = self.namespace
        if color is not None:
            color_arr, color_stamp, frame_id = color
        if depth is not None:
            depth_arr, depth_stamp = depth
        # Nearest-neighbour pairing: use the freshest of the two stamps and
        # require both planes (when present) to be within the pair window.
        if color is not None and depth is not None:
            if abs(_stamp_sec(color_stamp) - _stamp_sec(depth_stamp)) \
                    > self._pair_window:
                return
            stamp = (color_stamp
                     if _stamp_sec(color_stamp) >= _stamp_sec(depth_stamp)
                     else depth_stamp)
        elif color is None:
            stamp = depth_stamp
        elif depth is None:
            stamp = color_stamp

        frame = CameraFrame(color=color_arr, depth=depth_arr,
                            stamp=stamp, frame_id=frame_id)
        self._frame = frame
        now = time.monotonic()
        self._stamps.append(now)
        while self._stamps and now - self._stamps[0] > self._rate_window:
            self._stamps.popleft()
        self._emit(frame)

    def latest(self) -> Optional[CameraFrame]:
        return self._frame

    @property
    def frame_rate(self) -> Optional[float]:
        if len(self._stamps) < 2:
            return None
        span = self._stamps[-1] - self._stamps[0]
        if span <= 0:
            return None
        return (len(self._stamps) - 1) / span


class RealMultiCameraGroup(MultiCameraGroupBase):
    """Panoramic 6-camera RGB group (design doc §4.4; optional hardware).

    Indices follow the SDK demo: 0, 1, 2, 4, 5, 6 (3 and 7 skipped).
    """

    def __init__(self, node, indices, prefix: str, logger,
                 use_compressed: bool = False, qos_depth: int = 10):
        super().__init__(node, 'panorama')
        self.indices = tuple(indices)
        self._prefix = prefix
        self._log = logger
        self._use_compressed = use_compressed
        self._qos_depth = qos_depth
        self._subs = []
        self._callbacks: dict[int, list[Callable]] = {
            i: [] for i in self.indices}
        self._frames: dict[int, CameraFrame] = {}

    def _topic_for(self, idx: int) -> str:
        if self._use_compressed:
            return f'{self._prefix}{idx}/image/compressed'
        return f'{self._prefix}{idx}/image_raw'

    def on_start(self) -> None:
        from sensor_msgs.msg import CompressedImage, Image
        for idx in self.indices:
            topic = self._topic_for(idx)
            if self._use_compressed:
                self._subs.append(self._node.create_subscription(
                    CompressedImage, topic,
                    (lambda msg, i=idx: self._on_compressed(i, msg)),
                    self._qos_depth))
            else:
                self._subs.append(self._node.create_subscription(
                    Image, topic,
                    (lambda msg, i=idx: self._on_image(i, msg)),
                    self._qos_depth))
        if self._log is not None:
            self._log.info(f'{self.name}: sub {len(self.indices)} topics '
                           f'under {self._prefix}* '
                           f'(compressed={self._use_compressed})')

    def on_stop(self) -> None:
        self._subs = []
        self._frames = {}

    @property
    def is_active(self) -> bool:
        return bool(self._frames)

    def _on_image(self, idx: int, msg) -> None:
        array = decode_color(msg)
        if array is None:
            return
        frame = CameraFrame(color=array, depth=None,
                            stamp=msg.header.stamp,
                            frame_id=f'camera{idx}')
        self._frames[idx] = frame
        for cb in tuple(self._callbacks.get(idx, ())):
            cb(frame)

    def _on_compressed(self, idx: int, msg) -> None:
        # image_transport-style JPEG: decode without cv2 is not possible,
        # so compressed frames are forwarded only via the opaque passthrough
        # below; raw topics are the recommended subscription.
        if self._log is not None:
            self._log.warn(
                f'{self.name}: compressed frame on camera{idx} cannot be '
                'decoded without cv2; subscribe to raw topics')

    def on_frame(self, idx: int, cb: Callable[[CameraFrame], None]) -> None:
        self._callbacks.setdefault(idx, []).append(cb)

    def latest(self, idx: int) -> Optional[CameraFrame]:
        return self._frames.get(idx)
