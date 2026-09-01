#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Wrist Driver Node (Python version)

Publishes RGB (bgr8) and depth (16UC1, millimetres) images from one
wrist-mounted Intel RealSense D405 camera on orbbec-style topics,
aligned with the camera_display subscription contract:

    /ob_camera_wrist_<side>/color/image_raw    (sensor_msgs/Image, bgr8)
    /ob_camera_wrist_<side>/depth/image_raw    (sensor_msgs/Image, 16UC1, mm)

QoS: BEST_EFFORT + KEEP_LAST(10) + VOLATILE - matches camera_display.
"""

import logging
import os
import signal
import struct
import time
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None  # checked at node startup with a clear error message

try:
    import cv2
except ImportError:
    cv2 = None  # compressed topics require OpenCV; raw topics still work

logger = logging.getLogger(__name__)

# QoS matching camera_display subscriptions (BEST_EFFORT + VOLATILE).
CAMERA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class RealSenseD405Driver:
    """Single Intel RealSense D405 camera driver (color + depth streams).

    Thin pyrealsense2 pipeline wrapper so the node stays testable without
    ROS 2: start() -> get_frames() -> stop().
    """

    def __init__(
        self,
        serial: str,
        width: int = 640,
        height: int = 480,
        fps: int = 15,
        enable_depth: bool = True,
    ):
        self._serial = serial
        self._width = width
        self._height = height
        self._fps = fps
        self._enable_depth = enable_depth

        self._pipeline = None
        self._profile = None
        self._is_running = False
        # Scale converting raw z16 counts to millimetres (D405 depth_scale
        # is 0.1 mm per count, the published contract is mm).
        self._depth_to_mm = 1.0

    def start(self) -> None:
        """Open the device and start color (+ depth) streaming."""
        if self._is_running:
            return

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self._serial)
        config.enable_stream(
            rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)
        if self._enable_depth:
            config.enable_stream(
                rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)

        logger.info(
            'Starting RealSense D405 SN=%s at %dx%d@%d FPS (depth=%s)',
            self._serial, self._width, self._height, self._fps, self._enable_depth)

        self._profile = pipeline.start(config)

        # Verify the device is accessible
        if self._profile.get_device() is None:
            pipeline.stop()
            raise RuntimeError(
                f'Failed to get device for RealSense D405 SN={self._serial}. '
                f'Check USB connection and permissions.')

        if self._enable_depth:
            depth_scale_m = self._profile.get_device().first_depth_sensor().get_depth_scale()
            self._depth_to_mm = depth_scale_m * 1000.0

        self._pipeline = pipeline
        self._is_running = True

    def stop(self) -> None:
        """Stop streaming and release the device (idempotent)."""
        self._is_running = False
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as e:
                logger.debug('Error stopping pipeline SN=%s: %s', self._serial, e)
            self._pipeline = None
            self._profile = None

    def get_frames(
        self, timeout_ms: int = 3000
    ) -> Optional[Tuple[np.ndarray, Optional[np.ndarray]]]:
        """Wait for and return the next color (+ depth) frames.

        Returns (color, depth) where color is a BGR uint8 image (H, W, 3)
        and depth is a uint16 image in mm (H, W), or None if the depth
        stream is disabled or the frame is missing. Returns None entirely
        on timeout or capture error.
        """
        if not self._is_running or self._pipeline is None:
            return None

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=timeout_ms)

            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            color = np.asanyarray(color_frame.get_data())

            # Depth may drop out while color keeps flowing: still publish color.
            depth = None
            if self._enable_depth:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    # RealSense z16 counts are in device units (0.1 mm on
                    # the D405); convert to the documented mm contract.
                    # 65535 is the out-of-range saturation sentinel -> 0
                    # (invalid), so statistics/colormaps ignore it.
                    z16 = np.asanyarray(depth_frame.get_data())
                    saturated = z16 == 65535
                    depth = np.rint(z16 * self._depth_to_mm).astype(np.uint16)
                    depth[saturated] = 0

            return color, depth
        except Exception as e:
            logger.warning(
                'Failed to capture frames from RealSense SN=%s: %s', self._serial, e)
            return None

    @staticmethod
    def discover() -> list:
        """Discover all connected RealSense devices.

        Returns a list of dicts with serial/name/usb_type/firmware/product_line.
        """
        if rs is None:
            logger.warning('pyrealsense2 not available; cannot discover devices.')
            return []

        devices = []
        for dev in rs.context().query_devices():
            devices.append({
                'serial': dev.get_info(rs.camera_info.serial_number),
                'name': dev.get_info(rs.camera_info.name),
                'usb_type': dev.get_info(rs.camera_info.usb_type_descriptor),
                'firmware': dev.get_info(rs.camera_info.firmware_version),
                'product_line': dev.get_info(rs.camera_info.product_line),
            })
        return devices


class CameraWristDisplayNode(Node):
    """ROS2 Node publishing one wrist D405 camera's color + depth images."""

    @staticmethod
    def _env_int(name, default):
        """Parse an integer env var; fall back to default on bad input.

        The env file is user-editable (README troubleshooting): a stray
        trailing space must not turn the node into a systemd crash loop.
        """
        value = os.environ.get(name)
        if value is None:
            return default
        try:
            return int(value.strip())
        except ValueError:
            logger.warning(
                'Invalid integer for %s=%r; using default %d', name, value, default)
            return default

    @staticmethod
    def _env_float(name, default):
        value = os.environ.get(name)
        if value is None:
            return default
        try:
            return float(value.strip())
        except ValueError:
            logger.warning(
                'Invalid number for %s=%r; using default %g', name, value, default)
            return default

    @staticmethod
    def _env_bool(name, default):
        value = os.environ.get(name)
        if value is None:
            return default
        value = value.strip().lower()
        if value in ('0', 'false', 'off', 'no', ''):
            return False
        if value in ('1', 'true', 'on', 'yes'):
            return True
        logger.warning(
            'Invalid boolean for %s=%r; using default %s', name, value, default)
        return default

    def __init__(self):
        super().__init__('camera_wrist_driver_node')

        # serial/side fall back to the WRIST_* environment variables
        # injected by the systemd EnvironmentFile (see scripts/install.sh).
        self.declare_parameter('serial', os.environ.get('WRIST_SERIAL', ''))
        self.declare_parameter('side', os.environ.get('WRIST_SIDE', 'left'))
        self.declare_parameter('width', self._env_int('WRIST_WIDTH', 640))
        self.declare_parameter('height', self._env_int('WRIST_HEIGHT', 480))
        self.declare_parameter('fps', self._env_int('WRIST_FPS', 15))
        self.declare_parameter('topic_prefix', 'ob_camera_wrist')
        self.declare_parameter('enable_depth', True)
        # Camera mounting rotation in degrees (0/90/180/270, clockwise).
        # D405 has no IMU, so orientation cannot be auto-detected.
        self.declare_parameter('rotation', self._env_int('WRIST_ROTATE', 0))
        # Compressed image topics (image_transport style, matches the head
        # camera pattern): <topic>/compressed (JPEG) and
        # <topic>/compressedDepth (lossless PNG of the 16-bit depth map).
        self.declare_parameter(
            'enable_compressed', self._env_bool('WRIST_COMPRESSED', True))
        self.declare_parameter(
            'jpeg_quality', self._env_int('WRIST_JPEG_QUALITY', 80))
        # compressedDepth clips values above this to 0 (invalid), in meters.
        self.declare_parameter(
            'depth_max', self._env_float('WRIST_DEPTH_MAX', 10.0))

        serial = str(self.get_parameter('serial').value)
        side = str(self.get_parameter('side').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        topic_prefix = str(self.get_parameter('topic_prefix').value)
        enable_depth = bool(self.get_parameter('enable_depth').value)
        rotation = int(self.get_parameter('rotation').value)
        enable_compressed = bool(self.get_parameter('enable_compressed').value)
        jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        depth_max_m = float(self.get_parameter('depth_max').value)

        if not serial:
            raise RuntimeError(
                "no camera serial: set the 'serial' parameter or the "
                'WRIST_SERIAL environment variable')
        if side not in ('left', 'right'):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        if rotation not in (0, 90, 180, 270):
            raise ValueError(f'rotation must be 0/90/180/270, got {rotation!r}')
        if width <= 0 or height <= 0:
            raise ValueError(f'invalid resolution {width}x{height}')
        if fps not in (5, 15, 30):
            raise ValueError(f'fps must be 5/15/30, got {fps!r}')
        if rs is None:
            raise ImportError(
                'pyrealsense2 is required for the wrist camera driver. '
                'Install it with: pip install pyrealsense2')

        self._enable_compressed = enable_compressed
        if self._enable_compressed and cv2 is None:
            self._enable_compressed = False
            logger.warning(
                'cv2 not available; compressed topics disabled '
                '(pip install opencv-python to enable)')
        self._jpeg_quality = jpeg_quality
        self._depth_max_mm = int(depth_max_m * 1000.0)

        self._side = side
        self._frame_id = f'wrist_{side}'
        # np.rot90 rotates counter-clockwise; k = (4 - degrees/90) % 4 for CW.
        self._rot_k = (4 - rotation // 90) % 4

        self._driver = RealSenseD405Driver(
            serial=serial,
            width=width,
            height=height,
            fps=fps,
            enable_depth=enable_depth,
        )

        self._color_pub = self.create_publisher(
            Image, f'/{topic_prefix}_{side}/color/image_raw', CAMERA_QOS)
        self._depth_pub = None
        if enable_depth:
            self._depth_pub = self.create_publisher(
                Image, f'/{topic_prefix}_{side}/depth/image_raw', CAMERA_QOS)

        self._color_comp_pub = None
        self._depth_comp_pub = None
        if self._enable_compressed:
            self._color_comp_pub = self.create_publisher(
                CompressedImage,
                f'/{topic_prefix}_{side}/color/image_raw/compressed', CAMERA_QOS)
            if enable_depth:
                self._depth_comp_pub = self.create_publisher(
                    CompressedImage,
                    f'/{topic_prefix}_{side}/depth/image_raw/compressedDepth',
                    CAMERA_QOS)

        self.get_logger().info(
            f'Wrist camera ({side}) SN={serial} -> '
            f'/{topic_prefix}_{side}/{{color,depth}}/image_raw  '
            f'{width}x{height}@{fps}FPS rotation={rotation}deg '
            f'compressed={self._enable_compressed}')

    def run(self) -> None:
        """Open the camera and publish frames until shutdown.

        On repeated capture failures (e.g. USB disconnect) the pipeline is
        restarted once; if the device is still gone the node exits so that
        systemd's Restart=on-failure keeps retrying until the camera
        re-enumerates.
        """
        self._driver.start()
        self.get_logger().info(f'Publish loop started for wrist camera {self._side!r}.')

        consecutive_failures = 0
        try:
            while rclpy.ok():
                frames = self._driver.get_frames(timeout_ms=3000)
                if frames is None:
                    consecutive_failures += 1
                    if consecutive_failures >= 5:
                        self.get_logger().error(
                            f'{consecutive_failures} consecutive capture failures '
                            f'for {self._side!r}: attempting pipeline restart')
                        try:
                            self._driver.stop()
                            self._driver.start()
                            consecutive_failures = 0
                            continue
                        except Exception as e:
                            self.get_logger().error(
                                f'Failed to restart camera {self._side!r}: {e}. '
                                'Exiting so systemd can restart the node.')
                            raise
                    # No frame - small sleep to avoid busy-wait on
                    # persistent errors (USB glitch, camera re-enumerating).
                    time.sleep(0.1)
                    continue

                consecutive_failures = 0

                color, depth = frames
                try:
                    if self._rot_k:
                        # np.rot90 returns a non-contiguous view; cv2.imencode
                        # needs C-contiguous input, so copy both planes.
                        color = np.ascontiguousarray(np.rot90(color, self._rot_k))
                        if depth is not None:
                            depth = np.ascontiguousarray(np.rot90(depth, self._rot_k))
                    # All four messages share one stamp so raw and
                    # compressed streams can be matched per frame.
                    stamp = self.get_clock().now().to_msg()
                    self._publish_color(color, stamp)
                    if depth is not None and self._depth_pub is not None:
                        self._publish_depth(depth, stamp)
                    if self._color_comp_pub is not None:
                        self._publish_color_compressed(color, stamp)
                    if (depth is not None and self._depth_comp_pub is not None):
                        self._publish_depth_compressed(depth, stamp)
                except Exception as e:
                    self.get_logger().warning(
                        f'Failed to build/publish message for {self._side!r}: {e}')
        finally:
            self._driver.stop()
            self.get_logger().info(f'Wrist camera {self._side!r} stopped.')

    def _publish_color(self, img: np.ndarray, stamp) -> None:
        height, width = img.shape[:2]
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = height
        msg.width = width
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = img.tobytes()
        self._color_pub.publish(msg)

    def _publish_depth(self, depth: np.ndarray, stamp) -> None:
        height, width = depth.shape[:2]
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = height
        msg.width = width
        msg.encoding = '16UC1'
        msg.is_bigendian = False
        msg.step = width * 2
        msg.data = depth.tobytes()
        self._depth_pub.publish(msg)

    def _publish_color_compressed(self, img: np.ndarray, stamp) -> None:
        """JPEG-encode and publish on <topic>/compressed (image_transport)."""
        ok, buf = cv2.imencode(
            '.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self._color_comp_pub.publish(msg)

    def _publish_depth_compressed(self, depth: np.ndarray, stamp) -> None:
        """Publish <topic>/compressedDepth using the
        compressed_depth_image_transport 16UC1 format:
        a 12-byte ConfigHeader (int32 INV_DEPTH + 2 floats) followed by a
        lossless PNG of the 16-bit depth image. Values above depth_max are
        set to 0 (invalid), matching the C++ plugin's behaviour.
        """
        if np.any(depth > self._depth_max_mm):
            # np.where already returns uint16: no extra copy needed.
            depth = np.where(depth > self._depth_max_mm, 0, depth)
        ok, buf = cv2.imencode(
            '.png', depth, [cv2.IMWRITE_PNG_COMPRESSION, 3])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.format = '16UC1; compressedDepth png'
        # INV_DEPTH(=0) + depthParam[2] (unused for 16-bit input).
        msg.data = struct.pack('<iff', 0, 0.0, 0.0) + buf.tobytes()
        self._depth_comp_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CameraWristDisplayNode()
    except Exception as e:
        print(f'ERROR: {e}', file=__import__('sys').stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    try:
        # Graceful shutdown on SIGTERM (systemd); SIGINT (Ctrl+C) is
        # handled by rclpy and raises KeyboardInterrupt below.
        signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
