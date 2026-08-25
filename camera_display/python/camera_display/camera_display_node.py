#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Display Node (Python version)

Simultaneously displays RGB and depth images from both head and waist cameras.
Supports depth colormap, histogram, and statistics overlay.
"""

import threading
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


@dataclass
class CameraData:
    """Holds image data and statistics for one camera."""
    label: str
    depth_image: Optional[np.ndarray] = None
    colored_depth: Optional[np.ndarray] = None
    color_image: Optional[np.ndarray] = None
    min_depth_value: int = 0
    max_depth_value: int = 0
    mean_depth_value: float = 0.0
    valid_pixel_count: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)


class CameraDisplayNode(Node):
    """ROS2 Node for dual-camera (head + waist) image display"""

    COLORMAP_CODES = {
        0: None,  # GRAY
        1: cv2.COLORMAP_JET,
        2: cv2.COLORMAP_RAINBOW,
        3: cv2.COLORMAP_TURBO,
    }

    HEAD_NS = 'ob_camera_head'
    WAIST_NS = 'ob_camera_waist'

    def __init__(self):
        super().__init__('camera_display_node')

        # Declare parameters
        self.declare_parameter('colormap', 2)
        self.declare_parameter('max_depth', 5000.0)
        self.declare_parameter('min_depth', 0.0)
        self.declare_parameter('display_scale', 0.5)
        self.declare_parameter('show_histogram', True)
        self.declare_parameter('show_statistics', True)
        self.declare_parameter('enable_head', True)
        self.declare_parameter('enable_waist', True)

        self.colormap = self.get_parameter('colormap').get_parameter_value().integer_value
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value
        self.display_scale = self.get_parameter('display_scale').get_parameter_value().double_value
        self.show_histogram = self.get_parameter('show_histogram').get_parameter_value().bool_value
        self.show_statistics = self.get_parameter('show_statistics').get_parameter_value().bool_value
        self.enable_head = self.get_parameter('enable_head').get_parameter_value().bool_value
        self.enable_waist = self.get_parameter('enable_waist').get_parameter_value().bool_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cameras: dict[str, CameraData] = {}

        if self.enable_head:
            self._setup_camera(self.HEAD_NS, 'Head', qos)
        if self.enable_waist:
            self._setup_camera(self.WAIST_NS, 'Waist', qos)

        if not self.cameras:
            self.get_logger().error('No camera enabled! Set enable_head or enable_waist to true.')
            return

        self.display_running = True
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

        self.get_logger().info('Camera Display Node initialized')
        for ns, cam in self.cameras.items():
            self.get_logger().info(f'  {cam.label} camera:')
            self.get_logger().info(f'    Depth: {ns}/depth/image_raw')
            self.get_logger().info(f'    Color: {ns}/color/image_raw')
        self.get_logger().info(f'  Colormap: {self.colormap}')
        self.get_logger().info(f'  Depth range: {self.min_depth:.0f} - {self.max_depth:.0f} mm')

    def _setup_camera(self, namespace, label, qos):
        cam = CameraData(label=label)
        self.cameras[namespace] = cam

        self.create_subscription(
            Image, f'{namespace}/depth/image_raw',
            lambda msg, c=cam: self._depth_callback(msg, c), qos
        )
        self.create_subscription(
            Image, f'{namespace}/color/image_raw',
            lambda msg, c=cam: self._color_callback(msg, c), qos
        )

    def _depth_callback(self, msg, cam: CameraData):
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            elif msg.encoding == '32FC1':
                depth_float = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
                depth = (depth_float * 1000).astype(np.uint16)
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
                return

            with cam.lock:
                cam.depth_image = depth.copy()
                self._update_depth_statistics(depth, cam)
                cam.colored_depth = self._apply_colormap(depth)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image ({cam.label}): {e}')

    def _color_callback(self, msg, cam: CameraData):
        try:
            if msg.encoding in ('rgb8', 'RGB8'):
                color = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            elif msg.encoding in ('bgr8', 'BGR8'):
                color = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding in ('rgba8', 'RGBA8'):
                color = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                color = cv2.cvtColor(color, cv2.COLOR_RGBA2BGR)
            elif msg.encoding in ('bgra8', 'BGRA8'):
                color = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                color = cv2.cvtColor(color, cv2.COLOR_BGRA2BGR)
            else:
                self.get_logger().warn(f'Unsupported color encoding: {msg.encoding}')
                return

            with cam.lock:
                cam.color_image = color.copy()

        except Exception as e:
            self.get_logger().error(f'Error processing color image ({cam.label}): {e}')

    @staticmethod
    def _update_depth_statistics(depth, cam: CameraData):
        mask = depth > 0
        if not np.any(mask):
            cam.min_depth_value = 0
            cam.max_depth_value = 0
            cam.mean_depth_value = 0.0
            cam.valid_pixel_count = 0
            return
        cam.min_depth_value = int(np.min(depth[mask]))
        cam.max_depth_value = int(np.max(depth[mask]))
        cam.mean_depth_value = float(np.mean(depth[mask]))
        cam.valid_pixel_count = int(np.sum(mask))

    def _apply_colormap(self, depth):
        normalized = np.clip(
            (depth - self.min_depth) * 255 / (self.max_depth - self.min_depth),
            0, 255
        ).astype(np.uint8)

        colormap_code = self.COLORMAP_CODES.get(self.colormap, cv2.COLORMAP_JET)
        if colormap_code is None:
            colored = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        else:
            colored = cv2.applyColorMap(normalized, colormap_code)

        colored[depth == 0] = [0, 0, 0]
        return colored

    def _create_histogram(self, depth):
        mask = depth > 0
        if not np.any(mask):
            return None

        hist = cv2.calcHist([depth], [0], mask.astype(np.uint8), [256],
                            [self.min_depth, self.max_depth])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)

        hist_h, hist_w = 200, 512
        hist_image = np.zeros((hist_h, hist_w, 3), dtype=np.uint8)
        bin_w = hist_w // 256

        for i in range(1, 256):
            cv2.line(
                hist_image,
                (bin_w * (i - 1), hist_h - int(hist[i - 1] * hist_h)),
                (bin_w * i, hist_h - int(hist[i] * hist_h)),
                (255, 255, 255), 2
            )

        label = f'Range: {int(self.min_depth)}-{int(self.max_depth)} mm'
        cv2.putText(hist_image, label, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return hist_image

    def display_loop(self):
        # Create windows for enabled cameras
        for cam in self.cameras.values():
            cv2.namedWindow(f'{cam.label} - Depth', cv2.WINDOW_NORMAL)
            cv2.namedWindow(f'{cam.label} - Color', cv2.WINDOW_NORMAL)

        while self.display_running and rclpy.ok():
            for cam in self.cameras.values():
                with cam.lock:
                    if cam.colored_depth is not None:
                        depth_display = cam.colored_depth.copy()

                        if self.show_statistics:
                            cv2.putText(depth_display, f'Min: {cam.min_depth_value} mm',
                                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                            cv2.putText(depth_display, f'Max: {cam.max_depth_value} mm',
                                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                            cv2.putText(depth_display, f'Mean: {int(cam.mean_depth_value)} mm',
                                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                            cv2.putText(depth_display, f'Valid: {cam.valid_pixel_count}',
                                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                        resized = cv2.resize(depth_display, None,
                                             fx=self.display_scale, fy=self.display_scale)
                        cv2.imshow(f'{cam.label} - Depth', resized)

                    if self.show_histogram and cam.depth_image is not None:
                        hist = self._create_histogram(cam.depth_image)
                        if hist is not None:
                            cv2.imshow(f'{cam.label} - Depth Histogram', hist)

                    if cam.color_image is not None:
                        resized = cv2.resize(cam.color_image, None,
                                             fx=self.display_scale, fy=self.display_scale)
                        cv2.imshow(f'{cam.label} - Color', resized)

            key = cv2.waitKey(33)
            if key == 27 or key == ord('q'):
                rclpy.shutdown()
                break
            elif key == ord('c'):
                self.colormap = (self.colormap + 1) % 4
                self.get_logger().info(f'Colormap changed to: {self.colormap}')
            elif key == ord('h'):
                self.show_histogram = not self.show_histogram
                self.get_logger().info(f'Histogram display: {"ON" if self.show_histogram else "OFF"}')
            elif key == ord('s'):
                self.show_statistics = not self.show_statistics
                self.get_logger().info(f'Statistics display: {"ON" if self.show_statistics else "OFF"}')

    def destroy_node(self):
        self.display_running = False
        if hasattr(self, 'display_thread') and self.display_thread.is_alive():
            self.display_thread.join(timeout=1.0)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
