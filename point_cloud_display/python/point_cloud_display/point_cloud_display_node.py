#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Point Cloud Display Node (Python version)

This node subscribes to Livox point cloud data and provides:
- Point cloud statistics (point count, bounds)
- Basic filtering options
- Republishing for visualization
"""

import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudDisplayNode(Node):
    """ROS2 Node for point cloud display"""

    def __init__(self):
        super().__init__('point_cloud_display_node')

        # Statistics
        self.total_frames = 0
        self.total_points = 0
        self.last_processing_time_ms = 0

        # Bounds
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        self.min_z = float('inf')
        self.max_z = float('-inf')

        # Declare parameters
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/point_cloud/filtered')
        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('filter_enable', False)
        self.declare_parameter('voxel_leaf_size', 0.05)
        self.declare_parameter('sor_enable', False)
        self.declare_parameter('sor_mean_k', 50)
        self.declare_parameter('sor_stddev_mul_thresh', 1.0)
        self.declare_parameter('publish_interval', 1.0)
        self.declare_parameter('publish_rate', 0.0)

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.filter_enable = self.get_parameter('filter_enable').get_parameter_value().bool_value
        self.voxel_leaf_size = self.get_parameter('voxel_leaf_size').get_parameter_value().double_value
        self.sor_enable = self.get_parameter('sor_enable').get_parameter_value().bool_value
        self.sor_mean_k = self.get_parameter('sor_mean_k').get_parameter_value().integer_value
        self.sor_stddev_mul_thresh = self.get_parameter('sor_stddev_mul_thresh').get_parameter_value().double_value
        publish_interval = self.get_parameter('publish_interval').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._last_publish_time = 0.0

        # Create subscriber
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.point_cloud_callback,
            qos_profile
        )

        # Create publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile
        )

        # Create statistics timer
        self.stats_timer = self.create_timer(
            publish_interval,
            self.log_statistics
        )

        self.get_logger().info('Point Cloud Display Node initialized')
        self.get_logger().info(f'  Input topic: {self.input_topic}')
        self.get_logger().info(f'  Output topic: {self.output_topic}')
        self.get_logger().info(f'  Frame ID: {self.frame_id}')
        self.get_logger().info(f'  Filter enable: {self.filter_enable}')
        if self.filter_enable:
            self.get_logger().info(f'  Voxel leaf size: {self.voxel_leaf_size:.3f}')
        self.get_logger().info(f'  Statistical Outlier Removal: {self.sor_enable}')

    def point_cloud_callback(self, msg):
        """Process incoming point cloud data"""
        import time
        start_time = time.time()

        # Update frame count
        self.total_frames += 1
        input_points = msg.width * msg.height
        self.total_points += input_points

        # Parse point cloud
        points = self._parse_point_cloud(msg)

        # Calculate bounds
        self._update_bounds(points)

        # Apply filtering if enabled
        if self.filter_enable or self.sor_enable:
            points = self._apply_filters(points)

        # Create output message
        output_msg = self._create_point_cloud_msg(points, msg.header.stamp)

        # Throttled publishing: keep newest frame, publish at most publish_rate Hz.
        # publish_rate <= 0 means publish every frame (original behaviour).
        if self.publish_rate > 0:
            now = time.time()
            period = 1.0 / self.publish_rate
            if now - self._last_publish_time >= period:
                self.publisher.publish(output_msg)
                self._last_publish_time = now
        else:
            self.publisher.publish(output_msg)

        # Calculate processing time
        self.last_processing_time_ms = int((time.time() - start_time) * 1000)

    def _parse_point_cloud(self, msg):
        """Parse PointCloud2 to (N,4) float32 [x,y,z,intensity], vectorized.

        PointCloud2.data 是 point_step 紧凑布局，np.frombuffer(offset=..) 只能
        读连续流、无法按步长取字段（会读到错位的字节）。先把整帧 reshape 成
        (N, point_step/4) 的 float32 槽阵，再按各字段 4B 对齐的 offset 取列。
        """
        n = msg.width * msg.height
        if not msg.data or n == 0:
            return np.zeros((0, 4), dtype=np.float32)
        ps = msg.point_step
        if ps % 4 != 0 or len(msg.data) < n * ps:
            return np.zeros((0, 4), dtype=np.float32)
        dtype = '>f4' if msg.is_bigendian else '<f4'
        arr = np.frombuffer(
            msg.data, dtype=dtype, count=n * (ps // 4)).reshape(n, ps // 4)
        fields = {f.name: f for f in msg.fields}
        idx = []
        for name in ('x', 'y', 'z', 'intensity'):
            field = fields.get(name)
            if (field is None
                    or field.datatype != point_cloud2.PointField.FLOAT32
                    or field.offset % 4 != 0
                    or field.offset // 4 >= ps // 4):
                return np.zeros((0, 4), dtype=np.float32)
            idx.append(field.offset // 4)
        pts = np.column_stack([arr[:, i] for i in idx])
        # livox 无效点常是 (0,0,0) 或 ±1e34 / 369.64 等有限乱码；
        # 同时滤除零坐标无效点与 |坐标|>100m 的假点。
        keep = np.isfinite(pts).all(axis=1) \
            & (pts[:, :3] != 0).any(axis=1) \
            & (np.abs(pts[:, :3]) < 100).all(axis=1)
        pts = pts[keep]
        return np.ascontiguousarray(pts, dtype=np.float32)

    def _update_bounds(self, points):
        """Update point cloud bounds"""
        if len(points) == 0:
            return

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # Filter finite values
        mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x = x[mask]
        y = y[mask]
        z = z[mask]

        if len(x) > 0:
            self.min_x = min(self.min_x, float(np.min(x)))
            self.max_x = max(self.max_x, float(np.max(x)))
            self.min_y = min(self.min_y, float(np.min(y)))
            self.max_y = max(self.max_y, float(np.max(y)))
            self.min_z = min(self.min_z, float(np.min(z)))
            self.max_z = max(self.max_z, float(np.max(z)))

    def _apply_filters(self, points):
        """Apply point cloud filters"""
        if len(points) == 0:
            return points

        # Voxel grid filter (simple implementation)
        if self.filter_enable:
            points = self._voxel_grid_filter(points)

        return points

    def _voxel_grid_filter(self, points):
        """Voxel centroid downsampling, fully vectorized.

        每个体素输出其内部点的质心与平均 intensity；原先逐体素 Python 循环
        在 2 万点时单帧 ~0.6s，跟不上 10Hz，改 bincount 聚合。
        """
        if len(points) == 0:
            return points

        leaf = self.voxel_leaf_size
        vox = np.floor(points[:, :3] / leaf).astype(np.int64)
        _, inverse = np.unique(vox, axis=0, return_inverse=True)

        counts = np.bincount(inverse)
        cx = np.bincount(inverse, weights=points[:, 0]) / counts
        cy = np.bincount(inverse, weights=points[:, 1]) / counts
        cz = np.bincount(inverse, weights=points[:, 2]) / counts
        ci = np.bincount(inverse, weights=points[:, 3]) / counts
        return np.ascontiguousarray(
            np.column_stack((cx, cy, cz, ci)), dtype=np.float32)

    def _create_point_cloud_msg(self, points, stamp):
        """Create PointCloud2 message from numpy array"""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(points)

        # Define fields
        msg.fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='intensity', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        # Pack data
        if len(points) > 0:
            msg.data = points.tobytes()

        return msg

    def log_statistics(self):
        """Log statistics"""
        if self.total_frames == 0:
            self.get_logger().info('No point cloud data received yet...')
            return

        avg_points = self.total_points / self.total_frames if self.total_frames > 0 else 0

        self.get_logger().info(
            f'Statistics - Frames: {self.total_frames}, Total Points: {self.total_points}, '
            f'Avg Points/Frame: {avg_points:.0f}'
        )

        if self.min_x != float('inf'):
            self.get_logger().info(
                f'Bounds - X: [{self.min_x:.2f}, {self.max_x:.2f}], '
                f'Y: [{self.min_y:.2f}, {self.max_y:.2f}], '
                f'Z: [{self.min_z:.2f}, {self.max_z:.2f}]'
            )

        self.get_logger().info(f'Last Processing Time: {self.last_processing_time_ms} ms')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()