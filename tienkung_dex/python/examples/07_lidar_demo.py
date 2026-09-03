#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""07 激光雷达点云（livox PointCloud2 透传）＋可选 RViz 实时预览。

mock:  inject 一个占位对象，验证回调与 latest()。
real:  spin 5 秒订阅 /livox/lidar。数据验证通过后若给了 --view，
       自动拉起 rviz2 预览点云：
         - 配置文件默认 examples/07_lidar.rviz（结构对齐
           point_cloud_display/python/rviz/point_cloud.rviz 与 livox 驱动自带
           display_point_cloud_ROS2.rviz：Topic 平面键 + Intensity 着色 +
           Fixed Frame 放在 Global Options 下）；
         - 配置缺失时按点云实际 frame_id 自动生成同构模板；该文件可随时用
           rviz2 里 File → Save Config As 覆盖微调，无需改 demo 代码。

用法:
    python3 examples/07_lidar_demo.py
    python3 examples/07_lidar_demo.py --backend real --view   # 验证后弹 RViz 点云
    python3 examples/07_lidar_demo.py --view --rviz-config /path/to/any.rviz
"""

import os
import shutil
import subprocess
import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


# SDK 默认订阅的 livox 点云话题（与 core/topics.py 一致）。
_POINTCLOUD_TOPIC = '/livox/lidar'


class LidarDemo(DemoBase):

    def __init__(self, backend='real', view=False, rviz_config=None):
        super().__init__(backend, enable={'lidar'})
        self._view = bool(view and backend == 'real')
        self._rviz_config = rviz_config or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), '07_lidar.rviz')

    def _demo(self):
        clouds = []
        self.lidar.on_cloud(clouds.append)

        if self.backend == 'mock':
            self.lidar.inject(object())    # 点云消息对 SDK 是不透明的
        else:
            self.spin_for(self._observe)

        latest = self.lidar.latest()
        ok = self.check('latest() 非空', latest is not None)
        self.check('on_cloud 回调', len(clouds) >= 1, f'{len(clouds)} 帧')
        if ok and latest is not None:
            self._maybe_preview(latest)

    def _maybe_preview(self, cloud) -> None:
        """有真实点云后按配置拉起 rviz2 预览（默认不弹，需 --view）。

        配置文件缺失时按实际点云 frame_id 自动生成一份最小可用模板
        （examples/07_lidar.rviz），之后再手工微调亦可被覆盖。
        """
        if not self._view:
            return
        if shutil.which('rviz2') is None:
            print('  --view: 未找到 rviz2（不在 PATH），跳过预览')
            return
        if not os.path.isfile(self._rviz_config):
            frame = getattr(getattr(cloud, 'header', None), 'frame_id',
                            '') or 'livox_frame'
            try:
                self._write_default_rviz_config(frame)
                print(f'  --view: 已自动生成 RViz 配置: {self._rviz_config}'
                      f'（Fixed Frame={frame}）')
            except OSError as exc:
                print(f'  --view: 写配置失败: {exc}，跳过预览')
                return
        try:
            proc = subprocess.Popen(['rviz2', '-d', self._rviz_config])
        except OSError as exc:
            print(f'  --view: 启动 rviz2 失败: {exc}')
            return
        print(f'  RViz 预览已拉起 (pid={proc.pid})，cfg={self._rviz_config}')
        print('    RViz 为独立进程，demo 结束不会关闭它；在 RViz 中关闭即退出。')

    def _write_default_rviz_config(self, frame_id: str) -> None:
        """生成与仓库内 examples/07_lidar.rviz 同构的最小配置。

        结构要点（对齐 point_cloud_display 的参考配置）：
          - 点云 display 的 Topic 是平面字符串键；
          - Fixed Frame 位于 Global Options 下；
          - 默认 QoS（Unreliable: false，与发布端匹配），不写嵌套策略。
        """
        content = (
            'Visualization Manager:\n'
            '  Displays:\n'
            '    - Alpha: 1\n'
            '      Autocompute Intensity Bounds: true\n'
            '      Axis: Z\n'
            '      Channel Name: intensity\n'
            '      Class: rviz_default_plugins/PointCloud2\n'
            '      Color: 255; 255; 255\n'
            '      Color Transformer: Intensity\n'
            '      Decay Time: 0.2\n'
            '      Enabled: true\n'
            '      Invert Rainbow: false\n'
            '      Max Color: 255; 255; 255\n'
            '      Max Intensity: 255\n'
            '      Min Color: 0; 0; 0\n'
            '      Min Intensity: 0\n'
            '      Name: PointCloud2 (livox)\n'
            '      Position Transformer: XYZ\n'
            '      Queue Size: 10\n'
            '      Selectable: true\n'
            '      Size (Pixels): 3\n'
            '      Size (m): 0.01\n'
            '      Style: Flat Squares\n'
            f'      Topic: {_POINTCLOUD_TOPIC}\n'
            '      Unreliable: false\n'
            '      Use Fixed Frame: true\n'
            '      Use rainbow: true\n'
            '      Value: true\n'
            '  Enabled: true\n'
            '  Global Options:\n'
            '    Background Color: 48; 48; 48\n'
            f'    Fixed Frame: {frame_id}\n'
            '    Frame Rate: 30\n'
            '  Name: root\n'
            '  Value: true\n'
            '  Views:\n'
            '    Current:\n'
            '      Class: rviz_default_plugins/Orbit\n'
            '      Distance: 15\n'
            '      Focal Point:\n'
            '        X: 0\n'
            '        Y: 0\n'
            '        Z: 0\n'
            '      Name: Current View\n'
            '      Near Clip Distance: 0.009999999776482582\n'
            '      Pitch: 0.9503982067108154\n'
            '      Target Frame: <Fixed Frame>\n'
            '      Value: Orbit (rviz)\n'
            '      Yaw: 2.6603963375091553\n'
            '    Saved: ~\n'
        )
        with open(self._rviz_config, 'w', encoding='utf-8') as fh:
            fh.write(content)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--view', action='store_true',
                        help='real：数据验证通过后自动拉起 rviz2 预览点云')
    parser.add_argument('--rviz-config', default=None,
                        help='--view 使用的 RViz 配置（默认同目录 07_lidar.rviz）')
    args = parser.parse_args(argv)
    return LidarDemo(backend=args.backend, view=args.view,
                     rviz_config=args.rviz_config).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
