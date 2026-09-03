#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""09 相机图像（4 路 RGB-D + 可选 6 路全景）＋可选实时预览。

mock:  对每路相机 publish_frame ×2，验证 latest() / frame_rate()。
real:  spin 5 秒统计各路帧数；--panorama 附加 6 路全景相机。数据验证
       通过后若给 --view，进入内嵌 OpenCV 多窗口实时预览（对齐仓库
       camera_display 方案，非 RViz）：
         - 每路独立 {label} - Color / {label} - Depth 窗口；wrist 近距
           D405 用 100-600mm 独立深度量程，head/waist RGB-D 用 0-5000mm；
         - 深度图带伪彩（灰/JET/RAINBOW/TURBO）与直方图、统计叠加；
         - q/ESC 退出预览，c/h/s 切换伪彩/直方图/统计，随后 demo 照常
           输出健康报告。

用法:
    python3 examples/09_camera_demo.py
    python3 examples/09_camera_demo.py --backend real --view
    python3 examples/09_camera_demo.py --panorama
    python3 examples/09_camera_demo.py --backend real --view --panorama
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


# camera_display 风格的路名 -> 窗口标签。
_CAMERA_LABELS = {
    'ob_camera_head': 'Head',
    'ob_camera_waist': 'Waist',
    'ob_camera_wrist_left': 'Wrist Left',
    'ob_camera_wrist_right': 'Wrist Right',
}
# 深度显示量程（mm）：腕部 D405 为近距相机（~0.1-0.5m），独立量程防深度
# 视图全黑；与仓库 camera_display 的默认参数保持一致。
_RGBD_DEPTH_RANGE = (0.0, 5000.0)
_WRIST_DEPTH_RANGE = (100.0, 600.0)


class CameraDemo(DemoBase):

    def __init__(self, backend='real', panorama=False, view=False):
        enable = {'camera', 'panorama'} if panorama else {'camera'}
        super().__init__(backend, enable=enable)
        # 预览需要真实订阅数据流：mock/sim 无 ROS 图或无意义，直接忽略。
        self._view = bool(view and backend == 'real')

    def _demo(self):
        import numpy as np
        from tienkung_dex import CameraFrame

        def make_frame(ns):
            return CameraFrame(color=np.zeros((4, 4, 3), dtype=np.uint8),
                               depth=None, frame_id=ns)

        if self.backend == 'mock':
            for ns, camera in self.cameras.items():
                for _ in range(2):        # 两帧才能算出 frame_rate
                    camera.publish_frame(make_frame(ns))
                frame = camera.latest()
                ok = frame is not None and frame.frame_id == ns
                self.check(f'{ns} latest()', ok)
                self.check(f'{ns} frame_rate()', camera.frame_rate is not None,
                           f'{camera.frame_rate:.1f} fps'
                           if camera.frame_rate else '')
            if self.panorama is not None:
                for idx in self.panorama.indices:
                    self.panorama.publish_frame(idx, make_frame(f'cam{idx}'))
                    self.check(f'panorama[{idx}] latest()',
                               self.panorama.latest(idx) is not None)
        else:
            self.spin_for(self._observe)
            for ns, camera in self.cameras.items():
                frame = camera.latest()
                rate = camera.frame_rate
                self.check(f'{ns} 收到图像', frame is not None,
                           f'{rate:.1f} fps' if rate else '无帧')
            if self.panorama is not None:
                for idx in self.panorama.indices:
                    self.check(f'panorama[{idx}] latest()',
                               self.panorama.latest(idx) is not None)
            # 数据链路通过（至少一路有帧）才值得弹预览；无帧说明发布端没通，
            # 窗口里全是空画面没有意义。
            if self._view and any(cam.latest() is not None
                                  for cam in self.cameras.values()):
                self._open_preview()

    # -- --view：内嵌 OpenCV 多窗口预览（对齐仓库 camera_display 方案） -------
    def _open_preview(self) -> None:
        """阻塞式实时预览 4 路相机的 Color/Depth，直到 q/ESC 退出。

        循环内持续 spin_once 喂订阅，直接从各路 latest() 的 CameraFrame 取
        BGR 彩色与毫米深度渲染，无需 cv_bridge 二次订阅。运行结束后返回
        run() 收尾（健康快照/shutdown 不受影响）。
        """
        import os
        if self._node is None:
            return
        try:
            import cv2
        except ImportError:
            print('  --view: 未安装 opencv-python（cv2），跳过预览')
            return
        import numpy as np

        cameras = self.cameras
        if not any(cam.latest() is not None for cam in cameras.values()):
            print('  --view: 未收到任何相机帧，跳过预览')
            return
        if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
            print('  --view: 无图形界面（DISPLAY 未设置）。本地终端请先'
                  ' export DISPLAY=:0，SSH 请用 -X 转发或 VNC 后重试')
            return

        # 颜色映射序号与 camera_display 一致：0 灰 / 1 JET / 2 RAINBOW /
        # 3 TURBO，按 'c' 轮换。
        colormap = 2
        show_hist = True
        show_stat = True
        scale = 0.5
        settings_seq = 0

        # 每个相机一个渲染状态；窗口内容只在对应平面更新或设置变化时重绘。
        state = {}
        for ns, cam in cameras.items():
            mn, mx = (_WRIST_DEPTH_RANGE if 'wrist' in ns
                      else _RGBD_DEPTH_RANGE)
            state[ns] = {
                'label': _CAMERA_LABELS.get(ns, ns),
                'min_d': mn,
                'max_d': mx,
                # color/depth 各自由独立缓存驱动：SDK 配对快照可能因两流
                # 不同步而交替为单平面帧，但缓存只记录“真实到达”的平面，
                # 缺失一侧保留最近画面，不会被顶成 no color / no depth。
                'color': None,               # 最近收到的 color 平面（判新用 is）
                'depth': None,               # 最近收到的 depth 平面
                'color_shown': None,         # 已绘制到窗口的 color 平面
                'depth_shown': None,
                'color_disp': None,
                'depth_disp': None,
                'hist_disp': None,
                'no_color': None,            # 占位黑板（从未收到 color 平面时）
                'no_depth': None,            # 占位黑板（从未收到 depth 平面时）
                'no_frame': None,            # 占位黑板（该路从未收到帧时）
                'shown_seq': -1,
            }
        for st in state.values():
            cv2.namedWindow(f"{st['label']} - Color", cv2.WINDOW_NORMAL)
            cv2.namedWindow(f"{st['label']} - Depth", cv2.WINDOW_NORMAL)

        def blank(text: str, width: int = 640, height: int = 360):
            canvas = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(canvas, text, (20, height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1)
            return canvas

        def colorize(depth, st):
            """毫米深度 -> BGR 伪彩（量程外 clip，无效值 0 显示为黑）。"""
            mn, mx = st['min_d'], st['max_d']
            norm = np.clip((depth - mn) * 255.0 / (mx - mn), 0, 255
                           ).astype(np.uint8)
            code = (None, cv2.COLORMAP_JET, cv2.COLORMAP_RAINBOW,
                    cv2.COLORMAP_TURBO)[colormap]
            if code is None:
                out = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
            else:
                out = cv2.applyColorMap(norm, code)
            out[depth == 0] = (0, 0, 0)
            return out

        def overlay_stats(depth_disp, depth, st):
            """统计叠加沿用 camera_display：Min/Max/Mean/Valid（有效=非零）。"""
            mask = depth > 0
            if np.any(mask):
                lines = (f'Min: {int(np.min(depth[mask]))} mm',
                         f'Max: {int(np.max(depth[mask]))} mm',
                         f'Mean: {int(np.mean(depth[mask]))} mm',
                         f'Valid: {int(np.sum(mask))}')
            else:
                lines = ('Min: 0 mm', 'Max: 0 mm', 'Mean: 0 mm', 'Valid: 0')
            y = 30
            for line in lines:
                cv2.putText(depth_disp, line, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                y += 30
            return depth_disp

        def make_hist(depth, st):
            mask = depth > 0
            if not np.any(mask):
                return None
            # calcHist 返回 (256,1) 列向量，绘制前展平为一维，否则 hist[i]
            # 是单元素数组，int() 转标量会触发 NumPy 1.25+ 的
            # DeprecationWarning（并将在未来版本报错）。
            hist = cv2.calcHist([depth], [0], mask.astype(np.uint8), [256],
                                [st['min_d'], st['max_d'] + 1]).ravel()
            cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
            hist_h, hist_w = 200, 512
            img = np.zeros((hist_h, hist_w, 3), dtype=np.uint8)
            bin_w = hist_w // 256
            for i in range(1, 256):
                cv2.line(img,
                         (bin_w * (i - 1),
                          hist_h - int(hist[i - 1] * hist_h)),
                         (bin_w * i, hist_h - int(hist[i] * hist_h)),
                         (255, 255, 255), 2)
            cv2.putText(img, f'Range: {int(st["min_d"])}-{int(st["max_d"])} mm',
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)
            return img

        print('  --view: OpenCV 预览已启动'
              '（q/ESC 退出，c 切换伪彩，h 直方图，s 统计）')
        try:
            while True:
                # 预览期间保持订阅回调运转，latest() 才会持续更新。
                self._rclpy.spin_once(self._node, timeout_sec=0.03)
                for ns, cam in cameras.items():
                    st = state[ns]
                    frame = cam.latest()
                    if frame is None:
                        # 该路从未收到帧：双窗口都显示提示而不是黑屏。
                        if st['no_frame'] is None:
                            st['no_frame'] = blank(f'{st["label"]}: no frame')
                        cv2.imshow(f"{st['label']} - Color", st['no_frame'])
                        cv2.imshow(f"{st['label']} - Depth", st['no_frame'])
                        continue
                    # 独立缓存各自平面：配对快照可能因两流时间差超出
                    # pair_window 而交替为 color-only / depth-only，这里只把
                    # 真实到达的平面收进缓存，未更新的那一侧画面保持不变。
                    if frame.color is not None:
                        st['color'] = frame.color
                    if frame.depth is not None:
                        st['depth'] = frame.depth
                    color, depth = st['color'], st['depth']
                    settings_dirty = settings_seq != st['shown_seq']

                    if color is None:
                        # 该路从未收到 color：占位提示（此后有图则不再出现）。
                        if st['no_color'] is None:
                            st['no_color'] = blank(f'{st["label"]}: no color')
                        cv2.imshow(f"{st['label']} - Color", st['no_color'])
                    elif color is not st['color_shown']:
                        st['color_shown'] = color
                        st['color_disp'] = cv2.resize(
                            color, None, fx=scale, fy=scale)
                        cv2.imshow(f"{st['label']} - Color", st['color_disp'])

                    if depth is None:
                        # 该路从未收到 depth：占位提示；收到过的帧会一直保留
                        # 在窗口里直到新的 depth 平面到达再刷新。
                        if st['no_depth'] is None:
                            st['no_depth'] = blank(f'{st["label"]}: no depth')
                        cv2.imshow(f"{st['label']} - Depth", st['no_depth'])
                        st['hist_disp'] = None
                    elif depth is not st['depth_shown'] or settings_dirty:
                        st['depth_shown'] = depth
                        st['shown_seq'] = settings_seq
                        disp = colorize(depth, st)
                        if show_stat:
                            disp = overlay_stats(disp, depth, st)
                        st['depth_disp'] = cv2.resize(
                            disp, None, fx=scale, fy=scale)
                        cv2.imshow(f"{st['label']} - Depth", st['depth_disp'])
                        if show_hist:
                            st['hist_disp'] = make_hist(depth, st)
                        else:
                            st['hist_disp'] = None

                    if show_hist and st['hist_disp'] is not None:
                        cv2.imshow(f"{st['label']} - Depth Histogram",
                                   st['hist_disp'])
                key = cv2.waitKey(33) & 0xFF
                if key in (27, ord('q')):
                    break
                elif key == ord('c'):
                    colormap = (colormap + 1) % 4
                    settings_seq += 1
                    print(f'  --view: 伪彩 -> {("灰", "JET", "RAINBOW",
                                               "TURBO")[colormap]}')
                elif key == ord('h'):
                    show_hist = not show_hist
                    settings_seq += 1
                    print(f'  --view: 直方图 {"开" if show_hist else "关"}')
                elif key == ord('s'):
                    show_stat = not show_stat
                    settings_seq += 1
                    print(f'  --view: 统计 {"开" if show_stat else "关"}')
        finally:
            cv2.destroyAllWindows()


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--panorama', action='store_true',
                        help='附加 6 路全景相机（可选硬件）')
    parser.add_argument('--view', action='store_true',
                        help='real：数据验证通过后内嵌 OpenCV 窗口实时预览'
                             '各相机（q/ESC 退出，c 切伪彩，h 直方图，s 统计）')
    args = parser.parse_args(argv)
    return CameraDemo(backend=args.backend,
                      panorama=args.panorama,
                      view=args.view).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
