#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""11 TTS 播放控制（对应 ROS 包 speaker_play_demo：speak / play_file / stop）。

speaker_play_demo 支持 text/file/url 三种资源 + stop/query 控制；SDK 侧
只封装了 text/file/append 与 cmd=stop（url/query 未暴露），故本示例覆盖：
- speak(text)            -> type=text,  cmd=append
- play_file(path)        -> type=file,  cmd=append
- stop_playback()        -> cmd=stop（终止当前队列播放）

mock:  三个调用均返回桩结果，check 返回值。
real:  speak 出声 ->（可选）play_file 播放文件 -> stop_playback 停止。

用法:
    python3 examples/11_speaker_play_demo.py
    python3 examples/11_speaker_play_demo.py --backend real --play-file /tmp/a.wav
    python3 examples/11_speaker_play_demo.py --backend real --text '' --play-file /tmp/a.wav
"""

import os
import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class SpeakerPlayDemo(DemoBase):

    def __init__(self, backend='real', text='你好，我是天工',
                 play_file=None, play_wait=5.0):
        super().__init__(backend, enable={'audio'})
        self.text = text
        self.play_file = play_file
        self.play_wait = play_wait

    def _demo(self):
        blocking = self.backend != 'mock'

        # 1) 文字合成（type=text, cmd=append）
        if self.text:
            ok = self.audio.speak(self.text, blocking=blocking, timeout=10.0)
            self.check(f"speak({self.text!r})", ok, '服务端接受播报')
            if ok and self.backend != 'mock':
                print(f'  （TTS 播放中… {self.play_wait:.0f}s）')
                self.spin_for(self.play_wait)

        # 2) 本地文件播放（type=file, cmd=append）
        if self.play_file is not None:
            if os.path.exists(self.play_file):
                ok = self.audio.play_file(self.play_file,
                                          blocking=blocking, timeout=10.0)
                self.check(f"play_file({self.play_file!r})", ok,
                           '服务端接受文件播报')
                if ok and self.backend != 'mock':
                    print(f'  （文件播放中… {self.play_wait:.0f}s）')
                    self.spin_for(self.play_wait)
            else:
                self.check(f'play_file({self.play_file!r}) 文件存在',
                           False, '文件不存在，跳过播报')

        # 3) 停止播放（cmd=stop）
        self.check('stop_playback()', self.audio.stop_playback(),
                   '终止当前播放队列')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--text', default='你好，我是天工',
                        help='要合成的文字（空串则跳过 speak）')
    parser.add_argument('--play-file', default=None,
                        help='要播放的本地音频文件路径')
    parser.add_argument('--play-wait', type=float, default=5.0,
                        help='real/sim 出声确认窗口秒数')
    args = parser.parse_args(argv)
    return SpeakerPlayDemo(backend=args.backend, text=args.text,
                           play_file=args.play_file,
                           play_wait=args.play_wait).run()


if __name__ == '__main__':
    sys.exit(main())
