#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""10 TTS 文字合成播报（对应 ROS 包 play_text_demo 的 text 能力）。

mock:  speak 返回值检查（不实际出声）。
real:  speak(blocking=True) 等服务端接受，随后保留出声窗口确认真正发声
       （服务端响应到达 != 音频播放完毕，过早 shutdown 会掐断播放）。

用法:
    python3 examples/10_play_text_demo.py
    python3 examples/10_play_text_demo.py --backend real --text 你好我是天工 --play-wait 6
    python3 examples/10_play_text_demo.py --backend mock
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class PlayTextDemo(DemoBase):

    def __init__(self, backend='real', text='你好，我是天工', play_wait=5.0):
        super().__init__(backend, enable={'audio'})
        self.text = text
        self.play_wait = play_wait

    def _demo(self):
        blocking = self.backend != 'mock'
        ok = self.audio.speak(self.text, blocking=blocking, timeout=10.0)
        self.check(f"speak({self.text!r}, blocking={blocking})", ok,
                   '服务端接受播报')

        if ok and self.backend != 'mock':
            print(f'  （TTS 播放中，请听机器人是否出声… {self.play_wait:.0f}s）')
            self.spin_for(self.play_wait)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--text', default='你好，我是天工',
                        help='要合成的文字')
    parser.add_argument('--play-wait', type=float, default=5.0,
                        help='real/sim 出声确认窗口秒数')
    args = parser.parse_args(argv)
    return PlayTextDemo(backend=args.backend, text=args.text,
                        play_wait=args.play_wait).run()


if __name__ == '__main__':
    sys.exit(main())
