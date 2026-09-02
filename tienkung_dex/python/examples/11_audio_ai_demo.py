#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""11 语音交互（vendor demo 11 对应：TTS speak / play_file / 语音事件）。

mock:  speak 验证返回值；inject_voice_event 验证 ASR 回调。
real:  speak 后 spin 5 秒观察真实语音事件（对着机器人说话）。

用法:
    python3 examples/11_audio_ai_demo.py
    python3 examples/11_audio_ai_demo.py --backend real --play-file /tmp/a.wav
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class AudioAiDemo(DemoBase):

    def __init__(self, backend='mock', play_file=None):
        super().__init__(backend, enable={'audio'})
        self.play_file = play_file

    def _demo(self):
        events = []
        self.audio.on_voice_event(events.append)

        self.check("speak('你好，我是天工')", self.audio.speak('你好，我是天工'))

        if self.backend == 'mock':
            self.audio.inject_voice_event(1, '你好')   # 1 = ASR 识别事件
        else:
            print('  （real 模式：请对机器人说一句话，观察 5 秒）')
            self.spin_for(self._observe)

        self.check('on_voice_event 回调', bool(events),
                   f'{len(events)} 个事件' +
                   (f'，最后一个: {events[-1].get("text")!r}'
                    if events else ''))

        if self.play_file is not None:
            self.check(f'play_file({self.play_file!r})',
                       self.audio.play_file(self.play_file))


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--play-file', help='播放的音频文件路径')
    args = parser.parse_args(argv)
    return AudioAiDemo(backend=args.backend,
                       play_file=args.play_file).run(observe_seconds=5.0)


if __name__ == '__main__':
    sys.exit(main())
