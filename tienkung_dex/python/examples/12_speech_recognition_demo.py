#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""12 语音识别事件（对应 ROS 包 speech_recognition_demo：/lyre/voice_activity）。

订阅 /lyre/voice_activity 并解析 aiui_event，事件类型：
    1=ASR 识别结果  4=关键词唤醒  5=退出对话  6=VAD  20=人脸识别唤醒
mock:  inject_voice_event 注入上述多种事件，核对解析出的 event_type/name。
real:  spin 观察窗口：先等唤醒事件（face_wake=20 / keyword_wake=4）进入
       对话，再对机器人说测试语；期望观察到 ASR 识别文本（event_type=1
       且 text 非空）。注意：唤醒前的说话只产生 VAD；若事件 content 含
       info=mock_* 字段则是模拟帧，非真人识别结果。

用法:
    python3 examples/12_speech_recognition_demo.py
    python3 examples/12_speech_recognition_demo.py --backend real --observe 20
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class SpeechRecognitionDemo(DemoBase):

    def __init__(self, backend='real'):
        super().__init__(backend, enable={'audio'})

    def _demo(self):
        events = []
        state = {'mock_hit': 0}

        def on_event(ev):
            events.append(ev)
            content = (ev.get('raw') or {}).get('content') or {}
            if str(content.get('info', '')).startswith('mock'):
                state['mock_hit'] += 1
            et = ev.get('event_type')
            if et in (4, 20):
                print(f'  [唤醒] {ev.get("name")} — 已进入对话，请说话')
            elif et == 1 and ev.get('text'):
                print(f"  [ASR] 识别到: {ev['text']!r}")

        self.audio.on_voice_event(on_event)

        if self.backend == 'mock':
            # 注入各类事件，覆盖解析器分支（含 name 映射）
            self.audio.inject_voice_event(1, '你好')      # ASR 文本
            self.audio.inject_voice_event(4, '', angle=30)  # 关键词唤醒
            self.audio.inject_voice_event(5)              # 退出对话
            self.audio.inject_voice_event(6)              # VAD
            self.audio.inject_voice_event(20)             # 人脸唤醒
        else:
            print('  （观察中… 先触发唤醒进入对话（关键词 / 人脸），')
            print('   唤醒成功后再对机器人说测试语）')
            self.spin_for(self._observe)

        asr = [e for e in events if e.get('event_type') == 1
               and e.get('text')]
        detail = f'{len(events)} 个事件'
        if events:
            names = [f"{e.get('event_type')}:{e.get('name')}"
                     for e in events]
            detail += ' [' + ', '.join(names) + ']'
        if asr:
            detail += f'，识别到: {asr[-1]["text"]!r}'
        self.check('on_voice_event 事件解析', bool(events), detail)

        if state['mock_hit'] and self.backend != 'mock':
            print('  [警告] 收到模拟帧(info=mock_*)，可能非真人识别结果；'
                  '请确认语音服务处于真实可用状态')
        # ASR 文本到达是识别链路真正出结果（real 下需要唤醒+说话）
        self.check('收到 ASR 识别文本', bool(asr),
                   f'识别到: {asr[-1]["text"]!r}' if asr else
                   '未收到 event_type=1 文本（确认已唤醒并说话）')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='real',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--observe', type=float, default=20.0,
                        help='real/sim 观察窗口秒数（需覆盖唤醒+说话全过程）')
    args = parser.parse_args(argv)
    return SpeechRecognitionDemo(backend=args.backend
                                 ).run(observe_seconds=args.observe)


if __name__ == '__main__':
    sys.exit(main())
