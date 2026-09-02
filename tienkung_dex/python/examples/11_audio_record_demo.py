#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""11 语音录制（项目特有：start/stop_recording + AudioRingBuffer 快照）。

mock:  注入 3 个合成音频块，验证录制时长合计。
real:  录制 --seconds 秒后停止；--out 用 stdlib wave 写 16-bit PCM 单声道。

用法:
    python3 examples/11_audio_record_demo.py
    python3 examples/11_audio_record_demo.py --out /tmp/record.wav
    python3 examples/11_audio_record_demo.py --backend real --seconds 5 --out /tmp/record.wav
"""

import sys

try:
    from ._demo_base import DemoBase
except ImportError:                       # 直接脚本运行（无包上下文）
    from _demo_base import DemoBase


class AudioRecordDemo(DemoBase):

    def __init__(self, backend='mock', seconds=5.0, out=None):
        super().__init__(backend, enable={'audio'})
        self.seconds = seconds
        self.out = out

    def _demo(self):
        self.check('start_recording()', self.audio.start_recording())

        if self.backend == 'mock':
            from tienkung_dex import AudioChunk
            # 8000 Hz · 16-bit · 单声道，每块 16000 字节 = 1 秒
            for _ in range(3):
                self.audio.inject_audio_chunk(AudioChunk(
                    data=b'\x00\x00' * 8000, sample_rate=8000,
                    channels=1, bits_per_sample=16))
        else:
            print(f'  （real 模式：录制 {self.seconds} 秒，请说话）')
            self.spin_for(self.seconds)

        chunks = self.audio.stop_recording()
        total = sum(c.duration_seconds for c in chunks)
        self.check('stop_recording() 返回数据块', len(chunks) > 0,
                   f'{len(chunks)} 块 / {total:.2f} 秒')
        if self.backend == 'mock':
            self.check('录制时长合计 ≈ 3 秒', abs(total - 3.0) < 0.1,
                       f'{total:.2f} 秒')

        if self.out is not None:
            self._write_wav(self.out, chunks)

    @staticmethod
    def _write_wav(path, chunks):
        import wave
        if not chunks:
            print(f'  ⚠️  无音频数据，跳过写文件 {path}')
            return
        data = b''.join(c.data for c in chunks)
        first = chunks[0]
        with wave.open(path, 'wb') as wf:
            wf.setnchannels(first.channels or 1)
            wf.setsampwidth(max(first.bits_per_sample // 8, 2))
            wf.setframerate(first.sample_rate or 16000)
            wf.writeframes(data)
        print(f'  💾 已写入 {path} ({len(data)} 字节)')


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='mock',
                        choices=['mock', 'real', 'sim'])
    parser.add_argument('--seconds', type=float, default=5.0,
                        help='real 模式录制秒数')
    parser.add_argument('--out', help='输出 wav 文件路径（可选）')
    args = parser.parse_args(argv)
    return AudioRecordDemo(backend=args.backend, seconds=args.seconds,
                           out=args.out).run()


if __name__ == '__main__':
    sys.exit(main())
