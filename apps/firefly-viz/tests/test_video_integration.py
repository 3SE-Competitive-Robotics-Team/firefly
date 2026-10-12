"""显式启用的真实编码契约：使用 viz 配置的编码器与外部 FFmpeg。"""

from dataclasses import replace
import os
import signal
import subprocess

import numpy as np
import pytest

from firefly_viz.config import load
from firefly_viz.video import Encoder, Frame, nal_types


@pytest.mark.skipif(os.environ.get("FIREFLY_RUN_VIDEO_ENCODER") != "1",
                    reason="requires an explicitly requested FFmpeg/encoder integration run")
def test_real_encoder_preserves_capture_order_and_decodable_frames():
    options = replace(load().video, queue_frames=32, max_inflight=32)
    width, height, count = 320, 240, 24
    frames = []
    for i in range(count):
        pixels = np.full((height, width), 24 + 6 * i, dtype=np.uint8)
        pixels[80:144, 8 * i:8 * i + 64] = 230
        # 非等间隔时间戳能识别误用编码器的名义 FPS 重建时间。
        frames.append(Frame("left", 2.0 + i * 0.1 + (i // 7) * 0.05, i, width, height, pixels.tobytes()))
    output, diagnostics = [], []
    encoder = Encoder(options, width, height, lambda *sample: output.append(sample), diagnostics.append)
    try:
        for frame in frames:
            encoder.submit(frame)
    finally:
        encoder.close()
    assert encoder.dropped == 0
    assert [sample[0].stamp for sample in output] == [frame.stamp for frame in frames]
    assert [sample[0].index for sample in output] == list(range(count))
    assert output[0][2] is True
    for _, data, keyframe, _ in output:
        if keyframe:
            assert {32, 33, 34}.issubset(nal_types(data))
    decoder = subprocess.Popen([options.ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin",
                                "-f", "hevc", "-i", "pipe:0", "-f", "rawvideo", "-pix_fmt", "gray", "pipe:1"],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        raw, errors = decoder.communicate(b"".join(sample[1] for sample in output), timeout=30)
    except subprocess.TimeoutExpired:
        decoder.send_signal(signal.SIGINT)
        try:
            decoder.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            decoder.terminate()
            decoder.communicate(timeout=10)
        raise
    assert decoder.returncode == 0, errors.decode(errors="replace")
    decoded = np.frombuffer(raw, dtype=np.uint8).reshape(count, height, width)
    for actual, frame in zip(decoded, frames, strict=True):
        expected = np.frombuffer(frame.pixels, dtype=np.uint8).reshape(height, width)
        assert np.abs(actual.astype(float) - expected).mean() < 8.0
