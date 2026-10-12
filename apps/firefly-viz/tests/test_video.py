"""编码边界契约；本文件不需要 GPU、FFmpeg 或 Viewer。"""

from dataclasses import replace
import io
import queue
import threading
import time
from unittest.mock import create_autospec
from dataclasses import dataclass
from typing import BinaryIO

import pytest

from firefly_viz.config import VideoOptions
from firefly_viz.video import AccessUnits, Encoder, Frame, command, nal_types


@dataclass
class ProcessPipes:
    stdout: BinaryIO | None = None
    stdin: BinaryIO | None = None


def nal(kind, payload=b'\x99', prefix=b'\x00\x00\x00\x01'):
    return prefix + bytes([kind << 1, 1]) + payload


def test_access_units_do_not_depend_on_pipe_read_boundaries():
    first = nal(35) + nal(32) + nal(33) + nal(34) + nal(19)
    second = nal(35, prefix=b'\x00\x00\x01') + nal(1, b'\xab\x00\x00\x03\x01\xcd')
    third = nal(35) + nal(1)
    stream = first + second + third
    for width in range(1, len(stream) + 1):
        parser = AccessUnits(1024)
        result = []
        for begin in range(0, len(stream), width):
            result.extend(parser.feed(stream[begin:begin + width]))
        result.extend(parser.feed(b'', final=True))
        assert result == [first, second, third]
        assert nal_types(result[0]) == [35, 32, 33, 34, 19]


def test_access_unit_rejects_missing_aud_and_missing_picture():
    for data in [nal(19), nal(35) + nal(32), b'garbage']:
        with pytest.raises(ValueError, match='access unit'):
            AccessUnits(1024).feed(data, final=True)


def test_access_unit_budget_applies_to_completed_and_pending_frames():
    with pytest.raises(ValueError, match='budget'):
        AccessUnits(32).feed(nal(35) + nal(1, b'X' * 100))
    with pytest.raises(ValueError, match='access unit'):
        AccessUnits(32).feed(nal(35) + nal(1, b'X' * 100) + nal(35))


def test_access_unit_requires_exactly_one_picture_but_allows_multiple_slices():
    with pytest.raises(ValueError, match='one picture'):
        AccessUnits(1024).feed(nal(35) + nal(19) + nal(1), final=True)
    sample = nal(35) + nal(19) + nal(19, b'\x01')
    assert AccessUnits(1024).feed(sample, final=True) == [sample]
    with pytest.raises(ValueError, match='truncated'):
        AccessUnits(1024).feed(nal(35) + nal(19) + b'\x00\x00\x01\x02', final=True)


def test_hardware_commands_are_explicit_and_have_no_b_frames():
    for encoder in ('hevc_vaapi', 'hevc_qsv', 'libx265'):
        args = command(replace(VideoOptions(), encoder=encoder), 320, 240)
        assert args[args.index('-c:v') + 1] == encoder
        assert args[args.index('-bf') + 1] == '0'
        assert args[args.index('-g') + 1] == '10'
        assert 'hevc_metadata=aud=insert' in args[args.index('-bsf:v') + 1]
        assert args[-3:] == ['-f', 'hevc', 'pipe:1']
        assert args[args.index('-pixel_format') + 1] == 'gray'


def test_reader_pairs_fifo_capture_times_and_repeats_headers_after_aud():
    samples = [nal(35) + nal(32) + nal(33) + nal(34) + nal(19),
               nal(35) + nal(1), nal(35) + nal(19) + nal(36, b'')]
    frames = [Frame('left', stamp, index, 2, 2, bytes([index] * 4))
              for index, stamp in [(2, 1.1), (7, 1.73), (8, 1.8)]]
    encoder = Encoder.__new__(Encoder)
    encoder.options = VideoOptions()
    encoder.process = ProcessPipes(stdout=io.BytesIO(b''.join(samples)))
    encoder.pending = queue.Queue(16)
    for frame in frames:
        encoder.pending.put((frame, time.monotonic()))
    output = []
    encoder.emit = lambda *args: output.append(args)
    encoder.emitted = 0
    encoder._read()
    assert [row[0] for row in output] == frames
    assert [row[2] for row in output] == [True, False, True]
    assert nal_types(output[-1][1]) == [35, 32, 33, 34, 19, 36]
    assert encoder.pending.empty()


def test_input_drains_a_full_queue_without_a_sentinel_slot():
    encoder = Encoder.__new__(Encoder)
    encoder.options = VideoOptions()
    encoder.frames, encoder.pending = queue.Queue(1), queue.Queue(16)
    frame = Frame('left', 7.25, 6, 2, 2, b'\x01' * 4)
    encoder.frames.put(frame)
    encoder.aborting, encoder.draining = threading.Event(), threading.Event()
    encoder.draining.set()
    writes, closed = [], []
    pipe = create_autospec(io.BufferedWriter, instance=True, spec_set=True)
    pipe.write.side_effect = writes.append
    pipe.close.side_effect = lambda: closed.append(True)
    encoder.process = ProcessPipes(stdin=pipe)
    encoder.submitted = 0
    encoder._write()
    assert writes == [frame.pixels]
    assert closed == [True]
    assert encoder.pending.get_nowait()[0] is frame
    assert encoder.submitted == 1
