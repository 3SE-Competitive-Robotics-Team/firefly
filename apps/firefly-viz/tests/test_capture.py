"""采样与触发窗口不改变原始像素或时间，实时 static 不进入文件流。"""

from dataclasses import replace
import threading

from firefly_viz.capture import Capture, Rate
from firefly_viz.config import Options, load
from firefly_viz.video import Frame
import pytest


class Recording:
    def __init__(self):
        self.calls = []
        self.stamp = None

    def reset_time(self):
        self.stamp = None

    def set_time(self, name, **kwargs):
        self.stamp = kwargs

    def log(self, *args, **kwargs):
        self.calls.append((self.stamp, args, kwargs))


def test_rate_uses_capture_time_and_does_not_backfill():
    rate = Rate(2)
    assert [rate.accept(t) for t in [1., 1.1, 1.49, 1.5, 3., 3.01]] == [True, False, False, True, True, False]
    assert not Rate(0).accept(1.)


def test_rate_preserves_phase_across_timestamp_jitter():
    rate = Rate(10)
    assert all(rate.accept(t) for t in [0., .1001, .2, .3001, .4])


def test_preview_and_archive_sampling_have_independent_cadence(recordings_factory):
    options = Options()
    options = replace(options, video=replace(options.video, enabled=False),
                      capture=replace(options.capture, before_frames=0, after_frames=0,
                                      preview_hz=3., right_hz=2.))
    archive, live = Recording(), Recording()
    recordings = recordings_factory(archive=archive, live=live)
    capture = Capture(recordings, options, threading.Event())
    for i in range(11):
        capture.handle(Frame('right', i / 10., i, 2, 2, b'\x00' * 4))
    assert [call[0]['duration'] for call in archive.calls] == [0., .5, 1.]
    assert len(live.calls) == 4


def test_close_consumes_last_trigger_and_still_closes_failed_encoder(recordings_factory):
    options = Options()
    options = replace(options, video=replace(options.video, enabled=False),
                      capture=replace(options.capture, before_frames=1, after_frames=0))
    archive = Recording()
    recordings = recordings_factory(archive=archive, live=None)
    trigger = threading.Event()
    capture = Capture(recordings, options, trigger)
    capture.handle(Frame('left', 1., 0, 2, 2, b'\x01' * 4))
    closed = []

    def fail_close():
        closed.append(True)
        raise RuntimeError('encoder failed')

    class FailedEncoder:
        emitted = 0
        dropped = 0

        def close(self):
            fail_close()

    capture.encoder = FailedEncoder()
    trigger.set()
    with pytest.raises(ExceptionGroup, match='capture cleanup failed'):
        capture.close()
    assert closed == [True]
    assert not trigger.is_set()
    assert archive.calls[0][0]['duration'] == 1.


def test_trigger_keeps_bounded_before_and_after_and_preview_is_not_archived(recordings_factory):
    options = Options()
    options = replace(options, video=replace(options.video, enabled=False),
                      capture=replace(options.capture, before_frames=2, after_frames=1, preview_hz=10.))
    archive, live = Recording(), Recording()
    recordings = recordings_factory(archive=archive, live=live)
    trigger = threading.Event()
    capture = Capture(recordings, options, trigger)
    for i in range(3):
        capture.handle(Frame('left', float(i), i, 2, 2, bytes([i] * 4)))
    assert not archive.calls
    assert len(capture.rings['left']) == 2
    assert all(call[2]['static'] for call in live.calls)
    trigger.set()
    capture.handle(Frame('left', 3., 3, 2, 2, bytes([3] * 4)))
    assert [call[0]['duration'] for call in archive.calls] == [1., 2., 3.]
    assert all(call[1][0] == 'snapshots/left/image' for call in archive.calls)
    assert all(not call[2].get('static', False) for call in archive.calls)
    assert capture.after['left'] == 0


def test_configuration_rejects_misspelled_keys_and_invalid_budgets(tmp_path):
    config = tmp_path / 'viz.toml'
    assert load(config) == Options()
    for text in ['[video]\nbitrat = 100', '[video]\nqueue_frames = 0',
                 '[capture]\ndepth_hz = -1', '[video]\nencoder = "automatic"']:
        config.write_text(text)
        with pytest.raises((TypeError, ValueError)):
            load(config)
    config.write_text('[capture]\nbefore_frames = 0\nafter_frames = 0')
    assert load(config).capture.before_frames == 0
