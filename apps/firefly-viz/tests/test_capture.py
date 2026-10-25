"""左目视频按采集时间采样，编码结束必须报告完整性。"""

from dataclasses import replace

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


def test_capture_has_no_image_output_when_video_is_disabled(recordings_factory):
    options = Options()
    options = replace(options, video=replace(options.video, enabled=False))
    archive, live = Recording(), Recording()
    capture = Capture(recordings_factory(archive=archive, live=live), options)
    capture.handle(Frame('left', 1., 0, 2, 2, b'\x00' * 4))
    capture.close()
    assert capture.encoder is None
    assert not archive.calls
    assert not live.calls
    with pytest.raises(ValueError, match='left camera'):
        capture.handle(Frame('right', 1., 0, 2, 2, b'\x00' * 4))


def test_close_reports_encoder_failure_and_records_final_counts(recordings_factory):
    from firefly_viz.video import Encoder

    class FailedEncoder(Encoder):
        emitted = 0
        dropped = 0

        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True
            raise RuntimeError('encoder failed')

    recordings = recordings_factory(archive=Recording())
    capture = Capture(recordings, Options())
    encoder = FailedEncoder()
    capture.encoder = encoder
    with pytest.raises(ExceptionGroup, match='capture cleanup failed'):
        capture.close()
    assert encoder.closed
    recordings.health.assert_called_once_with('video/frames', [0, 0, 0, 0], None)


def test_configuration_rejects_misspelled_keys_and_invalid_budgets(tmp_path):
    config = tmp_path / 'viz.toml'
    assert load(config) == Options()
    for text in ['[video]\nbitrat = 100', '[video]\nqueue_frames = 0',
                 '[capture]\nqueue_frames = 0', '[video]\nencoder = "automatic"',
                 '[capture]\nright_hz = 1.0', '[capture]\nbefore_frames = 0']:
        config.write_text(text)
        with pytest.raises((TypeError, ValueError)):
            load(config)
