"""持久化和静态预览的目的地必须分开；初始化失败也释放流。"""

from dataclasses import replace

import pytest

from firefly_viz import recording
from firefly_viz.config import Options


class Stream:
    instances = []

    def __init__(self, application, **kwargs):
        self.kwargs = kwargs
        self.sinks = []
        self.closed = False
        self.logs = []
        self.instances.append(self)

    def set_sinks(self, *sinks):
        self.sinks = sinks

    def is_enabled(self):
        return True

    def send_blueprint(self, blueprint):
        pass

    def log(self, *args, **kwargs):
        self.logs.append((args, kwargs))

    def flush(self, **kwargs):
        pass

    def disconnect(self):
        self.closed = True


def dependencies(stream_factory=Stream):
    Stream.instances = []
    return dict(stream_factory=stream_factory,
                file_sink=lambda path, write_footer: ('file', path, write_footer),
                grpc_sink=lambda url: ('grpc', url), blueprint_factory=lambda live: live)


def test_live_static_cannot_reach_file_sink_and_uses_same_recording(tmp_path):
    options = Options()
    options = replace(options, video=replace(options.video, enabled=False))
    path = tmp_path / 'run.rrd'
    streams = recording.Recordings(path, recording.URL, False, options, **dependencies())
    assert streams.archive.sinks == (('file', str(path), True), ('grpc', recording.URL))
    assert streams.live.sinks == (('grpc', recording.URL),)
    assert streams.archive.kwargs['recording_id'] == streams.live.kwargs['recording_id']
    streams.close()
    assert all(stream.closed for stream in Stream.instances)


def test_existing_evidence_is_never_overwritten(tmp_path):
    path = tmp_path / 'run.rrd'
    path.write_bytes(b'existing evidence')
    with pytest.raises(FileExistsError):
        recording.Recordings(path, None, False, Options(), **dependencies())
    assert path.read_bytes() == b'existing evidence'
    assert all(stream.closed for stream in Stream.instances)


def test_same_id_native_streams_keep_preview_out_of_indexed_rrd(tmp_path):
    """生产文件 sink 关闭后可索引读取，静态预览不落盘；不启动 Viewer 或编码器。"""
    from rerun.chunk import RrdReader

    rr = recording.rr
    path = tmp_path / 'archive.rrd'
    options = Options()
    options = replace(options, video=replace(options.video, enabled=False))
    streams = recording.Recordings(path, None, False, options)
    archive = streams.archive
    live = rr.RecordingStream(recording.APP_ID, recording_id=archive.get_recording_id(),
                              send_properties=False)
    preview = rr.binary_stream(recording=live)
    try:
        archive.log('calibration', rr.AnyValues(focal_length=200.), static=True)
        recording.at(archive, 2.5)
        archive.log('metrics', rr.Scalars(7.), strict=True)
        for value in range(3):
            live.log('live/preview', rr.AnyValues(value=value), static=True, strict=True)
        archive.flush(timeout_sec=5.)
        assert preview.read(flush_timeout_sec=5.)
    finally:
        live.disconnect()
        streams.close()
    reader = RrdReader(path)
    entities = {chunk.entity_path.lstrip('/'): chunk.is_static
                for store in reader.recordings() for chunk in reader.store(store=store).stream()}
    assert entities['calibration'] is True
    assert entities['metrics'] is False
    assert not any(entity == 'live' or entity.startswith('live/') for entity in entities)


def test_initialization_disconnects_both_streams_even_if_first_cleanup_fails(tmp_path):

    class FailingStream(Stream):
        def send_blueprint(self, blueprint):
            raise ValueError('blueprint failure')

        def disconnect(self):
            self.closed = True
            if self is Stream.instances[1]:
                raise RuntimeError('live disconnect failure')

    with pytest.raises(ExceptionGroup, match='initialization and cleanup failed') as caught:
        recording.Recordings(tmp_path / 'failed.rrd', recording.URL, False, Options(),
                             **dependencies(FailingStream))
    assert all(stream.closed for stream in Stream.instances)
    assert [str(error) for error in caught.value.exceptions] == ['blueprint failure', 'live disconnect failure']
