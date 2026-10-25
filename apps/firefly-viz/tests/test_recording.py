"""双目只保存历史录制，初始化失败也释放录制流。"""

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
                grpc_sink=lambda url: ('grpc', url), blueprint_factory=lambda: None)


def test_file_and_viewer_share_one_recording_without_live_stream(tmp_path):
    options = Options()
    options = replace(options, video=replace(options.video, enabled=False))
    path = tmp_path / 'run.rrd'
    streams = recording.Recordings(path, recording.URL, False, options, **dependencies())
    assert streams.archive.sinks == (('file', str(path), True), ('grpc', recording.URL))
    assert len(Stream.instances) == 1
    streams.close()
    assert all(stream.closed for stream in Stream.instances)


def test_existing_evidence_is_never_overwritten(tmp_path):
    path = tmp_path / 'run.rrd'
    path.write_bytes(b'existing evidence')
    with pytest.raises(FileExistsError):
        recording.Recordings(path, None, False, Options(), **dependencies())
    assert path.read_bytes() == b'existing evidence'
    assert all(stream.closed for stream in Stream.instances)


def test_stereo_video_and_overlays_are_the_only_camera_history(tmp_path):
    """写入原子视频样本与同帧点集，不启动 Viewer 或编码器。"""
    from rerun.chunk import RrdReader
    from firefly_viz.video import Frame

    path = tmp_path / 'archive.rrd'
    streams = recording.Recordings(path, None, False, Options())
    try:
        for camera, index in [('left', 3), ('right', 8)]:
            streams.video(Frame(camera, 2.5, index, 320, 240, b''), b'encoded-frame', True, .001)
            streams.matches.observe(camera, 2.5, [[12., 20.]], [index], [64, 255, 128])
    finally:
        streams.close()
    entities = {}
    for chunk in RrdReader(path).stream():
        name = chunk.entity_path.lstrip('/')
        entities.setdefault(name, []).append(chunk.to_record_batch())
    assert not any(name.startswith(('live/', 'okvis/debug/keypoints/')) for name in entities)
    for camera, index in [('left', 3), ('right', 8)]:
        assert f'cameras/{camera}/image' in entities
        points = entities[f'cameras/{camera}/points']
        assert any(row == [[12.5, 20.5]] for batch in points
                   for row in batch.column('Points2D:positions').to_pylist())
        assert all(f'{camera}_frame' in batch.schema.names for batch in points)
        assert all(value == index for batch in points
                   for value in batch.column(f'{camera}_frame').to_pylist())


def test_initialization_reports_blueprint_and_cleanup_failures(tmp_path):

    class FailingStream(Stream):
        def send_blueprint(self, blueprint):
            raise ValueError('blueprint failure')

        def disconnect(self):
            self.closed = True
            raise RuntimeError('disconnect failure')

    with pytest.raises(ExceptionGroup, match='initialization and cleanup failed') as caught:
        recording.Recordings(tmp_path / 'failed.rrd', recording.URL, False, Options(),
                             **dependencies(FailingStream))
    assert all(stream.closed for stream in Stream.instances)
    assert [str(error) for error in caught.value.exceptions] == ['blueprint failure', 'disconnect failure']


def test_camera_blueprint_contains_only_recorded_stereo_views():
    import rerun.blueprint as rrb

    root = recording.blueprint().root_container
    container = root.contents[0].contents[1]
    assert isinstance(container, rrb.Horizontal)
    cameras = container.contents
    assert [(view.origin, view.name) for view in cameras] == [
        ('/cameras/left', 'Left · recorded'),
        ('/cameras/right', 'Right · recorded')]
    assert root.contents[0].contents[0].origin == '/'


def test_spatial_blueprint_uses_one_view_for_ground_truth_vslam_and_planning():
    import rerun.blueprint as rrb

    spatial = recording.blueprint().root_container.contents[0].contents[0]
    assert isinstance(spatial, rrb.Spatial3DView)
    assert spatial.origin == '/'
    assert spatial.name == 'Ground truth / VSLAM / planning'


def test_okvis_diagnostics_uses_state_timeline_for_tracking_and_curves_for_counters():
    import rerun.blueprint as rrb

    diagnostics = recording.blueprint().root_container.contents[1].contents[0]
    assert diagnostics.name == 'OKVIS diagnostics'
    tracking, loops = diagnostics.contents
    assert isinstance(tracking, rrb.StateTimelineView)
    assert tracking.origin == '/okvis/tracking'
    assert isinstance(loops, rrb.TimeSeriesView)
    assert loops.origin == '/okvis/loop_commits'


def test_diagnostics_blueprint_excludes_optional_vision_and_imu_matrix_views():
    import rerun.blueprint as rrb

    panels = recording.blueprint().root_container.contents[1].contents
    assert all(panel.name != 'Vision matching diagnostics' for panel in panels)
    assert not any(isinstance(panel, rrb.TensorView) for panel in panels)
