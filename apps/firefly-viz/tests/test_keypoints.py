"""图像与叠加按采集时间精确配对；丢帧和延迟不能复用旧点。"""

import pytest
import rerun as rr

from firefly_viz.config import Options
from firefly_viz.messages import VIZ_KIND_KEYPOINTS_2D, VIZ_KIND_MATRIX, VizMessage
from firefly_viz.recording import MatchedPoints
from firefly_viz.writer import Writer


class Recording:
    def __init__(self):
        self.stamp = None
        self.calls = []
        self.timelines = {}
        self.times = []

    def reset_time(self):
        self.stamp = None
        self.timelines = {}

    def set_time(self, name, **kwargs):
        self.stamp = name, kwargs
        self.timelines[name] = kwargs

    def log(self, *args, **kwargs):
        self.calls.append((self.stamp, args))
        self.times.append(dict(self.timelines))


def test_camera_and_okvis_match_on_the_same_nanosecond_after_clock_conversion():
    rec = Recording()
    pairs = MatchedPoints(rec, 2)
    pairs.frame('left', 0.30000000000000004, 3)
    pairs.observe('left', 0.3, [[12., 20.]], [5], [64, 255, 128])
    assert rec.calls[-2][1][1].positions.as_arrow_array().to_pylist() == [[12.5, 20.5]]
    assert rec.times[-2] == {'sim_time': {'duration': 0.30000000000000004}, 'left_frame': {'sequence': 3}}


def test_out_of_order_annotation_only_matches_its_image_and_missing_frames_are_empty():
    rec = Recording()
    pairs = MatchedPoints(rec, 2)
    pairs.observe('left', 1., [[12.25, 41.5]], [42], [64, 255, 128])
    assert not rec.calls
    pairs.frame('left', 1.)
    pairs.frame('left', 2.)
    points = [(stamp, args[1]) for stamp, args in rec.calls if isinstance(args[1], rr.Points2D)]
    assert points[0][0] == ('sim_time', {'duration': 1.})
    assert points[0][1].positions.as_arrow_array().to_pylist() == [[12.75, 42.]]
    assert points[1][1].positions.as_arrow_array().to_pylist() == []
    pairs.observe('left', 2., [[1, 2]], [7], [255, 0, 0])
    assert rec.calls[-2][0] == ('sim_time', {'duration': 2.})
    pairs.frame('left', 3.)
    before = len(rec.calls)
    pairs.observe('left', 1., [[3, 4]], [8], [0, 0, 0])
    assert len(rec.calls) == before
    assert len(pairs.frames['left']) == 2
    assert len(pairs.points['left']) == 2


def test_matrix_owns_storage_and_rejects_invalid_shapes(recordings_factory):
    rec = Recording()
    writer = Writer(recordings_factory(archive=rec, matches=MatchedPoints(rec, 4)), Options())
    msg = VizMessage()
    name = b'okvis/debug/pose_imu/test'
    msg.entity_len, msg.entity[:len(name)] = len(name), name
    msg.kind = VIZ_KIND_MATRIX
    msg.timestamp = 1.
    msg.scalar_count = 2
    msg.scalars[:2] = [2, 3]
    msg.point_count = 6
    msg.points[0][:] = [1, 2, 3]
    msg.points[1][:] = [4, 5, 6]
    writer.handle(msg)
    msg.points[0][0] = 999
    data = rec.calls[0][1][1].data.as_arrow_array().to_pylist()[0]
    assert data['shape'] == [2, 3]
    assert data['buffer'] == [1, 2, 3, 4, 5, 6]
    for rows, cols, count in [(0, 3, 0), (2, 3, 5), (2.5, 2, 5), (float('nan'), 2, 6)]:
        msg.scalars[:2], msg.point_count = [rows, cols], count
        with pytest.raises(ValueError, match='matrix'):
            writer.handle(msg)


def test_late_points_keep_original_video_index_and_do_not_cross_cameras():
    rec = Recording()
    pairs = MatchedPoints(rec, 4)
    pairs.frame('left', 1.125, 8)
    pairs.frame('left', 1.875, 11)
    pairs.observe('right', 1.125, [[9, 9]], [12], [0, 0, 0])
    assert len(rec.calls) == 4
    pairs.frame('right', 1.125, 5)
    assert rec.times[-2] == {'sim_time': {'duration': 1.125}, 'right_frame': {'sequence': 5}}
    assert rec.calls[-2][1][0] == 'cameras/right/points'
    assert rec.calls[-2][1][1].positions.as_arrow_array().to_pylist() == [[9.5, 9.5]]
    pairs.observe('left', 1.125, [[2, 3]], [42], [64, 255, 128])
    assert rec.times[-2] == {'sim_time': {'duration': 1.125}, 'left_frame': {'sequence': 8}}
    assert rec.calls[-2][1][1].positions.as_arrow_array().to_pylist() == [[2.5, 3.5]]
    before = len(rec.calls)
    pairs.observe('left', 1.125000001, [[4, 5]], [43], [0, 0, 0])
    assert len(rec.calls) == before


@pytest.mark.parametrize('camera', ['left', 'right'])
def test_keypoint_resolution_mismatch_cannot_reach_overlay(recordings_factory, camera):
    rec = Recording()
    pairs = MatchedPoints(rec, 4)
    writer = Writer(recordings_factory(archive=rec, matches=pairs), Options())
    msg = VizMessage()
    name = f'okvis/debug/keypoints/{camera}'.encode()
    msg.entity_len, msg.entity[:len(name)] = len(name), name
    msg.kind, msg.timestamp = VIZ_KIND_KEYPOINTS_2D, 1.
    msg.scalars[:3], msg.scalar_count = [640, 480, 1], 3
    msg.points[0][:], msg.point_count = [12, 20, 1], 1
    pairs.frame(camera, 1., 3)
    with pytest.raises(ValueError, match='resolution'):
        writer.handle(msg)
    assert not pairs.points[camera]
    assert len(rec.calls) == 2
