"""关键点 IPC → Rerun 的像素坐标、空帧与时间轴契约。"""

import ctypes

import rerun as rr

from firefly_viz import main
from firefly_viz.messages import VIZ_KIND_KEYPOINTS_2D, VizMessage


def test_keypoints_preserve_camera_time_and_clear_empty_frames(monkeypatch):
    calls = []
    times = []
    monkeypatch.setattr(main.rr, "log", lambda *args: calls.append(args))
    monkeypatch.setattr(main.rr, "set_time", lambda *args, **kwargs: times.append((args, kwargs)))
    msg = VizMessage()
    msg.kind = VIZ_KIND_KEYPOINTS_2D
    entity = b"vio/debug/keypoints/left"
    msg.entity_len = len(entity)
    msg.entity[:len(entity)] = entity
    msg.timestamp = 1.25
    msg.point_count = 1
    msg.points[0] = (ctypes.c_double * 3)(12.25, 41.5, 42.0)
    msg.color = (ctypes.c_uint8 * 3)(64, 255, 128)
    main._handle(msg, "")
    assert calls[0][0] == entity.decode()
    assert isinstance(calls[0][1], rr.Points2D)
    assert calls[0][1].positions.as_arrow_array().to_pylist() == [[12.25, 41.5]]
    assert calls[0][2].as_component_batches()[0].as_arrow_array().to_pylist() == [42]
    assert times[0] == (("sim_time",), {"duration": 1.25})
    msg.timestamp = 1.5
    msg.point_count = 0
    main._handle(msg, "")
    assert calls[1][1].positions.as_arrow_array().to_pylist() == []


def test_matrix_payload_preserves_shape_values_and_owns_storage(monkeypatch):
    calls = []
    monkeypatch.setattr(main.rr, "log", lambda *args: calls.append(args))
    monkeypatch.setattr(main.rr, "set_time", lambda *args, **kwargs: None)
    msg = VizMessage()
    msg.kind = main.VIZ_KIND_MATRIX
    msg.scalar_count = 2
    msg.scalars[:2] = [2, 3]
    msg.point_count = 6
    msg.points[0][:] = [1, 2, 3]
    msg.points[1][:] = [4, 5, 6]
    main._handle(msg, "")
    msg.points[0][0] = 999
    data = calls[0][1].data.as_arrow_array().to_pylist()[0]
    assert data["shape"] == [2, 3]
    assert data["buffer"] == [1, 2, 3, 4, 5, 6]
    for rows, cols, count in [(0, 3, 0), (2, 3, 5), (2.5, 2, 5), (float("nan"), 2, 6), (1000, 3, 3000)]:
        msg.scalars[:2] = [rows, cols]
        msg.point_count = count
        main._handle(msg, "")
    assert len(calls) == 1
