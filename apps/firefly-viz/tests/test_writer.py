"""异步故障必须传播；退出排空，不能把坏录制报告为成功。"""

import threading
from types import SimpleNamespace

import pytest

from firefly_viz.main import Continuity, Worker
from firefly_viz.config import Options
from firefly_viz.writer import Writer
from firefly_viz.messages import VoxelMessage


class Recording:
    def __init__(self):
        self.times = []
        self.calls = []

    def reset_time(self):
        self.times.append(('reset',))

    def set_time(self, name, **kwargs):
        self.times.append((name, kwargs))

    def log(self, *args, **kwargs):
        self.calls.append((args, kwargs))

    def flush(self, **kwargs):
        pass


def test_worker_shutdown_drains_in_order_and_finishes():
    received = []
    worker = Worker('contract', 16, received.append, lambda: None, lambda: received.append('done'), 2.)
    for i in range(10):
        worker.put(i)
    worker.close()
    assert received == [*range(10), 'done']
    assert not worker.thread.is_alive()


def test_worker_propagates_error_and_still_finishes():
    finished = threading.Event()

    def fail(item):
        raise ValueError('invalid sample')

    worker = Worker('contract', 16, fail, lambda: None, finished.set, 2.)
    worker.put(1)
    assert finished.wait(2.)
    with pytest.raises(RuntimeError, match='invalid sample'):
        worker.close()
    assert not worker.thread.is_alive()


def test_full_queue_shutdown_timeout_does_not_lose_close_signal():
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    received = []

    def consume(item):
        entered.set()
        assert release.wait(2.)
        received.append(item)

    worker = Worker('full-queue', 1, consume, lambda: None, finished.set, 0.01)
    try:
        worker.put(1)
        assert entered.wait(2.)
        worker.put(2)
        assert worker.inbox.full()
        with pytest.raises(RuntimeError, match='did not finish draining'):
            worker.close()
        with pytest.raises(RuntimeError, match='closing'):
            worker.put(3)
    finally:
        release.set()
        worker.thread.join(2.)
    assert finished.is_set()
    assert not worker.thread.is_alive()
    assert received == [1, 2]
    worker.close()


def test_diagnostic_continuity_is_per_publisher_and_bounded():
    continuity = Continuity(2)
    for publisher, sequence in [(1, 5), (2, 0), (1, 6), (2, 1)]:
        continuity.observe(publisher, sequence)
    with pytest.raises(ValueError, match='sequence gap'):
        continuity.observe(1, 8)
    with pytest.raises(ValueError, match='budget'):
        continuity.observe(3, 0)


def voxel(kind, points, stamp=1., sequence=0, chunk_index=0, chunk_count=1):
    msg = VoxelMessage()
    msg.kind, msg.timestamp = kind, stamp
    msg.sequence, msg.chunk_index, msg.chunk_count = sequence, chunk_index, chunk_count
    path = b'plan/voxels'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    msg.voxel_size[:] = [0.1] * 3
    msg.voxel_count = len(points)
    for i, point in enumerate(points):
        msg.voxels[i][:] = point
    return msg


def test_voxel_history_uses_complete_snapshots_after_ordered_deltas():
    archive, live = Recording(), Recording()
    writer = Writer(SimpleNamespace(archive=archive, live=live), Options(), threading.Event())
    writer.handle(voxel(3, [(1, 2, 3), (4, 5, 6)]))
    writer.handle(voxel(8, [(7, 8, 9)], 2., sequence=1))
    writer.handle(voxel(9, [(1, 2, 3)], 2., sequence=2))
    writer.tick(final=True)
    assert writer.voxels['plan/voxels'][0] == {(4, 5, 6), (7, 8, 9)}
    assert len(archive.calls) == 1
    assert archive.calls[0][0][0] == 'plan/voxels'
    assert not archive.calls[0][1].get('static', False)
    assert live.calls[0][0][0] == 'live/plan/voxels'
    assert live.calls[0][1]['static'] is True
    with pytest.raises(ValueError, match='out-of-order'):
        writer.handle(voxel(9, [(4, 5, 6)], 1., sequence=3))


def test_voxel_delta_cannot_create_an_unknown_baseline():
    writer = Writer(SimpleNamespace(archive=Recording(), live=None), Options(), threading.Event())
    with pytest.raises(ValueError, match='baseline'):
        writer.handle(voxel(8, [(1, 2, 3)]))


def test_voxel_chunk_commit_is_atomic_and_gaps_are_fatal():
    archive = Recording()
    writer = Writer(SimpleNamespace(archive=archive, live=None), Options(), threading.Event())
    writer.handle(voxel(3, [(1, 2, 3)], chunk_count=2))
    writer.tick()
    assert not archive.calls
    assert not writer.voxels
    writer.handle(voxel(3, [(4, 5, 6)], sequence=1, chunk_index=1, chunk_count=2))
    writer.tick(final=True)
    assert writer.voxels['plan/voxels'][0] == {(1, 2, 3), (4, 5, 6)}
    with pytest.raises(ValueError, match='sequence gap'):
        writer.handle(voxel(9, [(1, 2, 3)], 2., sequence=3))


def test_voxel_partial_update_fails_at_shutdown_and_empty_snapshot_clears():
    archive = Recording()
    writer = Writer(SimpleNamespace(archive=archive, live=None), Options(), threading.Event())
    writer.handle(voxel(3, [], chunk_count=1))
    writer.tick(final=True)
    assert writer.voxels['plan/voxels'][0] == set()
    writer.handle(voxel(3, [(1, 2, 3)], 2., sequence=1, chunk_count=2))
    with pytest.raises(ValueError, match='during a voxel update'):
        writer.tick(final=True)
