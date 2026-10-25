"""异步故障必须传播；退出排空，不能把坏录制报告为成功。"""

import threading

import pytest
from firefly_mujoco import OdomMessage

from firefly_viz.main import Continuity, Worker
from firefly_viz.config import Options
from firefly_viz.writer import Writer
from firefly_viz.messages import VIZ_KIND_CLEAR, VIZ_KIND_POSE, VIZ_KIND_SCALARS, VizMessage, VoxelMessage


class Recording:
    def __init__(self):
        self.times = []
        self.calls = []
        self.events = []
        self.stamp = None

    def reset_time(self):
        self.times.append(('reset',))
        self.stamp = None

    def set_time(self, name, **kwargs):
        self.times.append((name, kwargs))
        self.stamp = name, kwargs

    def log(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        self.events.append((self.stamp, args))

    def flush(self, **kwargs):
        pass


def okvis_pose(stamp, xyz):
    msg = VizMessage()
    path = b'okvis/odom'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    msg.kind, msg.timestamp = VIZ_KIND_POSE, stamp
    msg.xyz[:], msg.quat_xyzw[:] = xyz, [0., 0., 0., 1.]
    return msg


def test_vio_bias_is_recorded_directly_under_okvis(recordings_factory):
    from firefly_viz.messages import AttitudeMessage

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    writer.consume(('attitude', AttitudeMessage(timestamp=2., gyro_bias=(.01, -.02, .03))))
    stamp, args = archive.events[0]
    assert stamp == ('sim_time', {'duration': 2.})
    assert args[0] == 'okvis/gyro_bias'
    assert args[1].scalars.as_arrow_array().to_pylist() == [.01, -.02, .03]


def test_control_quaternion_is_only_recorded_as_acceptance_evidence(recordings_factory):
    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage(timestamp=2., kind=VIZ_KIND_SCALARS, scalar_count=4)
    msg.scalars[:] = [0., 0., 0., 1.]
    path = b'fc/imu/quaternion_xyzw'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    writer.handle(msg)
    assert not archive.events
    path = b'acceptance/imu_attitude'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    writer.handle(msg)
    assert archive.events[0][1][0] == 'acceptance/imu_attitude'
    assert archive.events[0][1][1].scalars.as_arrow_array().to_pylist() == [0., 0., 0., 1.]


def test_tracking_is_recorded_as_state_transitions_and_counters_stay_numeric(recordings_factory):
    import rerun as rr

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage()
    path = b'okvis/tracking'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    msg.kind, msg.scalar_count = VIZ_KIND_SCALARS, 1
    for stamp, state in [(1., 0), (2., 1), (3., 1), (4., 2), (5., 0)]:
        msg.timestamp, msg.scalars[0] = stamp, state
        writer.handle(msg)
    assert [stamp for stamp, _ in archive.events] == [
        ('sim_time', {'duration': stamp}) for stamp in [1., 2., 4., 5.]]
    assert all(isinstance(args[1], rr.StateChange) for _, args in archive.events)
    assert [args[1].state.as_arrow_array().to_pylist() for _, args in archive.events] == [
        ['Lost'], ['Marginal'], ['Good'], ['Lost']]
    msg.scalars[0] = 3
    with pytest.raises(ValueError, match='tracking state'):
        writer.handle(msg)
    path = b'okvis/loop_commits'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    writer.handle(msg)
    assert isinstance(archive.events[-1][1][1], rr.Scalars)


def test_clear_tracking_allows_same_state_to_be_logged_again(recordings_factory):
    import rerun as rr

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage()
    path = b'okvis/tracking'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    msg.timestamp, msg.kind, msg.scalar_count, msg.scalars[0] = 1., VIZ_KIND_SCALARS, 1, 2
    writer.handle(msg)
    msg.timestamp, msg.kind = 2., VIZ_KIND_CLEAR
    writer.handle(msg)
    msg.timestamp, msg.kind = 3., VIZ_KIND_SCALARS
    writer.handle(msg)
    assert len(archive.events) == 3
    assert isinstance(archive.events[-1][1][1], rr.StateChange)


def test_standby_accel_gate_records_transitions_and_nis_stays_numeric(recordings_factory):
    import rerun as rr

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage(kind=VIZ_KIND_SCALARS, scalar_count=1)
    path = b'fc/standby_eskf/accel_gate'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    for stamp, state in [(1., 0), (2., 1), (3., 1), (4., 0)]:
        msg.timestamp, msg.scalars[0] = stamp, state
        writer.handle(msg)
    assert [stamp for stamp, _ in archive.events] == [
        ('sim_time', {'duration': stamp}) for stamp in [1., 2., 4.]]
    assert all(isinstance(args[1], rr.StateChange) for _, args in archive.events)
    assert [args[1].state.as_arrow_array().to_pylist() for _, args in archive.events] == [
        ['Not accepted'], ['Accepted'], ['Not accepted']]
    msg.scalars[0] = 2.
    with pytest.raises(ValueError, match='gate state'):
        writer.handle(msg)
    msg.kind = VIZ_KIND_CLEAR
    writer.handle(msg)
    msg.kind, msg.scalars[0] = VIZ_KIND_SCALARS, 0.
    writer.handle(msg)
    assert isinstance(archive.events[-1][1][1], rr.StateChange)
    path = b'fc/standby_eskf/accel_nis'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    msg.scalars[0] = 120.
    writer.handle(msg)
    assert isinstance(archive.events[-1][1][1], rr.Scalars)


def test_standby_health_is_state_and_imu_age_remains_recorded(recordings_factory):
    import rerun as rr

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage(kind=VIZ_KIND_SCALARS, scalar_count=1)
    path = b'fc/standby_eskf/health'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    for stamp, ready in [(1., 0), (2., 1), (3., 1), (4., 0)]:
        msg.timestamp, msg.scalars[0] = stamp, ready
        writer.handle(msg)
    assert [args[1].state.as_arrow_array().to_pylist() for _, args in archive.events] == [
        ['Not ready'], ['Ready'], ['Not ready']]
    assert all(isinstance(args[1], rr.StateChange) for _, args in archive.events)
    msg.scalars[0] = 2.
    with pytest.raises(ValueError, match='health state'):
        writer.handle(msg)
    msg.kind = VIZ_KIND_CLEAR
    writer.handle(msg)
    msg.kind, msg.scalars[0] = VIZ_KIND_SCALARS, 0.
    writer.handle(msg)
    assert isinstance(archive.events[-1][1][1], rr.StateChange)
    path = b'fc/standby_eskf/imu_age_s'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    msg.scalars[0] = .01
    writer.handle(msg)
    assert archive.events[-1][1][0] == 'fc/standby_eskf/imu_age_s'
    assert archive.events[-1][1][1].scalars.as_arrow_array().to_pylist() == [.01]


def test_imu_source_records_state_transitions(recordings_factory):
    import rerun as rr

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage(kind=VIZ_KIND_SCALARS, scalar_count=1)
    path = b'fc/imu/source'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    for stamp, source in [(1., 0), (2., 1), (3., 1), (4., 2), (5., 0)]:
        msg.timestamp, msg.scalars[0] = stamp, source
        writer.handle(msg)
    assert [args[1].state.as_arrow_array().to_pylist() for _, args in archive.events] == [
        ['Unavailable'], ['OKVIS'], ['Standby ESKF'], ['Unavailable']]
    assert all(isinstance(args[1], rr.StateChange) for _, args in archive.events)
    msg.scalars[0] = 3.
    with pytest.raises(ValueError, match='source state'):
        writer.handle(msg)
    msg.kind = VIZ_KIND_CLEAR
    writer.handle(msg)
    msg.kind, msg.scalars[0] = VIZ_KIND_SCALARS, 0.
    writer.handle(msg)
    assert isinstance(archive.events[-1][1][1], rr.StateChange)


def test_imu_health_separates_states_from_age(recordings_factory):
    import rerun as rr

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage(kind=VIZ_KIND_SCALARS, scalar_count=3)
    path = b'fc/imu/health'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    for stamp, values in [(1., [0., 0., -1.]), (2., [1., 1., .1]),
                          (3., [1., 1., .2]), (4., [1., 0., .6])]:
        msg.timestamp, msg.scalars[:3] = stamp, values
        writer.handle(msg)
    states = [(args[0], args[1].state.as_arrow_array().to_pylist())
              for _, args in archive.events if isinstance(args[1], rr.StateChange)]
    assert states == [('fc/imu/attitude_ready', ['Unavailable']), ('fc/imu/vio_fresh', ['Stale']),
                      ('fc/imu/attitude_ready', ['Available']), ('fc/imu/vio_fresh', ['Fresh']),
                      ('fc/imu/vio_fresh', ['Stale'])]
    ages = [args[1].scalars.as_arrow_array().to_pylist()
            for _, args in archive.events if args[0] == 'fc/imu/posterior_age_s']
    assert ages == [[-1.], [.1], [.2], [.6]]
    msg.scalars[0] = 2.
    with pytest.raises(ValueError, match='invalid IMU health'):
        writer.handle(msg)


@pytest.mark.parametrize('entity,values,expected', [
    ('fc/debug/state', [0., 3., 4., 6.], ['Disarmed', 'Hold', 'Track', 'Attitude fallback']),
    ('fc/failsafe/action', [0., 1., 2., 3.], ['None', 'Hold', 'Attitude fallback', 'Disarm']),
    ('fc/debug/saturated', [0., 1., 0.], ['Not saturated', 'Saturated', 'Not saturated']),
    ('fc/arming/failed', [0., 2., 32.], ['No blocking check', 'Estimator not ready', 'IMU stale']),
    ('fc/failsafe/flags', [0., 9., 128.], ['No active failsafe', 'Attitude invalid, IMU stale', 'Reference lost']),
])
def test_flight_modes_and_safety_codes_are_named_states(recordings_factory, entity, values, expected):
    import rerun as rr

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage(kind=VIZ_KIND_SCALARS, scalar_count=1)
    path = entity.encode()
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    for stamp, value in enumerate(values, 1):
        msg.timestamp, msg.scalars[0] = stamp, value
        writer.handle(msg)
        writer.handle(msg)
    assert all(isinstance(args[1], rr.StateChange) for _, args in archive.events)
    assert [args[1].state.as_arrow_array().to_pylist()[0] for _, args in archive.events] == expected


def test_saturation_count_is_a_separate_numeric_curve(recordings_factory):
    import rerun as rr

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage(kind=VIZ_KIND_SCALARS, scalar_count=1, timestamp=1.)
    path = b'fc/debug/saturated_ticks'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    msg.scalars[0] = 450.
    writer.handle(msg)
    assert isinstance(archive.events[0][1][1], rr.Scalars)
    path = b'fc/debug/saturated'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    with pytest.raises(ValueError, match='saturated state'):
        writer.handle(msg)


def test_optional_localization_diagnostics_separate_states_and_values(recordings_factory):
    import rerun as rr

    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = VizMessage(kind=VIZ_KIND_SCALARS, timestamp=1., scalar_count=4)
    path = b'corr/debug/gate'
    msg.entity_len, msg.entity[:len(path)] = len(path), path
    msg.scalars[:] = [3., 10., .2, 1.]
    writer.handle(msg)
    assert archive.events[0][1][0] == 'corr/gate/accepted'
    assert isinstance(archive.events[0][1][1], rr.StateChange)
    assert [args[0] for _, args in archive.events[1:]] == [
        'corr/gate/metric', 'corr/gate/limit', 'corr/gate/applied_translation_m']
    msg.kind = VIZ_KIND_CLEAR
    writer.handle(msg)
    msg.kind = VIZ_KIND_SCALARS
    writer.handle(msg)
    assert sum(isinstance(args[1], rr.StateChange) for _, args in archive.events) == 2


def test_ground_truth_records_world_pose_and_incremental_trajectory(recordings_factory):
    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = OdomMessage(timestamp=1., position_x=1., position_y=2., position_z=3., quat_w=1.)
    writer.consume(('gt', msg))
    assert not archive.calls
    msg.is_initialized = True
    writer.consume(('gt', msg))
    msg.timestamp, msg.position_x = 2., 4.
    msg.quat_z, msg.quat_w = 1., 0.
    writer.consume(('gt', msg))
    assert [args[0] for _, args in archive.events] == ['gt/pose', 'gt/pose', 'gt/traj']
    assert [stamp for stamp, _ in archive.events] == [
        ('sim_time', {'duration': 1.}), ('sim_time', {'duration': 2.}), ('sim_time', {'duration': 2.})]
    assert archive.calls[0][0][1].translation.as_arrow_array().to_pylist() == [[1., 2., 3.]]
    assert archive.calls[1][0][1].quaternion.as_arrow_array().to_pylist() == [[0., 0., 1., 0.]]
    assert archive.calls[2][0][1].strips.as_arrow_array().to_pylist() == [[[1., 2., 3.], [4., 2., 3.]]]
    assert all(not kwargs.get('static', False) for _, kwargs in archive.calls)


def test_corrected_odometry_stays_in_map_frame_with_an_independent_trajectory(recordings_factory):
    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    writer.consume(('viz', okvis_pose(1., [0., 0., 1.])))
    for stamp, x in [(1., 10.), (2., 11.)]:
        writer.consume(('map', OdomMessage(timestamp=stamp, position_x=x, quat_w=1., is_initialized=True)))
    assert [args[0] for args, _ in archive.calls] == [
        'okvis/odom', 'okvis/map/pose', 'okvis/map/pose', 'okvis/map/traj']
    assert archive.calls[-1][0][1].strips.as_arrow_array().to_pylist() == [[[10., 0., 0.], [11., 0., 0.]]]
    assert writer.previous_poses['okvis/odom'] == (1., [0., 0., 1.])


def test_okvis_trajectory_stores_two_point_segments_with_one_cached_endpoint(recordings_factory):
    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    for i in range(32):
        writer.consume(('viz', okvis_pose(float(i), [float(i), 0., 1.])))
    segments = [args[1].strips.as_arrow_array().to_pylist() for args, _ in archive.calls
                if args[0] == 'okvis/traj']
    assert segments == [[[[float(i - 1), 0., 1.], [float(i), 0., 1.]]] for i in range(1, 32)]
    assert writer.previous_poses == {'okvis/odom': (31., [31., 0., 1.])}
    assert not any(args[0].startswith('gt/') for args, _ in archive.calls)
    calls = len(archive.calls)
    writer.consume(('viz', okvis_pose(31., [31., 0., 1.])))
    assert len(archive.calls) == calls
    with pytest.raises(ValueError, match='regressed pose timestamp'):
        writer.consume(('viz', okvis_pose(30., [30., 0., 1.])))
    assert len(archive.calls) == calls


@pytest.mark.parametrize('field,value', [('timestamp', float('nan')), ('timestamp', -1.),
                                        ('position_z', float('inf')), ('quat_w', 0.)])
def test_invalid_ground_truth_cannot_reach_recording(recordings_factory, field, value):
    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    msg = OdomMessage(timestamp=1., quat_w=1., is_initialized=True)
    setattr(msg, field, value)
    with pytest.raises(ValueError, match='invalid pose'):
        writer.consume(('gt', msg))
    assert not archive.calls
    assert not writer.previous_poses


def test_clear_trajectory_breaks_the_next_segment(recordings_factory):
    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    writer.handle(okvis_pose(1., [0., 0., 1.]))
    clear = VizMessage()
    path = b'okvis/traj'
    clear.entity_len, clear.entity[:len(path)] = len(path), path
    clear.kind, clear.timestamp = VIZ_KIND_CLEAR, 2.
    writer.handle(clear)
    writer.handle(okvis_pose(3., [5., 0., 1.]))
    assert len(archive.calls) == 3
    assert writer.previous_poses == {'okvis/odom': (3., [5., 0., 1.])}


def test_cold_start_empty_keypoints_preserve_typed_ids_and_clear_previous_points(tmp_path):
    import signal
    import subprocess
    import sys
    import textwrap

    import pyarrow as pa
    from rerun.chunk import RrdReader

    path = tmp_path / 'keypoints.rrd'
    # 独立 SDK 进程保证首帧没有 track_id 类型缓存。
    code = textwrap.dedent('''
        import sys
        from unittest.mock import create_autospec
        import rerun as rr
        from firefly_viz.config import Options
        from firefly_viz.messages import VizMessage, VIZ_KIND_KEYPOINTS_2D
        from firefly_viz.recording import MatchedPoints, RecordingOutputs
        from firefly_viz.writer import Writer

        archive = rr.RecordingStream('cold-start-keypoints')
        archive.set_sinks(rr.FileSink(sys.argv[1], write_footer=True))
        matches = MatchedPoints(archive, 4)
        outputs = create_autospec(RecordingOutputs, instance=True, spec_set=True)
        outputs.archive, outputs.matches = archive, matches
        writer = Writer(outputs, Options())
        try:
            matches.frame('left', 0.5, 0)
            for stamp, ids in [(1., []), (2., [42]), (3., [])]:
                msg = VizMessage()
                msg.kind, msg.timestamp = VIZ_KIND_KEYPOINTS_2D, stamp
                entity = b'okvis/debug/keypoints/left'
                msg.entity_len, msg.entity[:len(entity)] = len(entity), entity
                msg.scalar_count, msg.scalars[:3] = 3, [320., 240., float(len(ids))]
                msg.point_count = len(ids)
                for index, track_id in enumerate(ids):
                    msg.points[index][:] = [12.25, 41.5, track_id]
                writer.handle(msg)
                matches.frame('left', stamp, int(stamp))
            archive.flush(timeout_sec=5.)
        finally:
            archive.disconnect()
    ''')
    with subprocess.Popen([sys.executable, '-c', code, str(path)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as process:
        try:
            _, stderr = process.communicate()
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.communicate(timeout=5.)
        assert process.returncode == 0, stderr
    rows = {'/okvis/debug/keypoints/left': [], '/cameras/left/points': []}
    for chunk in RrdReader(path).stream():
        if chunk.entity_path not in rows:
            continue
        batch = chunk.to_record_batch()
        ids = batch.column('track_id')
        assert ids.type == pa.list_(pa.uint64())
        rows[chunk.entity_path].extend(zip(batch.column('sim_time').cast(pa.int64()).to_pylist(),
                                          batch.column('Points2D:positions').to_pylist(), ids.to_pylist()))
    assert rows['/okvis/debug/keypoints/left'] == []
    assert sorted(rows['/cameras/left/points']) == [
        (500_000_000, [], []), (1_000_000_000, [], []),
        (2_000_000_000, [[12.75, 42.]], [42]), (3_000_000_000, [], [])]


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


def test_voxel_history_uses_complete_snapshots_after_ordered_deltas(recordings_factory):
    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    writer.handle(voxel(3, [(1, 2, 3), (4, 5, 6)]))
    writer.handle(voxel(8, [(7, 8, 9)], 2., sequence=1))
    writer.handle(voxel(9, [(1, 2, 3)], 2., sequence=2))
    writer.tick(final=True)
    assert writer.voxels['plan/voxels'][0] == {(4, 5, 6), (7, 8, 9)}
    assert len(archive.calls) == 1
    assert archive.calls[0][0][0] == 'plan/voxels'
    assert not archive.calls[0][1].get('static', False)
    assert not any(args[0].startswith('live/') for args, _ in archive.calls)
    with pytest.raises(ValueError, match='out-of-order'):
        writer.handle(voxel(9, [(4, 5, 6)], 1., sequence=3))


def test_voxel_delta_cannot_create_an_unknown_baseline(recordings_factory):
    writer = Writer(recordings_factory(archive=Recording()), Options())
    with pytest.raises(ValueError, match='baseline'):
        writer.handle(voxel(8, [(1, 2, 3)]))


def test_voxel_chunk_commit_is_atomic_and_gaps_are_fatal(recordings_factory):
    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    writer.handle(voxel(3, [(1, 2, 3)], chunk_count=2))
    writer.tick()
    assert not archive.calls
    assert not writer.voxels
    writer.handle(voxel(3, [(4, 5, 6)], sequence=1, chunk_index=1, chunk_count=2))
    writer.tick(final=True)
    assert writer.voxels['plan/voxels'][0] == {(1, 2, 3), (4, 5, 6)}
    with pytest.raises(ValueError, match='sequence gap'):
        writer.handle(voxel(9, [(1, 2, 3)], 2., sequence=3))


def test_voxel_partial_update_fails_at_shutdown_and_empty_snapshot_clears(recordings_factory):
    archive = Recording()
    writer = Writer(recordings_factory(archive=archive), Options())
    writer.handle(voxel(3, [], chunk_count=1))
    writer.tick(final=True)
    assert writer.voxels['plan/voxels'][0] == set()
    writer.handle(voxel(3, [(1, 2, 3)], 2., sequence=1, chunk_count=2))
    with pytest.raises(ValueError, match='during a voxel update'):
        writer.tick(final=True)
