"""Lossless source validation and causal scheduling, without flight processes."""

import json
import os
from pathlib import Path
import sys
import threading
import time
import uuid

import iceoryx2 as iox2
import pytest
import rerun as rr

from firefly_evaluation.sensor_replay import (
    Event, FORMAT_VERSION, completion_evidence, frame_coverage, open_recording,
    posterior_frame_times, read_capture, schedule, select_segment, summarize, write_event,
)
from firefly_mujoco import GrayImageMessage, ImuMessage, OdomMessage
from firefly_mujoco.messages import IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH
from firefly_evaluation.mission_io import service
from firefly_evaluation.process_lifecycle import ManagedProcess
from firefly_evaluation.sensor_replay import TOPICS


def event(kind, order, stamp):
    cls = {"imu": ImuMessage, "ground_truth": OdomMessage,
           "left": GrayImageMessage, "right": GrayImageMessage}[kind]
    message = cls()
    message.timestamp = stamp
    if kind in ("left", "right"):
        message.sensor_id = 0 if kind == "left" else 1
        message.width, message.height = IMAGE_WIDTH, IMAGE_HEIGHT
        message.data[:] = bytes((index * 13 + message.sensor_id) % 256 for index in range(IMAGE_SIZE))
    return Event(kind, order, bytes(message))


def source():
    return [event(kind, order, stamp) for order, (kind, stamp) in enumerate([
        ("imu", 0.), ("ground_truth", 0.), ("left", 0.005),
        ("right", 0.005), ("imu", 0.01), ("imu", 0.02), ("imu", 0.03),
    ])]


def test_raw_rrd_roundtrip_preserves_every_payload_byte(tmp_path):
    events = source()
    path = tmp_path / "sensors.rrd"
    recording = open_recording(path)
    for value in events:
        write_event(recording, value, value.order * 1_000_000, 123)
    expected = summarize(events)
    recording.log("capture/manifest", rr.TextDocument(json.dumps({
        "format_version": FORMAT_VERSION, "complete": True, "streams": expected,
    })), static=True)
    recording.flush()
    recording.disconnect()
    actual, manifest = read_capture(path)
    assert sorted(actual, key=lambda e: e.order) == events
    assert manifest["streams"] == expected


def test_schedule_brackets_exact_stereo_payloads_and_never_publishes_truth():
    events = source()
    batches = schedule(events)
    assert [(stamp, tuple(e.kind for e in batch)) for stamp, batch in batches] == [
        (0., ("imu",)), (0.01, ("imu",)), (0.01, ("left", "right")), (0.02, ("imu",)), (0.03, ("imu",)),
    ]
    published = [e for _, batch in batches for e in batch]
    assert {e.order: e.payload for e in published} == {
        e.order: e.payload for e in events if e.kind != "ground_truth"
    }


def test_missing_raw_stream_and_unpaired_or_unbracketed_images_are_rejected():
    with pytest.raises(ValueError, match="missing raw stream"):
        summarize([e for e in source() if e.kind != "imu"])
    with pytest.raises(ValueError, match="matching frame"):
        schedule([e for e in source() if e.kind != "right"])
    bad = [event(e.kind, e.order, 0.03) if e.kind in ("left", "right") else e for e in source()]
    with pytest.raises(ValueError, match="bracketed"):
        schedule(bad)


def test_repeated_or_missing_input_sequence_is_rejected():
    events = source()
    events[-1] = event("imu", events[-1].order, 0.01)
    with pytest.raises(ValueError, match="regressed"):
        summarize(events)
    events = source()
    events[-1] = event("imu", 10, 0.03)
    with pytest.raises(ValueError, match="sequence"):
        summarize(events)


def test_explicit_segment_keeps_future_imu_without_silently_trimming_source():
    events = source() + [event("left", 7, 0.03), event("right", 8, 0.03)]
    with pytest.raises(ValueError, match="bracketed"):
        schedule(events)
    selected = select_segment(events, 0.005)
    assert [e.message().timestamp for e in selected if e.kind == "imu"] == [0., 0.01, 0.02, 0.03]
    assert [e.payload for e in selected if e.kind in ("left", "right")] == [events[2].payload, events[3].payload]
    assert len(events) == 9
    assert len(schedule(selected)) == 5
    with pytest.raises(ValueError, match="last frame"):
        select_segment(events, 0.03)
    with pytest.raises(ValueError, match="last frame"):
        select_segment(events, None)
    with pytest.raises(ValueError, match="last frame"):
        select_segment(source()[:-1], 0.005)
    assert [e.message().timestamp for e in select_segment(source(), 0.005, imu_tail_s=0.)
            if e.kind == "imu"] == [0., 0.01]


def test_missing_imu_intervals_are_reported_independently_of_capture_sequence():
    events = source()
    events[-1] = event("imu", 6, 0.05)
    imu = summarize(events, expected_imu_rate=100.)["imu"]
    assert imu["intervals_with_missing_samples"] == 1
    assert imu["inferred_missing_samples"] == 2
    assert imu["max_interval_s"] == pytest.approx(0.03)


def test_completion_requires_last_imu_and_exact_accepted_camera_posterior():
    assert completion_evidence((14.001, 14.), 14., 14., 14.001) == "VioAttitude.timestamp+posterior_time"
    assert completion_evidence((14.001, 13.98), 14., 14., 14.001) == "VioAttitude.timestamp+accepted_posterior_diagnostic"
    assert completion_evidence((14., 14.), 14., 14., 14.001) is None
    assert completion_evidence((14.001, 13.98), 13.98, 14., 14.001) is None
    assert completion_evidence((14.001, 14.01), 14.01, 14., 14.001) is None
    assert completion_evidence(None, 14., 14., 14.001) is None


def test_frame_coverage_accepts_only_initialization_prefix_and_timestamp_rounding():
    expected = [10, 20, 30, 40]
    full = frame_coverage(expected, expected)
    assert full["valid"] and full["initialization_prefix_frames"] == 0
    prefix = frame_coverage(expected, [22, 28, 40])
    assert prefix["valid"] and prefix["initialization_prefix_frames"] == 1
    assert prefix["matched_frames"] == prefix["posterior_rows"] == 3
    assert not frame_coverage(expected, [23, 30, 40])["valid"]
    with pytest.raises(ValueError, match="unambiguous"):
        frame_coverage([10, 10], [10])


@pytest.mark.parametrize(("posterior", "field", "values"), [
    ([20, 40], "missing_frame_ns", [30]),
    ([20, 30], "missing_frame_ns", [40]),
    ([20, 30, 31, 40], "duplicate_frame_ns", [30]),
    ([20, 30, 40, 50], "foreign_posterior_ns", [50]),
    ([5, 20, 30, 40], "foreign_posterior_ns", [5]),
])
def test_frame_coverage_rejects_missing_duplicate_and_foreign_posteriors(posterior, field, values):
    coverage = frame_coverage([10, 20, 30, 40], posterior)
    assert not coverage["valid"]
    assert coverage[field] == values


def test_frame_coverage_rejects_no_posteriors():
    coverage = frame_coverage([10, 20], [])
    assert not coverage["valid"]
    assert coverage["posterior_rows"] == 0
    assert coverage["first_posterior_ns"] is None


def test_posterior_frame_times_reads_real_rrd_rows_without_deduplicating(tmp_path):
    import numpy as np

    path = tmp_path / "vio.rrd"
    recording = open_recording(path)
    for stamp in [20, 20, 40]:
        recording.set_time("sim_time", duration=np.timedelta64(stamp, "ns"))
        recording.log("vio/posterior/velocity_world", rr.Scalars([0., 0., 0.]))
    recording.set_time("sim_time", duration=np.timedelta64(50, "ns"))
    recording.log("unrelated", rr.Scalars([1.]))
    recording.flush()
    recording.disconnect()
    actual = posterior_frame_times(path)
    assert sorted(actual) == [20, 20, 40]
    coverage = frame_coverage([10, 20, 30, 40], actual)
    assert not coverage["valid"]
    assert coverage["duplicate_frame_ns"] == [20]
    assert coverage["missing_frame_ns"] == [30]


def test_incomplete_capture_is_never_accepted(tmp_path):
    path = tmp_path / "incomplete.rrd"
    recording = open_recording(path)
    for value in source():
        write_event(recording, value, value.order)
    recording.flush()
    recording.disconnect()
    with pytest.raises(ValueError, match="complete supported manifest"):
        read_capture(path)


def test_real_ipc_capture_and_replay_preserve_raw_inputs(tmp_path):
    prefix = "FireflyReplayTest/" + uuid.uuid4().hex
    topics = {kind: (prefix + "/" + topic.rsplit("/", 1)[1], cls)
              for kind, (topic, cls) in TOPICS.items()}
    setup = """
import logging, sys
from pathlib import Path
from firefly_evaluation import sensor_replay as m
logging.basicConfig(level=logging.INFO)
topics = {k: (sys.argv[1] + '/' + t.rsplit('/', 1)[1], c) for k, (t, c) in m.TOPICS.items()}
"""
    source_path = tmp_path / "sensors.rrd"
    ready = threading.Event()
    child = ManagedProcess(
        [sys.executable, "-c", setup + "m.capture(Path(sys.argv[2]), topics=topics)", prefix, str(source_path)],
        cwd=Path.cwd(), env=dict(os.environ),
        on_line=lambda line: ready.set() if "raw capture ready" in line else None,
    )
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    publishers = {}
    try:
        assert ready.wait(10.), list(child.tail)
        for kind, (topic, cls) in topics.items():
            publishers[kind] = service(node, topic, cls, 128).publisher_builder().create()
        for value in source():
            assert publishers[value.kind].loan_uninit().write_payload(value.message()).send() == 1
        assert child.stop() == 0, list(child.tail)
    finally:
        child.stop()
        for publisher in publishers.values():
            publisher.delete()
    captured, _ = read_capture(source_path)
    assert {(e.kind, e.message().timestamp): e.payload for e in captured} == {
        (e.kind, e.message().timestamp): e.payload for e in source()
    }
    subscribers = {kind: service(node, topic, cls, 128).subscriber_builder().buffer_size(128).create()
                   for kind, (topic, cls) in topics.items()}
    listeners = {
        name: node.service_builder(iox2.ServiceName.new(prefix + "/" + name))
        .event().open_or_create().listener_builder().create() for name in ("Imu", "CameraPair")
    }
    delivery = tmp_path / "delivery.rrd"
    child = ManagedProcess(
        [sys.executable, "-c", setup + "m.replay(Path(sys.argv[2]), Path(sys.argv[3]), topics=topics)",
         prefix, str(source_path), str(delivery)], cwd=Path.cwd(), env=dict(os.environ),
    )
    received = {kind: [] for kind in subscribers}
    headers = []
    notifications = dict.fromkeys(listeners, 0)
    deadline = time.monotonic() + 10.
    try:
        while child.process.poll() is None and time.monotonic() < deadline:
            for kind, subscriber in subscribers.items():
                while (sample := subscriber.receive()) is not None:
                    received[kind].append(bytes(sample.payload().contents))
                    header = sample.user_header().contents
                    headers.append((header.version, header.send_ts_secs, header.send_ts_mono_ns))
                    del sample
            for name, listener in listeners.items():
                notifications[name] += len(listener.try_wait())
            time.sleep(0.0005)
        assert child.stop() == 0, list(child.tail)
    finally:
        child.stop()
        for subscriber in subscribers.values():
            subscriber.delete()
        for listener in listeners.values():
            listener.delete()
    for kind, payloads in received.items():
        assert payloads == ([] if kind == "ground_truth" else [e.payload for e in source() if e.kind == kind])
    assert headers and all(version == 1 and unix > 0 and mono > 0 for version, unix, mono in headers)
    assert notifications == {"Imu": 4, "CameraPair": 1}
