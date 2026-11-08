"""Real IPC → release mimir → indexed RRD; requires an idle workspace."""

from datetime import datetime, timezone
from pathlib import Path
import os
import signal
import threading
import time

import iceoryx2 as iox2
import psutil
import pytest
from rerun.chunk import RrdReader

from firefly_evaluation.process_lifecycle import ManagedProcess, isolated_run
from firefly_mujoco import TraceContext
from firefly_mujoco.messages import ScalarMessage, LogMessage, GrayImageMessage, VizMessage
from firefly_workspace import digest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.skipif(
    os.environ.get("FIREFLY_RUN_TELEMETRY_LOAD") != "1",
    reason="explicit native recorder test on an idle host with release mimir",
)


def publisher(node, topic, payload, capacity=None, publishers=None):
    builder = node.service_builder(iox2.ServiceName.new(topic)).publish_subscribe(payload).user_header(TraceContext)
    if capacity is not None:
        builder = builder.subscriber_max_buffer_size(capacity)
    if publishers is not None:
        builder = builder.max_publishers(publishers)
    return builder.open_or_create().publisher_builder().create()


def scalar(name, stamp, sequence, values):
    message = ScalarMessage(kind=4, timestamp=stamp, sequence=sequence, scalar_count=len(values))
    encoded = name.encode()
    message.entity_len, message.entity[:len(encoded)] = len(encoded), encoded
    message.scalars[:len(values)] = values
    return message


def event(port, sequence, text):
    message = LogMessage(sim_time=-1., wall_secs=int(time.time()), log_level=3, sequence=sequence)
    tag, data = b"test-mimir", text.encode()
    assert len(data) <= len(message.text)
    message.tag_len, message.tag[:len(tag)] = len(tag), tag
    message.text_len, message.text[:len(data)] = len(data), data
    port.loan_uninit().write_payload(message).send()


def recording():
    binary = ROOT / "target/release/mimir"
    assert binary.is_file(), "build release mimir first"
    path = ROOT / "logs" / f"test_mimir_{datetime.now(timezone.utc):%Y%m%dT%H%M%S%fZ}.rrd"
    ready = threading.Event()
    process = ManagedProcess([str(binary), "--save", str(path)], cwd=ROOT,
                             env={**os.environ, "RUST_LOG": "info"},
                             on_line=lambda line: ready.set() if "mimir subscriptions ready" in line else None)
    try:
        assert ready.wait(10.), list(process.tail)
    except BaseException:
        process.stop()
        raise
    return process, path


def read(path):
    reader = RrdReader(path)
    # Indexed access requires the completed footer, independently of sequential decoding.
    assert reader.store(store=reader.recordings()[0]).schema() is not None
    result, logs = {}, []
    for chunk in reader.stream():
        if chunk.is_static:
            continue
        batch = chunk.to_record_batch()
        if "Scalars:scalars" in batch.schema.names:
            result.setdefault(chunk.entity_path.lstrip("/"), []).extend(zip(
                batch.column("sim_time").cast("int64").to_pylist(),
                batch.column("Scalars:scalars").to_pylist(),
            ))
        if chunk.entity_path == "/logs/mimir" and "TextLog:text" in batch.schema.names:
            logs.extend(value for row in batch.column("TextLog:text").to_pylist() for value in row)
    return result, logs


@pytest.mark.parametrize("rate", [1000, 5000])
@pytest.mark.parametrize("repeat", range(3))
def test_ipc_recording_retains_every_sample(rate, repeat):
    with isolated_run(ROOT):
        iox2.set_log_level(iox2.LogLevel.Error)
        process, path = recording()
        node = port = events = camera_port = None
        cameras = []
        try:
            node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
            port = publisher(node, "Firefly/VizScalars", ScalarMessage, 1024, 8)
            cameras = [publisher(node, "Firefly/Camera" + side, GrayImageMessage) for side in ("Left", "Right")]
            events = publisher(node, "Firefly/Log", LogMessage, 256, 10)
            event(events, 0, f"rate={rate} repeat={repeat} cpu_percent={psutil.cpu_percent(interval=.1)} "
                  f"mimir_sha256={digest(ROOT / 'target/release/mimir')} "
                  "GPU/temperature/throttling counters unavailable in this portable test")
            names = ["fc/loop/" + name for name in ("timing", "roll", "pitch", "control", "motors")]
            expected = {name: [] for name in names}
            samples = rate * 3
            start = time.monotonic()
            for first in range(0, samples, 5):
                deadline = start + first / rate
                while time.monotonic() < deadline:
                    time.sleep(min(.0005, max(0., deadline - time.monotonic())))
                assert process.process.poll() is None, list(process.tail)
                if first % (rate // 10) == 0:
                    for camera, camera_port in enumerate(cameras):
                        frame = GrayImageMessage(timestamp=first / rate, sensor_id=camera, width=320, height=240)
                        frame.data[:] = bytes((pixel + first + camera) % 256 for pixel in range(len(frame.data)))
                        camera_port.loan_uninit().write_payload(frame).send()
                for index in range(first, first + 5):
                    name = names[index % 5]
                    # Each entity gets repeated timestamps, which must remain distinct rows.
                    stamp = (index // 10) / 1024
                    values = [index, -index, index / 4, index / 8]
                    port.loan_uninit().write_payload(scalar(name, stamp, index, values)).send()
                    expected[name].append((int(stamp * 1e9 + .5), values))
            time.sleep(.1)
            event(events, 1, f"samples={samples} cpu_percent={psutil.cpu_percent(interval=.1)}")
        finally:
            code = process.stop()
            cameras.clear()
            node = port = events = camera_port = None
            assert code == 0, list(process.tail)
        actual, logs = read(path)
        assert {name: sorted(actual.get(name, [])) for name in names} == expected
        assert "recording complete" in logs
        assert any(f"scalar_samples={samples} " in entry for entry in logs)
        print({"rate": rate, "repeat": repeat, "samples": samples,
               "seconds": time.monotonic() - start, "statistics": logs, "rrd": str(path)})


@pytest.mark.parametrize("fault", ["sequence", "producer_loss", "unknown_entity"])
def test_incomplete_recording_is_an_error(fault):
    with isolated_run(ROOT):
        process, path = recording()
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        try:
            if fault == "producer_loss":
                port = publisher(node, "Firefly/Log", LogMessage, 256, 10)
                message = LogMessage(dropped_records=1)
                port.loan_uninit().write_payload(message).send()
            else:
                port = publisher(node, "Firefly/VizScalars", ScalarMessage, 1024, 8)
                name = "unknown/value" if fault == "unknown_entity" else "fc/debug/altitude"
                for seq in (0, 2):
                    port.loan_uninit().write_payload(scalar(name, 1., seq, [1.])).send()
            assert process.process.wait(timeout=10.) != 0
        finally:
            assert process.stop() != 0
        _, logs = read(path)
        assert "recording complete" not in logs
        assert any(("sequence gap" in entry or "lost 1 log" in entry or "unregistered" in entry) for entry in logs)


@pytest.mark.parametrize("stream", ["scalars", "logs"])
def test_shutdown_drains_delivered_backlog(stream):
    with isolated_run(ROOT):
        process, path = recording()
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        ports = []
        try:
            if stream == "scalars":
                ports = [publisher(node, "Firefly/VizScalars", ScalarMessage, 1024, 8)]
                count = 600
            else:
                ports = [publisher(node, "Firefly/Log", LogMessage, 256, 10) for _ in range(3)]
                count = 200
            pid = process.process.pid
            os.kill(pid, signal.SIGSTOP)
            stopped_pid, status = os.waitpid(pid, os.WUNTRACED)
            assert stopped_pid == pid and os.WIFSTOPPED(status)
            for port_index, port in enumerate(ports):
                for index in range(count):
                    if stream == "scalars":
                        port.loan_uninit().write_payload(
                            scalar("fc/debug/altitude", index / 1024, index, [index])
                        ).send()
                    else:
                        event(port, index, f"shutdown backlog {port_index}:{index}")
            # All sends have completed; no producer writes after the stop request.
            os.kill(pid, signal.SIGINT)
            os.kill(pid, signal.SIGCONT)
            assert process.process.wait(timeout=10.) == 0
        finally:
            if process.process.poll() is None:
                os.kill(process.process.pid, signal.SIGCONT)
            code = process.stop()
            ports.clear()
            assert code == 0, list(process.tail)
        actual, logs = read(path)
        assert "recording complete" in logs
        if stream == "scalars":
            assert actual["fc/debug/altitude"] == [
                (int(index / 1024 * 1e9 + .5), [float(index)]) for index in range(count)
            ]
        else:
            recorded = []
            for chunk in RrdReader(path).stream():
                if chunk.entity_path == "/logs/test-mimir":
                    recorded.extend(text for row in chunk.to_record_batch().column("TextLog:text").to_pylist()
                                    for text in row)
            assert sorted(recorded) == sorted(
                f"shutdown backlog {port_index}:{index}" for port_index in range(3) for index in range(count)
            )


def test_stereo_images_and_keypoint_ids_survive_recording():
    with isolated_run(ROOT):
        process, path = recording()
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        images = [publisher(node, "Firefly/Camera" + side, GrayImageMessage) for side in ("Left", "Right")]
        geometry = publisher(node, "Firefly/Viz", VizMessage, 256, 8)
        try:
            for camera, port in enumerate(images):
                msg = GrayImageMessage(timestamp=2., sensor_id=camera, width=320, height=240)
                msg.data[:] = bytes([30 + camera]) * len(msg.data)
                port.loan_uninit().write_payload(msg).send()
            time.sleep(.05)
            for camera in range(2):
                msg = VizMessage(kind=10, timestamp=2., sequence=camera, scalar_count=3, point_count=2)
                name = ("vio/diagnostics/keypoints/" + ("left" if camera == 0 else "right")).encode()
                msg.entity_len, msg.entity[:len(name)] = len(name), name
                msg.scalars[:] = [320., 240., 2., 0.]
                msg.points[0][:], msg.points[1][:] = [4., 6., 42.], [8., 10., 43.]
                geometry.loan_uninit().write_payload(msg).send()
            time.sleep(.1)
        finally:
            assert process.stop() == 0, list(process.tail)
        found_images, found_points = set(), set()
        for chunk in RrdReader(path).stream():
            batch = chunk.to_record_batch()
            for camera, side in enumerate(("left", "right")):
                if chunk.entity_path == f"/cameras/{side}/image":
                    assert batch.column("sim_time").cast("int64").to_pylist() == [2_000_000_000]
                    assert bytes(batch.column("Image:buffer")[0].as_py()[0]) == bytes([30 + camera]) * (320 * 240)
                    found_images.add(side)
                if chunk.entity_path == f"/cameras/{side}/points" and "track_id" in batch.schema.names:
                    assert batch.column("track_id").to_pylist() == [[42., 43.]]
                    found_points.add(side)
        assert found_images == found_points == {"left", "right"}
