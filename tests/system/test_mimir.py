"""Real IPC → release mimir → indexed RRD; requires an idle workspace."""

from datetime import datetime, timezone
from pathlib import Path
import os
import signal
import threading
import time
import ctypes

import iceoryx2 as iox2
import psutil
import pytest
from rerun.chunk import RrdReader

from firefly_evaluation.process_lifecycle import ManagedProcess, isolated_run
from firefly_mujoco import TraceContext
from firefly_mujoco.messages import ScalarMessage, LogMessage, GrayImageMessage, VizMessage, OdomMessage
from firefly_workspace import digest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.skipif(
    os.environ.get("FIREFLY_RUN_TELEMETRY_LOAD") != "1",
    reason="explicit native recorder test on an idle host with release mimir",
)


class NavigationMessage(ctypes.Structure):
    """Test publisher for firefly_pubsub::navigation::NavigationMessage; IPC verifies the ABI."""

    _fields_ = [("odom", OdomMessage), ("translation", ctypes.c_double * 3),
                ("rotation", ctypes.c_double * 4), ("revision", ctypes.c_uint64),
                ("external_map", ctypes.c_uint64)]

    @staticmethod
    def type_name():
        return "FireflyNavigationMessage"


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
    assert len(reader.blueprints()) == 1
    assert reader.blueprints()[0].application_id == "firefly"
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
        node = port = events = camera_port = navigation_port = None
        cameras = []
        try:
            node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
            port = publisher(node, "Firefly/VizScalars", ScalarMessage, 1024, 8)
            cameras = [publisher(node, "Firefly/Camera" + side, GrayImageMessage) for side in ("Left", "Right")]
            events = publisher(node, "Firefly/Log", LogMessage, 256, 10)
            navigation_port = publisher(node, "Firefly/Navigation", NavigationMessage)
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
                navigation = NavigationMessage(odom=OdomMessage(timestamp=first / rate, quat_w=1., is_initialized=True))
                navigation.rotation[:] = (0., 0., 0., 1.)
                navigation_port.loan_uninit().write_payload(navigation).send()
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
            node = port = events = camera_port = navigation_port = None
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


def test_spatial_frames_preserve_raw_poses_and_follow_navigation():
    with isolated_run(ROOT):
        process, path = recording()
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        truth_port = publisher(node, "Firefly/GroundTruth", OdomMessage)
        navigation_port = publisher(node, "Firefly/Navigation", NavigationMessage)
        geometry = publisher(node, "Firefly/Viz", VizMessage, 256, 8)
        sequence = 0

        def spatial(name, kind, stamp, xyz=(0., 0., 0.)):
            nonlocal sequence
            msg = VizMessage(kind=kind, timestamp=stamp, sequence=sequence)
            encoded = name.encode()
            msg.entity_len, msg.entity[:len(encoded)] = len(encoded), encoded
            msg.xyz[:], msg.quat_xyzw[:] = xyz, (0., 0., 0., 1.)
            sequence += 1
            return msg

        try:
            physics_time = 0.
            for _ in range(1000):
                physics_time += .001
            navigation_time = round(physics_time * 1e9) * 1e-9
            assert physics_time != navigation_time
            truth = OdomMessage(timestamp=physics_time, position_x=-13., position_z=.4, quat_w=1., is_initialized=True)
            navigation = NavigationMessage(odom=OdomMessage(timestamp=navigation_time, quat_w=1., is_initialized=True))
            navigation.rotation[:] = (0., 0., 0., 1.)
            truth_port.loan_uninit().write_payload(truth).send()
            time.sleep(.03)
            navigation_port.loan_uninit().write_payload(navigation).send()
            for name in ("vio/odom", "vio/prediction/pose_body_odom", "acceptance/odom", "acceptance/reference"):
                geometry.loan_uninit().write_payload(spatial(name, 1, 1.)).send()
            camera = spatial("cameras/left", 12, 1., (.1, .05, 0.))
            camera.scalar_count, camera.point_count = 4, 1
            camera.scalars[:] = (200., 200., 159.5, 119.5)
            camera.points[0][:] = (320., 240., 0.)
            geometry.loan_uninit().write_payload(camera).send()
            time.sleep(.03)
            # A map correction moves odom, while the startup truth anchor stays fixed.
            navigation.odom.timestamp = 2.
            navigation.translation[:] = (2., 3., 0.)
            navigation.rotation[:] = (0., 0., 2 ** -.5, 2 ** -.5)
            navigation.revision = 1
            navigation_port.loan_uninit().write_payload(navigation).send()
            geometry.loan_uninit().write_payload(spatial("vio/odom", 1, 2., (1., 0., .026))).send()
            geometry.loan_uninit().write_payload(spatial("vio/prediction/pose_body_odom", 1, 2., (1., 0., .026))).send()
            geometry.loan_uninit().write_payload(spatial("plan", 7, 2.)).send()
            line = spatial("plan/local_traj", 2, 2.)
            line.point_count = 2
            line.points[0][:], line.points[1][:] = (2., 4., .026), (2., 5., .026)
            geometry.loan_uninit().write_payload(line).send()
            time.sleep(.1)
        finally:
            assert process.stop() == 0, list(process.tail)

        _, logs = read(path)
        assert "recording complete" in logs
        components = {}
        for chunk in RrdReader(path).stream():
            batch = chunk.to_record_batch()
            for name in batch.schema.names:
                if name.startswith(("Transform3D:", "CoordinateFrame:", "Pinhole:")):
                    components.setdefault((chunk.entity_path, name), []).extend(batch.column(name).to_pylist())
        def component(entity, name):
            return components[(entity, name)]
        assert component("/frames/world", "Transform3D:translation") == [[pytest.approx([13., 0., -.4])]]
        assert component("/frames/world", "Transform3D:parent_frame") == [["map"]]
        assert component("/frames/odom", "Transform3D:translation") == [[[0., 0., 0.]], [[2., 3., 0.]]]
        assert component("/frames/odom", "Transform3D:quaternion")[1][0] == pytest.approx([0., 0., 2 ** -.5, 2 ** -.5])
        assert component("/vio/odom", "Transform3D:translation") == [[[0., 0., 0.]], [pytest.approx([1., 0., .026])]]
        assert component("/vio/prediction/pose_body_odom", "Transform3D:translation") == component("/vio/odom", "Transform3D:translation")
        assert component("/gt/pose", "Transform3D:translation") == [[pytest.approx([-13., 0., .4])]]
        assert component("/gt/pose", "Transform3D:parent_frame") == [["world"]]
        assert component("/vio/odom", "Transform3D:parent_frame") == [["odom"], ["odom"]]
        assert component("/acceptance/odom", "Transform3D:parent_frame") == [["odom"]]
        assert component("/acceptance/reference", "Transform3D:parent_frame") == [["odom"]]
        assert component("/cameras/left", "Transform3D:parent_frame") == [["tf#/vio/prediction/pose_body_odom"]]
        assert component("/cameras/left", "Pinhole:parent_frame") == component("/cameras/left", "Transform3D:parent_frame")
        assert component("/plan/local_traj", "CoordinateFrame:frame") == [["odom"]]
        print({"rrd": str(path), "statistics": logs})


def test_metric_cards_use_measurement_time_and_preserve_sources():
    with isolated_run(ROOT):
        process, path = recording()
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        port = publisher(node, "Firefly/VizScalars", ScalarMessage, 1024, 8)
        samples = [("fc/debug/tick_rate", 1., [999.5]),
                   ("rootslam/frame_processing_ms", 1.1, [6.5]),
                   ("vio/prediction/posterior_age_s", 1.2, [.012]),
                   ("vio/diagnostics/tracks", 1.3, [90., 80., 70., 60.]),
                   ("fc/debug/state", 1.5, [1.]), ("fc/debug/state", 4.1, [1.])]
        samples.extend(("render/capture/publish_s", 1. + frame / 60., [0., 0., .008])
                       for frame in range(61))
        samples.sort(key=lambda sample: sample[1])
        try:
            for seq, (name, stamp, values) in enumerate(samples):
                port.loan_uninit().write_payload(scalar(name, stamp, seq, values)).send()
            time.sleep(.1)
        finally:
            assert process.stop() == 0, list(process.tail)
        actual, logs = read(path)
        expected = {}
        for name, stamp, values in samples:
            expected.setdefault(name, []).append((int(stamp * 1e9 + .5), values))
        for name, rows in expected.items():
            assert actual[name] == rows
        texts = {}
        for chunk in RrdReader(path).stream().filter(content="/mimir/status/**"):
            batch = chunk.to_record_batch()
            texts.setdefault(chunk.entity_path.removeprefix("/mimir/status/"), []).extend(
                zip(batch["sim_time"].cast("int64").to_pylist(),
                    [row[0] for row in batch["TextDocument:text"].to_pylist()]))
        assert len(texts) == 6
        for rows in texts.values():
            assert [stamp for stamp, _ in rows] == [1_000_000_000, 1_500_000_000, 2_000_000_000, 4_100_000_000]
            assert "—" in rows[-1][1]
        assert texts["fc/tick_rate"][1][1] == "**999.5 Hz**"
        assert texts["vio/processing"][1][1] == "**6.5 ms**"
        assert texts["vio/posterior_age"][1][1] == "**12.0 ms**"
        assert texts["vio/landmarks"][1][1] == "**60 IDs**"
        assert texts["camera/publish_rate"][2][1] == "**60.0 FPS**"
        assert texts["camera/publish_latency"][2][1] == "**8.0 ms**"
        assert "recording complete" in logs
        print({"rrd": str(path), "statistics": logs})
