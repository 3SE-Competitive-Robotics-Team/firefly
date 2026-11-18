"""Real IPC → release mimir → indexed RRD; requires an idle workspace."""

from pathlib import Path
import os
import signal
import threading
import time
from datetime import datetime
from zoneinfo import ZoneInfo

import iceoryx2 as iox2
import psutil
import pytest
from rerun.chunk import RrdReader

from firefly_workspace.process import ManagedProcess, isolated_run
from firefly_ipc.messages import (
    FC_STATUS_BUFFER_SIZE, FC_STATUS_TOPIC, FcStatusMessage,
    SCALAR_BUFFER_SIZE, VIZ_MAX_PUBLISHERS, DownImageMessage, GrayImageMessage,
    LandingTargetMessage, LogMessage, NavigationMessage, OdomMessage,
    ScalarMessage, TraceContext, VizMessage,
)
from firefly_workspace import digest

ROOT = Path(__file__).resolve().parents[2]


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


def recording(*, env=None):
    binary = ROOT / "target/release/mimir"
    assert binary.is_file(), "build release mimir first"
    ready = threading.Event()
    process = ManagedProcess([str(binary)], cwd=ROOT,
                             env={**os.environ, "RUST_LOG": "info", **(env or {})},
                             on_line=lambda line: ready.set() if "mimir subscriptions ready" in line else None)
    try:
        assert ready.wait(10.), list(process.tail)
    except BaseException:
        process.stop()
        raise
    return process, process.wait_recording()


@pytest.mark.parametrize("zone", ["Asia/Shanghai", "America/New_York"])
def test_recording_path_uses_local_time_and_only_writes_log_rrd(zone):
    with isolated_run(ROOT):
        start = datetime.now(ZoneInfo(zone)).strftime("%Y%m%d_%H%M%S")
        process, path = recording(env={"TZ": zone})
        try:
            end = datetime.now(ZoneInfo(zone)).strftime("%Y%m%d_%H%M%S")
            assert path.parent.parent == ROOT / "logs"
            assert start <= path.parent.name <= end
            assert len(path.parent.name) == 15
            assert path.name == "log.rrd"
        finally:
            assert process.stop() == 0, list(process.tail)
        assert list(path.parent.iterdir()) == [path]
        _, logs = read(path)
        assert "recording complete" in logs


def test_consecutive_recordings_preserve_existing_files():
    with isolated_run(ROOT):
        process, first = recording()
        assert process.stop() == 0
        fingerprint = digest(first)
        process, second = recording()
        assert process.stop() == 0
        assert second != first
        assert digest(first) == fingerprint
        for path in (first, second):
            _, logs = read(path)
            assert "recording complete" in logs


@pytest.mark.parametrize("argument", ["--save", "--output", "--output-dir"])
def test_output_path_overrides_are_rejected(argument):
    import subprocess
    result = subprocess.run([str(ROOT / "target/release/mimir"), argument, "alternate.rrd"],
                            cwd=ROOT, capture_output=True, text=True)
    assert result.returncode != 0
    assert "unknown argument" in result.stderr


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
            port = publisher(node, "Firefly/VizScalars", ScalarMessage, SCALAR_BUFFER_SIZE, VIZ_MAX_PUBLISHERS)
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


@pytest.mark.parametrize("fault", ["sequence", "producer_loss", "unknown_entity", "fc_status_gap"])
def test_incomplete_recording_is_an_error(fault):
    with isolated_run(ROOT):
        process, path = recording()
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        try:
            if fault == "producer_loss":
                port = publisher(node, "Firefly/Log", LogMessage, 256, 10)
                message = LogMessage(dropped_records=1)
                port.loan_uninit().write_payload(message).send()
            elif fault == "fc_status_gap":
                port = publisher(node, FC_STATUS_TOPIC, FcStatusMessage, FC_STATUS_BUFFER_SIZE, 1)
                for seq in (0, 2):
                    port.loan_uninit().write_payload(FcStatusMessage(1., seq, 3)).send()
            else:
                port = publisher(node, "Firefly/VizScalars", ScalarMessage, SCALAR_BUFFER_SIZE, VIZ_MAX_PUBLISHERS)
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
                ports = [publisher(node, "Firefly/VizScalars", ScalarMessage, SCALAR_BUFFER_SIZE, VIZ_MAX_PUBLISHERS)]
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
        down = publisher(node,"Firefly/CameraDown",DownImageMessage)
        target = publisher(node,"Firefly/LandingTarget",LandingTargetMessage)
        try:
            frame=DownImageMessage(timestamp=2.,sensor_id=3,width=352,height=288)
            frame.data[:]=bytes([173])*len(frame.data)
            down.loan_uninit().write_payload(frame).send()
            observation=LandingTargetMessage(timestamp=2.,session=42,position=(.1,-.2,.3),reprojection_px=.7,confidence=.91,team=1,valid=1)
            target.loan_uninit().write_payload(observation).send()
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
        actual, logs=read(path)
        assert actual["landing/target"]==[(2_000_000_000,[.1,-.2,.3,1.])]
        assert actual["landing/quality"]==[(2_000_000_000,[42.,1.,.7,.91])]
        assert "recording complete" in logs
        found_images, found_points = set(), set()
        for chunk in RrdReader(path).stream():
            batch = chunk.to_record_batch()
            if chunk.entity_path=="/cameras/down/image":
                assert batch.column("sim_time").cast("int64").to_pylist()==[2_000_000_000]
                assert bytes(batch.column("Image:buffer")[0].as_py()[0])==bytes([173])*(352*288)
                found_images.add("down")
            for camera, side in enumerate(("left", "right")):
                if chunk.entity_path == f"/cameras/{side}/image":
                    assert batch.column("sim_time").cast("int64").to_pylist() == [2_000_000_000]
                    assert bytes(batch.column("Image:buffer")[0].as_py()[0]) == bytes([30 + camera]) * (320 * 240)
                    found_images.add(side)
                if chunk.entity_path == f"/cameras/{side}/points" and "track_id" in batch.schema.names:
                    assert batch.column("track_id").to_pylist() == [[42., 43.]]
                    found_points.add(side)
        assert found_images == {"left", "right", "down"}
        assert found_points == {"left","right"}


def test_spatial_batches_retain_repeated_and_out_of_order_times():
    with isolated_run(ROOT):
        process, path = recording()
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        geometry = publisher(node, "Firefly/Viz", VizMessage, 256, 8)
        expected = []
        try:
            for index in range(129):
                # Delayed measurements and equal timestamps remain separate observations.
                stamp = (index // 2 if index < 96 else index - 96) / 1024
                msg = VizMessage(kind=1, timestamp=stamp, sequence=index)
                name = b"vio/odom"
                msg.entity_len, msg.entity[:len(name)] = len(name), name
                msg.xyz[:], msg.quat_xyzw[:] = (float(index), 0., .25), (0., 0., 0., 1.)
                geometry.loan_uninit().write_payload(msg).send()
                expected.append((int(stamp * 1e9 + .5), [[float(index), 0., .25]]))
                if index % 32 == 31:
                    time.sleep(.02)
        finally:
            assert process.stop() == 0, list(process.tail)
        _, logs = read(path)
        assert "recording complete" in logs
        chunks = list(RrdReader(path).stream().filter(content="/vio/odom"))
        actual = []
        for chunk in chunks:
            batch = chunk.to_record_batch()
            assert batch["Transform3D:parent_frame"].to_pylist() == [["odom"]] * len(batch)
            actual.extend(zip(batch["sim_time"].cast("int64").to_pylist(),
                              batch["Transform3D:translation"].to_pylist()))
        assert actual == expected
        assert len(chunks) < len(expected) // 4, "spatial samples must be batched, not one chunk per row"


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
            for name in ("vio/odom", "vio/prediction/pose_body_odom", "vio/posterior/pose_imu_world", "plan/drone"):
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
        assert component("/vio/posterior/pose_imu_world", "Transform3D:parent_frame") == [["odom"]]
        assert component("/plan/drone", "Transform3D:parent_frame") == [["odom"]]
        assert component("/cameras/left", "Transform3D:parent_frame") == [["tf#/vio/prediction/pose_body_odom"]]
        assert component("/cameras/left", "Pinhole:parent_frame") == component("/cameras/left", "Transform3D:parent_frame")
        assert component("/plan/local_traj", "CoordinateFrame:frame") == [["odom"]]
        print({"rrd": str(path), "statistics": logs})


def test_fc_status_preserves_modes_and_measurement_times_without_viz():
    with isolated_run(ROOT):
        process, path = recording()
        try:
            node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
            port = publisher(node, FC_STATUS_TOPIC, FcStatusMessage, FC_STATUS_BUFFER_SIZE, 1)
            modes = [(1., 0), (1.1, 1), (1.2, 2), (1.3, 3), (1.4, 3)]
            for seq, (stamp, mode) in enumerate(modes):
                port.loan_uninit().write_payload(FcStatusMessage(stamp, seq, mode)).send()
        finally:
            assert process.stop() == 0, list(process.tail)
        rows, logs = read(path)
        assert rows["fc/debug/state"] == [(round(t * 1e9), [mode]) for t, mode in modes]
        assert "recording complete" in logs


def test_metric_cards_use_measurement_time_and_preserve_sources():
    with isolated_run(ROOT):
        process, path = recording()
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        port = publisher(node, "Firefly/VizScalars", ScalarMessage, SCALAR_BUFFER_SIZE, VIZ_MAX_PUBLISHERS)
        samples = [("fc/debug/tick_rate", 1., [999.5]),
                   ("rootslam/frame_processing_ms", 1.1, [6.5]),
                   ("vio/prediction/posterior_age_s", 1.2, [.012]),
                   ("vio/diagnostics/tracks", 1.3, [90., 80., 70., 60.]),
                   ("fc/debug/altitude", 1.5, [1.]), ("fc/debug/altitude", 4.1, [1.])]
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
