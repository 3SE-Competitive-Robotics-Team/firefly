"""Lossless IPC sensor capture and deterministic input replay for online VIO.

Run ``python -m firefly_evaluation.sensor_replay --help``. Ground truth is
archived for independent scoring and never published by the replay process.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import ctypes
from dataclasses import dataclass
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import iceoryx2 as iox2
import numpy as np
import pyarrow as pa
import rerun as rr
from rerun.chunk import RrdReader

from firefly_mujoco import GrayImageMessage, ImuMessage, OdomMessage, TraceContext
from firefly_mujoco.messages import IMAGE_HEIGHT, IMAGE_WIDTH
from firefly_viz.messages import AttitudeMessage, ScalarMessage, SCALAR_BUFFER_SIZE
from firefly_workspace import digest, workspace_root

from .mission_io import service
from .process_lifecycle import ManagedProcess, isolated_run


LOG = logging.getLogger(__name__)
TOPICS = {
    "imu": ("Firefly/Imu", ImuMessage),
    "left": ("Firefly/CameraLeft", GrayImageMessage),
    "right": ("Firefly/CameraRight", GrayImageMessage),
    "ground_truth": ("Firefly/GroundTruth", OdomMessage),
}
FORMAT_VERSION = 1
# OKVIS runtime::threaded_slam::IMU_TEMPORAL_OVERLAP; identical input for both estimators.
IMU_TAIL_SECONDS = 0.02


@dataclass(frozen=True)
class Event:
    kind: str
    order: int
    payload: bytes

    def message(self):
        cls = TOPICS[self.kind][1]
        if len(self.payload) != ctypes.sizeof(cls):
            raise ValueError(f"{self.kind}: payload layout/size mismatch")
        message = cls.from_buffer_copy(self.payload)
        if not math.isfinite(message.timestamp) or message.timestamp < 0:
            raise ValueError(f"{self.kind}: invalid sensor timestamp")
        if self.kind in ("left", "right"):
            if (message.width, message.height) != (IMAGE_WIDTH, IMAGE_HEIGHT):
                raise ValueError(f"{self.kind}: image dimensions differ from IPC contract")
            if message.sensor_id != {"left": 0, "right": 1}[self.kind]:
                raise ValueError(f"{self.kind}: invalid sensor ID")
        return message


def summarize(events: list[Event], expected_imu_rate: float | None = None) -> dict:
    """Validate strict per-topic ordering and fingerprint the actual payloads."""
    streams = {}
    for kind in TOPICS:
        selected = sorted((e for e in events if e.kind == kind), key=lambda e: e.order)
        if not selected:
            raise ValueError(f"missing raw stream: {kind}")
        stamps = [e.message().timestamp for e in selected]
        differences = np.diff(stamps)
        if np.any(differences <= 0):
            raise ValueError(f"{kind}: duplicate or regressed timestamps")
        digest = hashlib.sha256()
        pixels = hashlib.sha256()
        for event in selected:
            digest.update(event.payload)
            if kind in ("left", "right"):
                pixels.update(bytes(event.message().data))
        streams[kind] = {
            "count": len(selected), "first_timestamp": stamps[0], "last_timestamp": stamps[-1],
            "max_interval_s": float(max(differences, default=0.)),
            "median_interval_s": float(np.median(differences)) if len(differences) else 0.,
            "payload_sha256": digest.hexdigest(),
        }
        if kind == "imu" and expected_imu_rate is not None:
            if not math.isfinite(expected_imu_rate) or expected_imu_rate <= 0:
                raise ValueError("expected IMU rate must be positive and finite")
            steps = differences * expected_imu_rate
            missing = np.maximum(0, np.rint(steps).astype(np.int64) - 1)
            streams[kind].update({
                "expected_rate_hz": expected_imu_rate,
                "intervals_with_missing_samples": int(np.count_nonzero(missing)),
                "inferred_missing_samples": int(missing.sum()),
                "off_grid_intervals": int(np.count_nonzero(np.abs(steps - np.rint(steps)) > 1e-5)),
            })
        if kind in ("left", "right"):
            streams[kind]["pixels_sha256"] = pixels.hexdigest()
    orders = sorted(e.order for e in events)
    if orders != list(range(len(events))):
        raise ValueError("capture sequence is incomplete or duplicated")
    return streams


def select_segment(events: list[Event], until_sim: float | None,
                   imu_tail_s: float = IMU_TAIL_SECONDS) -> list[Event]:
    """Keep the prefix and every original IMU through the required tail interval."""
    if until_sim is not None and (not math.isfinite(until_sim) or until_sim < 0):
        raise ValueError("segment end must be finite and nonnegative")
    if not math.isfinite(imu_tail_s) or imu_tail_s < 0:
        raise ValueError("IMU tail must be finite and nonnegative")
    selected = [e for e in events if until_sim is None or e.message().timestamp <= until_sim]
    frames = [e.message().timestamp for e in selected if e.kind in ("left", "right")]
    if not frames:
        raise ValueError("selected segment contains no camera frames")
    last_frame = max(frames)
    coverage = min((e for e in events if e.kind == "imu" and e.message().timestamp > last_frame + imu_tail_s),
                   key=lambda e: e.message().timestamp, default=None)
    if coverage is None:
        raise ValueError(f"selected segment has no IMU beyond last frame + {imu_tail_s}s")
    if until_sim is not None:
        selected.extend(e for e in events if e.kind == "imu"
                        and until_sim < e.message().timestamp <= coverage.message().timestamp)
    # Segment order is a new contiguous index; payloads and sensor times are untouched.
    return [Event(e.kind, index, e.payload) for index, e in enumerate(sorted(selected, key=lambda e: e.order))]


def schedule(events: list[Event]) -> list[tuple[float, tuple[Event, ...]]]:
    """Publish each exact stereo pair after the first IMU strictly beyond it."""
    imu = sorted((e for e in events if e.kind == "imu"), key=lambda e: e.message().timestamp)
    times = [e.message().timestamp for e in imu]
    if len(times) < 2 or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("IMU stream must have at least two strictly ordered samples")
    cameras = {}
    for kind in ("left", "right"):
        frames = [e for e in events if e.kind == kind]
        cameras[kind] = {e.message().timestamp: e for e in frames}
        if len(frames) != len(cameras[kind]):
            raise ValueError(f"{kind}: duplicate frame timestamp")
    if not cameras["left"] or cameras["left"].keys() != cameras["right"].keys():
        raise ValueError("raw stereo streams are missing a matching frame")
    batches: dict[int, list[tuple[Event, ...]]] = {}
    for stamp in sorted(cameras["left"]):
        index = bisect_right(times, stamp)
        if index == 0 or index == len(times):
            raise ValueError("camera frame is not bracketed by the captured IMU stream")
        batches.setdefault(index, []).append((cameras["left"][stamp], cameras["right"][stamp]))
    output = []
    for index, event in enumerate(imu):
        output.append((times[index], (event,)))
        output.extend((times[index], pair) for pair in batches.get(index, []))
    return output


def write_event(recording, event: Event, receive_ns: int, source_send_ns: int = 0):
    message = event.message()
    recording.set_time("sim_time", duration=message.timestamp)
    recording.set_time("receive_order", sequence=event.order)
    recording.set_time("receive_time", duration=np.timedelta64(receive_ns, "ns"))
    recording.log(f"sensors/{event.kind}", rr.AnyValues(
        payload=np.frombuffer(event.payload, dtype=np.uint8), source_send_mono_ns=[source_send_ns]))


def read_capture(path: Path) -> tuple[list[Event], dict]:
    reader = RrdReader(path)
    stores = reader.recordings()
    if len(stores) != 1:
        raise ValueError("expected exactly one raw sensor recording")
    events, manifest = [], None
    for chunk in reader.stream(store=stores[0]).filter(content=["/sensors/**", "/capture/manifest"]):
        entity = chunk.entity_path.lstrip("/")
        batch = chunk.to_record_batch()
        if entity == "capture/manifest":
            values = batch.column("TextDocument:text").to_pylist()
            for value in values:
                if value:
                    manifest = json.loads(value[0])
            continue
        kind = entity.removeprefix("sensors/")
        if kind not in TOPICS:
            continue
        payloads = batch.column("payload")
        orders = batch.column("receive_order").to_pylist()
        for index, order in enumerate(orders):
            # Arrow uint8 buffers avoid expanding image bytes into Python integers.
            payload = payloads[index].values.to_numpy(zero_copy_only=False).tobytes()
            events.append(Event(kind, order, payload))
    if manifest is None or manifest.get("format_version") != FORMAT_VERSION or not manifest.get("complete"):
        raise ValueError("raw capture is missing a complete supported manifest")
    actual = summarize(events, manifest.get("expected_imu_rate_hz"))
    if actual != manifest["streams"]:
        raise ValueError("raw capture count/hash/timestamps differ from its manifest")
    return events, manifest


def open_recording(path: Path):
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    recording = rr.RecordingStream("firefly-sensor-replay")
    recording.save(path)
    return recording


def capture(path: Path, duration: float | None = None, expected_imu_rate: float | None = None,
            *, topics=TOPICS):
    """Start before producers; SIGINT drains the subscribers and flushes the RRD."""
    recording = open_recording(path)
    stop = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    events = []
    start = time.monotonic_ns()
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    subscriptions = []
    try:
        for kind, (topic, cls) in topics.items():
            # A first-started capture creates enough service capacity for its own
            # subscriber. Production consumers retain their configured depths.
            port = service(node, topic, cls, capacity=128).subscriber_builder().buffer_size(128).create()
            subscriptions.append((kind, port))
        recording.log("capture/events", rr.TextLog("ready: raw sensor subscribers connected"))
        recording.flush()
        LOG.info("raw capture ready: %s", path)
        while True:
            received = False
            for kind, subscriber in subscriptions:
                for _ in range(128):
                    sample = subscriber.receive()
                    if sample is None:
                        break
                    try:
                        event = Event(kind, len(events), bytes(sample.payload().contents))
                        received_ns = time.monotonic_ns() - start
                        source_ns = sample.user_header().contents.send_ts_mono_ns
                        write_event(recording, event, received_ns, source_ns)
                        events.append(event)
                        received = True
                    finally:
                        del sample
            if duration is not None and (time.monotonic_ns() - start) * 1e-9 >= duration:
                stop.set()
            if stop.is_set() and not received:
                break
            if not received:
                stop.wait(0.0005)
        streams = summarize(events, expected_imu_rate)
        manifest = {"format_version": FORMAT_VERSION, "complete": True, "streams": streams,
                    "expected_imu_rate_hz": expected_imu_rate,
                    "continuity": "receive sequence validates archive rows; sensor gaps are reported separately",
                    "ground_truth": "scoring only; never replayed to IPC"}
        recording.log("capture/manifest", rr.TextDocument(json.dumps(manifest)), static=True)
        LOG.info("raw capture complete: %s", {k: v["count"] for k, v in streams.items()})
    except BaseException as error:
        recording.log("capture/events", rr.TextLog(f"incomplete: {error}", level="ERROR"))
        raise
    finally:
        subscriptions.clear()
        recording.flush()
        recording.disconnect()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def replay(path: Path, output: Path, speed: float = 1., wait_seconds: float = 15.,
           until_sim: float | None = None, imu_tail_s: float = IMU_TAIL_SECONDS,
           *, topics=TOPICS):
    if not math.isfinite(speed) or speed <= 0 or not math.isfinite(wait_seconds) or wait_seconds <= 0:
        raise ValueError("replay speed and startup timeout must be positive and finite")
    source, manifest = read_capture(path)
    events = select_segment(source, until_sim, imu_tail_s)
    segment = summarize(events, manifest.get("expected_imu_rate_hz"))
    batches = schedule(events)
    recording = open_recording(output)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    ports, services, notifiers = {}, {}, {}
    stop = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for kind in ("imu", "left", "right"):
            topic, cls = topics[kind]
            services[kind] = service(node, topic, cls, capacity=128 if kind == "imu" else None)
            if services[kind].dynamic_config.number_of_publishers != 0:
                raise RuntimeError(f"{topic}: another sensor publisher is still active")
            ports[kind] = services[kind].publisher_builder().create()
        imu_topic = topics["imu"][0]
        pair_topic = topics["left"][0].rsplit("/", 1)[0] + "/CameraPair"
        for topic in (imu_topic, pair_topic):
            event_service = node.service_builder(iox2.ServiceName.new(topic)).event().open_or_create()
            notifiers[topic] = event_service.notifier_builder().create()
        deadline = time.monotonic() + wait_seconds
        while any(s.dynamic_config.number_of_subscribers < 1 for s in services.values()):
            if stop.is_set() or time.monotonic() >= deadline:
                raise TimeoutError("sensor replay requires subscribers on all three input topics")
            stop.wait(0.01)
        for port in ports.values():
            port.update_connections()
        recording.log("replay/input", rr.TextDocument(json.dumps({
            "source": str(path.resolve()), "capture": manifest, "speed": speed,
            "until_sim": until_sim, "segment": segment,
            "required_imu_tail_s": imu_tail_s,
            "actual_imu_tail_after_last_frame_s": segment["imu"]["last_timestamp"] - segment["left"]["last_timestamp"],
            "excluded_samples": {k: manifest["streams"][k]["count"] - v["count"] for k, v in segment.items()},
            "schedule": "sensor timestamp; first strictly later IMU precedes each stereo pair",
        })), static=True)
        LOG.info("sensor replay ready: %s, speed=%s", path, speed)
        start, first = time.monotonic(), batches[0][0]
        sent = {kind: 0 for kind in ports}
        digests = {kind: hashlib.sha256() for kind in ports}
        for stamp, batch in batches:
            due = start + (stamp - first) / speed
            if stop.wait(max(0., due - time.monotonic())):
                raise InterruptedError("sensor replay interrupted before all inputs were published")
            for event in batch:
                sample = ports[event.kind].loan_uninit()
                header = sample.user_header().contents
                ctypes.memset(ctypes.addressof(header), 0, ctypes.sizeof(TraceContext))
                now = time.time_ns()
                header.version = 1
                header.send_ts_secs, header.send_ts_nanos = divmod(now, 1_000_000_000)
                header.send_ts_mono_ns = time.monotonic_ns()
                receivers = sample.write_payload(event.message()).send()
                if receivers < 1:
                    raise RuntimeError(f"{event.kind}: no subscriber received sensor sample")
                sent[event.kind] += 1
                digests[event.kind].update(event.payload)
                recording.set_time("sim_time", duration=event.message().timestamp)
                recording.log(f"replay/{event.kind}", rr.Scalars([
                    sent[event.kind], receivers, max(0., time.monotonic() - due) * 1000.]))
            topic = imu_topic if len(batch) == 1 else pair_topic
            notifiers[topic].notify_with_custom_event_id(iox2.EventId.new(0))
        for kind in ports:
            expected = segment[kind]
            if sent[kind] != expected["count"] or digests[kind].hexdigest() != expected["payload_sha256"]:
                raise ValueError(f"{kind}: replay payload count/hash differs from capture")
        recording.log("replay/complete", rr.TextDocument(json.dumps({
            "sent": sent, "payload_sha256": {k: h.hexdigest() for k, h in digests.items()},
            "consumer_completion": "requires last-frame posterior and IMU watermarks from estimator",
        })), static=True)
        LOG.info("sensor replay publishing complete: %s", sent)
    except BaseException as error:
        recording.log("replay/events", rr.TextLog(str(error), level="ERROR"))
        raise
    finally:
        ports.clear()
        notifiers.clear()
        services.clear()
        recording.flush()
        recording.disconnect()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def completion_evidence(attitude: tuple[float, float] | None, accepted_posterior: float,
                        last_frame: float, last_imu: float) -> str | None:
    """Require actual IMU consumption and the exact final camera posterior."""
    if attitude is None or attitude[0] < last_imu - 1e-9:
        return None
    if abs(attitude[1] - last_frame) <= 1e-9:
        return "VioAttitude.timestamp+posterior_time"
    if abs(accepted_posterior - last_frame) <= 1e-9:
        return "VioAttitude.timestamp+accepted_posterior_diagnostic"
    return None


def posterior_frame_times(path: Path) -> list[int]:
    """Read each accepted posterior row, retaining duplicates for coverage checks."""
    reader = RrdReader(path)
    stores = reader.recordings()
    if len(stores) != 1:
        raise ValueError("expected exactly one VIO recording")
    times = []
    for chunk in reader.stream(store=stores[0]).filter(content=["/vio/posterior/velocity_world"]):
        batch = chunk.to_record_batch()
        if "Scalars:scalars" not in batch.column_names:
            continue
        stamps = batch.column("sim_time").cast(pa.int64()).to_pylist()
        for stamp, values in zip(stamps, batch.column("Scalars:scalars").to_pylist()):
            if values is not None and len(values):
                if stamp is None:
                    raise ValueError("posterior row has no sensor timestamp")
                times.append(stamp)
    return times


def frame_coverage(camera_times_ns: list[int], posterior_times_ns: list[int]) -> dict:
    """Allow an initialization prefix; require every subsequent input frame once.

    Sensor f64 timestamps and Rerun duration conversions may differ by up to 2 ns.
    This checks camera posteriors, not consumption of each individual IMU sample.
    """
    expected = sorted(camera_times_ns)
    if not expected or any(b - a <= 4 for a, b in zip(expected, expected[1:])):
        raise ValueError("camera timestamps must be nonempty, unique and unambiguous within 2 ns")
    matches = Counter()
    foreign = []
    for stamp in posterior_times_ns:
        index = bisect_left(expected, stamp)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(expected)]
        nearest = min(candidates, key=lambda i: abs(expected[i] - stamp))
        if abs(expected[nearest] - stamp) <= 2:
            matches[nearest] += 1
        else:
            foreign.append(stamp)
    first = min(matches, default=len(expected))
    missing = [expected[i] for i in range(first, len(expected)) if i not in matches]
    duplicates = [expected[i] for i in sorted(matches) if matches[i] > 1]
    return {
        "valid": bool(matches) and not (missing or duplicates or foreign),
        "timestamp_tolerance_ns": 2,
        "input_frames": len(expected), "posterior_rows": len(posterior_times_ns),
        "matched_frames": len(matches), "initialization_prefix_frames": first,
        "first_posterior_ns": min(posterior_times_ns, default=None),
        "last_posterior_ns": max(posterior_times_ns, default=None),
        "missing_frame_ns": missing, "duplicate_frame_ns": duplicates,
        "foreign_posterior_ns": sorted(foreign),
    }


def run(path: Path, directory: Path, estimator: str, speed: float, until_sim: float | None,
        diagnostics: bool = False, imu_tail_s: float = IMU_TAIL_SECONDS):
    """Own viz, the production estimator, replay and their verified shutdown."""
    if estimator not in ("firefly-rootslam", "firefly-okvis-x"):
        raise ValueError("unsupported online estimator")
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("replay speed must be positive and finite")
    root = workspace_root()
    path, directory = path.resolve(), directory.resolve()
    binary = root / "target/release" / estimator
    if not binary.is_file():
        raise FileNotFoundError(binary)
    config = (root / "configs" / {"firefly-rootslam": "firefly-rootslam.toml",
                                  "firefly-okvis-x": "okvis2-x.toml"}[estimator]).resolve()
    if not config.is_file():
        raise FileNotFoundError(config)
    source, manifest = read_capture(path)
    events = select_segment(source, until_sim, imu_tail_s)
    batches = schedule(events)
    segment = summarize(events, manifest.get("expected_imu_rate_hz"))
    last_frame = segment["left"]["last_timestamp"]
    last_imu = segment["imu"]["last_timestamp"]
    camera_times_ns = [round(e.message().timestamp * 1_000_000_000)
                       for e in events if e.kind == "left"]
    duration = (batches[-1][0] - batches[0][0]) / speed
    # The replay child reads the archive independently; release its large payloads here.
    del source, events, batches
    with isolated_run(root):
        directory.mkdir(parents=True, exist_ok=False)
        audit = open_recording(directory / "run.rrd")
        files = subprocess.check_output(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"], cwd=root,
        ).decode().split("\0")
        source_hashes = {name: digest(root / name) for name in files
                         if name and (root / name).is_file()
                         and Path(name).suffix in (".rs", ".py", ".toml", ".lock")}
        result = {
            "status": "running", "estimator": estimator, "diagnostics": diagnostics,
            "source": str(path), "source_sha256": digest(path), "segment": segment,
            "until_sim": until_sim, "speed": speed, "last_frame": last_frame, "last_imu": last_imu,
            "required_imu_tail_s": imu_tail_s,
            "actual_imu_tail_after_last_frame_s": last_imu - last_frame,
            "imu_tail_after_requested_end_s": max(0., last_imu - until_sim) if until_sim is not None else None,
            "excluded_samples": {k: manifest["streams"][k]["count"] - v["count"] for k, v in segment.items()},
            "provenance": {
                "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
                "git_status": subprocess.check_output(["git", "status", "--short"], cwd=root, text=True),
                "sources_sha256": source_hashes, "binary_sha256": digest(binary),
                "estimator_config": {"path": str(config), "sha256": digest(config)},
                "python": sys.version,
            },
            "recordings": {name: str(directory / name) for name in ("run.rrd", "vio.rrd", "delivery.rrd")},
        }
        audit.log("run/provenance", rr.TextDocument(json.dumps(result)), static=True)
        stop = threading.Event()
        previous = {sig: signal.signal(sig, lambda *_: stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        attitude_sub = service(node, "Firefly/VioAttitude", AttitudeMessage).subscriber_builder().buffer_size(2).create()
        scalar_sub = service(node, "Firefly/VizScalars", ScalarMessage, SCALAR_BUFFER_SIZE, 8).subscriber_builder().buffer_size(SCALAR_BUFFER_SIZE).create()
        children, exit_codes = {}, {}
        attitude, accepted_posterior = None, float("-inf")
        viz_ready = threading.Event()
        failure = None
        env = {**os.environ, "RUST_LOG": "info,vio-diagnostics=debug" if diagnostics else "info"}

        def start(name, command):
            def line(text):
                audit.log(f"processes/{name}", rr.TextLog(text))
                if name == "viz" and "RRD:" in text:
                    viz_ready.set()
            children[name] = ManagedProcess(command, cwd=root, env=env, on_line=line)
            audit.log("run/events", rr.TextLog(f"start {name} pid={children[name].pid} argv={command}"))

        def pump():
            nonlocal attitude, accepted_posterior
            if stop.is_set():
                raise InterruptedError("sensor replay run interrupted")
            for name, child in children.items():
                code = child.process.poll()
                if code is not None and (name != "replay" or code != 0):
                    raise RuntimeError(f"unexpected {name} exit {code}: {' | '.join(child.tail)}")
            while (sample := attitude_sub.receive()) is not None:
                msg = sample.payload().contents
                attitude = float(msg.timestamp), float(msg.posterior_time)
                del msg, sample
            while (sample := scalar_sub.receive()) is not None:
                msg = sample.payload().contents
                entity = bytes(msg.entity[:msg.entity_len]).decode()
                if entity == "vio/posterior/velocity_world":
                    accepted_posterior = max(accepted_posterior, float(msg.timestamp))
                del msg, sample

        def wait_for(predicate, timeout, description):
            deadline = time.monotonic() + timeout
            while True:
                pump()
                if predicate():
                    return
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{description}; attitude={attitude}, accepted_posterior={accepted_posterior}")
                stop.wait(0.002)

        try:
            start("viz", [sys.executable, "-m", "firefly_viz.main", "--save", str(directory / "vio.rrd")])
            wait_for(viz_ready.is_set, 15., "viz did not become ready")
            inputs = {kind: service(node, topic, cls, capacity=128 if kind == "imu" else None)
                      for kind, (topic, cls) in TOPICS.items() if kind != "ground_truth"}
            baseline = {kind: port.dynamic_config.number_of_subscribers for kind, port in inputs.items()}
            start("estimator", [str(binary), "--config", str(config), "--no-mapping", "--no-loop-closures"])
            wait_for(lambda: all(port.dynamic_config.number_of_subscribers > baseline[kind]
                                 for kind, port in inputs.items()), 15., "estimator input subscriptions missing")
            command = [sys.executable, "-m", "firefly_evaluation.sensor_replay", "replay", str(path),
                       "--output", str(directory / "delivery.rrd"), "--speed", str(speed),
                       "--imu-tail-s", str(imu_tail_s)]
            if until_sim is not None:
                command.extend(["--until-sim", str(until_sim)])
            start("replay", command)
            wait_for(lambda: children["replay"].process.poll() == 0, duration + 60., "replay publisher timed out")
            wait_for(lambda: completion_evidence(attitude, accepted_posterior, last_frame, last_imu) is not None,
                     15., "estimator did not consume final camera/IMU")
            result["completion_evidence"] = completion_evidence(attitude, accepted_posterior, last_frame, last_imu)
            result["final_attitude"] = attitude
            result["accepted_posterior"] = accepted_posterior if math.isfinite(accepted_posterior) else None
            result["status"] = "passed"
        except BaseException as error:
            failure = error
            result["status"], result["error"] = "failed", str(error)
            audit.log("run/events", rr.TextLog(str(error), level="ERROR"))
        finally:
            for name in ("replay", "estimator", "viz"):
                if name not in children:
                    continue
                try:
                    exit_codes[name] = children[name].stop()
                    if exit_codes[name] != 0:
                        raise RuntimeError(f"{name} exited with {exit_codes[name]}")
                except Exception as error:
                    result["status"] = "failed"
                    result.setdefault("shutdown_errors", []).append(str(error))
                    failure = failure or error
            result["exit_codes"] = exit_codes
            result["remaining_pids"] = [child.pid for child in children.values() if child.process.poll() is None]
            if result["status"] == "passed":
                try:
                    coverage = frame_coverage(camera_times_ns, posterior_frame_times(directory / "vio.rrd"))
                    result["frame_coverage"] = coverage
                    if not coverage["valid"]:
                        raise ValueError(f"posterior frame coverage failed: {coverage}")
                except Exception as error:
                    result["status"], result["error"] = "failed", str(error)
                    failure = failure or error
            result["final_attitude"] = attitude
            result["accepted_posterior"] = accepted_posterior if math.isfinite(accepted_posterior) else None
            attitude_sub.delete()
            scalar_sub.delete()
            audit.log("run/result", rr.TextDocument(json.dumps(result, allow_nan=False)), static=True)
            audit.flush()
            audit.disconnect()
            (directory / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
            for sig, handler in previous.items():
                signal.signal(sig, handler)
        if failure is not None:
            raise failure
        LOG.info("sensor replay run passed: %s", directory)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    record = commands.add_parser("capture")
    record.add_argument("output", type=Path)
    record.add_argument("--duration", type=float, help="wall seconds; otherwise stop with SIGINT")
    record.add_argument("--expected-imu-rate", type=float, help="source rate in Hz, for missing-interval accounting")
    play = commands.add_parser("replay")
    play.add_argument("input", type=Path)
    play.add_argument("--output", type=Path, required=True, help="replay delivery evidence RRD")
    play.add_argument("--speed", type=float, default=1.)
    play.add_argument("--wait-seconds", type=float, default=15.)
    play.add_argument("--until-sim", type=float, help="explicit sensor-time prefix end, plus IMU coverage of its last frame")
    play.add_argument("--imu-tail-s", type=float, default=IMU_TAIL_SECONDS,
                      help="original IMU tail after the last frame, shared by both estimators (default: 0.02)")
    inspect = commands.add_parser("inspect")
    inspect.add_argument("input", type=Path)
    execute = commands.add_parser("run")
    execute.add_argument("input", type=Path)
    execute.add_argument("--output-dir", type=Path, required=True)
    execute.add_argument("--estimator", choices=("firefly-rootslam", "firefly-okvis-x"), required=True)
    execute.add_argument("--speed", type=float, default=1.)
    execute.add_argument("--until-sim", type=float)
    execute.add_argument("--diagnostics", action="store_true")
    execute.add_argument("--imu-tail-s", type=float, default=IMU_TAIL_SECONDS)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    if args.command == "capture":
        if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
            parser.error("duration must be positive and finite")
        if args.expected_imu_rate is not None and (not math.isfinite(args.expected_imu_rate) or args.expected_imu_rate <= 0):
            parser.error("expected IMU rate must be positive and finite")
        capture(args.output, args.duration, args.expected_imu_rate)
    elif args.command == "replay":
        replay(args.input, args.output, args.speed, args.wait_seconds, args.until_sim, args.imu_tail_s)
    elif args.command == "run":
        run(args.input, args.output_dir, args.estimator, args.speed, args.until_sim, args.diagnostics, args.imu_tail_s)
    else:
        _, manifest = read_capture(args.input)
        LOG.info("verified raw capture: %s", json.dumps(manifest))


if __name__ == "__main__":
    main()
