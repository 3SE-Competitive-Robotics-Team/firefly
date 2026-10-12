"""独立录制进程：有界 IPC 接收、诊断写入和媒体处理分别调度。"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import faulthandler
import logging
import json
import math
from pathlib import Path
import queue
import signal
import threading
import time
import uuid

import iceoryx2 as iox2
import rerun as rr

from firefly_mujoco import (
    DepthImageMessage, GrayImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH,
    LogMessage, TraceContext, load_scene_name,
)

from .capture import Capture
from .config import ROOT, load
from .messages import ScalarMessage, VizMessage, VoxelMessage
from .recording import Recordings, URL
from .video import Frame
from .writer import Writer


logger = logging.getLogger("firefly-viz")
VIZ_BUFFER_SIZE = 256
VIZ_MAX_PUBLISHERS = 8
LOG_BUFFER_SIZE = 256
LOG_MAX_PUBLISHERS = 10
VOXEL_BUFFER_SIZE = 32


def detach_copy(msg):
    """IPC loan 归还前获得自有存储，不能把 ctypes 别名交给异步线程。"""
    return type(msg).from_buffer_copy(msg)


def _subscribe(node, topic, payload, capacity=VIZ_BUFFER_SIZE, publishers=VIZ_MAX_PUBLISHERS):
    builder = node.service_builder(iox2.ServiceName.new(topic)).publish_subscribe(payload).user_header(TraceContext)
    if capacity > 1:
        builder = builder.subscriber_max_buffer_size(capacity)
    if publishers is not None:
        builder = builder.max_publishers(publishers)
    return builder.open_or_create().subscriber_builder().buffer_size(capacity).create()


class Continuity:
    """每个 IPC 发布者的连续序号；有界记录发布者，丢诊断不能被当作采样。"""

    def __init__(self, capacity):
        self.capacity, self.last = capacity, {}

    def observe(self, publisher, sequence):
        if publisher in self.last:
            if sequence != self.last[publisher] + 1:
                raise ValueError(f"diagnostic sequence gap: publisher={publisher}, sequence={sequence}")
        elif len(self.last) >= self.capacity:
            raise ValueError("diagnostic publisher budget exceeded")
        self.last[publisher] = sequence


class Worker:
    def __init__(self, name, capacity, consume, tick, finish, timeout):
        self.inbox = queue.Queue(maxsize=capacity)
        self.errors = queue.SimpleQueue()
        self.failure = None
        self.closing = threading.Event()
        self.consume, self.tick, self.finish, self.timeout = consume, tick, finish, timeout
        self.high_water = 0
        self.thread = threading.Thread(target=self._run, name=name)
        self.thread.start()

    def _run(self):
        try:
            while True:
                try:
                    item = self.inbox.get(timeout=0.05)
                except queue.Empty:
                    if self.closing.is_set():
                        break
                    self.tick()
                    continue
                self.consume(item)
                self.tick()
        except Exception as error:
            self.errors.put(error)
        finally:
            try:
                self.finish()
            except Exception as error:
                self.errors.put(error)

    def check(self):
        if self.failure is None:
            try:
                self.failure = self.errors.get_nowait()
            except queue.Empty:
                pass
        if self.failure is not None:
            raise RuntimeError(f"{self.thread.name} failed: {self.failure}") from self.failure

    def put(self, item):
        self.check()
        if self.closing.is_set():
            raise RuntimeError(f"{self.thread.name} is closing")
        self.inbox.put_nowait(item)
        self.high_water = max(self.high_water, self.inbox.qsize())

    def close(self):
        # 唯一生产者已经停止；关闭信号不占队列容量，超时后也能继续排空退出。
        self.closing.set()
        if self.thread.is_alive():
            self.thread.join(self.timeout)
        if self.thread.is_alive():
            raise RuntimeError(f"{self.thread.name} did not finish draining")
        self.check()


def _parse_args():
    parser = argparse.ArgumentParser(prog="firefly-viz", description="Record diagnostics and H.265; visualize in Rerun")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/viz.toml")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--save", type=Path, help="RRD path; with no viewer option, record offline")
    target.add_argument("--no-save", action="store_true", help="Explicitly disable persistent recording")
    viewer = parser.add_mutually_exclusive_group()
    viewer.add_argument("--serve", action="store_true", help="Spawn native Rerun Viewer")
    viewer.add_argument("--connect", metavar="URL", help="Connect to an existing Rerun Viewer")
    return parser.parse_args()


def run(args):
    rr.set_strict_mode(True)
    options = load(args.config)
    url = args.connect or (URL if args.no_save or args.save is None else None)
    path = args.save
    if path is None and not args.no_save:
        now = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = ROOT / "logs" / f"{now}-{uuid.uuid4().hex[:8]}.rrd"
    load_scene_name()
    mesh = ROOT / "models/rmuc2026/field.glb"
    drone = ROOT / "models/drone/drone.glb"
    for asset in (mesh, drone):
        if not asset.is_file():
            raise FileNotFoundError(asset)
    recordings = Recordings(path, url, args.serve, options)
    workers = []
    subscriptions = []
    node = None
    subscriber = None
    cameras = []
    stop, trigger = threading.Event(), threading.Event()
    handlers = {}
    errors = []
    dropped_frames = 0
    last_health = time.monotonic()
    try:
        recordings.archive.log("world/rmuc2026", rr.Asset3D(path=mesh), static=True, strict=True)
        # 模型 +Y-up/+Z-forward → 机体 +Z-up/+X-forward，与 firefly-render::drone 一致。
        model = rr.Asset3D(path=drone)
        for parent in ("gt/pose", "vio/odom", "corr/odom", "plan/drone"):
            recordings.archive.log(f"{parent}/model", model,
                                   rr.Transform3D(quaternion=[0.5, 0.5, 0.5, 0.5]), static=True, strict=True)
            recordings.archive.log(f"{parent}/forward", rr.Arrows3D(origins=[[0, 0, 0]],
                                    vectors=[[0.15, 0, 0]], colors=[64, 255, 64]), static=True, strict=True)
        recordings.diagnostic(f"recording={path}; config={args.config}; video={options.video.encoder if options.video.enabled else 'disabled'}")
        recordings.archive.log("viz/config", rr.TextDocument(args.config.read_text()), static=True)
        recordings.archive.log("viz/effective_config", rr.TextDocument(json.dumps(asdict(options), indent=2)), static=True)
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        for sig, callback in ((signal.SIGINT, lambda *_: stop.set()), (signal.SIGTERM, lambda *_: stop.set()),
                              (signal.SIGUSR1, lambda *_: trigger.set())):
            handlers[sig] = signal.signal(sig, callback)
        writer = Writer(recordings, options, trigger)
        continuity = Continuity(options.capture.max_entities)
        telemetry = Worker("viz-telemetry", options.telemetry_queue,
                           lambda item: writer.log_text(item[1]) if item[0] == "log" else writer.handle(item[1]),
                           writer.tick, lambda: writer.tick(final=True), options.shutdown_seconds)
        workers.append(telemetry)
        capture = Capture(recordings, options, trigger)
        media = Worker("viz-media", options.capture.queue_frames, capture.handle, capture.tick,
                       capture.close, options.shutdown_seconds + 4 * options.video.shutdown_seconds)
        workers.append(media)
        for kind, topic, cls, publishers, capacity in (
            ("viz", "Firefly/VizScalars", ScalarMessage, VIZ_MAX_PUBLISHERS, VIZ_BUFFER_SIZE),
            ("viz", "Firefly/Viz", VizMessage, VIZ_MAX_PUBLISHERS, VIZ_BUFFER_SIZE),
            ("viz", "Firefly/VizVoxels", VoxelMessage, 1, VOXEL_BUFFER_SIZE),
            ("log", "Firefly/Log", LogMessage, LOG_MAX_PUBLISHERS, LOG_BUFFER_SIZE),
        ):
            subscriptions.append((kind, _subscribe(node, topic, cls, capacity, publishers)))
        for name, topic, cls in (("left", "CameraLeft", GrayImageMessage),
                                  ("right", "CameraRight", GrayImageMessage), ("depth", "Depth", DepthImageMessage)):
            cameras.append((name, _subscribe(node, f"Firefly/{topic}", cls, capacity=1, publishers=None)))
        # 各目的地只在 Capture 中采样一次，避免不同频率的串联采样互相混叠。
        needed = {
            name: (name == "left" and options.video.enabled)
            or (recordings.live is not None and options.capture.preview_hz > 0)
            or getattr(options.capture, f"{name}_hz", 0) > 0 for name, _ in cameras
        }
        last_stamp = dict.fromkeys(needed, float("-inf"))
        indices = dict.fromkeys(needed, 0)
        retain_window = options.capture.before_frames > 0 or options.capture.after_frames > 0
        logger.info("RRD: %s; SIGUSR1 captures a lossless window", path)
        while not stop.is_set():
            for worker in workers:
                worker.check()
            for kind, subscriber in subscriptions:
                # 公平排空，连续高频日志不能饿死相机接收。
                for _ in range(VIZ_BUFFER_SIZE):
                    sample = subscriber.receive()
                    if sample is None:
                        break
                    try:
                        message = detach_copy(sample.payload().contents)
                        try:
                            if kind == "viz":
                                continuity.observe(sample.header.publisher_id.value, message.sequence)
                            telemetry.put((kind, message))
                        finally:
                            del message
                    except queue.Full as error:
                        recordings.health("telemetry/overflow", [1, telemetry.high_water])
                        raise RuntimeError("telemetry queue overflow; recording is incomplete") from error
                    finally:
                        del sample
            for name, subscriber in cameras:
                sample = subscriber.receive()
                if sample is None:
                    continue
                msg = None
                try:
                    msg = sample.payload().contents
                    stamp = float(msg.timestamp)
                    if not math.isfinite(stamp) or stamp < 0 or stamp <= last_stamp[name]:
                        raise ValueError(f"{name}: invalid or regressed capture timestamp {stamp}")
                    if (msg.width, msg.height) != (IMAGE_WIDTH, IMAGE_HEIGHT):
                        raise ValueError(f"{name}: unexpected sensor resolution")
                    if msg.sensor_id != (1 if name == "right" else 0):
                        raise ValueError(f"{name}: unexpected sensor id")
                    last_stamp[name] = stamp
                    index = indices[name]
                    indices[name] += 1
                    if not retain_window and not needed[name]:
                        continue
                    if media.inbox.full():
                        dropped_frames += 1
                        continue
                    media.put(Frame(name, stamp, index, msg.width, msg.height, bytes(msg.data)))
                finally:
                    del msg
                    del sample
            if time.monotonic() - last_health >= 1.0:
                recordings.health("queues", [telemetry.inbox.qsize(), media.inbox.qsize(),
                                             telemetry.high_water, dropped_frames])
                last_health = time.monotonic()
            stop.wait(0.001)
    except Exception as error:
        errors.append(error)
        try:
            recordings.diagnostic(f"recording incomplete: {error}", level="ERROR")
        except Exception as diagnostic_error:
            errors.append(diagnostic_error)
    finally:
        # 先排空诊断（可能触发截图），再排空媒体，最后关闭 Rerun sinks。
        for worker in workers:
            try:
                worker.close()
            except Exception as error:
                errors.append(error)
                logger.error("shutdown failed: %s", error)
        try:
            recordings.health("camera/dropped_before_processing", [dropped_frames])
            recordings.diagnostic("recording incomplete" if errors else "producers drained; flushing recording", level="ERROR" if errors else "INFO")
        except Exception as error:
            errors.append(error)
        alive = [worker.thread.name for worker in workers if worker.thread.is_alive()]
        # Encoder 的子线程也可能在媒体 finish 失败后仍持有 sink。
        if len(workers) == 2 and capture.encoder:
            alive.extend(thread.name for thread in capture.encoder.threads if thread.is_alive())
            if capture.encoder.process.poll() is None:
                alive.append(f"ffmpeg pid={capture.encoder.process.pid}")
        if alive:
            errors.append(RuntimeError(f"cleanup incomplete; keeping sinks open for active writers: {alive}"))
        else:
            try:
                recordings.close()
            except Exception as error:
                errors.append(error)
        subscriptions.clear()
        # iceoryx 端口必须先于其节点释放。
        cameras.clear()
        subscriber = None
        node = None
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    if errors:
        raise ExceptionGroup("firefly-viz recording failed", errors)


def main():
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    faulthandler.enable()
    iox2.set_log_level(iox2.LogLevel.Error)
    run(_parse_args())


if __name__ == "__main__":
    main()
