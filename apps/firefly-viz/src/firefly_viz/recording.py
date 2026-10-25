"""左右目视频与同帧关键点的历史录制，所有时间戳属于原始测量时刻。"""

from collections import OrderedDict
from typing import Protocol
from pathlib import Path
import threading
import time
import uuid

import numpy as np
import pyarrow as pa
import rerun as rr
import rerun.blueprint as rrb

from .telemetry import log_scalars


APP_ID = "firefly"
URL = "rerun+http://127.0.0.1:9876/proxy"
# Rerun 0.38.1 visualizers/video/mod.rs: 图像左上边界为 (0,0)，首个像素中心为 (.5,.5)。
# 仅显示层转换；IPC/算法使用整数像素中心。
PIXEL_CENTER_OFFSET = 0.5


def at(recording, stamp: float | None):
    # SDK 时间是线程局部的；每次写入显式重置，防止 wall_time 与 sim_time 串轴。
    recording.reset_time()
    if stamp is None:
        recording.set_time("wall_time", timestamp=time.time())
    else:
        recording.set_time("sim_time", duration=stamp)


def blueprint():
    history = [rrb.VisibleTimeRange("sim_time", start=rrb.TimeRangeBoundary.infinite(),
                                    end=rrb.TimeRangeBoundary.cursor_relative())]
    spatial = rrb.Spatial3DView(
        origin="/", name="Ground truth / VSLAM / planning",
        contents=["+ /gt/**", "+ /okvis/odom/**", "+ /okvis/traj/**",
                  "+ /okvis/map/**", "+ /plan/**"],
        overrides={path: rrb.VisibleTimeRanges(history)
                   for path in ("/gt/traj", "/okvis/traj", "/okvis/map/traj")},
    )
    cameras = [rrb.Spatial2DView(origin=f"/cameras/{name}", name=f"{name.title()} · recorded",
                                 contents=[f"+ /cameras/{name}/**"])
               for name in ("left", "right")]
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                spatial, *cameras,
            ),
            rrb.Tabs(
                rrb.Vertical(
                    rrb.StateTimelineView(origin="/okvis/tracking", name="Tracking state"),
                    rrb.TimeSeriesView(origin="/okvis/loop_commits", name="Loop updates"),
                    rrb.TimeSeriesView(origin="/okvis/frame_processing_ms", name="Frame processing · ms"),
                    name="OKVIS diagnostics",
                ),
                rrb.Vertical(
                    rrb.TimeSeriesView(origin="/fc/debug/tick_rate", name="FC tick rate · Hz"),
                    rrb.Horizontal(
                        rrb.StateTimelineView(origin="/fc/imu", name="IMU attitude states",
                                              contents=["+ /fc/imu/source", "+ /fc/imu/attitude_ready",
                                                        "+ /fc/imu/vio_fresh"]),
                        rrb.StateTimelineView(origin="/fc", name="Flight mode / safety",
                                              contents=["+ /fc/debug/state", "+ /fc/arming/failed",
                                                        "+ /fc/failsafe/flags", "+ /fc/failsafe/action",
                                                        "+ /fc/debug/saturated"]),
                        rrb.StateTimelineView(origin="/fc/standby_eskf", name="Standby ESKF states",
                                              contents=["+ /fc/standby_eskf/accel_gate",
                                                        "+ /fc/standby_eskf/health"]),
                    ),
                    rrb.Horizontal(
                        rrb.TimeSeriesView(origin="/fc/debug/thrust_total", name="Total thrust · N"),
                        rrb.TimeSeriesView(origin="/fc/debug", name="Altitude / position error · m",
                                           contents=["+ /fc/debug/altitude", "+ /fc/debug/position_error_odom"]),
                    ),
                    name="Flight control",
                ),
                rrb.TextLogView(origin="/logs", name="Logs"),
                active_tab=1,
            ),
        ),
        rrb.TimePanel(timeline="sim_time", state=rrb.PanelState.Collapsed,
                      play_state=rrb.components.PlayState.Paused),
        rrb.SelectionPanel(state=rrb.PanelState.Collapsed),
        rrb.BlueprintPanel(state=rrb.PanelState.Collapsed),
    )


class RecordingOutputs(Protocol):
    """写入器与采集器所需的录制端口；成员和方法签名供测试替身校验。"""

    @property
    def archive(self) -> rr.RecordingStream: ...

    @property
    def matches(self) -> "MatchedPoints": ...

    def diagnostic(self, text: str, *, stamp=None, level="INFO") -> None: ...

    def health(self, name: str, values, stamp=None) -> None: ...

    def video(self, frame, sample, keyframe, latency) -> None: ...

    def close(self) -> None: ...


class Recordings:
    def __init__(self, path: Path | None, url: str | None, serve: bool, options, *,
                 stream_factory=rr.RecordingStream, file_sink=rr.FileSink,
                 grpc_sink=rr.GrpcSink, blueprint_factory=blueprint):
        self.options = options
        self.path = path
        recording_id = str(uuid.uuid4())
        self.archive = stream_factory(APP_ID, recording_id=recording_id)
        try:
            if not self.archive.is_enabled():
                raise RuntimeError("Rerun recording is disabled by the environment")
            self._connect(path, url, serve, options, recording_id, file_sink, grpc_sink, blueprint_factory)
        except Exception as error:
            try:
                self.archive.disconnect()
            except Exception as cleanup_error:
                raise ExceptionGroup("Rerun initialization and cleanup failed", [error, cleanup_error]) from error
            raise
        self.matches = MatchedPoints(self.archive, options.capture.match_frames)

    def _connect(self, path, url, serve, options, recording_id, file_sink, grpc_sink, blueprint_factory):
        sinks = []
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            # 不覆盖既有证据；预先占位，文件 sink 仅打开本次创建的文件。
            with path.open("xb"):
                pass
            # 持久证据必须带完整索引，供 Viewer 与随机访问读取器直接打开。
            sinks.append(file_sink(str(path), write_footer=True))
        if serve:
            self.archive.spawn(connect=False, memory_limit=options.viewer_memory)
            url = url or URL
        if url:
            sinks.append(grpc_sink(url=url))
        if not sinks:
            raise ValueError("at least one file or viewer destination is required")
        self.archive.set_sinks(*sinks)
        self.archive.send_blueprint(blueprint_factory())
        self.archive.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self.archive.log("viz/session", rr.AnyValues(recording_id=recording_id, rerun_version=rr.__version__), static=True)
        if options.video.enabled:
            for camera in ("left", "right"):
                self.archive.log(f"cameras/{camera}/image", rr.VideoStream(codec=rr.VideoCodec.H265), static=True)

    def diagnostic(self, text: str, *, stamp=None, level="INFO"):
        at(self.archive, stamp)
        self.archive.log("logs/viz", rr.TextLog(text, level=level), strict=True)

    def health(self, name: str, values, stamp=None):
        at(self.archive, stamp)
        log_scalars(self.archive, f"viz/{name}", values)

    def video(self, frame, sample, keyframe, latency):
        at(self.archive, frame.stamp)
        self.archive.set_time(f"{frame.camera}_frame", sequence=frame.index)
        self.archive.log(f"cameras/{frame.camera}/image", rr.VideoStream.from_fields(sample=sample, is_keyframe=keyframe),
                         rr.AnyValues(capture_time=frame.stamp, frame_index=frame.index), strict=True)
        self.matches.frame(frame.camera, frame.stamp, frame.index)
        self.health(f"video/{frame.camera}/encoded", [len(sample), latency, frame.index], frame.stamp)

    def close(self):
        errors = []
        try:
            self.archive.flush(timeout_sec=self.options.shutdown_seconds)
        except Exception as error:
            errors.append(error)
        finally:
            try:
                self.archive.disconnect()
            except Exception as error:
                errors.append(error)
        if errors:
            raise ExceptionGroup("Rerun flush failed", errors)


class MatchedPoints:
    """点集匹配容差内最近的已保存图像；缓存严格有界。"""

    MATCH_TOLERANCE_NS = 1_000_000

    def __init__(self, recording, capacity: int):
        self.recording, self.capacity = recording, capacity
        self.frames = {camera: OrderedDict() for camera in ("left", "right")}
        self.points = {camera: OrderedDict() for camera in ("left", "right")}
        self.lock = threading.Lock()

    def _remember(self, cache, key, value):
        cache[key] = value
        while len(cache) > self.capacity:
            cache.popitem(last=False)

    def _nearest(self, cache, key):
        nearest = min(cache, key=lambda candidate: (abs(candidate - key), candidate), default=None)
        return nearest if nearest is not None and abs(nearest - key) <= self.MATCH_TOLERANCE_NS else None

    def frame(self, camera, stamp, index=None):
        if camera not in self.frames:
            return
        with self.lock:
            key = round(stamp * 1e9)
            self._remember(self.frames[camera], key, (stamp, index))
            self._write(camera, stamp, self.points[camera].get(self._nearest(self.points[camera], key)))

    def observe(self, camera, stamp, points, ids, color):
        if camera not in self.points:
            return
        with self.lock:
            value = points, ids, color
            key = round(stamp * 1e9)
            self._remember(self.points[camera], key, value)
            frame_key = self._nearest(self.frames[camera], key)
            if frame_key is not None:
                point_key = self._nearest(self.points[camera], frame_key)
                self._write(camera, self.frames[camera][frame_key][0], self.points[camera][point_key])

    def _write(self, camera, stamp, value):
        points, ids, color = value if value is not None else (np.empty((0, 2)), [], [64, 255, 128])
        at(self.recording, stamp)
        if (index := self.frames[camera][round(stamp * 1e9)][1]) is not None:
            self.recording.set_time(f"{camera}_frame", sequence=index)
        display_points = np.asarray(points, dtype=np.float32).reshape(-1, 2) + PIXEL_CENTER_OFFSET
        self.recording.log(f"cameras/{camera}/points", rr.Points2D(display_points, radii=2, colors=color),
                           rr.AnyValues(track_id=pa.array(ids, type=pa.uint64())), strict=True)
        self.recording.log(f"cameras/{camera}/annotation", rr.AnyValues(matched=value is not None,
                                                                      capture_time=stamp), strict=True)
