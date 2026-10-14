"""持久记录与 latest-only 预览分流，所有时间戳属于原始测量时刻。"""

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


def blueprint(live: bool):
    history = [rrb.VisibleTimeRange("sim_time", start=rrb.TimeRangeBoundary.infinite(),
                                    end=rrb.TimeRangeBoundary.cursor_relative())]
    # 三种空间坐标各自成图；未提供对齐变换时绝不把 odom/map 当成仿真世界。
    spatial_views = [
        rrb.Spatial3DView(origin="/", name="Simulation world",
                          contents=["+ /world/**", "+ /gt/**"],
                          overrides={"/gt/traj": rrb.VisibleTimeRanges(history)}),
        rrb.Spatial3DView(origin="/", name="Map / planning",
                          contents=["+ /corr/**", "+ /plan/**"],
                          overrides={"/corr/traj": rrb.VisibleTimeRanges(history)}),
        rrb.Spatial3DView(origin="/vio", name="VIO odom", contents=["+ /vio/odom/**", "+ /vio/traj/**"],
                          overrides={"/vio/traj": rrb.VisibleTimeRanges(history)}),
    ]
    if live:
        spatial_views.append(rrb.Spatial3DView(origin="/live", name="Map · latest only",
                                              contents=["+ /live/plan/**"]))
    spatial = rrb.Tabs(*spatial_views)
    cameras = [rrb.Spatial2DView(origin="/cameras/left", name="Left · recorded",
                                 contents=["+ /cameras/left/**"])]
    for name in ("right", "depth"):
        cameras.append(rrb.Spatial2DView(origin=f"/cameras/{name}", name=f"{name.title()} · sampled"))
    cameras += [rrb.Spatial2DView(origin=f"/snapshots/{name}", name=f"{name.title()} · lossless snapshot")
                for name in ("left", "right", "depth")]
    if live:
        cameras += [rrb.Spatial2DView(origin=f"/live/cameras/{name}", name=f"{name.title()} · latest only")
                    for name in ("left", "right", "depth")]
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(spatial, rrb.Tabs(*cameras), column_shares=[3, 2]),
            rrb.Tabs(
                rrb.TimeSeriesView(origin="/vio", name="VIO diagnostics"),
                rrb.TimeSeriesView(origin="/fc", name="Flight control"),
                rrb.TimeSeriesView(origin="/viz", name="Recorder health"),
                rrb.TimeSeriesView(origin="/render", name="Sensor latency"),
                rrb.TextLogView(origin="/logs", name="Logs"),
                rrb.TensorView(origin="/vio/update", name="Update matrices"),
            ), row_shares=[3, 2],
        ),
        rrb.TimePanel(timeline="sim_time", play_state=rrb.components.PlayState.Following),
    )


class RecordingOutputs(Protocol):
    """写入器与采集器所需的录制端口；成员和方法签名供测试替身校验。"""

    @property
    def archive(self) -> rr.RecordingStream: ...

    @property
    def live(self) -> rr.RecordingStream | None: ...

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
        self.live = None
        try:
            if not self.archive.is_enabled():
                raise RuntimeError("Rerun recording is disabled by the environment")
            self._connect(path, url, serve, options, recording_id,
                          stream_factory, file_sink, grpc_sink, blueprint_factory)
        except Exception as error:
            errors = [error]
            for recording in (self.live, self.archive):
                if recording is not None:
                    try:
                        recording.disconnect()
                    except Exception as cleanup_error:
                        errors.append(cleanup_error)
            if len(errors) > 1:
                raise ExceptionGroup("Rerun initialization and cleanup failed", errors) from error
            raise
        self.matches = MatchedPoints(self.archive, options.capture.match_frames)

    def _connect(self, path, url, serve, options, recording_id,
                 stream_factory, file_sink, grpc_sink, blueprint_factory):
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
            # 同一 recording_id 合并视图；只有这个 stream 接收 /live，且没有 FileSink。
            self.live = stream_factory(APP_ID, recording_id=recording_id, send_properties=False)
            self.live.set_sinks(grpc_sink(url=url))
        if not sinks:
            raise ValueError("at least one file or viewer destination is required")
        self.archive.set_sinks(*sinks)
        self.archive.send_blueprint(blueprint_factory(bool(self.live)))
        self.archive.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self.archive.log("viz/session", rr.AnyValues(recording_id=recording_id, rerun_version=rr.__version__), static=True)
        if options.video.enabled:
            self.archive.log("cameras/left/image", rr.VideoStream(codec=rr.VideoCodec.H265), static=True)

    def diagnostic(self, text: str, *, stamp=None, level="INFO"):
        at(self.archive, stamp)
        self.archive.log("logs/viz", rr.TextLog(text, level=level), strict=True)

    def health(self, name: str, values, stamp=None):
        at(self.archive, stamp)
        self.archive.log(f"viz/{name}", rr.Scalars(values), strict=True)

    def video(self, frame, sample, keyframe, latency):
        at(self.archive, frame.stamp)
        self.archive.set_time("left_frame", sequence=frame.index)
        self.archive.log("cameras/left/image", rr.VideoStream.from_fields(sample=sample, is_keyframe=keyframe),
                         rr.AnyValues(capture_time=frame.stamp, frame_index=frame.index), strict=True)
        self.matches.frame("left", frame.stamp, frame.index)
        self.health("video/encoded", [len(sample), latency, int(keyframe), frame.index], frame.stamp)

    def close(self):
        errors = []
        for recording in (self.live, self.archive):
            if recording is not None:
                try:
                    recording.flush(timeout_sec=self.options.shutdown_seconds)
                except Exception as error:
                    errors.append(error)
                finally:
                    try:
                        recording.disconnect()
                    except Exception as error:
                        errors.append(error)
        if errors:
            raise ExceptionGroup("Rerun flush failed", errors)


class MatchedPoints:
    """点集只有在同时间图像已保存时才进入叠加层；缓存严格有界。"""

    def __init__(self, recording, capacity: int):
        self.recording, self.capacity = recording, capacity
        self.frames = {name: OrderedDict() for name in ("left", "right")}
        self.points = {name: OrderedDict() for name in ("left", "right")}
        self.lock = threading.Lock()

    def _remember(self, cache, key, value):
        cache[key] = value
        while len(cache) > self.capacity:
            cache.popitem(last=False)

    def frame(self, camera, stamp, index=None):
        if camera not in self.frames:
            return
        with self.lock:
            self._remember(self.frames[camera], stamp, index)
            self._write(camera, stamp, self.points[camera].get(stamp))

    def observe(self, camera, stamp, points, ids, color):
        with self.lock:
            value = points, ids, color
            self._remember(self.points[camera], stamp, value)
            if stamp in self.frames[camera]:
                self._write(camera, stamp, value)

    def _write(self, camera, stamp, value):
        points, ids, color = value if value is not None else (np.empty((0, 2)), [], [64, 255, 128])
        at(self.recording, stamp)
        if camera == "left" and (index := self.frames[camera].get(stamp)) is not None:
            self.recording.set_time("left_frame", sequence=index)
        display_points = np.asarray(points, dtype=np.float32).reshape(-1, 2) + PIXEL_CENTER_OFFSET
        self.recording.log(f"cameras/{camera}/points", rr.Points2D(display_points, radii=2, colors=color),
                           rr.AnyValues(track_id=pa.array(ids, type=pa.uint64())), strict=True)
        self.recording.log(f"cameras/{camera}/annotation", rr.AnyValues(matched=value is not None,
                                                                      capture_time=stamp), strict=True)
