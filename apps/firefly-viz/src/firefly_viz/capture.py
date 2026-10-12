"""大数据预算：按采集时间采样、最新预览、触发式无损窗口。"""

from collections import deque
import math
import time

import numpy as np
import rerun as rr

from .recording import RecordingOutputs, at
from .video import Encoder


class Rate:
    def __init__(self, hz: float):
        self.period = 1.0 / hz if hz else float("inf")
        self.next_due = None

    def accept(self, stamp: float) -> bool:
        if self.period == float("inf"):
            return False
        if self.next_due is not None and stamp < self.next_due - 1e-9:
            return False
        if self.next_due is None:
            self.next_due = stamp + self.period
        else:
            # 保留采样相位，小幅抖动不能把相邻目标帧的间隔再延长一整周期。
            periods = max(1, math.floor((stamp - self.next_due + 1e-9) / self.period) + 1)
            self.next_due += periods * self.period
        return True


def pixels(frame):
    dtype = "<f4" if frame.camera == "depth" else "u1"
    data = np.frombuffer(frame.pixels, dtype=dtype).reshape(frame.height, frame.width)
    return rr.DepthImage(data, meter=1.0) if frame.camera == "depth" else rr.Image(data)


class Capture:
    def __init__(self, recordings: RecordingOutputs, options, trigger):
        self.recordings, self.options, self.trigger = recordings, options, trigger
        self.encoder = None
        self.shape = None
        capture = options.capture
        self.rings = {name: deque(maxlen=capture.before_frames) for name in ("left", "right", "depth")}
        self.after = dict.fromkeys(self.rings, 0)
        self.saved = dict.fromkeys(self.rings, float("-inf"))
        self.preview = {name: Rate(capture.preview_hz) for name in self.rings}
        self.sample = {"right": Rate(capture.right_hz), "depth": Rate(capture.depth_hz)}
        self.video_rate = Rate(options.video.fps)
        self.last_trigger = float("-inf")
        self.last_stamp = None
        self.accepted = self.skipped = 0

    def tick(self):
        if self.encoder:
            self.encoder.check()
        self._capture_trigger()

    def _capture_trigger(self):
        if self.trigger.is_set():
            self.trigger.clear()
            now = time.monotonic()
            if now - self.last_trigger >= self.options.capture.trigger_cooldown_seconds:
                self.last_trigger = now
                for name, ring in self.rings.items():
                    for frame in ring:
                        self._snapshot(frame)
                    self.after[name] = self.options.capture.after_frames
                self.recordings.diagnostic("lossless capture triggered", stamp=self.last_stamp)

    def handle(self, frame):
        self.tick()
        self.last_stamp = frame.stamp
        if frame.camera == "left" and self.options.video.enabled:
            if self.video_rate.accept(frame.stamp):
                if self.encoder is None:
                    self.shape = frame.width, frame.height
                    self.encoder = Encoder(self.options.video, *self.shape, self.recordings.video,
                                           lambda text: self.recordings.diagnostic(text, level="WARN"), start=False)
                    # 先取得所有权；部分线程启动失败后，退出路径仍能找到并回收编码器。
                    self.encoder.start()
                if self.shape != (frame.width, frame.height):
                    raise ValueError("video resolution changed within a recording")
                self.encoder.submit(frame)
                self.accepted += 1
            else:
                self.skipped += 1
        if frame.camera in self.sample and self.sample[frame.camera].accept(frame.stamp):
            at(self.recordings.archive, frame.stamp)
            self.recordings.archive.log(f"cameras/{frame.camera}/image", pixels(frame), strict=True)
            self.recordings.matches.frame(frame.camera, frame.stamp)
        if self.recordings.live and self.preview[frame.camera].accept(frame.stamp):
            # 独立 namespace 与只连 gRPC 的 stream，static 不进入持久录制。
            self.recordings.live.log(f"live/cameras/{frame.camera}/image", pixels(frame),
                                     rr.AnyValues(capture_time=frame.stamp, frame_index=frame.index),
                                     static=True, strict=True)
        if self.after[frame.camera]:
            self._snapshot(frame)
            self.after[frame.camera] -= 1
        self.rings[frame.camera].append(frame)

    def _snapshot(self, frame):
        if frame.stamp <= self.saved[frame.camera]:
            return
        at(self.recordings.archive, frame.stamp)
        self.recordings.archive.log(f"snapshots/{frame.camera}/image", pixels(frame),
                                    rr.AnyValues(capture_time=frame.stamp, frame_index=frame.index), strict=True)
        self.saved[frame.camera] = frame.stamp

    def close(self):
        errors = []
        # 最后一条诊断也可能触发；编码器失败不能阻止其余退出清理。
        try:
            self._capture_trigger()
        except Exception as error:
            errors.append(error)
        if self.encoder:
            try:
                self.encoder.close()
            except Exception as error:
                errors.append(error)
        try:
            if self.encoder:
                self.recordings.health("video/frames", [self.accepted, self.encoder.emitted,
                                                        self.encoder.dropped, self.skipped], self.last_stamp)
            self.recordings.health("snapshots/pending", list(self.after.values()), self.last_stamp)
        except Exception as error:
            errors.append(error)
        if errors:
            raise ExceptionGroup("capture cleanup failed", errors)
