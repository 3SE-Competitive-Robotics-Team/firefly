"""按采集时间采样左目视频，管理编码器及有界媒体队列。"""

import math

from .recording import RecordingOutputs
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


class Capture:
    def __init__(self, recordings: RecordingOutputs, options):
        self.recordings, self.options = recordings, options
        self.encoder = None
        self.shape = None
        self.video_rate = Rate(options.video.fps)
        self.last_stamp = None
        self.accepted = self.skipped = 0

    def tick(self):
        if self.encoder:
            self.encoder.check()

    def handle(self, frame):
        self.tick()
        if frame.camera != "left":
            raise ValueError("video input must be the left camera")
        self.last_stamp = frame.stamp
        if not self.options.video.enabled:
            return
        if not self.video_rate.accept(frame.stamp):
            self.skipped += 1
            return
        if self.encoder is None:
            self.shape = frame.width, frame.height
            self.encoder = Encoder(self.options.video, *self.shape, self.recordings.video,
                                   lambda text: self.recordings.diagnostic(text, level="WARN"), start=False)
            # 先取得所有权，退出路径须能回收部分启动的编码器。
            self.encoder.start()
        if self.shape != (frame.width, frame.height):
            raise ValueError("video resolution changed within a recording")
        self.encoder.submit(frame)
        self.accepted += 1

    def close(self):
        errors = []
        if self.encoder:
            try:
                self.encoder.close()
            except Exception as error:
                errors.append(error)
            try:
                self.recordings.health("video/frames", [self.accepted, self.encoder.emitted,
                                                        self.encoder.dropped, self.skipped], self.last_stamp)
            except Exception as error:
                errors.append(error)
        if errors:
            raise ExceptionGroup("capture cleanup failed", errors)
