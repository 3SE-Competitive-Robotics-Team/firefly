"""FFmpeg H.265 worker：有界原图输入、Annex B 帧边界、采集时间 FIFO。

FFmpeg 插入 AUD，并禁用 B 帧。每个 Rerun sample 是一个 access unit；
pipe.read 的分块边界不参与帧划分。只在编码前丢帧，已编码样本必须完整写出。
"""

from collections import deque
from dataclasses import dataclass
import queue
import re
import signal
import subprocess
import threading
import time

from .config import VideoOptions


@dataclass(frozen=True)
class Frame:
    camera: str
    stamp: float
    index: int
    width: int
    height: int
    pixels: bytes


START = re.compile(b"\x00\x00(?:\x00)?\x01")


def nal_types(data: bytes) -> list[int]:
    return [(data[m.end()] >> 1) & 63 for m in START.finditer(data) if m.end() + 1 < len(data)]


class AccessUnits:
    def __init__(self, max_bytes: int):
        self.buffer = bytearray()
        self.max_bytes = max_bytes

    def feed(self, data: bytes, *, final: bool = False) -> list[bytes]:
        self.buffer.extend(data)
        boundaries = [m.start() for m in START.finditer(self.buffer)
                      if m.end() + 1 < len(self.buffer) and (self.buffer[m.end()] >> 1) & 63 == 35]
        samples = []
        begin = 0
        for end in boundaries[1:]:
            samples.append(bytes(self.buffer[begin:end]))
            begin = end
        if begin:
            del self.buffer[:begin]
        if len(self.buffer) > self.max_bytes:
            raise ValueError("H.265 access unit exceeds budget or AUD is missing")
        if final and self.buffer:
            samples.append(bytes(self.buffer))
            self.buffer.clear()
        for sample in samples:
            kinds = nal_types(sample)
            if len(sample) > self.max_bytes or kinds.count(35) != 1 or not any(k < 32 for k in kinds):
                raise ValueError("invalid H.265 access unit: expected AUD and coded picture")
            starts = list(START.finditer(sample))
            pictures = 0
            for i, start in enumerate(starts):
                end = starts[i + 1].start() if i + 1 < len(starts) else len(sample)
                header = start.end()
                if end - header < 2 or sample[header] & 128 or not sample[header + 1] & 7:
                    raise ValueError("invalid H.265 access unit: truncated or invalid NAL")
                kind = (sample[header] >> 1) & 63
                if kind < 32:
                    if end - header < 3:
                        raise ValueError("invalid H.265 access unit: truncated slice")
                    if sample[header + 2] & 128:
                        pictures += 1
            if pictures != 1:
                raise ValueError("invalid H.265 access unit: expected exactly one picture")
        return samples


def command(options: VideoOptions, width: int, height: int) -> list[str]:
    args = [options.ffmpeg, "-hide_banner", "-loglevel", "warning", "-nostats", "-nostdin"]
    if options.encoder == "hevc_vaapi":
        args += ["-vaapi_device", options.device]
    elif options.encoder == "hevc_qsv":
        args += ["-init_hw_device", f"vaapi=va:{options.device}", "-init_hw_device", "qsv=qs@va"]
    args += ["-f", "rawvideo", "-pixel_format", "gray", "-video_size", f"{width}x{height}",
             "-framerate", str(options.fps), "-i", "pipe:0", "-an"]
    if options.encoder == "hevc_vaapi":
        args += ["-vf", "format=nv12,hwupload"]
    else:
        args += ["-vf", "format=yuv420p" if options.encoder == "libx265" else "format=nv12"]
    args += ["-c:v", options.encoder, "-bf", "0", "-g", str(options.keyframe_interval),
             "-b:v", str(options.bitrate), "-maxrate", str(options.bitrate),
             "-bufsize", str(options.bitrate * 2)]
    if options.encoder == "libx265":
        args += ["-preset", "ultrafast", "-tune", "zerolatency", "-x265-params",
                 f"bframes=0:open-gop=0:scenecut=0:repeat-headers=1:aud=1:keyint={options.keyframe_interval}:pools=1"]
    elif options.encoder == "hevc_vaapi":
        args += ["-idr_interval", "0", "-async_depth", "1"]
    else:
        args += ["-look_ahead_depth", "0", "-async_depth", "1", "-forced_idr", "1", "-idr_interval", "0"]
    args += ["-bsf:v", "dump_extra=freq=keyframe,hevc_metadata=aud=insert",
             "-fps_mode", "passthrough", "-flush_packets", "1", "-f", "hevc", "pipe:1"]
    return args


class Encoder:
    """错误在 check/close 传播；不重启编码器、不静默切换实现。"""

    def __init__(self, options: VideoOptions, width: int, height: int, emit, diagnostic, *, start=True):
        self.options, self.emit, self.diagnostic = options, emit, diagnostic
        self.frames = queue.Queue(maxsize=options.queue_frames)
        self.pending = queue.Queue(maxsize=options.max_inflight)
        self.errors = queue.SimpleQueue()
        self.failure = None
        self.tail = deque(maxlen=16)
        self.submitted = self.emitted = self.dropped = 0
        self.closed = False
        self.draining = threading.Event()
        self.aborting = threading.Event()
        self.process = subprocess.Popen(command(options, width, height), stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        self.threads = [threading.Thread(target=self._guard, args=(target,), name=name)
                        for target, name in ((self._write, "video-input"), (self._read, "video-output"),
                                             (self._stderr, "video-diagnostics"))]
        self.started = []
        if start:
            self.start()

    def start(self):
        try:
            for thread in self.threads:
                thread.start()
                self.started.append(thread)
        except Exception as error:
            self.errors.put(error)
            self._abort()
            self.close()
            raise

    def _guard(self, target):
        try:
            target()
        except Exception as error:
            self.errors.put(error)
            self._abort()

    def _abort(self):
        self.aborting.set()
        self.draining.set()
        if self.process.poll() is None:
            try:
                self.process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            except OSError as error:
                self.errors.put(error)

    def check(self):
        if self.failure is None:
            try:
                self.failure = self.errors.get_nowait()
            except queue.Empty:
                pass
        if self.failure is not None:
            raise RuntimeError(f"H.265 worker failed: {self.failure}; {' | '.join(tuple(self.tail))}") from self.failure
        if not self.closed and self.process.poll() is not None:
            raise RuntimeError(f"FFmpeg exited {self.process.returncode}: {' | '.join(tuple(self.tail))}")

    def submit(self, frame: Frame):
        self.check()
        if self.draining.is_set():
            raise RuntimeError("H.265 encoder is closing")
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        try:
            while not self.aborting.is_set():
                try:
                    frame = self.frames.get(timeout=0.05)
                except queue.Empty:
                    if self.draining.is_set():
                        break
                    continue
                self.pending.put((frame, time.monotonic()), timeout=self.options.shutdown_seconds)
                if self.aborting.is_set():
                    break
                self.process.stdin.write(frame.pixels)
                self.process.stdin.flush()
                self.submitted += 1
        finally:
            self.process.stdin.close()

    def _read(self):
        parser = AccessUnits(self.options.max_sample_bytes)
        parameters = {}
        while True:
            data = self.process.stdout.read1(65536)
            for sample in parser.feed(data, final=not data):
                starts = list(START.finditer(sample))
                for i, start in enumerate(starts):
                    kind = (sample[start.end()] >> 1) & 63
                    if kind in (32, 33, 34):
                        end = starts[i + 1].start() if i + 1 < len(starts) else len(sample)
                        parameters[kind] = sample[start.start():end]
                kinds = nal_types(sample)
                keyframe = any(k in (19, 20) for k in kinds)
                if self.emitted == 0 and not keyframe:
                    raise ValueError("video must start with IDR")
                if keyframe:
                    if len(parameters) != 3:
                        raise ValueError("IDR requires VPS/SPS/PPS")
                    # 每个 IDR 自含参数，Viewer 跳转或重新连接无需依赖早期帧。
                    nals = [sample[start.start():starts[i + 1].start() if i + 1 < len(starts) else len(sample)]
                            for i, start in enumerate(starts)]
                    # AUD 必须仍是该 access unit 的第一个 NAL。
                    sample = (b"".join(nal for kind, nal in zip(kinds, nals) if kind == 35)
                              + b"".join(parameters[k] for k in (32, 33, 34))
                              + b"".join(nal for kind, nal in zip(kinds, nals) if kind not in (35, 32, 33, 34)))
                if len(sample) > self.options.max_sample_bytes:
                    raise ValueError("self-contained H.265 access unit exceeds budget")
                frame, submitted_at = self.pending.get_nowait()
                self.emit(frame, sample, keyframe, time.monotonic() - submitted_at)
                self.emitted += 1
            if not data:
                return

    def _stderr(self):
        for raw in iter(lambda: self.process.stderr.readline(4096), b""):
            line = raw.decode(errors="replace").strip()
            if line:
                self.tail.append(line)
                self.diagnostic(line)

    def close(self):
        if self.closed:
            self.check()
            return
        self.draining.set()
        timeout = self.options.shutdown_seconds
        problems = []
        deadline = time.monotonic() + timeout
        try:
            if self.threads[0] in self.started:
                self.threads[0].join(max(0., deadline - time.monotonic()))
                if self.threads[0].is_alive():
                    raise TimeoutError("FFmpeg input did not drain")
            else:
                self.process.stdin.close()
            self.process.wait(timeout=max(0., deadline - time.monotonic()))
            for thread in self.started:
                thread.join(max(0., deadline - time.monotonic()))
            if any(thread.is_alive() for thread in self.started):
                raise TimeoutError("FFmpeg output did not drain")
        except Exception as error:
            problems.append(error)
            self._abort()
            if self.process.poll() is None:
                try:
                    self.process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    try:
                        self.process.terminate()
                        self.process.wait(timeout=timeout)
                    except (OSError, subprocess.TimeoutExpired) as error:
                        problems.append(error)
                except OSError as error:
                    problems.append(error)
            deadline = time.monotonic() + timeout
            for thread in self.started:
                thread.join(max(0., deadline - time.monotonic()))
        alive = [t.name for t in self.started if t.is_alive()]
        if alive or self.process.poll() is None:
            raise RuntimeError(f"video cleanup incomplete: pid={self.process.pid}, workers={alive}, errors={problems}")
        for pipe in (self.process.stdin, self.process.stdout, self.process.stderr):
            if not pipe.closed:
                try:
                    pipe.close()
                except OSError as error:
                    problems.append(error)
        self.closed = True
        if problems or self.process.returncode or self.submitted != self.emitted or not self.pending.empty() or not self.frames.empty():
            self.errors.put(RuntimeError(f"incomplete video: {self.submitted} inputs, {self.emitted} outputs, "
                                         f"exit={self.process.returncode}, {problems}"))
        self.check()
