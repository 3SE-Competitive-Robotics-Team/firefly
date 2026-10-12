"""录制预算与采样策略；TOML 只覆盖默认值，未知键拒绝启动。"""

from dataclasses import dataclass, field, fields
import math
from pathlib import Path
from firefly_workspace import load_toml


ROOT = Path(__file__).resolve().parents[4]


@dataclass(frozen=True)
class VideoOptions:
    enabled: bool = True
    ffmpeg: str = "ffmpeg"
    encoder: str = "hevc_vaapi"
    device: str = "/dev/dri/renderD128"
    fps: int = 10
    bitrate: int = 256_000
    keyframe_interval: int = 10
    queue_frames: int = 3
    max_inflight: int = 16
    max_sample_bytes: int = 2 * 1024 * 1024
    shutdown_seconds: float = 10.0


@dataclass(frozen=True)
class CaptureOptions:
    preview_hz: float = 2.0
    right_hz: float = 0.0
    depth_hz: float = 0.0
    voxel_hz: float = 1.0
    # 原始图像环形缓存的帧数；0 关闭事前窗口，事后窗口仍可触发。
    before_frames: int = 10
    after_frames: int = 10
    trigger_on_error: bool = True
    trigger_cooldown_seconds: float = 10.0
    queue_frames: int = 16
    max_voxels: int = 100_000
    max_entities: int = 256
    match_frames: int = 128


@dataclass(frozen=True)
class Options:
    telemetry_queue: int = 1024
    flush_seconds: float = 0.2
    shutdown_seconds: float = 15.0
    viewer_memory: str = "2GiB"
    # 完整 footer 的 chunk 索引随时长增长；长录制默认不在内存中积累该索引。
    write_footer: bool = False
    video: VideoOptions = field(default_factory=VideoOptions)
    capture: CaptureOptions = field(default_factory=CaptureOptions)


def load(path: str | Path = ROOT / "configs/viz.toml") -> Options:
    return decode(load_toml(path))


def decode(values: dict) -> Options:
    """构造并校验录制配置；不创建录制、编码或 IPC 资源。"""
    values = values.copy()
    video = VideoOptions(**values.pop("video", {}))
    capture = CaptureOptions(**values.pop("capture", {}))
    options = Options(video=video, capture=capture, **values)
    for group in (options, video, capture):
        for item in fields(group):
            value = getattr(group, item.name)
            if item.type is bool and type(value) is not bool:
                raise ValueError(f"{item.name} must be a boolean")
            if item.type is str and (not isinstance(value, str) or not value.strip()):
                raise ValueError(f"{item.name} must be a nonempty string")
            if item.type is float and (type(value) not in (int, float)):
                raise ValueError(f"{item.name} must be a number")
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value) or value < 0:
                    raise ValueError(f"{item.name} must be finite and nonnegative")
            if item.type is int and (type(value) is not int or value <= 0):
                # before/after_frames can explicitly disable the corresponding window.
                if item.name not in ("before_frames", "after_frames") or type(value) is not int or value < 0:
                    raise ValueError(f"{item.name} must be a positive integer")
    if min(options.flush_seconds, options.shutdown_seconds, video.shutdown_seconds) <= 0:
        raise ValueError("flush/shutdown timeouts must be positive")
    if video.encoder not in ("hevc_vaapi", "hevc_qsv", "libx265"):
        raise ValueError("video.encoder must be hevc_vaapi, hevc_qsv or libx265 (explicit software mode)")
    if video.max_inflight < 4 or capture.match_frames < video.max_inflight:
        raise ValueError("match_frames >= max_inflight >= 4 is required")
    return options
