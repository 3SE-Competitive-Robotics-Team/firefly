"""按仿真位姿的发布周期创建 `sim-cycle` 根 span。

周期内发布的 IMU 和真值位姿使用同一 trace ID。IPC 消息头保存 trace ID、
span ID 和发送时间，供下游关联。`--no-trace` 跳过 span 创建。
"""

from __future__ import annotations

import time
from typing import Optional

from opentelemetry import trace
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.trace import Span, set_span_in_context

from firefly_ipc.messages import TraceContext

_tracer: trace.Tracer
_enabled: bool = True


def init(*, enabled: bool = True) -> None:
    """初始化 OTel tracer provider。

    Args:
        enabled: False 时跳过 span 创建和导出。
    """
    global _tracer, _enabled
    _enabled = enabled
    if enabled:
        trace.set_tracer_provider(TracerProvider())
        _tracer = trace.get_tracer("firefly-sim")
    else:
        _tracer = trace.get_tracer("noop")


def start_cycle() -> Optional[Span]:
    """开一个新的传感器周期 trace（root span，未结束）。禁用时返回 None。"""
    if not _enabled:
        return None
    return _tracer.start_span("sim-cycle")


def end_cycle(cycle: Optional[Span]) -> None:
    """结束当前周期 root span。禁用时无操作。"""
    if cycle is not None:
        cycle.end()


def child_span(cycle: Optional[Span], name: str) -> Optional[Span]:
    """周期内创建一个发布 span。禁用时返回 None。"""
    if not _enabled or cycle is None:
        return None
    return _tracer.start_span(name, context=set_span_in_context(cycle))


def fill_header(header: TraceContext, span: Optional[Span], _ts: float) -> None:
    """把 OTel span 的 W3C id 与墙钟发送时间填进 iceoryx2 `TraceContext`。

    `_ts` 是仿真时刻，不写入 header：header 的发送时间必须与 Rust 侧一致，
    使用墙钟（Unix 秒）才能计算跨进程的接收年龄。禁用时无操作。
    """
    if span is None:
        return
    sc = span.get_span_context()
    header.version = 1
    header.flags = 1  # sampled
    header.trace_id_hi = (sc.trace_id >> 64) & 0xFFFFFFFFFFFFFFFF
    header.trace_id_lo = sc.trace_id & 0xFFFFFFFFFFFFFFFF
    header.span_id = sc.span_id
    now = time.time()
    header.send_ts_secs = int(now)
    header.send_ts_nanos = int((now - int(now)) * 1e9)
    header.send_ts_mono_ns = time.monotonic_ns()


def header_trace_id(header: TraceContext) -> int:
    """从 header 重组 128-bit trace_id（日志关联用）。"""
    return (int(header.trace_id_hi) << 64) | int(header.trace_id_lo)
