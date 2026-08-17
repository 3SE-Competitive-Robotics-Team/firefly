"""OTel trace 集成：每个传感器周期一条 trace，跨语言续接。

- 每个相机帧周期（10Hz）开一条新 trace（root span `sim-cycle`）；
- 周期内所有发布（imu/双目/深度/真值）为 cycle 的子 span，共享同一
  `trace_id`，`trace_id`/`span_id` 填进 iceoryx2 `TraceContext` header；
- Rust 侧（vio/demo）续接该 trace，参考回到本进程后周期闭合，
  下一相机帧开新 trace（可区分每次输入）。
"""

from __future__ import annotations

import time

from opentelemetry import trace
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.trace import Span, set_span_in_context

from firefly_mujoco import TraceContext

_tracer: trace.Tracer


def init() -> None:
    """初始化 OTel tracer provider（无 exporter，仅本地 span + W3C id）。"""
    global _tracer
    trace.set_tracer_provider(TracerProvider())
    _tracer = trace.get_tracer("firefly-sim")


def start_cycle() -> Span:
    """开一个新的传感器周期 trace（root span，未结束）。"""
    return _tracer.start_span("sim-cycle")


def end_cycle(cycle: Span) -> None:
    """结束当前周期 root span。"""
    cycle.end()


def child_span(cycle: Span, name: str) -> Span:
    """周期内创建一个发布 span（cycle 的子 span，同 trace，未结束）。"""
    return _tracer.start_span(name, context=set_span_in_context(cycle))


def fill_header(header: TraceContext, span: Span, ts: float) -> None:
    """把 OTel span 的 W3C id 填进 iceoryx2 `TraceContext` header。"""
    sc = span.get_span_context()
    header.version = 1
    header.flags = 1  # sampled
    header.trace_id_hi = (sc.trace_id >> 64) & 0xFFFFFFFFFFFFFFFF
    header.trace_id_lo = sc.trace_id & 0xFFFFFFFFFFFFFFFF
    header.span_id = sc.span_id
    header.send_ts_secs = int(ts)
    header.send_ts_nanos = int((ts - int(ts)) * 1e9)
    header.send_ts_mono_ns = time.monotonic_ns()


def header_trace_id(header: TraceContext) -> int:
    """从 header 重组 128-bit trace_id（日志关联用）。"""
    return (int(header.trace_id_hi) << 64) | int(header.trace_id_lo)
