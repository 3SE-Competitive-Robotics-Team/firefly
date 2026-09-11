"""firefly-viz IPC 拷贝回归锁（loan 别名滞留 = 撕裂读 + SIGSEGV）。

运行：uv run --with pytest pytest apps/firefly-viz/tests/（纯 ctypes，无需 IPC/rrd）
"""

import ctypes
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from firefly_viz.main import _entity, detach_copy  # noqa: E402
from firefly_viz.messages import VizMessage  # noqa: E402


def _make_pose() -> VizMessage:
    m = VizMessage()
    m.kind = 1
    b = b"vio/odom"
    m.entity_len = len(b)
    for i, c in enumerate(b):
        m.entity[i] = c
    m.timestamp = 12.5
    m.xyz = (ctypes.c_double * 3)(1.0, 2.0, 3.0)
    m.quat_xyzw = (ctypes.c_double * 4)(0.0, 0.0, 0.0, 1.0)
    return m


def test_detach_is_independent():
    """改原对象后拷贝不变（别名则必变——回归即失败）。"""
    src = _make_pose()
    dst = detach_copy(src)
    assert _entity(dst) == "vio/odom"
    assert dst.timestamp == 12.5
    # 模拟 loan 复用覆盖 / 释放
    src.entity_len = 7
    for i, c in enumerate(b"vio/deb"):
        src.entity[i] = c
    src.timestamp = -999.0
    src.xyz = (ctypes.c_double * 3)(0.0, 0.0, 0.0)
    assert _entity(dst) == "vio/odom"
    assert dst.timestamp == 12.5
    assert list(dst.xyz) == [1.0, 2.0, 3.0]


def test_detach_preserves_type():
    """泛型：同函数处理 `VizMessage`（`LogMessage` 同构，略）。"""
    dst = detach_copy(_make_pose())
    assert isinstance(dst, VizMessage)
    assert dst.kind == 1


def test_entity_tolerates_garbage():
    """坏实体不炸（fuzz 实证过 `rr.log` 容忍；此处只锁解码不抛异常）。"""
    m = _make_pose()
    m.entity_len = 64
    for i in range(64):
        m.entity[i] = 0xFF
    assert isinstance(_entity(m), str)
    m.entity_len = 20
    raw = b"vio/odom\x00\x00garbage!!"
    for i, c in enumerate(raw):
        m.entity[i] = c
    assert _entity(m).startswith("vio/odom")
