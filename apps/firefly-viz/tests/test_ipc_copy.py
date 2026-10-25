"""firefly-viz IPC 拷贝回归锁（loan 别名滞留 = 撕裂读 + SIGSEGV）。

运行：uv run --with pytest pytest apps/firefly-viz/tests/（纯 ctypes，无需 IPC/rrd）
"""

import ctypes
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from firefly_viz.main import detach_copy  # noqa: E402
from firefly_viz.writer import entity as _entity  # noqa: E402
from firefly_viz.messages import VizMessage  # noqa: E402


def _make_pose() -> VizMessage:
    m = VizMessage()
    m.kind = 1
    b = b"okvis/odom"
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
    assert _entity(dst) == "okvis/odom"
    assert dst.timestamp == 12.5
    # 模拟 loan 复用覆盖 / 释放
    src.entity_len = 7
    for i, c in enumerate(b"okvis/d"):
        src.entity[i] = c
    src.timestamp = -999.0
    src.xyz = (ctypes.c_double * 3)(0.0, 0.0, 0.0)
    assert _entity(dst) == "okvis/odom"
    assert dst.timestamp == 12.5
    assert list(dst.xyz) == [1.0, 2.0, 3.0]


def test_detach_preserves_type():
    """泛型：同函数处理 `VizMessage`（`LogMessage` 同构，略）。"""
    dst = detach_copy(_make_pose())
    assert isinstance(dst, VizMessage)
    assert dst.kind == 1


def test_ground_truth_detach_owns_pose_storage():
    from firefly_mujoco import OdomMessage

    src = OdomMessage(timestamp=2., position_x=3., quat_z=1., is_initialized=True)
    dst = detach_copy(src)
    src.timestamp, src.position_x, src.quat_z = 0., 0., 0.
    assert isinstance(dst, OdomMessage)
    assert (dst.timestamp, dst.position_x, dst.quat_z, dst.is_initialized) == (2., 3., 1., True)


def test_entity_rejects_corrupt_payload():
    import pytest
    m = _make_pose()
    m.entity_len = 65
    with pytest.raises(ValueError):
        _entity(m)
    m.entity_len = 1
    m.entity[0] = 0xFF
    with pytest.raises(UnicodeDecodeError):
        _entity(m)


def test_wire_layouts_match_rust():
    from firefly_viz.messages import ScalarMessage, VoxelMessage
    assert ctypes.sizeof(ScalarMessage) == 128
    assert ctypes.sizeof(VizMessage) == 25312
    assert ctypes.sizeof(VoxelMessage) == 196736
    assert ScalarMessage.timestamp.offset == 8
    assert ScalarMessage.scalars.offset == 80
    assert ScalarMessage.sequence.offset == 120
    assert VoxelMessage.voxels.offset == 80
    assert VoxelMessage.sequence.offset == 196720
    assert VizMessage.points.offset == 144
