"""iceoryx2 跨语言消息契约（与 firefly-pubsub Rust `#[repr(C)]` 布局严格一致）。

`type_name()` 必须与 Rust 侧 `#[type_name("...")]` 完全一致，iceoryx2 按
字符串 + 布局做跨语言签名校验。修改任一字段/类型名都要同步 Rust 侧
`crates/firefly-pubsub/src/viz.rs`（ctypes 的 `_pack_` 显式设为 8 与 Rust
`#[repr(C)]` 对齐一致，避免平台默认对齐差异）。
"""

from __future__ import annotations

import ctypes

#: 消息类型（与 Rust `firefly_pubsub::viz::kind` 一致）
VIZ_KIND_POSE = 1
VIZ_KIND_LINE_STRIP = 2
VIZ_KIND_VOXELS = 3
VIZ_KIND_SCALARS = 4
VIZ_KIND_BAR_CHART = 5
VIZ_KIND_ARROWS = 6
VIZ_KIND_CLEAR = 7

#: 定长数组上限（与 Rust `viz.rs` 常量一致）
ENTITY_MAX = 64
POINTS_MAX = 512
ARROWS_MAX = 256
VOXELS_MAX = 16384
BINS_MAX = 64
SCALARS_MAX = 4


class VizMessage(ctypes.Structure):
    """统一可视化消息：与 Rust `VizMessage`（`FireflyVizMessage`）布局一致。

    字段顺序与 `#[repr(C)]` 完全对应（含隐式 padding），
    `sizeof` 自检见 [`_self_check`]。
    """

    _pack_ = 8
    _fields_ = [
        ("kind", ctypes.c_uint32),
        ("entity_len", ctypes.c_uint32),
        ("color", ctypes.c_uint8 * 3),
        ("timestamp", ctypes.c_double),
        ("entity", ctypes.c_uint8 * ENTITY_MAX),
        ("xyz", ctypes.c_double * 3),
        ("quat_xyzw", ctypes.c_double * 4),
        ("points", (ctypes.c_double * 3) * POINTS_MAX),
        ("point_count", ctypes.c_uint32),
        ("arrow_count", ctypes.c_uint32),
        ("arrow_origins", (ctypes.c_double * 3) * ARROWS_MAX),
        ("arrow_vectors", (ctypes.c_double * 3) * ARROWS_MAX),
        ("voxels", (ctypes.c_int32 * 3) * VOXELS_MAX),
        ("voxel_count", ctypes.c_uint32),
        ("voxel_size", ctypes.c_float * 3),
        ("voxel_origin", ctypes.c_float * 3),
        ("scalars", ctypes.c_double * SCALARS_MAX),
        ("scalar_count", ctypes.c_uint32),
        ("bins", ctypes.c_uint64 * BINS_MAX),
        ("bin_count", ctypes.c_uint32),
        ("bin_start", ctypes.c_int64),
        ("bin_width", ctypes.c_int64),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyVizMessage"


def _self_check() -> None:
    """布局自检：与 Rust 侧 `viz_message_is_plain_old_data` 断言一致。"""
    assert ctypes.sizeof(VizMessage) == 221944, ctypes.sizeof(VizMessage)
    # kind 常量与 Rust `viz::kind` 一致
    assert VIZ_KIND_ARROWS == 6
    assert VIZ_KIND_CLEAR == 7


_self_check()
