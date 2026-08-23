"""iceoryx2 跨语言消息契约（与 firefly-pubsub Rust `#[repr(C)]` 布局严格一致）。

`type_name()` 必须与 Rust 侧 `#[type_name("...")]` 完全一致，iceoryx2 按
字符串 + 布局做跨语言签名校验。修改任一字段/类型名都要同步 Rust 侧。
"""

from __future__ import annotations

import ctypes

#: 图像分辨率（与 Rust `firefly_pubsub::camera::IMAGE_WIDTH/HEIGHT` 一致）
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT


class TraceContext(ctypes.Structure):
    """User Header：与 Rust `firefly_pubsub::trace::TraceContext` 布局一致。"""

    _fields_ = [
        ("version", ctypes.c_uint8),
        ("flags", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
        ("trace_id_hi", ctypes.c_uint64),
        ("trace_id_lo", ctypes.c_uint64),
        ("span_id", ctypes.c_uint64),
        ("send_ts_secs", ctypes.c_int64),
        ("send_ts_nanos", ctypes.c_uint32),
        ("send_ts_mono_ns", ctypes.c_uint64),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyTraceContext"


class ImuMessage(ctypes.Structure):
    """IMU：与 Rust `ImuMessage`（`FireflyImuMessage`）一致。"""

    _fields_ = [
        ("timestamp", ctypes.c_double),
        ("angular_velocity_x", ctypes.c_double),
        ("angular_velocity_y", ctypes.c_double),
        ("angular_velocity_z", ctypes.c_double),
        ("linear_acceleration_x", ctypes.c_double),
        ("linear_acceleration_y", ctypes.c_double),
        ("linear_acceleration_z", ctypes.c_double),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyImuMessage"


class GrayImageMessage(ctypes.Structure):
    """灰度图：与 Rust `GrayImageMessage`（`FireflyGrayImageMessage`）一致。"""

    _fields_ = [
        ("timestamp", ctypes.c_double),
        ("sensor_id", ctypes.c_int32),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("data", ctypes.c_uint8 * IMAGE_SIZE),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyGrayImageMessage"


class DepthImageMessage(ctypes.Structure):
    """深度图（米制 f32）：与 Rust `DepthImageMessage`（`FireflyDepthImageMessage`）一致。"""

    _fields_ = [
        ("timestamp", ctypes.c_double),
        ("sensor_id", ctypes.c_int32),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("data", ctypes.c_float * IMAGE_SIZE),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyDepthImageMessage"


class ReferenceMessage(ctypes.Structure):
    """参考状态：与 Rust `ReferenceMessage`（`FireflyReferenceMessage`）一致。"""

    _fields_ = [
        ("timestamp", ctypes.c_double),
        ("position_x", ctypes.c_double),
        ("position_y", ctypes.c_double),
        ("position_z", ctypes.c_double),
        ("velocity_x", ctypes.c_double),
        ("velocity_y", ctypes.c_double),
        ("velocity_z", ctypes.c_double),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyReferenceMessage"


class GoalMessage(ctypes.Structure):
    """飞行目标：与 Rust `GoalMessage`（`FireflyGoalMessage`）一致。"""

    _fields_ = [
        ("timestamp", ctypes.c_double),
        ("position_x", ctypes.c_double),
        ("position_y", ctypes.c_double),
        ("position_z", ctypes.c_double),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyGoalMessage"


class OdomMessage(ctypes.Structure):
    """里程计：与 Rust `OdomMessage`（`FireflyOdomMessage`）一致。"""

    _fields_ = [
        ("timestamp", ctypes.c_double),
        ("position_x", ctypes.c_double),
        ("position_y", ctypes.c_double),
        ("position_z", ctypes.c_double),
        ("velocity_x", ctypes.c_double),
        ("velocity_y", ctypes.c_double),
        ("velocity_z", ctypes.c_double),
        ("quat_x", ctypes.c_double),
        ("quat_y", ctypes.c_double),
        ("quat_z", ctypes.c_double),
        ("quat_w", ctypes.c_double),
        ("is_initialized", ctypes.c_bool),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyOdomMessage"


def _self_check() -> None:
    """布局自检：与 Rust 侧测试一致（改布局时同步更新两侧断言）。"""
    assert ctypes.sizeof(TraceContext) == 56, ctypes.sizeof(TraceContext)
    assert ctypes.sizeof(ImuMessage) == 56
    assert ctypes.sizeof(GrayImageMessage) == 76824
    assert ctypes.sizeof(DepthImageMessage) == 307224
    assert ctypes.sizeof(ReferenceMessage) == 56
    assert ctypes.sizeof(OdomMessage) == 96


_self_check()
