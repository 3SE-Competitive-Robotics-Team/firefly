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
        ("acceleration_x", ctypes.c_double),
        ("acceleration_y", ctypes.c_double),
        ("acceleration_z", ctypes.c_double),
        ("yaw", ctypes.c_double),
        ("yaw_dot", ctypes.c_double),
        ("alignment_translation", ctypes.c_double * 3),
        ("alignment_rotation", ctypes.c_double * 4),
        ("has_alignment", ctypes.c_uint64),
        ("goal_timestamp", ctypes.c_double),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyReferenceMessage"


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


class PlantStateMessage(ctypes.Structure):
    """被控对象状态（真值，每物理步）：与 Rust `PlantStateMessage`（`FireflyPlantStateMessage`）一致。

    姿态为**机体→世界** Hamilton 四元数 `[x, y, z, w]`（与 `firefly-flight` 的
    `QuadState::attitude` 同约定）；角速度为机体系。注意与 `OdomMessage` 的
    JPL `q_GtoI` 不同约定，不得混用。
    """

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
        ("angular_velocity_x", ctypes.c_double),
        ("angular_velocity_y", ctypes.c_double),
        ("angular_velocity_z", ctypes.c_double),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyPlantStateMessage"


class AirframeMessage(ctypes.Structure):
    """机体/执行器描述（1Hz 电平）：与 Rust `AirframeMessage`（`FireflyAirframeMessage`）一致。

    质量/惯量/阻尼是被控对象属性，旋翼位置/旋向/单电机推力上限/反扭矩系数是执行器
    属性——两者都从被控对象发布，飞控不另抄一份。
    """

    _fields_ = [
        ("timestamp", ctypes.c_double),
        ("mass", ctypes.c_double),
        ("inertia_x", ctypes.c_double),
        ("inertia_y", ctypes.c_double),
        ("inertia_z", ctypes.c_double),
        ("linear_drag", ctypes.c_double),
        ("angular_drag", ctypes.c_double),
        ("rotor_positions", (ctypes.c_double * 3) * 4),
        ("rotor_spins", ctypes.c_double * 4),
        ("max_thrust_per_motor", ctypes.c_double),
        ("torque_coefficient", ctypes.c_double),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyAirframeMessage"


class ControlMessage(ctypes.Structure):
    """飞控指令（1kHz，plant 取最新）：与 Rust `ControlMessage`（`FireflyControlMessage`）一致。

    `state_time` 是飞控用于本指令的状态采样时刻（plant 据此判陈旧）；`thrust` 是
    4 电机推力（N，与 `scene.ROTORS` 同序），plant 按机体几何合成 wrench。
    """

    _fields_ = [
        ("state_time", ctypes.c_double),
        ("thrust", ctypes.c_double * 4),
        ("tick", ctypes.c_uint64),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyControlMessage"


#: 被控对象状态话题（sim → 飞控，每物理步）
PLANT_STATE_TOPIC = "Firefly/PlantState"
#: 机体/执行器描述话题（sim → 飞控，1Hz 电平）
AIRFRAME_TOPIC = "Firefly/Airframe"
#: 飞控指令话题（飞控 → sim，1kHz）
CONTROL_TOPIC = "Firefly/Control"

#: 统一日志话题（与 Rust `firefly_pubsub::log::LOG_TOPIC` 一致）
LOG_TOPIC = "Firefly/Log"

#: 日志级别（与 Rust `firefly_pubsub::log::level` 一致）
LOG_LEVEL_ERROR = 1
LOG_LEVEL_WARN = 2
LOG_LEVEL_INFO = 3
LOG_LEVEL_DEBUG = 4
LOG_LEVEL_TRACE = 5

#: 定长上限（与 Rust `log.rs` 常量一致）
LOG_TAG_MAX = 64
LOG_TEXT_MAX = 512


class LogMessage(ctypes.Structure):
    """结构化日志：与 Rust `LogMessage`（`FireflyLogMessage`）一致。"""

    _pack_ = 8
    _fields_ = [
        ("log_level", ctypes.c_uint8),
        ("tag_len", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
        ("text_len", ctypes.c_uint32),
        ("tag", ctypes.c_uint8 * LOG_TAG_MAX),
        ("text", ctypes.c_uint8 * LOG_TEXT_MAX),
        ("sim_time", ctypes.c_double),
        ("wall_secs", ctypes.c_int64),
        ("wall_nanos", ctypes.c_uint32),
        ("trace_id_hi", ctypes.c_uint64),
        ("trace_id_lo", ctypes.c_uint64),
        ("sequence", ctypes.c_uint64),
        ("dropped_records", ctypes.c_uint64),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyLogMessage"


def _self_check() -> None:
    """布局自检：与 Rust 侧测试一致（改布局时同步更新两侧断言）。"""
    assert ctypes.sizeof(TraceContext) == 56, ctypes.sizeof(TraceContext)
    assert ctypes.sizeof(ImuMessage) == 56
    assert ctypes.sizeof(GrayImageMessage) == 76824
    assert ctypes.sizeof(DepthImageMessage) == 307224
    assert ctypes.sizeof(ReferenceMessage) == 168
    assert ctypes.sizeof(OdomMessage) == 96
    assert ctypes.sizeof(LogMessage) == 640, ctypes.sizeof(LogMessage)
    assert ctypes.sizeof(PlantStateMessage) == 112, ctypes.sizeof(PlantStateMessage)
    assert ctypes.sizeof(AirframeMessage) == 200, ctypes.sizeof(AirframeMessage)
    assert ctypes.sizeof(ControlMessage) == 48, ctypes.sizeof(ControlMessage)


_self_check()


#: 消息类型（与 Rust `firefly_pubsub::viz::kind` 一致）
VIZ_KIND_POSE = 1
VIZ_KIND_LINE_STRIP = 2
VIZ_KIND_VOXELS = 3
VIZ_KIND_SCALARS = 4
VIZ_KIND_BAR_CHART = 5
VIZ_KIND_ARROWS = 6
VIZ_KIND_CLEAR = 7
VIZ_KIND_VOXELS_ADD = 8
VIZ_KIND_VOXELS_REMOVE = 9
VIZ_KIND_KEYPOINTS_2D = 10
VIZ_KIND_CAMERA = 12

#: 定长数组上限（与 Rust `viz.rs` 常量一致）
ENTITY_MAX = 64
POINTS_MAX = 512
ARROWS_MAX = 256
VOXELS_MAX = 16384
BINS_MAX = 64
SCALARS_MAX = 4
#: 1kHz IMU 双标量与相机诊断突发的接收容量，与 Rust SCALAR_BUFFER_SIZE 一致。
SCALAR_BUFFER_SIZE = 1024


class AttitudeMessage(ctypes.Structure):
    """Firefly/VioAttitude，与 Rust attitude::AttitudeMessage 一致。"""

    _pack_ = 8
    _fields_ = [
        ("timestamp", ctypes.c_double),
        ("posterior_time", ctypes.c_double),
        ("session", ctypes.c_uint64),
        ("quat_xyzw", ctypes.c_double * 4),
        ("angular_velocity", ctypes.c_double * 3),
        ("gyro_bias", ctypes.c_double * 3),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyVioAttitude"


class VizMessage(ctypes.Structure):
    """空间诊断消息：与 Rust `VizMessage`（`FireflyVizMessage`）布局一致。

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
        ("scalars", ctypes.c_double * SCALARS_MAX),
        ("scalar_count", ctypes.c_uint32),
        ("bins", ctypes.c_uint64 * BINS_MAX),
        ("bin_count", ctypes.c_uint32),
        ("bin_start", ctypes.c_int64),
        ("bin_width", ctypes.c_int64),
        ("sequence", ctypes.c_uint64),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyVizMessage"


def _self_check() -> None:
    """布局自检：与 Rust 侧 `viz_message_is_plain_old_data` 断言一致。"""
    assert ctypes.sizeof(VizMessage) == 25312, ctypes.sizeof(VizMessage)
    # kind 常量与 Rust `viz::kind` 一致
    assert VIZ_KIND_ARROWS == 6
    assert VIZ_KIND_CLEAR == 7


_self_check()


class ScalarMessage(ctypes.Structure):
    """Firefly/VizScalars，与 Rust ScalarMessage 一致。"""
    _pack_ = 8
    _fields_ = [
        ("kind", ctypes.c_uint32), ("entity_len", ctypes.c_uint32),
        ("timestamp", ctypes.c_double), ("entity", ctypes.c_uint8 * ENTITY_MAX),
        ("scalars", ctypes.c_double * SCALARS_MAX), ("scalar_count", ctypes.c_uint32),
        ("sequence", ctypes.c_uint64),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyScalarMessage"


class VoxelMessage(ctypes.Structure):
    """Firefly/VizVoxels，全量替换或有序增量。"""
    _pack_ = 8
    _fields_ = [
        ("kind", ctypes.c_uint32), ("entity_len", ctypes.c_uint32),
        ("timestamp", ctypes.c_double), ("entity", ctypes.c_uint8 * ENTITY_MAX),
        ("voxels", (ctypes.c_int32 * 3) * VOXELS_MAX),
        ("voxel_count", ctypes.c_uint32), ("voxel_size", ctypes.c_float * 3),
        ("voxel_origin", ctypes.c_float * 3),
        ("sequence", ctypes.c_uint64),
        ("chunk_index", ctypes.c_uint32), ("chunk_count", ctypes.c_uint32),
    ]

    @staticmethod
    def type_name() -> str:
        return "FireflyVoxelMessage"


assert ctypes.sizeof(ScalarMessage) == 128
assert ctypes.sizeof(VoxelMessage) == 196736
