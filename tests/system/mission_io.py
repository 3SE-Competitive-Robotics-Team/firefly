"""任务验收的实时 IPC 记录；原始观测与事件经共享 viz 写入 RRD。"""
import ctypes
import math
import time

import iceoryx2 as iox2
from firefly_mujoco import LogMessage, TraceContext
from firefly_viz.messages import VizMessage


class ReferenceMessage(ctypes.Structure):
    _pack_ = 8
    _fields_ = [(name, ctypes.c_double) for name in (
        "timestamp", "position_x", "position_y", "position_z", "velocity_x",
        "velocity_y", "velocity_z", "acceleration_x", "acceleration_y",
        "acceleration_z", "yaw", "yaw_dot")]

    @staticmethod
    def type_name():
        return "FireflyReferenceMessage"


def service(node, topic, cls, capacity=None, publishers=None):
    builder = node.service_builder(iox2.ServiceName.new(topic)).publish_subscribe(cls).user_header(TraceContext)
    if capacity is not None:
        builder = builder.subscriber_max_buffer_size(capacity)
    if publishers is not None:
        builder = builder.max_publishers(publishers)
    return builder.open_or_create()


class Recorder:
    def __init__(self, node):
        self.viz = service(node, "Firefly/Viz", VizMessage, 256, 8).publisher_builder().create()
        self.logs = service(node, "Firefly/Log", LogMessage, capacity=256, publishers=10).publisher_builder().create()

    def message(self, kind, entity, stamp):
        msg = VizMessage()
        msg.kind, msg.timestamp = kind, stamp
        raw = entity.encode()
        if len(raw) > 64:
            raise ValueError("entity path too long")
        msg.entity[:len(raw)], msg.entity_len = raw, len(raw)
        return msg

    def scalars(self, entity, stamp, values):
        msg = self.message(4, entity, stamp)
        msg.scalars[:len(values)], msg.scalar_count = values, len(values)
        self.viz.loan_uninit().write_payload(msg).send()

    def pose(self, entity, stamp, position, quaternion):
        msg = self.message(1, entity, stamp)
        msg.xyz[:], msg.quat_xyzw[:] = position, quaternion
        self.viz.loan_uninit().write_payload(msg).send()

    def event(self, text, stamp=-1., error=False):
        msg = LogMessage()
        wall = time.time()
        msg.sim_time, msg.wall_secs, msg.wall_nanos = stamp, int(wall), int(wall % 1 * 1e9)
        msg.log_level = 1 if error else 3
        tag, raw = b"acceptance", text.encode()[:512]
        msg.tag[:len(tag)], msg.tag_len = tag, len(tag)
        msg.text[:len(raw)], msg.text_len = raw, len(raw)
        self.logs.loan_uninit().write_payload(msg).send()


def pose_values(msg):
    position = [msg.position_x, msg.position_y, msg.position_z]
    velocity = [msg.velocity_x, msg.velocity_y, msg.velocity_z]
    if hasattr(msg, "yaw"):
        quaternion = [0., 0., math.sin(msg.yaw / 2), math.cos(msg.yaw / 2)]
    else:
        quaternion = [msg.quat_x, msg.quat_y, msg.quat_z, msg.quat_w]
    return position, velocity, quaternion
