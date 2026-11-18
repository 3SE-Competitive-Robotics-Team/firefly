"""HIL 悬停 PoC 的真值与控制监视器：打印高度、推力与飞行模式。

只订阅主机侧话题，不参与控制；用于确认 HIL 闭环确实在飞，而 RRD 仍是
原始证据。运行：

    uv run --isolated --package firefly-ipc --no-dev python scripts/hil_hover_monitor.py --seconds 40
"""

from __future__ import annotations

import argparse
import time

import iceoryx2 as iox2
from firefly_ipc.messages import (
    CONTROL_SERVICE_MAX, CONTROL_TOPIC, SCALAR_BUFFER_SIZE, TOPIC_GT, VIZ_MAX_PUBLISHERS,
    FC_STATUS_BUFFER_SIZE, FC_STATUS_TOPIC, ControlMessage, FcStatusMessage, FlightMode,
    OdomMessage, ScalarMessage, TraceContext,
)


def open_sub(node, topic, payload, capacity=None, *, publishers=None):
    builder = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(payload)
        .user_header(TraceContext)
    )
    if capacity is not None:
        builder = builder.subscriber_max_buffer_size(capacity)
    if publishers is not None:
        builder = builder.max_publishers(publishers)
    service = builder.open_or_create()
    return service.subscriber_builder().buffer_size(capacity or 2).create()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--period", type=float, default=0.5)
    args = parser.parse_args()
    iox2.set_log_level(iox2.LogLevel.Error)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    gt = open_sub(node, TOPIC_GT, OdomMessage)
    odom = open_sub(node, "Firefly/Odometry", OdomMessage)
    control = open_sub(node, CONTROL_TOPIC, ControlMessage, CONTROL_SERVICE_MAX)
    status = open_sub(node, FC_STATUS_TOPIC, FcStatusMessage, FC_STATUS_BUFFER_SIZE, publishers=1)
    scalars = open_sub(node, "Firefly/VizScalars", ScalarMessage, SCALAR_BUFFER_SIZE,
                       publishers=VIZ_MAX_PUBLISHERS)
    started = time.monotonic()
    next_print = started
    altitude = 0.0
    odom_z = 0.0
    error_z = 0.0
    thrust = 0.0
    mode = "?"
    while time.monotonic() - started < args.seconds:
        while (sample := gt.receive()) is not None:
            altitude = float(sample.payload().contents.position_z)
        while (sample := odom.receive()) is not None:
            odom_z = float(sample.payload().contents.position_z)
        while (sample := control.receive()) is not None:
            thrust = max(float(v) for v in sample.payload().contents.thrust)
        while (sample := status.receive()) is not None:
            mode = FlightMode(sample.payload().contents.mode).name
        while (sample := scalars.receive()) is not None:
            message = sample.payload().contents
            entity = bytes(message.entity[: message.entity_len]).decode(errors="replace")
            if entity == "fc/debug/position_error_odom" and message.scalar_count >= 3:
                error_z = float(message.scalars[2])
        if time.monotonic() >= next_print:
            print(
                f"t={time.monotonic() - started:6.1f}s  gt_z={altitude:6.3f}m  "
                f"odom_z={odom_z:6.3f}m  err_z={error_z:7.3f}m  "
                f"max_thrust={thrust:6.3f}N  mode={mode}",
                flush=True,
            )
            next_print += args.period
        time.sleep(0.01)


if __name__ == "__main__":
    main()
