#!/usr/bin/env python3
"""读 5 条 VoidOdom，打印姿态相对恒等的角度（验证机体系发布）。"""

import sys
import time

import iceoryx2 as iox2
import numpy as np

sys.path.insert(0, "packages/firefly-mujoco/src")

from firefly_mujoco import OdomMessage, TraceContext  # noqa: E402

node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
sub = (
    node.service_builder(iox2.ServiceName.new("Firefly/VoidOdom"))
    .publish_subscribe(OdomMessage)
    .user_header(TraceContext)
    .open_or_create()
    .subscriber_builder()
    .create()
)
time.sleep(3)
n = 0
while n < 5:
    s = sub.receive()
    if s is None:
        time.sleep(0.05)
        continue
    m = s.payload().contents
    ang = 2 * np.degrees(np.arccos(np.clip(abs(m.quat_w), -1, 1)))
    print(f"{ang:.1f} deg vs identity | pos {m.position_x:.2f} {m.position_y:.2f} {m.position_z:.2f}")
    n += 1
