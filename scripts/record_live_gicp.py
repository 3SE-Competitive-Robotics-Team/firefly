#!/usr/bin/env python3
"""录制线上真实数据供 GICP 回放：depth 帧 + VoidOdom 位姿 + GT。

自包含：起 sim（--script）+ void，采 35s，优雅杀进程。
输出 logs/bench/replay/live_<traj>/frame_<nnn>.bin
（u64 W, u64 H, f32 depth, f64 七元 void_xyz_xyzw, f64 GT xyz, f64 三时间戳）。

用法：uv run python scripts/record_live_gicp.py ff_cross
"""

from __future__ import annotations

import struct
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "packages" / "firefly-mujoco" / "src"))

TRAJ = sys.argv[1] if len(sys.argv) > 1 else "ff_cross"
DUR = 35.0
OUT = REPO_ROOT / "logs" / "bench" / "replay" / f"live_{TRAJ}"
OUT.mkdir(parents=True, exist_ok=True)


def main() -> None:
    import iceoryx2 as iox2
    import numpy as np

    from firefly_mujoco import DepthImageMessage, OdomMessage, TraceContext

    subprocess.run(["pkill", "-9", "-f", "firefly-sim --no-trace"], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "target/release/void"], capture_output=True)
    time.sleep(1)
    subprocess.run(
        ["bash", "-lc", "rm -rf /tmp/iceoryx2/services 2>/dev/null; true"],
        capture_output=True,
    )
    sim = subprocess.Popen(
        ["uv", "run", "firefly-sim", "--no-trace", "--script", TRAJ],
        cwd=REPO_ROOT,
        stdout=open(OUT / "sim.log", "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(8)
    void = subprocess.Popen(
        ["cargo", "run", "--release", "-p", "void"],
        cwd=REPO_ROOT,
        env={**__import__("os").environ, "RUST_LOG": "warn"},
        stdout=open(OUT / "void.log", "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(6)

    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)

    def sub(topic, cls):
        return (
            node.service_builder(iox2.ServiceName.new(topic))
            .publish_subscribe(cls)
            .user_header(TraceContext)
            .open_or_create()
            .subscriber_builder()
            .create()
        )

    gt_sub = sub("Firefly/GroundTruth", OdomMessage)
    vo_sub = sub("Firefly/VoidOdom", OdomMessage)
    dp_sub = sub("Firefly/Depth", DepthImageMessage)

    n = 0
    t_end = time.perf_counter() + DUR
    latest_vo = None
    latest_gt = None
    while time.perf_counter() < t_end:
        while (s := gt_sub.receive()) is not None:
            m = s.payload().contents
            latest_gt = (
                m.timestamp,
                np.array([m.position_x, m.position_y, m.position_z]),
            )
        while (s := vo_sub.receive()) is not None:
            m = s.payload().contents
            latest_vo = (
                m.timestamp,
                np.array([m.position_x, m.position_y, m.position_z]),
                np.array([m.quat_x, m.quat_y, m.quat_z, m.quat_w]),
            )
        while (s := dp_sub.receive()) is not None:
            m = s.payload().contents
            if latest_vo is None or latest_gt is None:
                continue
            w, h = int(m.width), int(m.height)
            depth = np.array(m.data[: w * h], dtype=np.float64)
            with open(OUT / f"frame_{n:03d}.bin", "wb") as f:
                f.write(struct.pack("<QQ", w, h))
                f.write(depth.astype("<f4").tobytes())
                f.write(latest_vo[1].astype("<f8").tobytes())
                f.write(latest_vo[2].astype("<f8").tobytes())
                f.write(latest_gt[1].astype("<f8").tobytes())
                f.write(
                    struct.pack(
                        "<ddd", m.timestamp, latest_vo[0], latest_gt[0]
                    )
                )
            n += 1
        time.sleep(0.005)
    print(f"[record] {n} frames -> {OUT}")

    for pat in ["firefly-sim --no-trace", "target/release/void"]:
        subprocess.run(["pkill", "-INT", "-f", pat], capture_output=True)
    time.sleep(3)
    for pat in ["firefly-sim --no-trace", "target/release/void"]:
        subprocess.run(["pkill", "-9", "-f", pat], capture_output=True)


if __name__ == "__main__":
    main()
