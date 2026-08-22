"""VIO 端到端精度测试：采集 odometry + ground truth，按时间对齐算误差。

用法：先启动 sim（--no-trace），再运行此脚本，最后 Ctrl-C 结束并输出报告。

    uv run firefly-sim --script --no-trace &
    sleep 3
    python scripts/vio_accuracy_test.py [--duration 30] [--output vio_eval.json]
"""

from __future__ import annotations

import argparse
import ctypes
import json
import select
import signal
import sys
import time

import iceoryx2 as iox2
import numpy as np


TOPIC_ODOM = "Firefly/Odometry"
TOPIC_GT = "Firefly/GroundTruth"


def _make_sub(node, topic, payload_cls, header_cls=None):
    svc = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(payload_cls)
    )
    if header_cls is not None:
        svc = svc.user_header(header_cls)
    svc = svc.open_or_create()
    return svc.subscriber_builder().buffer_size(1).create()


def main() -> None:
    parser = argparse.ArgumentParser(description="VIO 端到端精度测试")
    parser.add_argument("--duration", type=float, default=30.0, help="采集时长（秒）")
    parser.add_argument("--output", type=str, default="vio_eval.json", help="输出文件")
    args = parser.parse_args()

    # 直接用 firefly_mujoco 的消息类型（它们有正确的 type_name()）
    from firefly_mujoco import OdomMessage, TraceContext

    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    odom_sub = _make_sub(node, TOPIC_ODOM, OdomMessage, TraceContext)
    gt_sub = _make_sub(node, TOPIC_GT, OdomMessage, TraceContext)

    print(f"已订阅 {TOPIC_ODOM} + {TOPIC_GT}，采集 {args.duration}s...")

    # 打印 Python 端类型详情（用于调试兼容性）
    print("═══ Python 端 TypeDetails ═══")
    print(f"  OdomMessage:    name={OdomMessage.type_name()} size={ctypes.sizeof(OdomMessage)} align={ctypes.alignment(OdomMessage)}")
    print(f"  TraceContext:   name={TraceContext.type_name()} size={ctypes.sizeof(TraceContext)} align={ctypes.alignment(TraceContext)}")
    print("════════════════════════════")

    print("启动 sim 后按 Enter 开始采集（或等 5s 自动开始）")

    # 等待 sim 就绪
    try:
        signal.signal(signal.SIGINT, lambda *_: None)
        _, ready, _ = select.select([sys.stdin], [], [], 5.0)
        if ready:
            sys.stdin.readline()
    except Exception:
        time.sleep(5)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    odom_data: list[dict] = []
    gt_data: list[dict] = []
    t_start = time.perf_counter()
    wall = 0.0

    print("采集中... Ctrl-C 结束")
    while wall < args.duration:
        wall = time.perf_counter() - t_start
        t_sim = wall  # 1x real-time: wall ≈ sim

        # 消费所有 pending 消息
        while (s := odom_sub.receive()) is not None:
            m = s.payload().contents
            odom_data.append({
                "t": m.timestamp,
                "px": m.position_x, "py": m.position_y, "pz": m.position_z,
                "vx": m.velocity_x, "vy": m.velocity_y, "vz": m.velocity_z,
                "qx": m.quat_x, "qy": m.quat_y, "qz": m.quat_z, "qw": m.quat_w,
            })

        while (s := gt_sub.receive()) is not None:
            m = s.payload().contents
            gt_data.append({
                "t": m.timestamp,
                "px": m.position_x, "py": m.position_y, "pz": m.position_z,
                "vx": m.velocity_x, "vy": m.velocity_y, "vz": m.velocity_z,
                "qx": m.quat_x, "qy": m.quat_y, "qz": m.quat_z, "qw": m.quat_w,
            })

        time.sleep(0.001)

    print(f"\n采集完成：odom {len(odom_data)} 帧，gt {len(gt_data)} 帧")

    if not odom_data or not gt_data:
        print("ERROR: 无数据，确保 sim 正在运行")
        return

    # ── 按时间对齐（最近邻） ──
    gt_times = np.array([d["t"] for d in gt_data])
    aligned: list[dict] = []

    for o in odom_data:
        idx = np.argmin(np.abs(gt_times - o["t"]))
        g = gt_data[idx]
        if abs(gt_times[idx] - o["t"]) > 0.15:  # >150ms 偏差跳过
            continue
        aligned.append({"odom": o, "gt": g})

    if not aligned:
        print("ERROR: 时间对齐失败（odom 和 gt 时间戳偏差过大）")
        return

    print(f"对齐后 {len(aligned)} 对")

    # ── 计算误差 ──
    pos_err = np.array([
        [a["odom"]["px"] - a["gt"]["px"],
         a["odom"]["py"] - a["gt"]["py"],
         a["odom"]["pz"] - a["gt"]["pz"]]
        for a in aligned
    ])
    pos_norm = np.linalg.norm(pos_err, axis=1)

    vel_err = np.array([
        [a["odom"]["vx"] - a["gt"]["vx"],
         a["odom"]["vy"] - a["gt"]["vy"],
         a["odom"]["vz"] - a["gt"]["vz"]]
        for a in aligned
    ])
    vel_norm = np.linalg.norm(vel_err, axis=1)

    t_arr = np.array([a["odom"]["t"] for a in aligned])

    def stats(name: str, vals: np.ndarray) -> None:
        print(f"  {name:20s}  mean={np.mean(vals):.4f}  std={np.std(vals):.4f}  "
              f"max={np.max(vals):.4f}  RMSE={np.sqrt(np.mean(vals**2)):.4f}")

    print("\n═══ VIO 精度报告 ═══")
    print(f"  采集时长: {t_arr[-1] - t_arr[0]:.1f}s  帧数: {len(aligned)}")
    stats("位置误差 X (m)", pos_err[:, 0])
    stats("位置误差 Y (m)", pos_err[:, 1])
    stats("位置误差 Z (m)", pos_err[:, 2])
    stats("位置误差 |.| (m)", pos_norm)
    stats("速度误差 |.| (m/s)", vel_norm)

    # ── 保存 ──
    result = {
        "duration_s": float(t_arr[-1] - t_arr[0]),
        "num_frames": len(aligned),
        "position_error_mean_m": float(np.mean(pos_norm)),
        "position_error_rmse_m": float(np.sqrt(np.mean(pos_norm**2))),
        "position_error_max_m": float(np.max(pos_norm)),
        "velocity_error_mean_ms": float(np.mean(vel_norm)),
        "trajectory": [
            {
                "t": float(t_arr[i]),
                "odom": aligned[i]["odom"],
                "gt": aligned[i]["gt"],
                "pos_err_m": float(pos_norm[i]),
            }
            for i in range(0, len(aligned), max(1, len(aligned) // 500))  # 降采样到 ~500 点
        ],
    }
    with open(args.output, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\n轨迹已保存 → {args.output}")


if __name__ == "__main__":
    main()
