#!/usr/bin/env python3
"""场景视觉素材库采集：自己拉起 `sim` + `render`，在不同方位拍照，落关键帧素材。

用途：为视觉全局定位（`aliked --build-map` → `lightglue`）采集**本场景**的带位姿
关键帧。库图必须与场景同源——MuJoCo 侧（`collect_vision_frames.py`）只对
warehouse/boxes 有效，rmuc2026 的 MuJoCo 场景无视觉 mesh，故本脚本从在线
`render` 取图。

流程：
1. 清残留进程与 iceoryx2 → 起 `render` + `sim`（`--no-camera`；sim 用真值反馈
   `--odom-topic Firefly/VoidOdom`，航点悬停稳定）。
2. 按 `--x-range/--y-range/--step` 生成栅格航点（撞地形盒的自动跳过），逐个用
   `Firefly/Reference` 驱动真机；稳定后落一帧「左目灰度 + 深度 + 真值位姿」。
3. 可选 `--build`：调 `aliked --build-map` 直接产出 `ffvmap` 库图。
4. 优雅退出（SIGINT）并清理。

用法：
  uv run python scripts/collect_vision_map.py --prefix rmuc --x-range 0 6 --y-range -2 2 --step 2
  uv run python scripts/collect_vision_map.py --prefix rmuc --build --out apps/planner/maps/rmuc2026.ffvmap
"""

from __future__ import annotations

import argparse
import json
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "packages" / "firefly-mujoco" / "src"))

import iceoryx2 as iox2  # noqa: E402
from firefly_mujoco import (  # noqa: E402
    DepthImageMessage,
    GrayImageMessage,
    OdomMessage,
    ReferenceMessage,
    TraceContext,
    load_scene_name,
)

W, H = 320, 240
#: 各场景独立的 bin 帧目录（避免不同场景的关键帧混进同一张库图）。
FRAMES_ROOT = REPO_ROOT / "logs" / "bench" / "vision_eval"
#: 悬停稳定判据：位姿误差（m）与速度（m/s）持续 `SETTLE_TIME` 低于阈值。
SETTLE_DIST = 0.12
SETTLE_SPEED = 0.08
SETTLE_TIME = 0.6
PER_WP_TIMEOUT = 8.0
#: 航点离地形盒的安全裕度（米）：机半宽 + PD 瞬态余量。
CLEARANCE = 0.15


def collision_boxes(scene: str) -> np.ndarray:
    """场景碰撞盒 `[cx,cy,cz,hx,hy,hz]`（缺文件返回空）。"""
    path = REPO_ROOT / "models" / scene / f"{scene}_collision.json"
    if not path.is_file():
        return np.empty((0, 6))
    return np.asarray(json.loads(path.read_text())["boxes"], dtype=float)


def grid_waypoints(args: argparse.Namespace, scene: str) -> list[tuple[float, float, float]]:
    """栅格航点，剔除撞地形盒的点（无碰撞文件时不剔除）。"""
    boxes = collision_boxes(scene)
    xs = np.arange(args.x_range[0], args.x_range[1] + 1e-9, args.step)
    ys = np.arange(args.y_range[0], args.y_range[1] + 1e-9, args.step)
    pts: list[tuple[float, float, float]] = []
    for x in xs:
        for y in ys:
            p = np.array([x, y, args.z])
            if boxes.size and ((np.abs(boxes[:, :3] - p) <= boxes[:, 3:] + CLEARANCE).all(axis=1)).any():
                continue
            pts.append((float(x), float(y), float(args.z)))
    return pts


def free_port(node, topic: str, cls, publish: bool = False):
    builder = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(cls)
        .user_header(TraceContext)
        .open_or_create()
    )
    return builder.publisher_builder().create() if publish else builder.subscriber_builder().create()


def main() -> None:
    ap = argparse.ArgumentParser(description="从 render 采集场景视觉关键帧（库图素材）")
    ap.add_argument("--prefix", default="rmuc", help="输出文件名前缀（默认 rmuc）")
    ap.add_argument("--z", type=float, default=1.2, help="采集高度（米，默认 1.2）")
    ap.add_argument("--x-range", type=float, nargs=2, default=[0.0, 6.0], help="x 范围（米）")
    ap.add_argument("--y-range", type=float, nargs=2, default=[-2.0, 2.0], help="y 范围（米）")
    ap.add_argument("--step", type=float, default=2.0, help="栅格步长（米，默认 2.0）")
    ap.add_argument("--yaws", default="0", help="每个航点的偏航角组（度，逗号分隔；默认 0）")
    ap.add_argument("--frames-dir", type=Path, default=None, help="bin 输出目录（缺省 logs/bench/vision_eval/frames_<scene>）")
    ap.add_argument("--build", action="store_true", help="采完直接建库（aliked --build-map）")
    ap.add_argument("--out", type=Path, default=None, help="库图输出路径（--build 时必需）")
    args = ap.parse_args()

    scene = load_scene_name()
    frames_dir = args.frames_dir or (FRAMES_ROOT / f"frames_{scene}")
    waypoints = grid_waypoints(args, scene)
    if not waypoints:
        sys.exit("[collect] 无可用航点（范围/步长/裕度？）")
    frames_dir.mkdir(parents=True, exist_ok=True)
    print(f"[collect] 场景 {scene}：{len(waypoints)} 个航点，z={args.z}")

    # 清残留 + 起进程（render 最先，sim 最后；不删 iceoryx2 直到全部退出）。
    subprocess.run(["pkill", "-INT", "-f", "target/release/render"], capture_output=True)
    subprocess.run(["pkill", "-INT", "-f", "firefly-sim"], capture_output=True)
    subprocess.run(["pkill", "-INT", "-f", "target/release/(vio|aliked|lightglue|gicp|planner)"], capture_output=True)
    time.sleep(3)
    subprocess.run(
        ["bash", "-lc", "rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state"],
        capture_output=True,
    )
    render = subprocess.Popen(
        [str(REPO_ROOT / "target" / "release" / "render")],
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(6)
    sim = subprocess.Popen(
        ["uv", "run", "firefly-sim", "--no-trace", "--no-camera", "--odom-topic", "Firefly/VoidOdom"],
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        iox2.set_log_level(iox2.LogLevel.Error)
        node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        ref = free_port(node, "Firefly/Reference", ReferenceMessage, publish=True)
        gt_sub = free_port(node, "Firefly/GroundTruth", OdomMessage)
        left_sub = free_port(node, "Firefly/CameraLeft", GrayImageMessage)
        depth_sub = free_port(node, "Firefly/Depth", DepthImageMessage)
        # 等第一帧相机（render 就绪 + sim 发真值）。
        print("[collect] 等待 render/sim 就绪...")
        deadline = time.time() + 30
        slots: dict[str, object] = {}
        while time.time() < deadline and len(slots) < 3:
            for s, key in ((gt_sub, "gt"), (left_sub, "left"), (depth_sub, "depth")):
                while (sample := s.receive()) is not None:
                    slots[key] = sample.payload().contents
            time.sleep(0.05)
        if len(slots) < 3:
            sys.exit(f"[collect] 就绪超时（缺 {set(['gt', 'left', 'depth']) - set(slots)}）")

        index = 0
        yaws = [float(v) * 3.141592653589793 / 180.0 for v in args.yaws.split(",") if v.strip()]
        for x, y, z in waypoints:
            for yaw in yaws:
                yaw_deg = yaw * 180.0 / 3.141592653589793
                t0 = time.time()
                settled_since = None
                while time.time() - t0 < PER_WP_TIMEOUT:
                    msg = ReferenceMessage()
                    msg.timestamp = time.time()
                    msg.position_x, msg.position_y, msg.position_z = x, y, z
                    msg.yaw = yaw
                    ref.loan_uninit().write_payload(msg).send()
                    for s, key in ((gt_sub, "gt"), (left_sub, "left"), (depth_sub, "depth")):
                        while (sample := s.receive()) is not None:
                            slots[key] = sample.payload().contents
                    gt = slots["gt"]
                    err = ((gt.position_x - x) ** 2 + (gt.position_y - y) ** 2 + (gt.position_z - z) ** 2) ** 0.5
                    spd = (gt.velocity_x**2 + gt.velocity_y**2 + gt.velocity_z**2) ** 0.5
                    # 真机偏航 = atan2(R10, R00)：由 xyzw 四元数算机体 x 轴在世界 xy 的方位。
                    qx, qy, qz, qw = gt.quat_x, gt.quat_y, gt.quat_z, gt.quat_w
                    yaw_now = np.arctan2(
                        2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
                    )
                    yaw_err = abs((yaw_now - yaw + np.pi) % (2.0 * np.pi) - np.pi)
                    if err < SETTLE_DIST and spd < SETTLE_SPEED and yaw_err < 0.09:
                        settled_since = settled_since or time.time()
                        if time.time() - settled_since >= SETTLE_TIME:
                            break
                    else:
                        settled_since = None
                    time.sleep(0.02)
                gt, left, depth = slots["gt"], slots["left"], slots["depth"]
                img = np.frombuffer(bytes(left.data), np.uint8).reshape(H, W)
                dep = np.frombuffer(bytes(depth.data), np.float32).reshape(H, W)
                path = frames_dir / f"{args.prefix}_{index:03d}.bin"
                with open(path, "wb") as f:
                    f.write(struct.pack("<QQ", W, H))
                    f.write(img.tobytes())
                    f.write(dep.astype("<f4").tobytes())
                    f.write(struct.pack("<3d", gt.position_x, gt.position_y, gt.position_z))
                    f.write(struct.pack("<4d", gt.quat_x, gt.quat_y, gt.quat_z, gt.quat_w))
                    f.write(struct.pack("<d", gt.timestamp))
                print(
                    f"[collect] {index:02d} 目标({x:.1f},{y:.1f},{z:.1f}) yaw{yaw_deg:+.0f} 实际"
                    f"({gt.position_x:.2f},{gt.position_y:.2f},{gt.position_z:.2f}) -> {path.name}"
                )
                index += 1
    finally:
        render.send_signal(signal.SIGINT)
        sim.send_signal(signal.SIGINT)
        render.wait(timeout=10)
        sim.wait(timeout=10)

    print(f"[collect] {index} 帧 -> {frames_dir}")
    if args.build:
        out = args.out or (REPO_ROOT / "apps" / "planner" / "maps" / f"{scene}.ffvmap")
        subprocess.run(
            [
                "cargo",
                "run",
                "--release",
                "-p",
                "aliked",
                "--",
                "--build-map",
                str(frames_dir),
                "--out",
                str(out),
            ],
            cwd=REPO_ROOT,
            check=True,
        )
        print(f"[collect] 库图 -> {out}")


if __name__ == "__main__":
    main()
