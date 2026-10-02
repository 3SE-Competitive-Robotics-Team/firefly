#!/usr/bin/env python3
"""独立离线库图采集：隔离话题摆放渲染相机，按同一时间戳配对左图、深度和位姿。

只运行 render，不运行 sim、估计器或飞控；标签是离线已知相机位姿。
输出为 aliked --build-map 的二进制输入资产，采集清单记录接受/拒绝及资产 hash。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import logging
from pathlib import Path
import signal
import struct
import subprocess
import time
import tomllib

import iceoryx2 as iox2
import numpy as np

from firefly_mujoco import DepthImageMessage, GrayImageMessage, OdomMessage, TraceContext

ROOT = Path(__file__).resolve().parents[1]
W, H = 320, 240
log = logging.getLogger(__name__)


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def capture_assets():
    """渲染内容、坐标与采集行为的输入指纹。"""
    return {name: digest(ROOT / name) for name in [
        "models/rmuc2026/field.glb", "models/rmuc2026/rmuc2026_collision.json",
        "configs/render.toml", "configs/scene.toml", "configs/vision_map.toml", "target/release/render",
        "scripts/collect_vision_map.py",
    ]}


def reusable_capture(directory, assets):
    """清单完整且输入、逐帧产物均匹配才可复用；空采集不是成功。"""
    try:
        manifest = json.loads((directory / "capture_manifest.json").read_text())
        rows = manifest["frames"]
        if (manifest["status"] != "complete" or manifest["assets"] != assets
                or len(rows) != manifest["requested"]):
            return False
        expected = {f"frame_{row['index']:05d}.bin": row["sha256"] for row in rows if row["accepted"]}
        actual = {path.name: digest(path) for path in directory.glob("*.bin")}
        return bool(expected) and expected == actual
    except (OSError, ValueError, KeyError, TypeError):
        return False


def grid_poses(boxes, step, heights, yaws, x_range=None, y_range=None):
    """避开碰撞盒外扩 0.15m 的离散机体原点；姿态绕世界 Z 轴。"""
    if not np.isfinite(step) or step <= 0 or not np.isfinite([*heights, *yaws]).all():
        raise ValueError("invalid capture grid")
    low = np.min(boxes[:, :3] - boxes[:, 3:], axis=0)
    high = np.max(boxes[:, :3] + boxes[:, 3:], axis=0)
    xr = x_range or (low[0] + 1., high[0] - 1.)
    yr = y_range or (low[1] + 1., high[1] - 1.)
    for z in heights:
        for x in np.arange(xr[0], xr[1] + 1e-9, step):
            for y in np.arange(yr[0], yr[1] + 1e-9, step):
                position = np.array([x, y, z])
                if np.any(np.all(np.abs(boxes[:, :3] - position) <= boxes[:, 3:] + 0.15, axis=1)):
                    continue
                for yaw in yaws:
                    angle = np.deg2rad(yaw) / 2
                    yield position.tolist(), [0., 0., float(np.sin(angle)), float(np.cos(angle))]


def configured_poses(boxes, configuration):
    """全场网格与作业区补密共享高度/朝向，重叠位姿只采集一次。"""
    base = {"step": 5., "heights": [1.2, 2.2], "yaws": [0., 90., 180., 270.], **configuration}
    regions = base.pop("regions", [])
    seen = set()
    for grid in [base, *(base | region for region in regions)]:
        for position, quaternion in grid_poses(boxes, **grid):
            key = tuple(position + quaternion)
            if key not in seen:
                seen.add(key)
                yield position, quaternion


def paired_frame(left, depth, stamp):
    """只有同帧、定标尺寸正确的数据才可作为几何标签的输入。"""
    if left.timestamp != stamp or depth.timestamp != stamp:
        raise ValueError("image/depth/pose timestamp mismatch")
    if (left.width, left.height, depth.width, depth.height) != (W, H, W, H):
        raise ValueError("capture dimensions differ from calibration")
    image = np.ctypeslib.as_array(left.data).copy().reshape(H, W)
    distance = np.ctypeslib.as_array(depth.data).copy().reshape(H, W)
    fraction = float(np.mean(np.isfinite(distance) & (distance > 0.2) & (distance < 20.)))
    quality = {"gray_std": float(image.std()), "valid_depth_fraction": fraction}
    return image, distance, quality


def write_frame(path, image, depth, position, quaternion, stamp):
    with path.open("wb") as stream:
        stream.write(struct.pack("<QQ", W, H))
        stream.write(image.astype(np.uint8).tobytes())
        stream.write(depth.astype("<f4").tobytes())
        stream.write(struct.pack("<3d4dd", *position, *quaternion, stamp))


def port(node, name, cls, publish=False):
    service = (node.service_builder(iox2.ServiceName.new("Firefly/Offline/" + name))
               .publish_subscribe(cls).user_header(TraceContext).open_or_create())
    return service.publisher_builder().create() if publish else service.subscriber_builder().create()


def collect(poses, directory, timeout):
    if any(directory.glob("*.bin")):
        raise FileExistsError(f"capture directory contains frames: {directory}")
    directory.mkdir(parents=True, exist_ok=True)
    iox2.set_log_level(iox2.LogLevel.Error)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    pose_pub = port(node, "GroundTruth", OdomMessage, True)
    left_sub = port(node, "CameraLeft", GrayImageMessage)
    depth_sub = port(node, "Depth", DepthImageMessage)
    assets = capture_assets()
    child = subprocess.Popen([str(ROOT / "target/release/render"), "--offline"], cwd=ROOT)
    results = []
    stamp = 0.
    start_stamp = 0.
    complete = False
    try:
        for index, (position, quaternion) in enumerate(poses):
            deadline = time.monotonic() + (max(timeout, 180.) if index == 0 else timeout)
            lefts, depths = {}, {}
            next_send = 0.
            accepted = False
            quality = {}
            # 同一位姿保持至配对成功；递增时间戳允许着色器/资产就绪后继续出图。
            while time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError(f"offline render exited: {child.returncode}")
                if time.monotonic() >= next_send:
                    stamp = round(stamp + 0.2, 6)
                    msg = OdomMessage()
                    msg.timestamp = stamp
                    msg.position_x, msg.position_y, msg.position_z = position
                    msg.quat_x, msg.quat_y, msg.quat_z, msg.quat_w = quaternion
                    msg.is_initialized = True
                    pose_pub.loan_uninit().write_payload(msg).send()
                    next_send = time.monotonic() + 0.2
                for sub, cls, frames in [(left_sub, GrayImageMessage, lefts), (depth_sub, DepthImageMessage, depths)]:
                    while (sample := sub.receive()) is not None:
                        message = cls.from_buffer_copy(bytes(sample.payload().contents))
                        if message.timestamp > start_stamp:
                            frames[message.timestamp] = message
                    for old in sorted(frames)[:-16]:
                        del frames[old]
                common = sorted(lefts.keys() & depths.keys())
                if common:
                    frame_stamp = common[-1]
                    image, depth, quality = paired_frame(lefts.pop(frame_stamp), depths.pop(frame_stamp), frame_stamp)
                    if quality["gray_std"] >= 2 and quality["valid_depth_fraction"] >= 0.05:
                        path = directory / f"frame_{index:05d}.bin"
                        write_frame(path, image, depth, position, quaternion, frame_stamp)
                        quality.update(sha256=digest(path), timestamp=frame_stamp)
                        accepted = True
                        break
                time.sleep(0.01)
            results.append({"index": index, "position": position, "quat_xyzw": quaternion,
                            "accepted": accepted, **quality})
            if not accepted or (index + 1) % 10 == 0 or index + 1 == len(poses):
                log.info("库图 %d/%d accepted=%s %s", index + 1, len(poses), accepted, quality)
            start_stamp = stamp
        complete = any(row["accepted"] for row in results)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=30)
        (directory / "capture_manifest.json").write_text(json.dumps({"status": "complete" if complete else "incomplete", "assets": assets, "requested": len(poses), "frames": results}, indent=2) + "\n")
    if not any(row["accepted"] for row in results):
        raise RuntimeError("no usable synchronized frames")
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames-dir", type=Path, default=ROOT / "models/rmuc2026/derived/vision_frames")
    parser.add_argument("--step", type=float)
    parser.add_argument("--heights", type=float, nargs="+")
    parser.add_argument("--yaws", type=float, nargs="+")
    parser.add_argument("--x-range", type=float, nargs=2)
    parser.add_argument("--y-range", type=float, nargs=2)
    parser.add_argument("--timeout", type=float, default=15.)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--model", type=Path, default=ROOT / "models/aliked-n16-k512.onnx")
    parser.add_argument("--out", type=Path, default=ROOT / "apps/planner/maps/rmuc2026.ffvmap")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    if args.build and not args.model.is_file():
        raise FileNotFoundError(f"ALIKED model missing: {args.model}")
    collision = ROOT / "models/rmuc2026/rmuc2026_collision.json"
    boxes = np.asarray(json.loads(collision.read_text())["boxes"])
    with (ROOT / "configs/vision_map.toml").open("rb") as stream:
        configuration = tomllib.load(stream)
    configuration.update({key: getattr(args, key) for key in ["step", "heights", "yaws", "x_range", "y_range"]
                          if getattr(args, key) is not None})
    poses = list(configured_poses(boxes, configuration))
    if not poses:
        raise ValueError("capture grid has no free poses")
    collect(poses, args.frames_dir, args.timeout)
    if args.build:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([str(ROOT / "target/release/aliked"), "--build-map", str(args.frames_dir),
                        "--model", str(args.model), "--out", str(args.out)], cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
