#!/usr/bin/env python3
"""视觉全局定位离线评测数据：摆拍 npz → 原始 bin（对照 gen_gicp_eval_data.py）。

输入：`logs/visamp/<traj>/frame_III.npz`（collect_vision_frames.py 产物：
left uint8[240,320]、depth f32、pos f64[3]、t f64，水平姿态）。
输出：`logs/bench/vision_eval/frames/<traj>_<idx>.bin`，布局
`u64 W, u64 H, u8 left[W*H], f32 depth[W*H], f64 pos[3], f64 quat_xyzw[4], f64 t`
（quat 为水平姿态恒等 [0,0,0,1]，与采集约定一致）。

用法：uv run python scripts/gen_vision_eval_data.py [--traj straight_forward]
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent

SRC = REPO_ROOT / "logs" / "visamp"
OUT = REPO_ROOT / "logs" / "bench" / "vision_eval"
QUAT_XYZW_IDENTITY = (0.0, 0.0, 0.0, 1.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", default="straight_forward")
    args = ap.parse_args()
    src = SRC / args.traj
    if not src.is_dir():
        sys.exit(f"[gen] 无采集帧目录 {src}（先跑 scripts/collect_vision_frames.py）")
    out = OUT / "frames"
    out.mkdir(parents=True, exist_ok=True)

    n = 0
    for npz in sorted(src.glob("frame_*.npz")):
        d = np.load(npz)
        left = d["left"].astype(np.uint8)
        depth = d["depth"].astype(np.float32)
        pos = d["pos"].astype(np.float64)
        if left.ndim != 2 or depth.shape != left.shape or pos.shape != (3,):
            sys.exit(f"[gen] {npz} 布局异常: left {left.shape} depth {depth.shape} pos {pos.shape}")
        h, w = left.shape
        stem = npz.stem  # frame_III
        name = f"{args.traj}_{stem.removeprefix('frame_')}.bin"
        with open(out / name, "wb") as f:
            f.write(struct.pack("<QQ", w, h))
            f.write(left.tobytes())
            f.write(depth.tobytes())
            f.write(pos.astype("<f8").tobytes())
            f.write(struct.pack("<4d", *QUAT_XYZW_IDENTITY))
            f.write(struct.pack("<d", float(d["t"])))
        n += 1
    print(f"[gen] {args.traj}: {n} frames -> {out}")


if __name__ == "__main__":
    main()
