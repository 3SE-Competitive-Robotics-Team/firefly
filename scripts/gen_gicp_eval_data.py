#!/usr/bin/env python3
"""GICP 离线评测数据生成：先验点云 + 轨迹深度帧。

- 先验云：解析 MuJoCo SCENE_XML（box 外表面 + 地面），0.08m 采样，
  与 export_prior_planes.py 同几何源。
- 轨迹帧：logs/bench/ff_fix/<traj>/run1 GT，每 2s 取一位姿（位置真值 +
  水平姿态），MuJoCo 直接摆位渲染深度（含默认噪声，诚实口径）。
- 输出 logs/bench/gicp_eval/：prior_cloud.bin（u64 条数 + f32 xyz），
  frames/<traj>_<idx>.bin（u64 W, u64 H + f32 depth + f64 xyz）。

用法：uv run python scripts/gen_gicp_eval_data.py
"""

from __future__ import annotations

import re
import struct
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "packages" / "firefly-mujoco" / "src"))

from firefly_mujoco import DroneEnv  # noqa: E402
from firefly_mujoco.scene import SCENE_XML  # noqa: E402

OUT = REPO_ROOT / "logs" / "bench" / "gicp_eval"
TRAJS = [
    "straight_forward",
    "ff_cross",
    "ff_zigzag",
    "ff_sway",
    "ff_spiral",
    "ff_climb",
    "ff_fast",
    "ff_low",
    "ff_diag",
    "ff_estop",
    "ff_laps",
]


def parse_boxes(xml: str) -> list[tuple[np.ndarray, np.ndarray]]:
    """解析 box geom → [(center(3), half(3))]（跳过无人机体小盒：half < 0.2 全轴）。"""
    out = []
    for m in re.finditer(
        r'<geom type="box" pos="([\d\.\- ]+)" size="([\d\.\- ]+)"', xml
    ):
        c = np.fromstring(m.group(1), sep=" ")
        h = np.fromstring(m.group(2), sep=" ")
        if np.all(h < 0.2):
            continue  # 无人机体
        out.append((c, h))
    return out


def sample_box(c: np.ndarray, h: np.ndarray, step: float) -> np.ndarray:
    """6 外表面网格采样（含底面，无害）。"""
    pts = []
    for ax in range(3):
        for s in (-1.0, 1.0):
            u, v = [a for a in range(3) if a != ax]
            nu = max(2, int(round(2 * h[u] / step)) + 1)
            nv = max(2, int(round(2 * h[v] / step)) + 1)
            gu = np.linspace(c[u] - h[u], c[u] + h[u], nu)
            gv = np.linspace(c[v] - h[v], c[v] + h[v], nv)
            uu, vv = np.meshgrid(gu, gv)
            p = np.zeros((uu.size, 3))
            p[:, u] = uu.ravel()
            p[:, v] = vv.ravel()
            p[:, ax] = c[ax] + s * h[ax]
            pts.append(p)
    return np.vstack(pts)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "frames").mkdir(exist_ok=True)

    boxes = parse_boxes(SCENE_XML)
    print(f"[gen] boxes={len(boxes)}")
    cloud = [sample_box(c, h, 0.08) for c, h in boxes]
    gx, gy = np.meshgrid(
        np.arange(-3.0, 13.01, 0.2), np.arange(-1.0, 9.01, 0.2)
    )
    cloud.append(
        np.stack([gx.ravel(), gy.ravel(), np.zeros_like(gx.ravel())], axis=1)
    )
    prior = np.vstack(cloud).astype(np.float64)
    with open(OUT / "prior_cloud.bin", "wb") as f:
        f.write(struct.pack("<Q", len(prior)))
        f.write(prior.astype("<f4").tobytes())
    print(f"[gen] prior points={len(prior)}")

    env = DroneEnv()  # 默认噪声（depth_noise=0.02）：诚实口径
    n_frames = 0
    for traj in TRAJS:
        run = REPO_ROOT / "logs" / "bench" / "ff_fix" / traj / "run1"
        gt = np.load(run / "gt.npy")
        gtt = np.load(run / "gt_t.npy")
        t = gtt[0]
        idx = 0
        while t <= gtt[-1]:
            j = int(np.argmin(np.abs(gtt - t)))
            env.reset(
                gt[j].astype(float), np.array([0.0, 0.0, 0.0, 1.0])
            )
            import mujoco

            mujoco.mj_forward(env.model, env.data)
            depth = env.render_depth().astype(np.float64)
            h, w = depth.shape
            with open(OUT / "frames" / f"{traj}_{idx:03d}.bin", "wb") as f:
                f.write(struct.pack("<QQ", w, h))
                f.write(depth.astype("<f4").tobytes())
                f.write(gt[j].astype("<f8").tobytes())
            idx += 1
            n_frames += 1
            t += 2.0
        print(f"[gen] {traj}: {idx} frames")
    print(f"[gen] total frames={n_frames} -> {OUT}")


if __name__ == "__main__":
    main()
