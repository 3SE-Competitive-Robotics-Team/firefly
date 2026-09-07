#!/usr/bin/env python3
"""视觉库图采集：沿轨迹摆位渲染（左目灰度 + 深度 + 真值位姿）。

离线摆拍模式（对照 scripts/gen_gicp_eval_data.py）：不飞 live，
按轨迹参考位置、水平姿态逐帧 reset → mj_forward → 渲染。
输出 logs/visamp/<traj>/frame_III.npz（left uint8, depth f32, pos f64[3]）。

用法：uv run python scripts/collect_vision_frames.py [--traj straight_forward]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "packages" / "firefly-mujoco" / "src"))
sys.path.insert(0, str(REPO_ROOT / "apps" / "firefly-sim" / "src"))

from firefly_mujoco import DroneEnv  # noqa: E402
from firefly_sim.trajectories import TRAJECTORIES  # noqa: E402

OUT = REPO_ROOT / "logs" / "visamp"
SAMPLE_DT = 2.0
DURATION = 34.0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", default="straight_forward")
    args = ap.parse_args()
    traj = TRAJECTORIES[args.traj]
    out = OUT / args.traj
    out.mkdir(parents=True, exist_ok=True)

    env = DroneEnv(depth_noise=0.0)
    level = np.array([0.0, 0.0, 0.0, 1.0])
    n = 0
    t = 0.0
    while t <= DURATION:
        pos, _ = traj.ref(t)
        env.reset(np.asarray(pos, dtype=float), level)
        import mujoco

        mujoco.mj_forward(env.model, env.data)
        left = env.render_left()
        # 左目视角深度（与左目图像严格配准，零配准误差；建库用零噪声真值）
        env._renderer.update_scene(env.data, camera="cam_left")
        env._renderer.enable_depth_rendering()
        depth = env._renderer.render().copy().astype(np.float32)
        env._renderer.disable_depth_rendering()
        np.savez_compressed(
            out / f"frame_{n:03d}.npz",
            left=left,
            depth=depth,
            pos=np.asarray(pos, dtype=np.float64),
            t=np.float64(t),
        )
        n += 1
        t += SAMPLE_DT
    print(f"[collect] {args.traj}: {n} frames -> {out}")


if __name__ == "__main__":
    main()
