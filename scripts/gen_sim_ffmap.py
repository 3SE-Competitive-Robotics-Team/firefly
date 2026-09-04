#!/usr/bin/env python3
"""生成 MuJoCo 默认场景的 ffmap（gicp 在线靶图用）。

与 export_prior_planes.py 同几何源（SCENE_XML 解析）：围墙 + 南北箱带 +
中线矮箱 + 地面，外表面 0.1m 采样。输出 apps/planner/maps/sim_scene.ffmap
（gitignored，见 .gitignore）。

用法：uv run python scripts/gen_sim_ffmap.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "packages" / "firefly-mujoco" / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from firefly_mujoco.scene import SCENE_XML  # noqa: E402
from gen_gicp_eval_data import parse_boxes, sample_box  # noqa: E402

OUT = REPO_ROOT / "apps" / "planner" / "maps" / "sim_scene.ffmap"


def main() -> None:
    boxes = parse_boxes(SCENE_XML)
    pts = [sample_box(c, h, 0.1) for c, h in boxes]
    gx, gy = np.meshgrid(np.arange(0.0, 34.01, 0.2), np.arange(0.0, 20.01, 0.2))
    pts.append(np.stack([gx.ravel(), gy.ravel(), np.zeros_like(gx.ravel())], axis=1))
    cloud = np.vstack(pts)
    # 裁到 DIMS 内（origin (0,-5,0)，0.1m，dims [360,260,52] → x[0,36] y[-5,21] z[0,5.2]）
    m = (
        (cloud[:, 0] >= 0)
        & (cloud[:, 0] < 36)
        & (cloud[:, 1] >= -5)
        & (cloud[:, 1] < 21)
        & (cloud[:, 2] >= 0)
        & (cloud[:, 2] < 5.2)
    )
    cloud = cloud[m]
    with open(OUT, "w") as f:
        f.write("FORMAT     firefly-map   1\n")
        f.write("RESOLUTION 0.100\n")
        f.write("ORIGIN     0.000 -5.000 0.000\n")
        f.write("DIMS       360 260 52\n")
        f.write("OCCUPANCY\n")
        for p in cloud:
            f.write(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
    print(f"[ffmap] {len(cloud)} points -> {OUT}")


if __name__ == "__main__":
    main()
