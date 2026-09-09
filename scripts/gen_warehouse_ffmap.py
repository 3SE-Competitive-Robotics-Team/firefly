#!/usr/bin/env python3
"""生成仓库场景的 ffmap（gicp 在线靶图 + planner 静态先验同源）。

几何源 = firefly_mujoco/scene.py 的 _WAREHOUSE_COLLIDERS（6 透明碰撞盒）：
两侧货架墙（y=±6.5）/两端墙（x=0/46）/顶（z=5）/地面。外表面 0.1m 采样，
输出 apps/planner/maps/warehouse.ffmap（gitignored）。

与 gen_sim_ffmap.py 同惯例（boxes 时代残留，仅回归对照）。

用法：uv run python scripts/gen_warehouse_ffmap.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "packages" / "firefly-mujoco" / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from firefly_mujoco.scene import _WAREHOUSE_COLLIDERS  # noqa: E402
from gen_gicp_eval_data import sample_box  # noqa: E402

OUT = REPO_ROOT / "apps" / "planner" / "maps" / "warehouse.ffmap"


def main() -> None:
    pts = [
        sample_box(np.asarray(c, dtype=float), np.asarray(h, dtype=float), 0.1)
        for c, h in ((b[:3], b[3:]) for b in _WAREHOUSE_COLLIDERS)
    ]
    cloud = np.vstack(pts)
    # 裁到 DIMS 内（origin (-2,-9,0)，0.1m，dims [500,180,52] → x[-2,48] y[-9,9] z[0,5.2]）
    m = (
        (cloud[:, 0] >= -2)
        & (cloud[:, 0] < 48)
        & (cloud[:, 1] >= -9)
        & (cloud[:, 1] < 9)
        & (cloud[:, 2] >= 0)
        & (cloud[:, 2] < 5.2)
    )
    cloud = cloud[m]
    with open(OUT, "w") as f:
        f.write("FORMAT     firefly-map   1\n")
        f.write("RESOLUTION 0.100\n")
        f.write("ORIGIN     -2.000 -9.000 0.000\n")
        f.write("DIMS       500 180 52\n")
        f.write("OCCUPANCY\n")
        for p in cloud:
            f.write(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
    print(f"[ffmap] {len(cloud)} points -> {OUT}")


if __name__ == "__main__":
    main()
