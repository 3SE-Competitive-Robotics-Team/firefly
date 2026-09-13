#!/usr/bin/env python3
"""下载并归一化微无人机模型（Poly Pizza，CC-BY-4.0）。

产出 `models/drone/drone.glb`：居中、按水平尺寸缩放；保持 glTF Y-up、机头 +Z
（`apps/quad` 挂载时转到世界 Z-up、机头 +X）。模型不入 git（`models/` 已 ignore），
本脚本即复现依据。

来源：Poly Pizza "Drone" by NateGazzard（https://poly.pizza/m/DNbUoMtG3H），
CC-BY-4.0，使用需保留署名。

用法：uv run python scripts/fetch_drone_model.py
"""

from __future__ import annotations

from pathlib import Path
from urllib.request import urlretrieve

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent.parent
RAW = ROOT / "models" / "drone" / "drone_raw.glb"
OUT = ROOT / "models" / "drone" / "drone.glb"
URL = "https://static.poly.pizza/4eb88feb-cb2d-46c3-980d-3e704868361c.glb"
#: 归一化后的水平最大尺寸（米，250g 级 2.5~3 寸量级）。
TARGET_SPAN = 0.16


def main() -> None:
    """下载（若缺）+ 居中缩放导出 `drone.glb`。"""
    RAW.parent.mkdir(parents=True, exist_ok=True)
    if not RAW.is_file():
        print(f"下载 {URL}")
        urlretrieve(URL, RAW)
    scene = trimesh.load(RAW, force="scene")
    mesh = scene.to_geometry()
    mesh.apply_translation(-mesh.bounds.mean(axis=0))
    # glTF 水平面是 x/z（Y 为竖直）。
    span = float(max(mesh.extents[0], mesh.extents[2]))
    mesh.apply_scale(TARGET_SPAN / span)
    mesh.export(OUT)
    print(
        f"写出 {OUT}（{mesh.faces.shape[0]} 三角，尺寸 "
        f"{np.round(mesh.extents, 4).tolist()} m）"
    )
    print("来源：Poly Pizza 'Drone' by NateGazzard（CC-BY-4.0）")


if __name__ == "__main__":
    main()
