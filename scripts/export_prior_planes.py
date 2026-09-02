#!/usr/bin/env python3
"""从 MuJoCo scene.py 已知几何导出静态先验平面（P11.2 先验图 P0 主路径）。

先验语义（对照 `~/Projects/liop_prior/Lidar_IMU_Localization/`）：LIOP 的
先验地图是 LIO-SAM 离线建的全局点云 kdtree；本脚本从仿真场景的**解析几何**
（最精确口径）导出等价先验：每个 box geom 的可见外露面 = 一个平面。

坐标：scene.py 的 geom 都在 worldbody（MuJoCo 世界系）；VOID 状态 pos/rot
与深度点 `p_w = R·p_l + p` 同在世界系 → 平面坐标直接可用，无需外参对齐。

输出格式（`PriorPlaneMap::parse_text` 兼容）：
`cx cy cz nx ny nz d var_scale radius npts`
- var_scale：Σ_nq ≈ var_scale·I₆（各向同性近似）；解析几何给 1e-6（m²，
  法向/中心不确定度远小于在线拟合——但 configs/void.toml `var_scale` 可放大）
- radius：面内接半径 = 面半高宽的对角一半（保守取 √(h²+w²)/2，覆盖整个面）
- 分组去重：5×5 箱体阶梯中，被上层遮挡的下层顶面/被相邻箱遮挡的侧面会
  被深度相机看到吗？——相机在箱区 x≥5 之外（轨迹 x≤4），只能看到朝 −x 面
  与顶面（下倾 20° 看得到部分顶面）。但仍导出全部外露面：匹配有径向判据
  + 卡方门控，被遮挡的背面点本来就不会投影到它（点在真实表面，匹配到
  几何上最近的可视面），冗余面不产生错误约束。

用法：uv run python scripts/export_prior_planes.py [--out configs/prior/void_scene.planes]
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "packages" / "firefly-mujoco" / "src"))

from firefly_mujoco.scene import SCENE_XML  # noqa: E402


def parse_box_geoms(xml: str) -> list[tuple[float, float, float, float, float, float]]:
    """解析 `<geom type="box" pos="x y z" size="hx hy hz"/>`，返回 (center, half)。"""
    out = []
    for m in re.finditer(
        r'<geom type="box"\s+pos="([-\d.]+) ([-\d.]+) ([-\d.]+)"\s+size="([-\d.]+) ([-\d.]+) ([-\d.]+)"',
        xml,
    ):
        center = np.array([float(m.group(i)) for i in range(1, 4)])
        half = np.array([float(m.group(i)) for i in range(4, 7)])
        out.append((center, half))
    return out


def box_faces(center: np.ndarray, half: np.ndarray) -> list[dict]:
    """box 六个外露面（世界系）。每面: center(面心), normal(朝外), radius。"""
    faces = []
    # 六面：±x, ±y, ±z；面心 = center ± 法向·半宽，面内尺寸取另两半宽
    for axis in range(3):
        for sign in (+1.0, -1.0):
            normal = np.zeros(3)
            normal[axis] = sign
            face_center = center + normal * half[axis]
            # 面内两轴的半宽 → 面内接圆半径（保守：半对角）
            other = [i for i in range(3) if i != axis]
            radius = float(np.hypot(half[other[0]], half[other[1]]))
            faces.append(
                {
                    "center": face_center,
                    "normal": normal,
                    "radius": radius,
                }
            )
    return faces


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--out",
        type=Path,
        default=REPO_ROOT / "configs" / "prior" / "void_scene.planes",
        help="输出平面文件（PriorPlaneMap 文本格式）",
    )
    args = ap.parse_args()

    planes = []
    # 地面：plane geom（世界 z=0 无限大；给保守大半径覆盖轨迹全程）
    planes.append(
        {
            "center": np.array([0.0, 0.0, 0.0]),
            "normal": np.array([0.0, 0.0, 1.0]),
            "radius": 20.0,
        }
    )
    # box geom（中线立柱 + 侧柱 + 前方箱子）
    for center, half in parse_box_geoms(SCENE_XML):
        planes.extend(box_faces(center, half))

    var_scale = 1e-6  # 解析几何：法向/中心不确定度 ~1e-6 m²
    npts = 200  # 合成点数的量级标记（拟合等效点数；无算法语义）

    lines = ["# 已知几何先验平面（scripts/export_prior_planes.py 导出；"
             "cx cy cz nx ny nz d var_scale radius npts）"]
    for p in planes:
        c = p["center"]
        n = p["normal"] / np.linalg.norm(p["normal"])
        d = -float(n @ c)
        lines.append(
            f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f} "
            f"{n[0]:.6f} {n[1]:.6f} {n[2]:.6f} {d:.6f} "
            f"{var_scale:.1e} {p['radius']:.4f} {npts}"
        )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[prior-export] {len(planes)} 面 -> {args.out}")
    # 摘要：各朝向面数
    import collections

    counts = collections.Counter(
        tuple(np.round(p["normal"] / np.linalg.norm(p["normal"]), 3))
        for p in planes
    )
    for n, cnt in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f"  法向 {n}: {cnt} 面")


if __name__ == "__main__":
    main()
