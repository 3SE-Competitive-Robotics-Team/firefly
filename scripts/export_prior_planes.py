#!/usr/bin/env python3
"""从 MuJoCo scene.py 已知几何导出静态先验平面（P11.2 先验图 P0 主路径）。

先验语义（对照 `~/Projects/liop_prior/Lidar_IMU_Localization/`）：LIOP 的
先验地图是 LIO-SAM 离线建的全局点云 kdtree；本脚本从仿真场景的**解析几何**
（最精确口径）导出等价先验：每个 box geom 的可见外露面 = 一个平面。

法向朝向约定（review H-3）：**法向必须朝外/朝相机侧**。点面残差带符号
`dis = n·(p_w − q)`，法向反了会镜像残差符号——对固定先验面，符号错会导致
ESIKF 沿错误方向拉状态。导出时统一保证法向朝外（box 面朝箱外），剔除背面。

坐标：scene.py 的 geom 都在 worldbody（MuJoCo 世界系）；VOID 状态 pos/rot
与深度点 `p_w = R·p_l + p` 同在世界系 → 平面坐标直接可用，无需外参对齐。

可见性粗筛（双向飞行：去程面朝 +x、返程面朝 -x，下倾 20°）：
- **-z 底面**恒剔除（深度相机看不到任何底面）；
- **场外面**（面心 x∉[0,34]、y∉[0,20]、z<0，如围墙外侧面）剔除——无人机
  不出场，那些面永远不可见，大漂移时误关联会把状态往错误方向拉；
- 保留：地面、场内 ±x/±y 侧面、+z 顶面（下倾 20° 能看到部分顶面）。
  顶面法向朝上不违反"朝相机侧"约定——匹配由点到面几何距离驱动。

输出格式（`PriorPlaneMap::parse_text` 兼容）：
`cx cy cz nx ny nz d var_scale radius npts`
- var_scale：Σ_nq ≈ var_scale·I₆（各向同性近似）；解析几何给 1e-6（m²，
  法向/中心不确定度远小于在线拟合——但 configs/void.toml `var_scale` 可放大）
- radius：面内接半径 = 面半高宽的对角一半（保守取 √(h²+w²)/2，覆盖整个面）

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
    """解析 `<geom type="box" pos="x y z" size="hx hy hz"/>`，返回 (center, half)。

    跳过无人机体小盒（三轴半宽全 < 0.2m，随动非静态，不能做先验）。
    """
    out = []
    for m in re.finditer(
        r'<geom type="box"\s+pos="([-\d.]+) ([-\d.]+) ([-\d.]+)"\s+size="([-\d.]+) ([-\d.]+) ([-\d.]+)"',
        xml,
    ):
        center = np.array([float(m.group(i)) for i in range(1, 4)])
        half = np.array([float(m.group(i)) for i in range(4, 7)])
        if bool(np.all(half < 0.2)):
            continue
        out.append((center, half))
    return out


def box_faces(center: np.ndarray, half: np.ndarray) -> list[dict]:
    """box 六个外露面（世界系）。每面: center(面心), normal(朝外), radius。

    返回前做可见性粗筛（见模块 docstring）：-z 底面恒剔除；面心落在
    场外（x∉[0,34]、y∉[0,20]、z<0，如围墙外侧面）的面剔除——无人机不
    出场，那些面永远不可见，大漂移时误关联会把状态往错误方向拉。
    双向飞行（去程面朝 +x、返程面朝 -x），场内 ±x 面全部保留。
    """
    faces = []
    # 六面：±x, ±y, ±z；面心 = center ± 法向·半宽，面内尺寸取另两半宽
    for axis in range(3):
        for sign in (+1.0, -1.0):
            normal = np.zeros(3)
            normal[axis] = sign
            face_center = center + normal * half[axis]
            if normal[2] < 0.0:
                continue  # 底面
            x, y, z = (float(face_center[i]) for i in range(3))
            if not (0.0 <= x <= 34.0 and 0.0 <= y <= 20.0 and z >= 0.0):
                continue  # 场外面
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
    # 地面：plane geom（世界 z=0 无限大；面心取场地中心，半径覆盖全场对角）
    planes.append(
        {
            "center": np.array([17.0, 10.0, 0.0]),
            "normal": np.array([0.0, 0.0, 1.0]),
            "radius": 25.0,
        }
    )
    # box geom（中线立柱 + 侧柱 + 前方箱子）；box_faces 已做背面粗筛
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
