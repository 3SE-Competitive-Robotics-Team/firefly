"""碰撞聚合：三角网格 → 实心体素 → 贪心 AABB 盒集合。

`MuJoCo` 的 `type="mesh"` 几何只取凸包，场地凹结构（高地/隧道/外墙内侧）会被
填平，飞行时会撞到空气，故碰撞用盒集合：体素填充为实心（逐列取上下界之间的
段——"所有东西合成一个实心物块"），再贪心合并成 AABB。低质量、保留凹结构、
必然撞得上。

产物 `<scene>_collision.json`：``{"res": r, "boxes": [[cx,cy,cz,hx,hy,hz], ...]}``
（米，`MuJoCo` 系）；`firefly_mujoco.scene` 读它生成 `<geom type="box">`。

体素化与网格 IO 用 `trimesh`（本包依赖），盒合并用 `numpy`。
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

import numpy as np

log = logging.getLogger(__name__)


def merge_boxes(occ: np.ndarray) -> np.ndarray:
    """布尔体素占据 → 贪心 AABB 盒（体素下标 ``[x0,x1,y0,y1,z0,z1]``，闭区间）。

    贪心顺序：先沿 +x 长条，再把同 x 跨度的相邻 y 并入，最后并入相邻 z。
    不保证最少盒数，但保证覆盖且不跨空腔。
    """
    nx, ny, nz = occ.shape
    visited = np.zeros_like(occ, dtype=bool)
    boxes: list[tuple[int, int, int, int, int, int]] = []
    for x, y, z in np.argwhere(occ).tolist():
        if visited[x, y, z]:
            continue
        x1 = x
        while x1 + 1 < nx and occ[x1 + 1, y, z] and not visited[x1 + 1, y, z]:
            x1 += 1
        y1 = y
        while (
            y1 + 1 < ny
            and occ[x : x1 + 1, y1 + 1, z].all()
            and not visited[x : x1 + 1, y1 + 1, z].any()
        ):
            y1 += 1
        z1 = z
        while (
            z1 + 1 < nz
            and occ[x : x1 + 1, y : y1 + 1, z1 + 1].all()
            and not visited[x : x1 + 1, y : y1 + 1, z1 + 1].any()
        ):
            z1 += 1
        visited[x : x1 + 1, y : y1 + 1, z : z1 + 1] = True
        boxes.append((x, x1, y, y1, z, z1))
    if not boxes:
        return np.zeros((0, 6), dtype=np.int64)
    return np.asarray(boxes, dtype=np.int64)


def boxes_for_mesh(mesh, res: float) -> list[list[float]]:
    """三角网格 → 实心体素 → AABB 盒集合（米，``[cx,cy,cz,hx,hy,hz]``）。

    体素占据用 `trimesh` 的 ``voxelized(pitch).fill()``（逐列正交填充，即实心）。
    """
    import trimesh

    if not isinstance(mesh, trimesh.Trimesh):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    grid = mesh.voxelized(pitch=res).fill()
    occ = np.asarray(grid.matrix, dtype=bool)
    merged = merge_boxes(occ)
    if merged.size == 0:
        return []
    # 体素下标 → 世界坐标：用体素中心标定（grid.indices_to_points 给中心）。
    # `pitch` 在 trimesh 中按轴给出（标量入参下三轴相等）。
    pitch = np.asarray(grid.pitch, dtype=float).reshape(3)
    out: list[list[float]] = []
    for x0, x1, y0, y1, z0, z1 in merged.tolist():
        center = grid.indices_to_points(
            np.asarray([[(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]], dtype=float)
        )[0]
        half = np.array([(x1 - x0 + 1), (y1 - y0 + 1), (z1 - z0 + 1)], dtype=float) * pitch / 2
        out.append([round(v, 5) for v in (*center, *half)])
    return out


def collide(paths: list[Path], res: float):
    """读若干网格文件（``.obj``/``.glb``/``.npz``），返回碰撞描述 dict。"""
    import trimesh

    meshes = []
    for path in paths:
        if path.suffix == ".npz":
            data = np.load(path)
            meshes.append(
                trimesh.Trimesh(
                    vertices=np.asarray(data["vertices"], dtype=np.float64),
                    faces=np.asarray(data["faces"], dtype=np.int64),
                    process=False,
                )
            )
        else:
            loaded = trimesh.load(path, force="mesh")
            meshes.append(loaded)
    mesh = trimesh.util.concatenate(meshes)
    boxes = boxes_for_mesh(mesh, res)
    log.info("碰撞聚合：%d 个输入 → %d 个盒（res=%.3f）", len(paths), len(boxes), res)
    return {"res": res, "boxes": boxes}


def write_collision(report: dict, out: Path) -> None:
    """落盘碰撞 JSON（下游 `firefly_mujoco.scene` 机器契约）。"""
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    log.info("碰撞落盘: %s（%d 盒）", out, len(report["boxes"]))


def main(argv: list[str] | None = None) -> int:
    """CLI：``firefly-cad-collide --out D.json [--res R] MESH...``。"""
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    parser = argparse.ArgumentParser(description="网格 → 低质量聚合碰撞盒")
    parser.add_argument("mesh", type=Path, nargs="+", help="输入网格（.obj/.glb/.npz）")
    parser.add_argument("--out", type=Path, required=True, help="输出碰撞 JSON")
    parser.add_argument("--res", type=float, default=0.15, help="体素尺度（米，缺省 0.15）")
    args = parser.parse_args(argv)
    for path in args.mesh:
        if not path.is_file():
            parser.error(f"输入网格不存在: {path}")
    write_collision(collide(args.mesh, args.res), args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
