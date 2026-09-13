"""STEP → 逐实例三角网格（米，归一化）+ 逐面 CAD 颜色。

读取：XCAF（`ColorMode` 开 / `NameMode` 关——名字全匿名，且 `NameMode` 会触发
逐子形状命名的平方级路径，见 `census`）。

遍历：自 free shapes 递归装配树（`GetComponents_s` / `GetReferredShape_s` /
`GetLocation_s`），每个实例取被引用产品的 shape 三角化并按累积位置摆放——
只有产品 label 的面能查到颜色，装配展开的匿名副本查不到，故必须按 label 递归，
不能直接对根 compound 取面。

三角化：`BRepMesh_IncrementalMesh`（线性偏差 2mm，全并行）。

颜色：逐面 `XCAFDoc_ColorSurf`→`ColorGen`→`ColorCurv`（sRGB 0~1）；无颜色记
`-1,-1,-1`（下游按默认材质处理）。

归一化：全部顶点（毫米，装配系）实测包围盒 → x/y 居中、z 底置 0、毫米转米。

输出 `derived/raw/field_raw.npz`：`vertices`(N,3 f32, 米) / `faces`(M,3 i32) /
`face_colors`(M,3 f32) / `part`(M, i32, 产品 tag)；`_manifest.json` 记包围盒、
变换参数与逐件统计。下游（分组/减面/碰撞）只认这一份契约。
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

import numpy as np

log = logging.getLogger(__name__)

#: 毫米转米（源 STEP 单位）。
MM_TO_M = 1e-3
#: 默认线性偏差（毫米）。
DEFAULT_DEFLECTION_MM = 2.0
#: 无逐面颜色时的占位（下游按默认材质处理）。
NO_COLOR = (-1.0, -1.0, -1.0)


def _trsf_to_matrix(trsf) -> np.ndarray:
    """`gp_Trsf` → 4×4 齐次矩阵（行主序，`p' = M @ p`）。"""
    m = np.eye(4)
    for r in range(1, 4):
        for c in range(1, 4):
            m[r - 1, c - 1] = trsf.Value(r, c)
    t = trsf.TranslationPart()
    m[0, 3], m[1, 3], m[2, 3] = t.X(), t.Y(), t.Z()
    return m


def _read_document(stp_path: Path):
    """XCAF 读取（快速模式），返回 `(shape_tool, color_tool)`。"""
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(False)
    reader.SetLayerMode(False)
    reader.SetPropsMode(False)
    reader.SetGDTMode(False)
    reader.SetSHUOMode(False)
    if reader.ReadFile(str(stp_path)) != IFSelect_RetDone:
        raise RuntimeError(f"STEP 读取失败: {stp_path}")
    doc = TDocStd_Document(TCollection_ExtendedString("tessellate"))
    if not reader.Transfer(doc):
        raise RuntimeError(f"STEP 装配 Transfer 失败: {stp_path}")
    return (
        XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()),
        XCAFDoc_DocumentTool.ColorTool_s(doc.Main()),
        doc,
    )


def _face_color(color_tool, face) -> np.ndarray:
    from OCP.Quantity import Quantity_Color
    from OCP.XCAFDoc import XCAFDoc_ColorCurv, XCAFDoc_ColorGen, XCAFDoc_ColorSurf

    for ctype in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen, XCAFDoc_ColorCurv):
        c = Quantity_Color()
        try:
            if color_tool.GetColor(face, ctype, c):
                return np.array([c.Red(), c.Green(), c.Blue()], dtype=np.float32)
        except Exception:  # noqa: BLE001 -- 单面取色失败按无颜色处理，不断整轮
            continue
    return np.array(NO_COLOR, dtype=np.float32)


def _append_shape_mesh(shape, color_tool, matrix, tag, acc) -> None:
    """把 `shape` 的面三角化（套 `matrix`）追加进累加器。"""
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    # 标签级整件色（`ColorGen`）作逐面色的兜底：合成件常只有 Gen 色。
    shape_color = _face_color(color_tool, shape)
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None and tri.NbTriangles() > 0:
            face_matrix = matrix @ _trsf_to_matrix(loc.Transformation())
            n = int(tri.NbNodes())
            nodes = np.empty((n, 3), dtype=np.float64)
            for i in range(1, n + 1):
                p = tri.Node(i)
                nodes[i - 1] = (p.X(), p.Y(), p.Z())
            world = np.empty((n, 3), dtype=np.float32)
            world[:] = (face_matrix[:3, :3] @ nodes.T).T + face_matrix[:3, 3]
            nt = int(tri.NbTriangles())
            tris = np.empty((nt, 3), dtype=np.int32)
            for i in range(1, nt + 1):
                n1, n2, n3 = tri.Triangle(i).Get()
                tris[i - 1] = (n1 - 1, n2 - 1, n3 - 1)
            color = _face_color(color_tool, face)
            if color[0] < 0.0:
                color = shape_color
            offset = acc["n_vertices"]
            acc["vertices"].append(world)
            acc["faces"].append(tris + offset)
            acc["face_colors"].append(np.tile(color, (nt, 1)))
            acc["part"].append(np.full(nt, tag, dtype=np.int32))
            acc["n_vertices"] += n
        ex.Next()


def _walk(shape_tool, color_tool, label, matrix, acc, depth=0) -> None:
    from OCP.TDF import TDF_Label
    from OCP.collections import Sequence_TDF_Label

    if shape_tool.IsAssembly_s(label):
        comps = Sequence_TDF_Label()
        shape_tool.GetComponents_s(label, comps)
        for i in range(1, comps.Length() + 1):
            comp = comps.Value(i)
            referred = TDF_Label()
            src = comp
            if shape_tool.GetReferredShape_s(comp, referred):
                src = referred
            loc = shape_tool.GetLocation_s(comp)
            _walk(shape_tool, color_tool, src, matrix @ _trsf_to_matrix(loc.Transformation()), acc, depth + 1)
    elif shape_tool.IsSimpleShape_s(label):
        shape = shape_tool.GetShape_s(label)
        if not shape.IsNull():
            _append_shape_mesh(shape, color_tool, matrix, int(label.Tag()), acc)


def tessellate(stp_path: Path, out_dir: Path, deflection_mm: float = DEFAULT_DEFLECTION_MM) -> dict:
    """三角化整个装配，落盘 `field_raw.npz` + `_manifest.json`。"""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.collections import Sequence_TDF_Label

    shape_tool, color_tool, _doc = _read_document(stp_path)
    acc: dict = {
        "vertices": [],
        "faces": [],
        "face_colors": [],
        "part": [],
        "n_vertices": 0,
    }
    free = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        shape = shape_tool.GetShape_s(free.Value(i))
        if not shape.IsNull():
            BRepMesh_IncrementalMesh(shape, deflection_mm, False, 0.5, True)
            _walk(shape_tool, color_tool, free.Value(i), np.eye(4), acc)

    vertices_mm = np.vstack(acc["vertices"])
    faces = np.vstack(acc["faces"]).astype(np.int32)
    face_colors = np.vstack(acc["face_colors"]).astype(np.float32)
    part = np.concatenate(acc["part"]).astype(np.int32)

    lo, hi = vertices_mm.min(axis=0), vertices_mm.max(axis=0)
    # 归一化：x/y 居中、z 底置 0、毫米转米。
    shift = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
    vertices = (vertices_mm - shift) * MM_TO_M

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out_dir / "field_raw.npz",
        vertices=vertices.astype(np.float32),
        faces=faces,
        face_colors=face_colors,
        part=part,
    )
    manifest = {
        "input": {"path": stp_path.name, "bytes": stp_path.stat().st_size},
        "deflection_mm": deflection_mm,
        "n_vertices": int(len(vertices)),
        "n_triangles": int(len(faces)),
        "bbox_mm": [round(float(v), 3) for v in (*lo, *hi)],
        "bbox_m": [round(float(v), 4) for v in (*vertices.min(axis=0), *vertices.max(axis=0))],
        "shift_mm": [round(float(v), 3) for v in shift],
        "parts": _part_stats(part, face_colors),
    }
    (out_dir / "_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    log.info(
        "三角化落盘: %s（顶点 %d / 三角 %d）",
        out_dir,
        len(vertices),
        len(faces),
    )
    return manifest


def _part_stats(part: np.ndarray, face_colors: np.ndarray) -> list[dict]:
    out = []
    for tag in np.unique(part):
        mask = part == tag
        colors = face_colors[mask]
        keys, counts = np.unique(colors, axis=0, return_counts=True)
        top = sorted(
            (
                {"rgb": [round(float(c), 4) for c in rgb], "faces": int(n)}
                for rgb, n in zip(keys, counts, strict=True)
            ),
            key=lambda d: d["faces"],
            reverse=True,
        )[:5]
        out.append({"tag": int(tag), "triangles": int(mask.sum()), "colors": top})
    out.sort(key=lambda d: d["triangles"], reverse=True)
    return out


def main(argv: list[str] | None = None) -> int:
    """CLI：`firefly-cad-tessellate <stp> [--out-dir D] [--deflection-mm X]`。"""
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    parser = argparse.ArgumentParser(description="STEP 装配三角化（逐实例 + 逐面颜色）")
    parser.add_argument("stp", type=Path, help="源 STEP 文件（只读）")
    parser.add_argument(
        "--out-dir", type=Path, default=None, help="输出目录（缺省 <stp 同目录>/derived/raw）"
    )
    parser.add_argument("--deflection-mm", type=float, default=DEFAULT_DEFLECTION_MM)
    args = parser.parse_args(argv)
    if not args.stp.is_file():
        parser.error(f"源文件不存在: {args.stp}")
    out_dir = args.out_dir or (args.stp.parent / "derived" / "raw")
    tessellate(args.stp, out_dir, args.deflection_mm)
    return 0


if __name__ == "__main__":
    sys.exit(main())
