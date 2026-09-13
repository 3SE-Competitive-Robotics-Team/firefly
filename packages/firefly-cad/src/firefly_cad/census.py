"""STEP 装配只读普查：产品清单、包围盒、颜色、面数估计。

输入源 STEP 一律只读（`ReadFile`，不写回）；输出 `products.json`
（下游机器契约）与 `SUMMARY.md`（人工确认清单）。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import sys
import time
from importlib.metadata import version as pkg_version
from pathlib import Path

log = logging.getLogger(__name__)

#: 毫米转米（源 STEP 单位，对照 strands：`SI_UNIT(.MILLI.,.METRE.)`）。
MM_TO_M = 1e-3


def sha256_of(path: Path) -> str:
    """流式 sha256（1.2G 源文件不一次读入内存）。"""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(8 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _count(shape, shape_type: int) -> int:
    from OCP.TopExp import TopExp_Explorer

    explorer, n = TopExp_Explorer(shape, shape_type), 0
    while explorer.More():
        n += 1
        explorer.Next()
    return n


def _label_name(label) -> str | None:
    from OCP.TDataStd import TDataStd_Name

    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        try:
            return str(attr.Get().ToExtString())
        except Exception:  # noqa: BLE001 -- 名字缺失只影响展示，不断整轮普查
            return None
    return None


def _shape_color(color_tool, shape):
    """按 Gen→Surf→Curv 顺序取色（STEP 往返会把 Gen 烘成 Surf，实测结论）。"""
    from OCP.Quantity import Quantity_Color
    from OCP.XCAFDoc import XCAFDoc_ColorCurv, XCAFDoc_ColorGen, XCAFDoc_ColorSurf

    for ctype, tag in (
        (XCAFDoc_ColorGen, "gen"),
        (XCAFDoc_ColorSurf, "surf"),
        (XCAFDoc_ColorCurv, "curv"),
    ):
        color = Quantity_Color()
        try:
            if color_tool.GetColor(shape, ctype, color):
                return tag, (
                    round(color.Red(), 4),
                    round(color.Green(), 4),
                    round(color.Blue(), 4),
                )
        except Exception:  # noqa: BLE001 -- 同上
            continue
    return None, None


def read_census(stp_path: Path, progress_every: int = 100) -> dict:
    """读 STEP 装配，返回普查报告 dict（见 `write_census` 的落盘格式）。"""
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
    from OCP.TopLoc import TopLoc_Location
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.collections import Sequence_TDF_Label
    from OCP.gp import gp_Pnt

    t0 = time.monotonic()
    reader_cls = __import__(
        "OCP.STEPCAFControl", fromlist=["STEPCAFControl_Reader"]
    ).STEPCAFControl_Reader
    reader = reader_cls()
    status = reader.ReadFile(str(stp_path))
    log.info("STEP 读取状态: %s", status)
    doc = TDocStd_Document(TCollection_ExtendedString("census"))
    if not reader.Transfer(doc):
        raise RuntimeError(f"STEP 装配 Transfer 失败: {stp_path}")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    shape_cls = type(shape_tool)

    seq = Sequence_TDF_Label()
    shape_tool.GetShapes(seq)
    labels = [seq.Value(i) for i in range(1, seq.Length() + 1)]

    free_seq = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(free_seq)
    free_tags = {free_seq.Value(i).Tag() for i in range(1, free_seq.Length() + 1)}

    #: 同原型共享未定位包围盒：tag→（角点，solid 数，face 数），实例只变换角点。
    cache: dict[int, tuple] = {}
    products = []
    for idx, label in enumerate(labels):
        tag = label.Tag()
        try:
            is_assembly = bool(shape_cls.IsAssembly_s(label))
            shape = shape_cls.GetShape_s(label)
            if shape.IsNull():
                log.warning("tag=%d 形状为空，跳过", tag)
                continue
            ref_tag = tag
            try:
                from OCP.TDF import TDF_Label

                referred = TDF_Label()
                if shape_cls.GetReferredShape_s(label, referred):
                    ref_tag = referred.Tag()
            except Exception:  # noqa: BLE001 -- 降级为按自身 tag 缓存，正确性不变
                pass
            if ref_tag not in cache:
                bare = shape.Located(TopLoc_Location())
                box = Bnd_Box()
                BRepBndLib.Add_s(bare, box, True)
                if box.IsVoid():
                    log.warning("tag=%d 包围盒为空，跳过", tag)
                    continue
                cache[ref_tag] = (
                    tuple(box.CornerMin().Coord()),
                    tuple(box.CornerMax().Coord()),
                    _count(bare, TopAbs_SOLID),
                    _count(bare, TopAbs_FACE),
                )
            (lo, hi, n_solids, n_faces) = cache[ref_tag]
            trsf = shape.Location().Transformation()
            corners = []
            for x in (lo[0], hi[0]):
                for y in (lo[1], hi[1]):
                    for z in (lo[2], hi[2]):
                        pt = gp_Pnt(x, y, z)
                        pt.Transform(trsf)
                        corners.append(pt.Coord())
            mins = [min(c[i] for c in corners) for i in range(3)]
            maxs = [max(c[i] for c in corners) for i in range(3)]
            color_kind, rgb = _shape_color(color_tool, shape)
            products.append(
                {
                    "tag": tag,
                    "name": _label_name(label) or f"tag_{tag}",
                    "is_assembly": is_assembly,
                    "is_top_level": tag in free_tags,
                    "n_components": int(shape_cls.NbComponents_s(label))
                    if is_assembly
                    else 0,
                    "n_solids": n_solids,
                    "n_faces": n_faces,
                    "bbox_mm": [round(v, 3) for v in (*mins, *maxs)],
                    "color_kind": color_kind,
                    "color_rgb": list(rgb) if rgb else None,
                }
            )
        except Exception:  # noqa: BLE001 -- 单件失败只记日志，不断整轮
            log.exception("tag=%d 处理失败，跳过", tag)
        if (idx + 1) % progress_every == 0:
            log.info("普查进度: %d/%d", idx + 1, len(labels))

    def volume(p):
        b = p["bbox_mm"]
        return (b[3] - b[0]) * (b[4] - b[1]) * (b[5] - b[2])

    products.sort(key=volume, reverse=True)
    top_boxes = [p["bbox_mm"] for p in products if p["is_top_level"]]
    overall = None
    if top_boxes:
        overall = [
            min(b[0] for b in top_boxes),
            min(b[1] for b in top_boxes),
            min(b[2] for b in top_boxes),
            max(b[3] for b in top_boxes),
            max(b[4] for b in top_boxes),
            max(b[5] for b in top_boxes),
        ]

    stp_path = Path(stp_path)
    return {
        "input": {
            "path": stp_path.name,
            "bytes": stp_path.stat().st_size,
            "sha256": sha256_of(stp_path),
        },
        "tool": {
            "ocp_wheel": pkg_version("cadquery-ocp-novtk"),
            "elapsed_s": round(time.monotonic() - t0, 1),
        },
        "summary": {
            "n_labels": len(labels),
            "n_products": sum(1 for p in products if not p["is_assembly"]),
            "n_assemblies": sum(1 for p in products if p["is_assembly"]),
            "n_solids": sum(p["n_solids"] for p in products if not p["is_assembly"]),
            "n_faces": sum(p["n_faces"] for p in products if not p["is_assembly"]),
            "bbox_mm": overall,
            "bbox_m": [round(v * MM_TO_M, 4) for v in overall] if overall else None,
        },
        "products": products,
    }


def write_census(report: dict, out_dir: Path, top_n: int = 60) -> None:
    """落盘 `products.json` + `SUMMARY.md`。"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "products.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8"
    )

    products = report["products"]
    colors: dict[str, int] = {}
    for p in products:
        if p["color_rgb"]:
            colors[str(p["color_rgb"])] = colors.get(str(p["color_rgb"]), 0) + 1
    top_colors = sorted(colors.items(), key=lambda kv: kv[1], reverse=True)[:20]

    lines = [
        "# 装配普查摘要（人工确认用）",
        "",
        f"- 输入：`{report['input']['path']}` "
        f"（{report['input']['bytes']} B，sha256 `{report['input']['sha256'][:16]}…`）",
        f"- 工具：cadquery-ocp-novtk {report['tool']['ocp_wheel']}，"
        f"耗时 {report['tool']['elapsed_s']}s",
        f"- 标签 {report['summary']['n_labels']} / 零件 "
        f"{report['summary']['n_products']} / 装配 {report['summary']['n_assemblies']}",
        f"- 实体 {report['summary']['n_solids']} / 面 {report['summary']['n_faces']}",
        f"- 总包围盒（米，装配系）：`{report['summary']['bbox_m']}`",
        "",
        "## 颜色直方图 Top20（`[r,g,b]`→件数，疑似灯饰从高亮小件里挑）",
        "",
    ]
    for rgb, n in top_colors:
        lines.append(f"- `{rgb}` → {n}")
    lines += ["", f"## 包围盒体积 Top{top_n}（候选功能件分组依据）", ""]
    lines.append("| 名称 | tag | 装配 | 实体 | 面 | 尺寸mm（dx×dy×dz） | 颜色 |")
    lines.append("|---|---|---|---|---|---|---|")
    for p in products[:top_n]:
        b = p["bbox_mm"]
        size = (
            f"{b[3] - b[0]:.0f}×{b[4] - b[1]:.0f}×{b[5] - b[2]:.0f}"
        )
        lines.append(
            f"| {p['name']} | {p['tag']} | {'Y' if p['is_assembly'] else ''} | "
            f"{p['n_solids']} | {p['n_faces']} | {size} | {p['color_rgb']} |"
        )
    (out_dir / "SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    log.info("普查落盘: %s", out_dir)


def main(argv: list[str] | None = None) -> int:
    """CLI：`firefly-cad-census <stp> [--out-dir D] [--top N]`。"""
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    parser = argparse.ArgumentParser(description="STEP 装配只读普查")
    parser.add_argument("stp", type=Path, help="源 STEP 文件（只读）")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="输出目录（缺省 <stp 同目录>/derived/census）",
    )
    parser.add_argument("--top", type=int, default=60, help="SUMMARY 表格行数")
    args = parser.parse_args(argv)
    if not args.stp.is_file():
        parser.error(f"源文件不存在: {args.stp}")
    out_dir = args.out_dir or (args.stp.parent / "derived" / "census")
    write_census(read_census(args.stp), out_dir, top_n=args.top)
    return 0


if __name__ == "__main__":
    sys.exit(main())
