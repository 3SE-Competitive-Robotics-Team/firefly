"""用两个带颜色和位移的平坦 STEP 零件验证普查结果与 CLI 产物。"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from firefly_cad.census import main, read_census


def _build_sample_stp(path: Path) -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.TDataStd import TDataStd_Name
    from OCP.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString("sample"))
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    base = shape_tool.AddShape(
        BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 10, 20, 30).Shape(), False
    )
    TDataStd_Name.Set_s(base, TCollection_ExtendedString("BasePlate"))
    color_tool.SetColor(
        base, Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB), XCAFDoc_ColorGen
    )

    # 两个独立零件用放置点表达相对位置。
    pillar = shape_tool.AddShape(
        BRepPrimAPI_MakeBox(gp_Pnt(100, 0, 0), 5, 5, 50).Shape(), False
    )
    TDataStd_Name.Set_s(pillar, TCollection_ExtendedString("Pillar"))
    color_tool.SetColor(
        pillar, Quantity_Color(0.0, 0.0, 1.0, Quantity_TOC_RGB), XCAFDoc_ColorGen
    )

    writer = STEPCAFControl_Writer()
    assert writer.Transfer(doc)
    assert str(writer.Write(str(path))).endswith("RetDone")


def test_census_roundtrip(tmp_path: Path) -> None:
    stp = tmp_path / "sample.stp"
    _build_sample_stp(stp)
    report = read_census(stp)

    assert report["summary"]["n_products"] == 2
    assert report["summary"]["n_assemblies"] == 0
    products = report["products"]

    base = next(p for p in products if p["bbox_mm"][3] == 10.0)
    assert base["bbox_mm"] == [0.0, 0.0, 0.0, 10.0, 20.0, 30.0]
    assert base["color_rgb"] == [1.0, 0.0, 0.0]
    assert base["n_solids"] == 1 and base["n_faces"] == 6

    pillar = next(p for p in products if p["bbox_mm"][0] == 100.0)
    assert pillar["color_rgb"] == [0.0, 0.0, 1.0]

    assert report["summary"]["bbox_m"] == [0., 0., 0., .105, .02, .05]
    assert report["input"]["sha256"] == hashlib.sha256(stp.read_bytes()).hexdigest()


def test_census_cli_writes_outputs(tmp_path: Path) -> None:
    stp = tmp_path / "sample.stp"
    _build_sample_stp(stp)
    out = tmp_path / "out"
    assert main([str(stp), "--out-dir", str(out), "--top", "5"]) == 0
    report = json.loads((out / "products.json").read_text())
    assert report["summary"]["n_products"] == 2
    assert report["summary"]["n_solids"] == 2
    summary = (out / "SUMMARY.md").read_text(encoding="utf-8")
    rows = [line.split("|") for line in summary.splitlines() if line.startswith("| tag_")]
    assert {row[1].strip() for row in rows} == {p["name"] for p in report["products"]}
    assert {row[6].strip() for row in rows} == {"10×20×30", "5×5×50"}
