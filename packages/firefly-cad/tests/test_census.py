"""census 合成往返测试（不依赖 1.2G 真文件）。

约定：fixture 用 OCP 现场造小装配（两零件 + 一装配，含名字/颜色/位移），
census 读回断言。OCP 8 绑定形态以本文件为准（`_s` 后缀静态方法等）。
"""

from __future__ import annotations

from pathlib import Path

from firefly_cad.census import main, read_census


def _build_sample_stp(path: Path) -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt, gp_Trsf, gp_Vec
    from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.TDataStd import TDataStd_Name
    from OCP.TopLoc import TopLoc_Location
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

    pillar = shape_tool.AddShape(
        BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 5, 5, 50).Shape(), False
    )
    TDataStd_Name.Set_s(pillar, TCollection_ExtendedString("Pillar"))
    color_tool.SetColor(
        pillar, Quantity_Color(0.0, 0.0, 1.0, Quantity_TOC_RGB), XCAFDoc_ColorGen
    )

    asm = shape_tool.NewShape()
    TDataStd_Name.Set_s(asm, TCollection_ExtendedString("FieldAsm"))
    trsf = gp_Trsf()
    trsf.SetTranslationPart(gp_Vec(100, 0, 0))
    shape_tool.AddComponent(asm, pillar, TopLoc_Location(trsf))

    writer = STEPCAFControl_Writer()
    assert writer.Transfer(doc)
    assert str(writer.Write(str(path))).endswith("RetDone")


def test_census_roundtrip(tmp_path: Path) -> None:
    stp = tmp_path / "sample.stp"
    _build_sample_stp(stp)
    report = read_census(stp)

    assert report["summary"]["n_products"] == 2
    assert report["summary"]["n_assemblies"] == 1
    by_name = {p["name"]: p for p in report["products"]}

    base = by_name["BasePlate"]
    assert base["bbox_mm"] == [0.0, 0.0, 0.0, 10.0, 20.0, 30.0]
    assert base["color_rgb"] == [1.0, 0.0, 0.0]
    assert base["n_solids"] == 1 and base["n_faces"] == 6

    pillar = by_name["Pillar"]
    assert pillar["bbox_mm"][0] == 0.0  # 原型位（实例位移挂在装配下）
    assert pillar["color_rgb"] == [0.0, 0.0, 1.0]

    asm = by_name["FieldAsm"]
    assert asm["is_assembly"] and asm["n_components"] == 1
    # 装配包围盒应包含位移后的实例（x≥100）
    assert asm["bbox_mm"][3] >= 100.0

    assert report["summary"]["bbox_m"][3] >= 0.1
    assert len(report["input"]["sha256"]) == 64


def test_census_cli_writes_outputs(tmp_path: Path) -> None:
    stp = tmp_path / "sample.stp"
    _build_sample_stp(stp)
    out = tmp_path / "out"
    assert main([str(stp), "--out-dir", str(out), "--top", "5"]) == 0
    assert (out / "products.json").is_file()
    summary = (out / "SUMMARY.md").read_text(encoding="utf-8")
    assert "BasePlate" in summary and "FieldAsm" in summary
