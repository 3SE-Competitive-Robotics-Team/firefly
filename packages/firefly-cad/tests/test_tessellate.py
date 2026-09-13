"""tessellate 合成往返：两零件 → 顶点/面/逐面颜色/归一化。

复用普查测试的合成 STEP fixture（平坦两零件 + 颜色）。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from firefly_cad.tessellate import tessellate
from test_census import _build_sample_stp


def test_tessellate_sample(tmp_path: Path) -> None:
    stp = tmp_path / "sample.stp"
    _build_sample_stp(stp)
    raw_dir = tmp_path / "raw"
    manifest = tessellate(stp, raw_dir)

    data = np.load(raw_dir / "field_raw.npz")
    vertices = data["vertices"]
    faces = data["faces"]
    colors = data["face_colors"]

    assert vertices.shape[1] == 3
    assert len(faces) > 0
    assert len(colors) == len(faces)
    assert manifest["n_triangles"] == len(faces)

    # 归一化：x/y 居中（包围盒关于 0 对称），z 底置 0。
    assert abs(float(vertices[:, 0].min() + vertices[:, 0].max())) < 1e-4
    assert abs(float(vertices[:, 1].min() + vertices[:, 1].max())) < 1e-4
    assert abs(float(vertices[:, 2].min())) < 1e-4

    # 逐面颜色保留：至少各有红/蓝面子集。
    assert ((colors[:, 0] > 0.5) & (colors[:, 1] < 0.1)).any()
    assert ((colors[:, 2] > 0.5) & (colors[:, 0] < 0.1)).any()
