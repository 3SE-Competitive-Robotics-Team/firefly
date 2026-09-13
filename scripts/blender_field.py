"""RMUC 场地 mesh（`field_raw.npz`）→ `field.glb`：逐面 CAD 颜色 → PBR 材质。

headless 调用：
```sh
/Applications/Blender.app/Contents/MacOS/Blender --background --python scripts/blender_field.py -- \
  models/rmuc2026/derived/raw/field_raw.npz models/rmuc2026/field.glb
```

几何：**不做减面、不做焊接**。CAD 逐 solid 三角化时顶点只在单个 BRep 面内共享
——弯曲面（圆柱/倒角）靠共享顶点平滑着色，面与面之间不共顶点而保留锐边，壳与壳
不粘连。建面前剔除零面积退化三角（渲染不产生像素，只会污染法线），其余拓扑原样
进入 glb。

材质：CAD 逐面色（无颜色记默认灰）；低饱和（灰/白）面压进灰黑区间 [`BODY_LO`,
`BODY_HI`] 避免日光曝光泛白，饱和色（标线/红蓝绿灯饰）原样保留并按 [`is_emissive`]
标自发光。调色板取全部去重色，不做量化丢色。

坐标：npz 已是 `MuJoCo` 世界系（米，Z 上、地面 z=0）。Blender 内部同为 Z 上，
导出必须 `export_yup=False`，否则 glTF 规范的 Y 上转换会让场地相对 `apps/render`
的 Z 上 rig 立起来。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

#: 低饱和面（灰/白体）压到的灰黑区间（线性 sRGB 0~1）。
BODY_LO = 0.05
BODY_HI = 0.16
#: 饱和度阈值（max-min 小于此值按灰/白体处理）。
SAT_THRESHOLD = 0.15
#: 未着色面（CAD 无 Surf 色）的体色。
DEFAULT_RGB = (0.12, 0.12, 0.12)
#: 退化三角面积下限（m²）：低于此值不产生像素，建面前剔除。
MIN_AREA = 1e-12
#: 材质粗糙度（低饱和体面带一点光泽，暗面也有高光层次）。
ROUGHNESS = 0.5
#: emissive 强度（配合 Bevy bloom）。
EMISSION_STRENGTH = 8.0


def is_emissive(rgb: tuple[float, float, float]) -> bool:
    """高饱和且以单通道为主（红/蓝/绿灯饰）。"""
    r, g, b = (float(c) for c in rgb)
    if max(r, g, b) < 0.4:
        return False
    return r > 1.5 * max(g, b) or b > 1.5 * max(r, g) or g > 1.5 * max(r, b)


def remap_body(colors: np.ndarray) -> np.ndarray:
    """逐面颜色 → 体色：低饱和（灰/白）压进灰黑区间，饱和色原样保留。

    必须在建调色板之前做——否则大片 off-white 会落到最近的饱和色项上。
    """
    out = colors.astype(np.float64, copy=True)
    uncolored = out[:, 0] < 0.0
    sat = out.max(axis=1) - out.min(axis=1)
    lum = out.mean(axis=1)
    low = sat < SAT_THRESHOLD
    v = np.clip(lum, BODY_LO, BODY_HI)
    out[low] = v[low, None]
    out[uncolored] = DEFAULT_RGB
    return out


def build_palette(colors: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """体色 → `(调色板, 逐面下标)`：去重色按面数降序，不量化。

    `np.unique` 的精确索引避免最近邻距离搜索（同一体色必落到同一材质）。
    """
    uniq, inverse, counts = np.unique(
        colors, axis=0, return_inverse=True, return_counts=True
    )
    order = np.argsort(-counts)
    rank = np.empty(len(uniq), dtype=np.int32)
    rank[order] = np.arange(len(uniq), dtype=np.int32)
    return uniq[order], rank[inverse]


def add_materials(mesh, palette: np.ndarray) -> None:
    import bpy

    for i, rgb in enumerate(palette):
        color = tuple(float(c) for c in rgb)
        mat = bpy.data.materials.new(name=f"cad_{i:02d}")
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        bsdf.inputs["Base Color"].default_value = (*color, 1.0)
        bsdf.inputs["Roughness"].default_value = ROUGHNESS
        bsdf.inputs["Metallic"].default_value = 0.0
        if is_emissive(color):
            bsdf.inputs["Emission Color"].default_value = (*color, 1.0)
            bsdf.inputs["Emission Strength"].default_value = EMISSION_STRENGTH
        mesh.materials.append(mat)


def main() -> None:
    import bpy

    argv = sys.argv[sys.argv.index("--") + 1 :]
    npz_path, out_path = Path(argv[0]), Path(argv[1])

    data = np.load(npz_path)
    vertices = data["vertices"].astype(np.float64)
    faces = data["faces"].astype(np.int32)
    face_colors = data["face_colors"].astype(np.float64)
    n_in = len(faces)

    # 剔除退化三角：重复顶点或零面积（渲染无像素，且污染顶点法线）。
    tri = vertices[faces]
    area = 0.5 * np.linalg.norm(
        np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1
    )
    degenerate = (
        (faces[:, 0] == faces[:, 1])
        | (faces[:, 1] == faces[:, 2])
        | (faces[:, 0] == faces[:, 2])
        | (area < MIN_AREA)
    )
    faces = faces[~degenerate]
    face_colors = face_colors[~degenerate]
    print(
        f"[blender_field] in: {len(vertices)} verts / {n_in} tris"
        f"（退化剔除 {int(degenerate.sum())}）",
        flush=True,
    )

    body = remap_body(face_colors)
    palette, face_palette = build_palette(body)
    n_emissive = sum(1 for rgb in palette if is_emissive(tuple(rgb)))
    print(f"[blender_field] palette {len(palette)}（emissive {n_emissive}）", flush=True)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    mesh = bpy.data.meshes.new("field")
    mesh.vertices.add(len(vertices))
    mesh.vertices.foreach_set("co", vertices.ravel())
    n_loops = len(faces) * 3
    mesh.loops.add(n_loops)
    mesh.loops.foreach_set("vertex_index", faces.ravel())
    mesh.polygons.add(len(faces))
    mesh.polygons.foreach_set("loop_start", np.arange(0, n_loops, 3, dtype=np.int32))
    if "loop_total" in mesh.polygons[0].bl_rna.properties:
        mesh.polygons.foreach_set("loop_total", np.full(len(faces), 3, dtype=np.int32))
    # 逐面平滑：弯曲面的顶点在 BRep 面内共享 → 平滑；面间不共享 → 锐边自然保留。
    mesh.polygons.foreach_set("use_smooth", np.ones(len(faces), dtype=bool))
    mesh.update()

    if mesh.validate(verbose=False):
        raise RuntimeError("mesh.validate 修正了非法几何——上游三角化不合约，请查")

    add_materials(mesh, palette)
    mesh.polygons.foreach_set("material_index", face_palette)
    mesh.update()

    obj = bpy.data.objects.new("field", mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(out_path),
        export_format="GLB",
        export_yup=False,
        export_apply=True,
    )
    print(f"[blender_field] wrote {out_path} ({out_path.stat().st_size} B)", flush=True)


if __name__ == "__main__":
    main()
