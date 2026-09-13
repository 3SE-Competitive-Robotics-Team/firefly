"""RMUC 场地 mesh（`field_raw.npz`）→ `field.glb`：CAD 逐面颜色量化成 PBR 材质、
红蓝灯饰 emissive、焊接 + 减面。

headless 调用：
```sh
/Applications/Blender.app/Contents/MacOS/Blender --background --python scripts/blender_field.py -- \
  models/rmuc2026/derived/raw/field_raw.npz models/rmuc2026/field.glb 100000
```

坐标：npz 已是 `MuJoCo` 世界系（米，Z 上、地面 z=0）。Blender 内部同为 Z 上，
导出必须 `export_yup=False`，否则 glTF 规范的 Y 上转换会让场地相对 `apps/render`
的 Z 上 rig 立起来。

材质：CAD 的灰/白体面（低饱和）统一压进灰黑区间 [`BODY_LO`, `BODY_HI`]，避免
大片 off-white 在日光曝光下发白；饱和色（标线/红蓝绿灯饰）保留，红/蓝/绿按
`is_emissive` 设自发光。减面用 Decimate(Collapse)，保材质索引。
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
#: 调色板项数。
PALETTE_SIZE = 24
#: 焊接阈值（米；顶点本就共点，只并精确重复）。
WELD_DIST = 1e-5
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


def build_palette(colors: np.ndarray) -> np.ndarray:
    """体色 → 调色板（Top-N 频次 + 默认灰）。"""
    uniq, counts = np.unique(colors, axis=0, return_counts=True)
    order = np.argsort(-counts)
    chosen = [np.array(DEFAULT_RGB, dtype=np.float64)]
    for idx in order:
        if len(chosen) >= PALETTE_SIZE:
            break
        chosen.append(uniq[idx])
    return np.vstack(chosen)


def map_to_palette(colors: np.ndarray, palette: np.ndarray) -> np.ndarray:
    """逐面体色 → 最近调色板下标（默认灰为 0）。"""
    out = np.empty(len(colors), dtype=np.int32)
    block = 200_000
    for start in range(0, len(colors), block):
        chunk = colors[start : start + block]
        d = ((chunk[:, None, :] - palette[None, :, :]) ** 2).sum(axis=2)
        out[start : start + block] = np.argmin(d, axis=1)
    return out


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
    import bmesh
    import bpy

    argv = sys.argv[sys.argv.index("--") + 1 :]
    npz_path, out_path = Path(argv[0]), Path(argv[1])
    target = int(argv[2]) if len(argv) > 2 else 100_000

    data = np.load(npz_path)
    vertices = data["vertices"].astype(np.float64)
    faces = data["faces"].astype(np.int32)
    colors = data["face_colors"].astype(np.float64)
    print(f"[blender_field] in: {len(vertices)} verts / {len(faces)} tris", flush=True)

    body = remap_body(colors)
    palette = build_palette(body)
    face_palette = map_to_palette(body, palette)
    n_emissive = sum(1 for i in range(len(palette)) if is_emissive(tuple(palette[i])))
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
    mesh.update()

    add_materials(mesh, palette)
    mesh.polygons.foreach_set("material_index", face_palette)
    mesh.update()

    obj = bpy.data.objects.new("field", mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=WELD_DIST)
    bm.to_mesh(mesh)
    bm.free()
    print(f"[blender_field] welded: {len(mesh.vertices)} verts / {len(mesh.polygons)} tris", flush=True)

    if target > 0 and len(mesh.polygons) > target:
        mod = obj.modifiers.new("decimate", "DECIMATE")
        mod.decimate_type = "COLLAPSE"
        mod.ratio = target / len(mesh.polygons)
        bpy.ops.object.modifier_apply(modifier=mod.name)
    print(f"[blender_field] decimated: {len(mesh.polygons)} tris", flush=True)

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
