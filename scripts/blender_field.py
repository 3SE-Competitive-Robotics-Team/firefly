"""RMUC 场地 mesh（`field_raw.npz`）→ `field.glb`：CAD 逐面颜色量化成 PBR 材质、
红蓝灯饰 emissive、焊接 + 减面。

headless 调用：
```sh
/Applications/Blender.app/Contents/MacOS/Blender --background --python scripts/blender_field.py -- \
  models/rmuc2026/derived/raw/field_raw.npz models/rmuc2026/field.glb 180000
```

坐标：npz 已是 `MuJoCo` 世界系（米，Z 上、地面 z=0）。Blender 内部同为 Z 上，
导出必须 `export_yup=False`，否则 glTF 规范的 Y 上转换会让场地相对 `apps/render`
的 Z 上 rig 立起来。

材质：取面颜色直方图 Top-N 作调色板（含默认灰），逐面映射到最近调色板项；
高饱和红/蓝调色板项设 emissive（场地的红蓝方灯饰）。减面用 Decimate(Collapse)，
保材质索引。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

#: 默认未着色面（CAD 无 Surf 色）→ 混凝土灰。
DEFAULT_RGB = (0.30, 0.30, 0.30)
#: 调色板项数（含默认灰）。
PALETTE_SIZE = 24
#: 焊接阈值（米；顶点本就共点，只并精确重复）。
WELD_DIST = 1e-5
#: emissive 强度（配合 Bevy bloom）。
EMISSION_STRENGTH = 8.0


def is_emissive(rgb: tuple[float, float, float]) -> bool:
    r, g, b = rgb
    if max(rgb) < 0.4:
        return False
    return r > 1.5 * max(g, b) or b > 1.5 * max(r, g)


def build_palette(colors: np.ndarray) -> np.ndarray:
    """面颜色 → 调色板（Top-N 频次 + 默认灰）。"""
    uniq, counts = np.unique(colors, axis=0, return_counts=True)
    order = np.argsort(-counts)
    chosen = []
    default = np.array(DEFAULT_RGB, dtype=np.float64)
    for idx in order:
        rgb = uniq[idx]
        if rgb[0] < 0.0:  # -1 占位（无颜色）
            continue
        chosen.append(rgb)
        if len(chosen) >= PALETTE_SIZE - 1:
            break
    return np.vstack([default, np.asarray(chosen, dtype=np.float64)])


def map_to_palette(colors: np.ndarray, palette: np.ndarray) -> np.ndarray:
    """逐面颜色 → 最近调色板下标（默认灰为 0）。"""
    clean = np.where(colors < 0.0, np.array(DEFAULT_RGB), colors)
    # 距离矩阵按块算，避免 (M, P, 3) 峰值内存。
    out = np.empty(len(clean), dtype=np.int32)
    block = 200_000
    for start in range(0, len(clean), block):
        chunk = clean[start : start + block]
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
        bsdf.inputs["Roughness"].default_value = 0.75
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
    target = int(argv[2]) if len(argv) > 2 else 180_000

    data = np.load(npz_path)
    vertices = data["vertices"].astype(np.float64)
    faces = data["faces"].astype(np.int32)
    colors = data["face_colors"].astype(np.float64)
    print(f"[blender_field] in: {len(vertices)} verts / {len(faces)} tris", flush=True)

    palette = build_palette(colors)
    face_palette = map_to_palette(colors, palette)
    print(f"[blender_field] palette {len(palette)}", flush=True)

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
