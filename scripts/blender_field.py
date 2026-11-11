"""RMUC 场地 mesh（`field_raw.npz`）→ `field.glb`：逐面 CAD 颜色 + 程序化微表面纹理。

headless 调用：
```sh
/Applications/Blender.app/Contents/MacOS/Blender --background --python scripts/blender_field.py -- \
  models/rmuc2027/v1.0.0/derived/raw/field_raw.npz models/rmuc2027/v1.0.0/field.glb
```

几何：**不做减面、不做焊接**。CAD 逐 solid 三角化时顶点只在单个 BRep 面内共享
——弯曲面（圆柱/倒角）靠共享顶点平滑着色，面与面之间不共顶点而保留锐边，壳与壳
不粘连。建面前剔除零面积退化三角（渲染不产生像素，只会污染法线），其余拓扑原样
进入 glb。

材质：CAD 逐面色（无颜色记默认灰）；低饱和（灰/白）表面保留相对明度、线性映射进
冷石板蓝灰 [`BODY_DARK`, `BODY_LIGHT`]（对照现场取色 `#3D4867`），CAD 近纯白面
保持 [`WHITE_RGB`] 白；饱和色（标线/灯饰）原样保留并按
[`is_emissive`] 标自发光。调色板取
全部去重色，不做量化丢色。

已登记的场地表面配置绑定原始网格 SHA-256 与零件 ID：地面和平台的中性体面使用
独立 PVC 纹理，明确标白和彩色的 CAD 面保留原有几何与颜色，并按普通涂装反射光线。
这些涂装不参与灯具自发光；纹理和粗糙度是外观近似，不是实测光学材质。

微表面纹理：程序化可平铺噪声（[`DETAIL_SIZE`]²，np.roll 环绕无缝）按世界空间三
平面投影生成 UV（[`UV_TILE`] 米一周期），进体面的 Base Color（乘性微变化）与
Roughness（[`ROUGH_LO`, `ROUGH_HI`]）表现大平面的粗糙度变化。灯饰材质不加纹理，
保持自发光外观。

坐标：npz 已是 `MuJoCo` 世界系（米，Z 上、地面 z=0）。Blender 内部同为 Z 上，
导出必须 `export_yup=False`，否则 glTF 规范的 Y 上转换会让场地相对 `apps/render`
的 Z 上 rig 立起来。
"""

from __future__ import annotations

import sys
import hashlib
import json
from pathlib import Path

import numpy as np

#: 低饱和（灰/白）面的线性基色区间（sRGB 0~1）：保留 CAD 逐面相对明度，取
#: **偏冷的浅灰**（蓝略高于红/绿，不做重蓝）。基色过暗会同时拖垮观感与 KLT 近场
#: 特征——低反照率靠加灯救不回，亮度必须落在基色上。
BODY_DARK = (0.040, 0.045, 0.060)
BODY_LIGHT = (0.150, 0.165, 0.200)
#: 饱和度阈值（max-min 小于此值按灰/白体处理）。
SAT_THRESHOLD = 0.15
#: CAD 近纯白面的体色（线性 sRGB）；颜色分类不依赖三角剖分。
WHITE_RGB = (0.9, 0.9, 0.9)
#: 近纯白判据（饱和度上限、明度下限）。
WHITE_SAT = 0.03
WHITE_LUM = 0.95
#: 未着色面（CAD 无 Surf 色，装配匿名副本）的体色。
DEFAULT_RGB = (0.060, 0.065, 0.085)
#: 退化三角面积下限（m²）：低于此值不产生像素，建面前剔除。
MIN_AREA = 1e-12
#: 灯饰外材质的基础粗糙度（带纹理的体面由纹理逐像素决定）。
ROUGHNESS = 0.25
#: emissive 强度（配合 Bevy bloom）。
EMISSION_STRENGTH = 8.0

#: 微表面细节纹理边长（像素）。
DETAIL_SIZE = 256
#: 细节纹理随机种子（确定性，各次运行一致）。
DETAIL_SEED = 7
#: 世界空间 UV 周期（米）：一个纹理周期覆盖的边长。
UV_TILE = 0.35
#: Base Color 乘性变化区间（贴图为 sRGB，实际线性约 ×0.79~1.0）。
BASE_LO = 0.90
BASE_HI = 1.00
#: Roughness 变化区间（Non-Color，直接取贴图值）：宽区间=光面与糙面斑驳共存，
#: 单一低粗糙度会整片塑料感。
ROUGH_LO = 0.16
ROUGH_HI = 0.60


def surface_profile(raw_path: Path) -> dict | None:
    """Resolve authored surface paint only for the exact reviewed CAD mesh."""
    with raw_path.open("rb") as stream:
        fingerprint = hashlib.file_digest(stream, "sha256").hexdigest()
    for path in sorted((Path(__file__).parent / "field_surfaces").glob("*.json")):
        profile = json.loads(path.read_text())
        if profile["raw_sha256"] == fingerprint:
            return profile
    return None


def surface_atlas(profile: dict, surface: str, width: int = 4096) -> np.ndarray:
    """World-XY PVC albedo; actual markings remain in the CAD surface colors."""
    xmin, ymin, xmax, ymax = profile["bounds_xy_m"]
    height = round(width * (ymax - ymin) / (xmax - xmin))
    x = xmin + (np.arange(width, dtype=np.float32) + 0.5) * (xmax - xmin) / width
    y = ymin + (np.arange(height, dtype=np.float32) + 0.5) * (ymax - ymin) / height
    rng = np.random.default_rng(DETAIL_SEED)
    noise = np.zeros((height, width), dtype=np.float32)
    # Broad PVC mottling remains visible at sensor resolution; fine grain has
    # low contrast and is not used to manufacture point features.
    for cells, weight in [(24, 0.16), (96, 0.09), (384, 0.025)]:
        low = rng.random((cells, cells)).astype(np.float32) - 0.5
        horizontal = np.array(
            [
                np.interp(np.linspace(0, cells - 1, width), np.arange(cells), row)
                for row in low
            ],
            dtype=np.float32,
        )
        layer = np.array(
            [
                np.interp(np.linspace(0, cells - 1, height), np.arange(cells), col)
                for col in horizontal.T
            ],
            dtype=np.float32,
        ).T
        noise += weight * layer * (1.0 if surface == "platform" else 0.35)
    color = np.asarray(profile["palette_srgb"][surface], dtype=np.float32)
    atlas = np.clip(color[None, None, :] + noise[:, :, None], 0, 1)
    if surface == "platform":
        joints = (np.mod(x - xmin, 1.2)[None, :] < 0.012) | (
            np.mod(y - ymin, 1.2)[:, None] < 0.012
        )
        atlas[joints] *= 0.92

    return atlas


def textured_faces(vertices, faces, parts, colors, profile):
    """Texture only upward neutral body surfaces; preserve explicit white and colored paint."""
    tri = vertices[faces]
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    upward = normal[:, 2] > 0.5 * np.linalg.norm(normal, axis=1)
    white = (np.ptp(colors, axis=1) < WHITE_SAT) & (colors.mean(axis=1) > WHITE_LUM)
    neutral = (np.ptp(colors, axis=1) < SAT_THRESHOLD) & (colors[:, 0] >= 0) & ~white
    return {
        kind: np.isin(parts, profile[kind + "_parts"]) & upward & neutral
        for kind in ("ground", "platform")
    }


def add_surface_materials(mesh, vertices, faces, parts, colors, profile):
    import bpy

    masks = textured_faces(vertices, faces, parts, colors, profile)
    xmin, ymin, xmax, ymax = profile["bounds_xy_m"]
    surface_uv = (vertices[faces][..., :2] - np.array([xmin, ymin])) / np.array(
        [xmax - xmin, ymax - ymin]
    )
    # Reuse the existing UV channel; unrelated CAD geometry needs no second
    # per-vertex attribute merely because the floor has a different texture.
    layer = mesh.attributes["UVMap"]
    uv = np.empty((len(faces), 3, 2), dtype=np.float32)
    layer.data.foreach_get("vector", uv.ravel())
    selected = masks["ground"] | masks["platform"]
    uv[selected] = surface_uv[selected]
    layer.data.foreach_set("vector", uv.ravel())
    indices = np.empty(len(faces), dtype=np.int32)
    mesh.polygons.foreach_get("material_index", indices)
    for kind, mask in masks.items():
        atlas = surface_atlas(profile, kind)
        h, w, _ = atlas.shape
        rgba = np.ones((h, w, 4), dtype=np.float32)
        rgba[:, :, :3] = atlas
        img = bpy.data.images.new("field_" + kind, width=w, height=h, alpha=False)
        img.colorspace_settings.name = "sRGB"
        img.pixels.foreach_set(rgba.ravel())
        img.pack()
        mat = bpy.data.materials.new("painted_" + kind)
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        links = mat.node_tree.links
        bsdf = nodes.get("Principled BSDF")
        bsdf.inputs["Roughness"].default_value = 0.72 if kind == "ground" else 0.62
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = img
        tex.extension = "EXTEND"
        coords = nodes.new("ShaderNodeUVMap")
        coords.uv_map = "UVMap"
        links.new(coords.outputs["UV"], tex.inputs["Vector"])
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        indices[mask] = len(mesh.materials)
        mesh.materials.append(mat)
    # Painted CAD markings reflect light; only the separate fixture materials
    # use the original emissive classification.
    region = np.isin(parts, profile["ground_parts"] + profile["platform_parts"])
    white = (np.ptp(colors, axis=1) < WHITE_SAT) & (colors.mean(axis=1) > WHITE_LUM)
    marks = region & (white | (np.ptp(colors, axis=1) >= SAT_THRESHOLD))
    for i, color in enumerate(np.unique(colors[marks], axis=0)):
        mat = bpy.data.materials.new("surface_paint_" + str(i))
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        rgb = remap_body(color[None, :])[0]
        bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
        bsdf.inputs["Roughness"].default_value = 0.7
        indices[marks & np.all(colors == color, axis=1)] = len(mesh.materials)
        mesh.materials.append(mat)
    mesh.polygons.foreach_set("material_index", indices)


def is_emissive(rgb: tuple[float, float, float]) -> bool:
    """高饱和且以单通道为主（红/蓝/绿灯饰）。"""
    r, g, b = (float(c) for c in rgb)
    if max(r, g, b) < 0.4:
        return False
    return r > 1.5 * max(g, b) or b > 1.5 * max(r, g) or g > 1.5 * max(r, b)


def remap_body(colors: np.ndarray) -> np.ndarray:
    """逐面颜色 → 体色。

    - 低饱和面（面板/地面）：保留相对明度，映射进冷石板蓝灰 [`BODY_DARK`,
      `BODY_LIGHT`]；饱和色（标线/灯饰）原样保留。
    - CAD 近纯白面：保持 [`WHITE_RGB`]；同色表面使用同一材质。
    - 无颜色面：默认体色。

    必须在建调色板之前做——否则大片 off-white 会落到最近的饱和色项上。
    """
    out = colors.astype(np.float64, copy=True)
    uncolored = out[:, 0] < 0.0
    sat = out.max(axis=1) - out.min(axis=1)
    lum = out.mean(axis=1)
    low = sat < SAT_THRESHOLD
    t = np.clip(lum, 0.0, 1.0)
    dark = np.asarray(BODY_DARK)
    body = dark + t[:, None] * (np.asarray(BODY_LIGHT) - dark)
    out[low] = body[low]
    white = (sat < WHITE_SAT) & (lum > WHITE_LUM)
    out[white] = WHITE_RGB
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


def detail_noise(size: int, seed: int) -> np.ndarray:
    """可平铺灰度噪声（0~1）：细粒 + 低频斑驳，`np.roll` 环绕保证无缝。"""
    rng = np.random.default_rng(seed)

    def blur(a: np.ndarray, iters: int) -> np.ndarray:
        for _ in range(iters):
            a = (
                a
                + np.roll(a, 1, 0)
                + np.roll(a, -1, 0)
                + np.roll(a, 1, 1)
                + np.roll(a, -1, 1)
            ) / 5.0
        return a

    def unit(a: np.ndarray) -> np.ndarray:
        lo, hi = float(a.min()), float(a.max())
        return (a - lo) / (hi - lo) if hi > lo else np.zeros_like(a)

    fine = unit(blur(rng.random((size, size)), 1))
    coarse = unit(blur(rng.random((size, size)), 12))
    return unit(0.6 * fine + 0.4 * coarse)


def make_image(name: str, gray: np.ndarray, non_color: bool):
    """灰度数组 → 打包进 glb 的 Blender 图像。"""
    import bpy

    h, w = gray.shape
    rgba = np.empty((h, w, 4), dtype=np.float32)
    rgba[..., :3] = gray[..., None]
    rgba[..., 3] = 1.0
    img = bpy.data.images.new(name, width=w, height=h, alpha=False)
    # colorspace 必须在写像素之前设：写入后再改会触发重载并把像素清零。
    img.colorspace_settings.name = "Non-Color" if non_color else "sRGB"
    img.pixels.foreach_set(rgba.ravel())
    img.pack()
    return img


def assign_uvs(mesh, vertices: np.ndarray, faces: np.ndarray) -> None:
    """世界空间三平面投影 UV：按面主法线选投影轴，1 周期 = [`UV_TILE`] 米。"""
    tri = vertices[faces]
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    axis = np.argmax(np.abs(normal), axis=1)
    uv = np.empty((len(faces), 3, 2), dtype=np.float32)
    for d, (a, b) in ((0, (1, 2)), (1, (0, 2)), (2, (0, 1))):
        m = axis == d
        if not m.any():
            continue
        vidx = faces[m]
        uv[m, :, 0] = vertices[vidx, a]
        uv[m, :, 1] = vertices[vidx, b]
    uv /= UV_TILE
    layer = mesh.attributes.new(name="UVMap", type="FLOAT2", domain="CORNER")
    layer.data.foreach_set("vector", uv.ravel())


def add_materials(mesh, palette: np.ndarray, base_img, rough_img) -> None:
    import bpy

    for i, rgb in enumerate(palette):
        color = tuple(float(c) for c in rgb)
        mat = bpy.data.materials.new(name=f"cad_{i:02d}")
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        links = mat.node_tree.links
        bsdf = nodes.get("Principled BSDF")
        bsdf.inputs["Base Color"].default_value = (*color, 1.0)
        bsdf.inputs["Roughness"].default_value = ROUGHNESS
        bsdf.inputs["Metallic"].default_value = 0.0
        if is_emissive(color):
            bsdf.inputs["Emission Color"].default_value = (*color, 1.0)
            bsdf.inputs["Emission Strength"].default_value = EMISSION_STRENGTH
            mesh.materials.append(mat)
            continue
        # Base Color = 调色板色 × 细节贴图（乘性微变化）。
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = base_img
        tex.interpolation = "Linear"
        tex.extension = "REPEAT"
        mix = nodes.new("ShaderNodeMix")
        mix.data_type = "RGBA"
        mix.blend_type = "MULTIPLY"
        mix.inputs[0].default_value = 1.0
        mix.inputs[6].default_value = (*color, 1.0)
        links.new(tex.outputs["Color"], mix.inputs[7])
        links.new(mix.outputs[2], bsdf.inputs["Base Color"])
        # Roughness = 细节贴图（磨砂/反光变化）。
        rtex = nodes.new("ShaderNodeTexImage")
        rtex.image = rough_img
        rtex.interpolation = "Linear"
        rtex.extension = "REPEAT"
        links.new(rtex.outputs["Color"], bsdf.inputs["Roughness"])
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
    print(
        f"[blender_field] palette {len(palette)}（emissive {n_emissive}）", flush=True
    )

    bpy.ops.wm.read_factory_settings(use_empty=True)

    noise = detail_noise(DETAIL_SIZE, DETAIL_SEED)
    base_img = make_image("detail_base", BASE_LO + (BASE_HI - BASE_LO) * noise, False)
    rough_img = make_image(
        "detail_rough", ROUGH_LO + (ROUGH_HI - ROUGH_LO) * noise, True
    )

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

    assign_uvs(mesh, vertices, faces)

    if mesh.validate(verbose=False):
        raise RuntimeError("mesh.validate 修正了非法几何——上游三角化不合约，请查")

    add_materials(mesh, palette, base_img, rough_img)
    mesh.polygons.foreach_set("material_index", face_palette)
    profile = surface_profile(npz_path)
    if profile is not None:
        add_surface_materials(
            mesh, vertices, faces, data["part"][~degenerate], face_colors, profile
        )
        print(
            f"[blender_field] surface profile: {profile['scene']} ({profile['scope']})",
            flush=True,
        )
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
