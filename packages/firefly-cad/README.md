# firefly-cad

RMUC 场地 `STEP→glTF` 资产管线（Blender 精修的上游，Blender 只做材质与导出，
场地约定见 `models/rmuc2026/README.md`）。

## 阶段（一步一包，可独立复现）

1. `census`（已实现）：只读装配普查——产品清单、包围盒（毫米，装配系）、
   颜色、面数。输出 `derived/census/products.json`（下游机器契约）与
   `SUMMARY.md`（人工确认清单）。不写任何 mesh，不修改源 STEP。
2. `tessellate`（已实现）：自 free shapes 递归装配树，逐实例三角化并套累积
   位置（只有被引用产品的面能查到颜色，装配展开的匿名副本查不到，故必须按
   label 递归）。输出 `derived/raw/field_raw.npz`（`vertices` 米已归一化 /
   `faces` / `face_colors` 逐面 sRGB，无颜色记 -1 / `part` 产品 tag）与
   `_manifest.json`。这是下游唯一契约。
3. `collide`（已实现）：`field_raw.npz` → 实心体素 → 贪心 AABB 盒 →
   `<scene>_collision.json`（供 `MuJoCo` 生成 box geom；凹结构不跨空腔）。
4. `export`（`scripts/blender_field.py`）：`field_raw.npz` → 逐面 CAD 颜色 →
   PBR 材质 → `field.glb`。**不减面、不焊接**——逐 solid 三角化拓扑原样保留
   （弯曲面平滑、面间锐边、壳不粘连），只剔除零面积退化三角；近黑体带程序化
   微表面纹理（世界空间 UV 噪声 → Base Color / Roughness）。

视觉导出用 headless Blender（CAD 颜色→PBR 灰黑体 + 红蓝绿 emissive），
不进本包源码。

## XCAF 读取约束

`STEPCAFControl_Reader` 必须关 `NameMode`：产品名全匿名，而 `NameMode` 会触发
逐子形状命名的平方级路径——实测 1.2G 装配的 `Transfer` 卡死 >1h，关闭后 ~37s
（`ReadFile` 约 21s）。颜色必须保留（灯饰识别与材质分色的依据），
其余 `Layer/Props/GDT/SHUO` 全关。

## 资产接入约定（换场地 = 新目录 + 注册一行）

- 视觉：`models/<scene>/<visual>.glb`，Bevy asset root = `models/`，
  场景由 `configs/scene.toml` 的 `scene` 选定（sim/render/viz 共用）。
- 物理：`models/<scene>/<scene>_collision.json`，盒集合 `[cx,cy,cz,hx,hy,hz]`（米，
  `MuJoCo` 系）。`MuJoCo` mesh 几何取凸包，凹结构不可用单 mesh，故用盒集合。
- 场景注册表 Rust（`apps/render`）与 Python（`firefly_mujoco/scene.py`）同构。

## 运行

```sh
# 普查（out 缺省为 <stp 同目录>/derived/census）
uv run --package firefly-cad firefly-cad-census models/rmuc2026/RMUC2026_2.0.0.stp

# 三角化（out 缺省为 <stp 同目录>/derived/raw）
uv run --package firefly-cad firefly-cad-tessellate models/rmuc2026/RMUC2026_2.0.0.stp

# 碰撞聚合（三角网格 → 实心体素 → 贪心 AABB 盒）
uv run --package firefly-cad firefly-cad-collide \
  --out models/rmuc2026/rmuc2026_collision.json --res 0.15 \
  models/rmuc2026/derived/raw/field_raw.npz

# 测试（合成小 STEP / 合成网格，不依赖 1.2G 真件；pytest 约定对照 CI）
uv run --package firefly-cad --with pytest pytest packages/firefly-cad/tests/ -q -p no:cacheprovider
```

## 记录

每次转换在 `CONVERSION.md` 追加一条：工具版本、偏差参数、红蓝灯饰清单、
三角数与材质数。`models/` 已 ignore，记录随本包进 git。

## 验证

接入后以人工实测为最终结论（出生点无接触、悬停、发布图像的亮度与特征密度），
不写自动验收脚本。
