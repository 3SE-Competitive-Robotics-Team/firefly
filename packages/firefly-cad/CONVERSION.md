# CONVERSION

RMUC 场地 STEP→glTF 的转换记录（工具版本、参数、映射；`models/` 已 ignore，
记录随本包进 git）。

## RMUC2026_2.0.0.stp → field.glb

- 源：`RMUC2026_2.0.0.stp`（1_254_821_405 B，sha256 `8dfe9ebd761e44d9…`）。
- 工具：cadquery-ocp-novtk 8.0.1.0.0；trimesh 5.1.0 + scipy 1.18.1；Blender 5.2.1 LTS。
- XCAF 读取：`ColorMode=on`、`NameMode=off`，其余 `Layer/Props/GDT/SHUO` 全关。
  `NameMode` 触发逐子形状命名的平方级路径（实测 1.2G 装配 `Transfer` >1h 未完成），
  关闭后 `ReadFile≈21s + Transfer≈37s`；产品名全匿名，关闭无损失。
- 装配：`PRODUCT_DEFINITION_SHAPE` 875；XCAF `GetShapes` 440 labels（433 零件 /
  7 装配）；展开 1025 solids / 292566 faces。
- 三角化：`BRepMesh_IncrementalMesh` 线性偏差 2.0mm，全并行 →
  **5,065,670 三角 / 5,312,112 顶点**。
- 归一化：装配系包围盒 `[-14876, -6377, -1941, 14876, 9626, 1860]` mm →
  x/y 居中、z 底置 0、mm→m → **29.75 × 16.00 × 3.80 m**。
- 颜色：逐面 `ColorSurf→ColorGen→ColorCurv`（sRGB 0~1），无颜色记 -1。
  调色板 756 色。红蓝灯饰：件 `429` 纯红 `(1,0,0)` 35592 面；件 `433`
  蓝 `(0, 0.521, 1)` 5344 面；另有绿/青/棕等。
- 视觉导出（`scripts/blender_field.py`）：**不做减面、不做焊接**——逐 solid 的
  三角化拓扑原样进 glb。顶点只在单个 BRep 面内共享：弯曲面（圆柱/倒角）平滑
  着色、面与面之间不共顶点而保留锐边、壳与壳不粘连。建面前剔除零面积退化三角
  （本次 2,344，渲染不产生像素且污染法线）。
- 材质：CAD 逐面色（无颜色记默认灰 0.12）；低饱和（灰/白）面压进灰黑区间
  `[0.05, 0.16]`（避免大片 off-white 在日光曝光下发白），饱和色（标线/红蓝绿灯
  饰）原样保留；调色板取全部去重色（本次 28 项，不做量化丢色），`is_emissive`
  命中 12 项设 emission，强度 8。**不加合成点阵/贴图平面**——特征来自场地自身
  的板缝、标线与灯饰。
- 坐标：Blender 导出 `export_yup=False`，场地系 Z 上，与 `apps/render` rig 一致。
- 碰撞：`field_raw.npz` → 实心体素 0.15m → 贪心 AABB **1061 盒** →
  `rmuc2026_collision.json`；MuJoCo 侧只放这 1061 个透明 box（无视觉 mesh）。
- 产出：`models/rmuc2026/field.glb`（**5,063,326 三角 / 5,311,296 顶点 / 28 材质**，
  186,482,284 B）。
