# firefly-cad

RMUC 场地 `STEP→glTF` 资产管线（Blender 精修的上游，Blender 只做美术与减面，
场地约定见 `models/rmuc2026/README.md`）。

## 阶段（一步一包，可独立复现）

1. `census`（已实现）：只读装配普查——产品清单、包围盒（毫米，装配系）、
   颜色、面数。输出 `derived/census/products.json`（下游机器契约）与
   `SUMMARY.md`（人工确认清单）。不写任何 mesh，不修改源 STEP。
2. `tessellate`：逐 solid 三角化（大平板偏差 2mm、细杆件 0.5mm），毫米转米，
   施加归一化变换（`transform.py`，原点/朝向锁定一次），按功能分组分批落盘
   `derived/raw/<group>.obj`（不一次性 hold 全装配，规避内存峰值）。
3. `group`：读 `groups.json`（人工可编辑：匿名 tag → 功能组），把 raw mesh
   归组；减面交 Blender `Decimate`（headless，见 `scripts/`）。
4. `collide`：全部 solid 体素化 → 贪心合并 AABB 盒 → `<scene>_collision.json`。

Blender 精修（PBR/红蓝 emissive/点阵贴花）与最终 `field.glb` 导出不在本包，
由 `scripts/` 下的 headless Blender 脚本承担。

## 资产接入约定（换场地 = 新目录 + 注册一行）

- 视觉：`models/<scene>/<visual>.glb`，Bevy asset root = `models/`，
  `FIREFLY_SCENE` 选场景。
- 物理：`models/<scene>/<scene>_collision.json`，盒集合 `[cx,cy,cz,hx,hy,hz]`（米，
  MuJoCo 系）。MuJoCo mesh 几何取凸包，凹结构不可用单 mesh，故用盒集合。
- 场景注册表 Rust（`apps/render`）与 Python（`firefly_mujoco/scene.py`）同构。

## 运行

```sh
# 普查（out 缺省为 <stp 同目录>/derived/census）
uv run --package firefly-cad firefly-cad-census models/rmuc2026/RMUC2026_2.0.0.stp

# 碰撞聚合（三角网格 → 实心体素 → 贪心 AABB 盒；供 MuJoCo 生成 box geom）
uv run --package firefly-cad firefly-cad-collide \
  --out models/rmuc2026/rmuc2026_collision.json --res 0.15 models/rmuc2026/derived/raw/*.obj

# 测试（合成小 STEP / 合成网格，不依赖 1.2G 真件；pytest 约定对照 CI）
uv run --package firefly-cad --with pytest pytest packages/firefly-cad/tests/ -q -p no:cacheprovider
```

## 记录

每次转换在 `CONVERSION.md` 追加一条：工具版本、偏差参数、匿名件→功能分组映射、
红蓝灯饰清单、减面前后面数。`models/` 已 ignore，记录随本包进 git。

## 验证

接入后以人工实测为最终结论（出生点无接触、悬停、发布图像的亮度与特征密度），
不写自动验收脚本。
