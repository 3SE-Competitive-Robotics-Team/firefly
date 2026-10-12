# firefly-cad

RMUC STEP 资产构建。源文件只读，统一归一化为米、Z 向上、XY 居中、最低点 Z=0。
视觉、物理碰撞和规划地图从同一份几何生成；全部生成资产留在本机，不进 Git。

## 入口

从仓库根执行：

```bash
# 几何资产：三角化 → 碰撞 → Blender GLB → FFMap → 独立读回验收
uv run --package firefly-cad firefly-cad-build /path/to/RMUC2026_V2.0.0.stp
```

数值配置统一在 `configs/cad.toml`，默认三角化偏差 2mm，碰撞/规划分辨率
0.15m，地图边距 1m、顶界至少 6m。`--blender` 指定 Blender 可执行文件。
若发行版 Blender 缺 NumPy，构建器使用 `uv` 将匹配 Python ABI 的 NumPy
安装到 `derived/blender-python-*`，仅给导出子进程设置路径，不修改系统 Python。

Rust 应用统一用 `cargo xtask build-all-release` 构建；物理资产检查、离线图像采集、
建库和任务验收各自使用 [开发指南](../../docs/development.md#assets) 中的入口。
任务、碰撞和故障验收由 `firefly-evaluation` 包的 `uv run firefly-evaluate` 执行，
报告保存在 `logs/acceptance/<运行ID>/`。几何验收与建库完成不能替代飞行或在线定位验收。

## 产物

| 文件 | 消费者与契约 |
|---|---|
| `models/rmuc2026/derived/raw/field_raw.npz` | 归一化顶点、三角形、逐面颜色、零件 tag |
| `models/rmuc2026/field.glb` | Bevy / Rerun；CAD 颜色与程序化材质，不减面、不焊接 |
| `models/rmuc2026/rmuc2026_collision.json` | MuJoCo；米制 `[cx,cy,cz,hx,hy,hz]` 碰撞盒 |
| `apps/planner/maps/rmuc2026.ffmap` | planner；与物理碰撞同一体素格，未额外膨胀 |
| `models/rmuc2026/asset_manifest.json` | 源 SHA-256、转换参数、依赖版本、坐标变换、产物 hash 与几何验收 |

`derived/stages/` 保存阶段输入与产物内容 hash；二者都匹配才复用。
离线采集也检查输入及逐帧内容 hash；损坏或未完成的采集保留，重试时通过 `--frames-dir` 指定新目录。
总清单只有 `status=passed` 才代表本次几何验收完成。图像采集与飞行记录分别放在
独立资产目录和 `logs/*.rrd`，不把原始传感器数据写成文本日志。

## 算法约束

- XCAF 禁用 `NameMode`，保留颜色，沿装配树累积实例变换。
- 三角形朝向组合 OCCT 面的 `TopAbs_REVERSED` 与实例变换手性；解析长方体测试
  以几何叉积验证外法线。
- 网格体素化后填充封闭内部，贪心合并仅合并占据格；开放通道与薄结构需要针对
  实际分辨率核验。盒合并本身不跨越空格。
- FFMap 保留相同占据格；独立读回检查格心、唯一性、物理盒覆盖与体积。
- GLB 读回包围盒须与归一化网格在 `1e-4m` 内一致；碰撞边界误差上限为一个体素。
  这些检查不证明每处 CAD 空腔都适合当前体素分辨率。

## 单独运行与测试

普查工具 `firefly-cad-census <stp>` 输出产品、包围盒、颜色与源 hash；
`firefly-cad-tessellate`、`firefly-cad-collide` 支持单阶段运行。
视觉导出脚本是 `scripts/blender_field.py`。

```bash
uv run --all-packages --extra test pytest packages/firefly-cad/tests/ tests/evaluation/ -q -p no:cacheprovider
FIREFLY_RUN_RMUC_ASSETS=1 uv run --all-packages --extra test pytest tests/system/test_rmuc_assets.py -q -p no:cacheprovider
```

实际转换记录见 [CONVERSION.md](CONVERSION.md)，系统验收边界见
[测试契约](../../docs/development.md#tests)。
