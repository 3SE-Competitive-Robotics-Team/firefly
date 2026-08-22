# FFMap：firefly 占据地图标准文件格式

FFMap 是 firefly 的 3D 占据地图标准格式，用于离线/仿真环境的地图表示。
静态环境以**占据体素**表达（与运行时 `GridMap`、在线感知的体素化一致），
动态障碍以**形状 + 时间航点**表达。

文件后缀 `.ffmap`，UTF-8 文本，行式指令，`#` 开头为注释。

## 头部

```text
FORMAT     firefly-map   1
RESOLUTION 0.4
ORIGIN     0 0 0
DIMS       50 20 8
```

| 指令 | 含义 |
|---|---|
| `FORMAT` | 格式名与版本（当前 `firefly-map 1`） |
| `RESOLUTION` | 体素分辨率（米） |
| `ORIGIN` | 地图原点（世界坐标，米） |
| `DIMS` | 体素维度（格数，`x y z`） |

## 静态占据：`OCCUPANCY` 段

`OCCUPANCY` 指令后每行一个**占据体素的世界坐标**（`x y z`，米），
坐标按 `ORIGIN + (idx + 0.5) * RESOLUTION` 取体素中心。

```text
OCCUPANCY
1.2 0.6 1.4
2.8 3.4 2.2
```

## 装饰层：`DECOR` 段

与 `OCCUPANCY` 同构的占据体素列表，语义为**不参与规划**的视觉元素
（草丛、地面装饰等），运行时独立渲染，不影响搜索与优化。

```text
DECOR
2.05 3.05 0.25
```

## 动态障碍：`MOTION` 段

`MOTION <shape> <参数>` 开始一个动态障碍，后续行给出时间-位置航点
（`t x y z`，t 为相对地图时刻 0 的秒数，位置为障碍**中心**），
`LOOP` 行表示航点循环（默认只在航点间单向运动）。

```text
MOTION box    9 2.5 1.5 0.8 3.0 1.2
0    1 2 1
4    18 2 1
8    1 2 1
LOOP

MOTION sphere 15 4 1 0.5
0    15 2 1
6    15 6 1
LOOP
```

| 形状 | 参数 | 含义 |
|---|---|---|
| `box` | `cx cy cz sx sy sz` | 中心 + 尺寸 |
| `sphere` | `cx cy cz r` | 中心 + 半径 |

航点时间必须单调递增。动态障碍不参与静态 `OCCUPANCY` 段，
由运行时按航点插值后体素化到 `GridMap`。

## 边界约定

- 指令大小写敏感，值以空白分隔；
- 未知指令报错，缺失头部字段报错；
- 空文件/无占据体素合法（全未知地图）。

## 实现

解析与序列化在 `crates/firefly-map/src/format.rs`（`MapFile`），
运行时地图为 `firefly-map::GridMap`。程序化场景用 `firefly-map::Scene`
设计后导出为 `.ffmap` 文件。
