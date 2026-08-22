---
name: rerun
description: Rerun 基础认知与 .rrd 录制文件的阅读方法。当需要检查/分析/提取 .rrd 或 .rbl 文件内容、理解 Rerun 数据模型、或用 CLI/Python 从录制文件取数据时使用。本仓库强制：debug 数据必须进 rrd（TextLog/组件），禁止 firefly-vio.log 之类文本日志。
---

# Rerun 基础与 .rrd 阅读

## 1. Rerun 是什么

- The Data Layer for Physical AI：log + visualize + query 一体，Python/C++/Rust SDK。
- `.rrd` = Rerun 持久化格式；`.rbl` = 仅 blueprint 的变体（字节格式相同）。
- 本机工具：`rerun` CLI（`rerun rrd` 子命令组）、Python `rerun` 包（`rerun.experimental`）。

## 2. RRD 文件格式

- 线性序列的 framed messages（`LogMsg`），三种变体：
  - `SetStoreInfo`：声明 store（`StoreId` = kind Recording/Blueprint + application_id + recording_id；`StoreInfo` 带 store_source/store_version）
  - `ArrowMsg`：数据主体，每个是一个 Apache Arrow IPC RecordBatch 编码的 chunk（Sorbet schema，元数据 `rerun:*` 前缀）
  - `BlueprintActivationCommand`：唯一非数据控制消息
- 可选 footer 索引：每 chunk 的偏移/大小/统计/schema hash → 随机访问；无 footer 时回退线性扫描（如不干净关闭）
- 一个 rrd 可含多个 store（recording + blueprint），靠 application_id + recording_id 匹配合并
- chunk 列结构：`RowId`（control 列）+ 每条 timeline 一个 index 列 + 每组件一个 data 列（如 `Points3D:positions`）

## 3. 能存什么（几乎一切）

- 结构化时序：标量、位姿、点云、图像、张量——各组件类型
- 原始二进制：`Blob` / `EncodedImage` / `Asset3D` 组件，任意字节都能塞
- 文本：`TextLog`（带时间戳和级别，比 print 日志更结构化——可按时间轴回放）
- 自定义类型：非标准字段用 `AnyValues` 兜底，或注册自己的组件
- 多时间轴：同一份数据可同时挂 `sim_time`（ns 时间戳）和 `frame`（序列号），查询时按任意轴切片

## 4. 能干什么

- **看**：viewer 直接渲染（图、3D、表格）
- **查**：viewer 内 DataframeView 当表格筛；Python `RrdReader` 按实体路径/时间窗取 chunk；CLI `rerun rrd print --entity` / `filter` / `split`
- **导出**：数据帧导出 parquet/CSV，喂给下游分析

## 5. 仓库规范（强制）

**本仓库所有 debug 必须使用 rerun rrd 进行**——数据进 viewer 看、用 `RrdReader`/CLI 查、按需导出。
`firefly-vio.log` / `firefly-sim.log` 这类文本日志是**不允许**的（结构化、可检索、可回放的数据一律进 rrd）。

## 6. 读 .rrd 的路径

- **CLI**：`rerun rrd print`（`-v` 到 `-vvv` 逐级展开，`--entity` 过滤，`--footers` 看索引）、`rerun rrd stats`（`--no-decode` 快速版）、`verify / migrate / optimize / split / merge / filter / compare`
- **Python**：`rerun.experimental.RrdReader(path)` → `.recordings()/.blueprints()`（StoreEntry: kind/application_id/recording_id）→ `.stream()/.store()`（索引访问）；`Chunk.format()` 直接打表
- **Chunk Processing API**：`LazyChunkStream` + lenses（MutateLens/DeriveLens/Selector jq 语法）；`write_rrd(application_id, recording_id)` 写文件，存储/查询场景用 `OptimizationProfile.OBJECT_STORE`
- **Viewer MCP**：`rerun viewer-mcp` 起 MCP server 控制 viewer（含截图）
- 坑：查询组件列是 `ListArray`，DataFrame 0-based 索引 vs SQL 1-based

## 7. 示例：读取 odom 与 GT 原始数据（本项目）

数据已就位：vio 进程写 `vio/odom` + `vio/traj`（橙）与 `gt/pose` + `gt/traj`（蓝），统一 `sim_time`。

1. 录制：`scripts/run_firefly.sh --save logs/run1.rrd`（viewer 后台运行并落盘；
   **`--no-viewer` 时无人写盘，不产 rrd**）。
2. 看：`rerun logs/run1.rrd`；数值快查 `rerun rrd print --entity gt/pose --entity vio/odom -vvv`。
3. 读出原始数据：`uv run python .agents/skills/rerun/scripts/read_poses.py logs/run1.rrd`
   （rerun-sdk 是根项目 dev 依赖，`uv add --dev rerun-sdk` 后 uv run 直接可用，
   勿用 `--with`（每次重建临时环境）或系统 python。默认读 `gt/pose` +
   `vio/odom`，可追加自定义实体；输出逐行 `sim_time_ns x y z qx qy qz qw`，
   可管道给 awk/python）。脚本内注释记录了实测坑：entity_path 带前导 `/`、
   sim_time 须 cast int64、translation/quaternion 嵌套取 `[0]`。

已实测（2026-08）：946/949 行读出成功。无 footer 的 rrd（viewer `--save`
被 SIGTERM 结束即如此）RrdReader 回退线性扫描并告警，`store()` 不可用，只用 `stream()`。
