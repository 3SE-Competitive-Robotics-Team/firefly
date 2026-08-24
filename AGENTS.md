# AGENTS.md

## 规范

- 强迫症：每次改动后审视全仓库一致性，不留冗余、死代码、临时文件。
- 新 crate：`cargo new crates/<name> --lib --edition 2024`（自动纳入 workspace）。
- DDD 拆 crate（firefly-*），依赖自上而下，领域层不依赖应用层。
- 错误用 firefly-error：kind/status 分类，模块边界 .with_context。
- 可观测性：关键函数 `#[fastrace::trace]`，日志用 log 宏，禁 println!。
- 依赖统一在根 `[workspace.dependencies]`。

## 注释规范

- **禁止过程叙事**：注释只描述现在的设计与约束，不写「曾/旧版/之前/原来/不再/
  修复了/从 X 改为 Y/当时」等演变史——历史归 git log 与 commit message。
- **禁止 bug 战争故事**：现象、复现步骤、排查过程、修复日期、新旧实测对比一律不进
  注释。背后有仍成立的陷阱时，改写成祈使句约束（「必须 X，否则 Y」），只留结论。
- **禁止阶段/计划叙事**：「wave N」「待移植」「尚未接入」「后续将」等不进代码——
  代码只陈述已存在的事实，计划放 issue。
- doc 注释（///、//!）：写用途、单位、不变量、边界条件；与参考实现的对应关系
  （如「对照 OpenVINS xxx」）保留。
- 行内注释（//）：只在代码无法自解释时解释「为什么」，不复述代码；注释掉的死代码
  直接删除。

## 配置（configs/）

- **TOML、一应用一份**：统一放仓库顶层 `configs/`（`sim.toml` / `vio.toml` / `planner.toml`），
  启动时加载（Rust 支持 `--config` 换文件），缺文件即报错。
- **最小化**：只写与代码默认值不同的键，缺失键回落 `*Options::default()`；
  纯数据 Options 直接 `serde::Deserialize` + `#[serde(default)]`，Python 用标准库
  `tomllib`；不引 YAML。

## VIO（firefly-vio*）约定

- 标定/噪声等数值参数：改对应 `*Options` 默认值并在 doc 注释标注单位与来源；
  需按部署调整的键同步进 `configs/vio.toml`。
- **进程必须优雅退出**（Ctrl-C → `node.wait` 返回 Err → 端口 Drop）：硬杀（pkill -9/SIGKILL）会留下孤儿内核 shm 对象与幽灵端口注册——后续订阅端会连上死端口的残留连接收不到任何数据，且幽灵占满 `max_publishers` 槽位后新发布器直接创建失败。排障清理：杀干净所有进程后 `rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state`（macOS；须在进程全死后执行）。

## 运行（MuJoCo 双语言闭环）

双语言闭环三个进程，iceoryx2 IPC（`Firefly/*` 话题）通信，fastrace
trace 跨进程续接（传感器→vio→planner→参考 单周期 trace）：

```
Python sim（MuJoCo 物理 + 传感器发布）→ vio（MSCKF 位姿估计）→ planner（重规划）
```

按顺序各开一个终端（可先开 viewer，见下）：

```bash
# 0. 可选：先开 rerun viewer（多进程共享；不开则每个进程自动起独立 viewer）
rerun

# 1. Python 物理环境：200Hz 物理；发布 IMU 100Hz / 双目+深度+真值 10Hz，
#    订阅 Firefly/Reference 做 PD 闭环控制
uv sync   # 首次：安装 firefly-mujoco / firefly-sim（根 workspace）
uv run firefly-sim

# 2. Rust VIO：订阅 MuJoCo IMU/双目灰度，MSCKF 视觉更新，发布 odom 10Hz；
#    传感器（双目/深度）与估计位姿写入共享 viewer
cargo run -p vio

# 3. Rust 重规划：订阅 odom 作为状态源（新鲜超时回退轨迹模拟），
#    未指定 --map 时加载 MuJoCo 默认场景静态地图（与 scene.py 同构，
#    深度感知在线补充），发布参考回传；
#    规划结果写入同一 viewer（与 vio 共用 sim_time 时间轴）
cargo run -p planner
```

rerun 可视化约定：`sensor/stereo_left|right`、`sensor/depth` 为传感器原图，
`vio/odom` 为估计位姿（3D 变换），规划/地图/轨迹由 planner 写入
（`plan/*` 前缀）——多进程共用 `sim_time` 时间轴（仿真秒），回放时跨进程
数据按同一时钟对齐。默认布局（场景 3D + 前端健康度面板）由进程启动时
自动发送，无需手工配置。

独立运行（不依赖闭环）：

```bash
cargo run -p planner -- --map apps/planner/maps/gate.ffmap  # 静态地图
```

## 构建

- Rust：`cargo build`（workspace 含 `apps/vio`、`apps/planner`，排除 `apps/firefly-sim`）。
- Python：`uv sync`（根 workspace 统一管理 `firefly-mujoco` / `firefly-sim`，
  依赖与脚本见各自 `pyproject.toml`）。

## Log / Debug（rerun rrd）

- **debug 数据一律进 rrd，禁止文本日志**
- 分工：log 宏（logforth）只做进程级诊断（启动、错误、关键事件）；可结构化数据（位姿、图像、标量、轨迹、中间状态）进 rrd, viewer 可回放，CLI/RrdReader 可检索。
- 写入（`firefly-rerun::Stream`）：
  - 入口：`connect_or_spawn()` 连共享 viewer，`save(path)` 离线录制。
  - 时间轴：`set_time(sim_time 秒)`。
  - 封装：`log_gray_image` / `log_depth_image` / `log_pose` /
    `log_line_strip` / `clear`；标量文本走 `stream()` 底层
    （`rerun::Scalars` / `rerun::TextLog`）。
  - 实体路径按 app 前缀（`sensor/*`、`vio/*`、`plan/*`），
    遵守上文 rerun 可视化约定。
- 读取：viewer 回放；`rerun rrd print --entity <path> -vvv` /
  `rerun rrd stats`；Python `RrdReader`，详见 `.agents/skills/rerun`。

## 性能

- 结论只认 trace 实测（ConsoleReporter span 树），不靠读代码推断。
- 主循环建 root span（`Span::root` + `set_local_parent`），结束 `flush()`。
- 热路径禁 `#[logcall]`（无条件格式化大对象，开销爆炸），只留 trace。
- 输出先写清晰，不写解析脚本。

## 验证

- `cargo test`
- `cargo test --release -p firefly-planner --test random_map_benchmark -- --ignored`
