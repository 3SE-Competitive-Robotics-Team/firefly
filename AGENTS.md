# AGENTS.md

## 规范

- 强迫症：每次改动后审视全仓库一致性，不留冗余、死代码、临时文件。
- 新 crate：`cargo new crates/<name> --lib --edition 2024`（自动纳入 workspace）。
- DDD 拆 crate（firefly-*），依赖自上而下，领域层不依赖应用层。
- 错误用 firefly-error：kind/status 分类，模块边界 .with_context。
- 可观测性：关键函数 `#[fastrace::trace]`，日志用 log 宏，禁 println!。
- 场景光照唯一一份（`firefly-render::lighting`），所有相机共用；视角差异只写相机
  配置（曝光/色调映射），禁止给单个相机加专属可聚簇光源（`PointLight`/`SpotLight`）
  ——Bevy GPU 聚簇在多视图同帧渲染时不按 `RenderLayers` 隔离，会渗进别的视角。
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

## 参考源码（本机 `~/Projects/`，实现须严格对照）

项目处于早期，各模块以对照移植为主——改行为前先读对应官方源码：

| 本机路径 | 对应 |
|---|---|
| `EGO-Planner-v2/` | 规划器官方 C++（`swarm-playground/*/src/planner/traj_opt/`） |
| `open_vins/` | MSCKF 官方 C++（firefly-vio* 的移植基准） |
| `FAST-LIVO2/` | DIVO 的 IMU 传播、ESIKF 更新与建图对照 |
| `iceoryx2/` | IPC 中间件源码 |
| `logforth/`、`fastrace/` | 日志 / tracing 库源码 |
| `purecv/` | 自研视觉库（LK 光流等，vio 前端引用） |

## 配置（configs/）

- **TOML、一应用一份**：统一放仓库顶层 `configs/`（`sim.toml` / `vio.toml` / `planner.toml`），
  启动时加载（Rust 支持 `--config` 换文件），缺文件即报错。
- **最小化**：只写与代码默认值不同的键，缺失键回落 `*Options::default()`；
  纯数据 Options 直接 `serde::Deserialize` + `#[serde(default)]`，Python 用标准库
  `tomllib`；不引 YAML。

## 真值、初始化与坐标系

- `Firefly/GroundTruth` 和 `Firefly/PlantState` 仅供评测、可视化对照；禁止进入
  估计器初始化、在线状态修正、飞控反馈、解锁判据与失效回退。
- VIO 从静止 IMU 与图像视差初始化；VOID 复用静态 IMU 初始化。测量不足或
  运动检查不通过时保持未就绪。飞控必须等待有效 IMU、已初始化且新鲜的里程计。
- 原始里程计使用重力对齐的局部坐标系：初始位置为零、航向为规范自由度。
  静态地图、先验平面和全局目标接入前必须有独立定位得到的坐标变换，禁止从
  真值生成该变换再反馈到算法。VOID 默认不启用仿真世界先验图。
- 轨迹评测允许固定尺度的航向和平移对齐，必须在结果中注明；原始数据保留，
  对齐结果不得回流估计或控制。
- 仿真内部状态用于物理推进、传感器生成、渲染；`--script` 的真值反馈属于
  运动生成夹具。它们不构成无人机可访问的状态源，也不证明真实反馈闭环成功。

## VIO（firefly-vio*）约定

- 标定/噪声等数值参数：改对应 `*Options` 默认值并在 doc 注释标注单位与来源；
  需按部署调整的键同步进 `configs/vio.toml`。
- **进程必须优雅退出**（Ctrl-C → `node.wait` 返回 Err → 端口 Drop）：硬杀（pkill -9/SIGKILL）会留下孤儿内核 shm 对象与幽灵端口注册——后续订阅端会连上死端口的残留连接收不到任何数据，且幽灵占满 `max_publishers` 槽位后新发布器直接创建失败。排障清理：杀干净所有进程后 `rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state`（macOS；须在进程全死后执行）。

## 运行（MuJoCo 双语言闭环）

最小闭环为 sim / vio / fc，另起 firefly-viz 记录（可先开 viewer，见下），iceoryx2 IPC（`Firefly/*` 话题）通信，fastrace
trace 上下文随 IPC 消息跨进程传递：

```
Python sim（MuJoCo 物理 + 传感器发布，被控对象）→ vio（MSCKF 位姿估计）
→ fc（1kHz 飞控，4 电机推力）→ sim（施加；planner 在时其参考进 fc）
```

上电停在停机坪（`firefly_mujoco.scene.PADS`：场景场地表面 + `PAD_CLEARANCE`），
**不会自己起飞**：解锁/起飞/降落
由地面站指令 `Firefly/Command`（`./target/release/ffctl fc arm | takeoff [alt] | hold |
track | land | disarm`；`cargo build --release -p ffctl`）驱动，模式与失效保护在
`firefly-flight::FlightFsm`（对照 `PX4`
`nav_state` + ArduPilot 模式机，细节见 `docs/how_to_run.md` §3.1）。

**锁步**：fc 每个 tick 都发 `Firefly/Control`（上锁时零推力），被控对象按指令新鲜度
（墙钟 50ms）决定物理是否推进——无有效指令则物理与传感器时间暂停，
`fc` 不起就不动；被控对象不自行供力。

按顺序各开一个终端（可先开 viewer，见下）：

```bash
# 0. 可视化：先起共享 viewer，再起统一写入进程（Rust 进程只发 IPC，不落盘、不开窗）
rerun &
uv run firefly-viz

# 1. Python 物理环境（被控对象）：200Hz 物理；发布 IMU 100Hz / 双目+深度+真值 10Hz
#    / 真值状态 PlantState 200Hz / 机体描述 Airframe 1Hz；订阅 Firefly/Control 施加推力
#    （锁步：无有效指令则物理不推进）。--script 是 VIO bench 的轨迹夹具（真值反馈），不走飞控。
uv sync   # 首次：安装 firefly-mujoco / firefly-sim（根 workspace）
uv run firefly-sim

# 2. Rust VIO：订阅 MuJoCo IMU/双目灰度，MSCKF 视觉更新，发布 odom 100Hz（视觉更新 10Hz）；
#    估计位姿与前端健康度写入共享 viewer
cargo run --release -p vio

# 3. Rust 飞控：1kHz 控制环，订阅 Airframe/odom/IMU/参考/Command；PlantState 仅供评测，
#    发布 Firefly/Control（4 电机推力）+ 控制量进共享 viewer（fc/debug/*）
cargo run --release -p fc

# 4.（可选，需先完成局部里程计与地图对齐）Rust 重规划：
#    订阅里程计，发布参考回传到飞控
cargo run --release -p planner

# 5. 等静止初始化完成后：解锁 → 起飞（被拒原因在 fc 日志与 rrd logs/fc）
./target/release/ffctl fc arm && ./target/release/ffctl fc takeoff 1.0
```

rerun 可视化约定：vio 写 `vio/odom` + `vio/traj`（估计位姿/轨迹）、
`gt/pose` + `gt/traj`（真值对照）与 `vio/debug/*`（前端健康度），飞控写
`fc/debug/*`（`state` 模式编码 / `altitude` 相对起飞点高度 / 推力与姿态误差），
规划/地图/轨迹由 planner 写入（`plan/*` 前缀）——多进程共用 `sim_time` 时间轴（仿真秒），回放时
跨进程数据按同一时钟对齐。传感器原图（双目/深度）不进 rrd，只在 `render` 进程
自带的调试面板里。默认布局（场景 3D + 前端健康度面板）由进程启动时自动发送，
无需手工配置。

独立运行（不依赖闭环）：

```bash
cargo run --release -p planner -- --map <实际存在的地图.ffmap>
```

## 构建

- Rust：`cargo build`（workspace 自动纳入 `crates/*` 与 Rust `apps/*`，排除 Python 应用 `firefly-sim` / `firefly-viz`）。
- **运行进程一律加 `--release`**：数值与逐像素代码必须使用优化构建，具体开销以当前版本 trace 实测为准；`cargo test` 仍用 debug 迭代。
- Python：`uv sync`（根 uv workspace 管理 `packages/*` 与 `firefly-sim` / `firefly-viz`，
  依赖与脚本见各自 `pyproject.toml`）。

## Log / Debug（rerun rrd，有且只有这一种记录方式）

- **数据记录有且只有一种方式：进 rrd（`logs/` 下），禁止文本日志与 tmpfs**——
  正式链路的录制产物一律落 `logs/`（如 `logs/wh_agg.rrd`），永远不写 `/tmp`
 （tmpfs 重启即丢、路径不可复现、排查轮子每次重造）。`firefly-vio.log` /
  `firefly-sim.log` 这类文本日志不允许存在；终端 stderr 只做进程级诊断的
  双 sink 之一，不做记录载体。
- 分工：log 宏（logforth）只做进程级诊断（启动、错误、关键事件）；可结构化数据（位姿、图像、标量、轨迹、中间状态）进 rrd, viewer 可回放，CLI/RrdReader 可检索。
- 写入（`firefly-viz` Python 进程统一写 rerun）：
  - Rust 计算线程零 IO：经 `Firefly/Viz` 话题发布 `VizMessage`（iceoryx2 零拷贝），
    `firefly-viz` 订阅后写 viewer/rrd。
  - 结构化日志同样聚合：各进程 `log!` 经 `Firefly/Log` 话题进 `firefly-viz`，
    落 `logs/<tag>` 的 `TextLog` 实体（`sim_time` 轴；`sim_time<0` 回落 `wall_time`
    轴）——日志本身也是 rrd 里可检索的实体，不是另一套系统。
  - 入口：`firefly-viz` 默认连共享 viewer（`127.0.0.1:9876`），`--save logs/<name>.rrd` 离线录制。
  - 时间轴：消息携带 `sim_time` 秒，Python 端 `set_time`。
  - 实体路径按 app 前缀（`vio/*`、`gt/*`、`plan/*`、`logs/*`），
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
- 无真值进程启动：先 release 构建 vio / void / fc，确保没有其他闭环进程；运行
  `uv run --no-dev python bench/check_sensor_startup.py`（VOID 加 `--estimator void`）。
  此检查需 Linux EGL，录制只进 `logs/*.rrd`，所有子进程通过 SIGINT 退出。
