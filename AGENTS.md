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
| `ORB_SLAM3/` | 回环候选、连续确认与固定尺度几何验证（`src/LoopClosing.cc` / `Sim3Solver.cc`） |
| `VINS-Fusion/` | 全局漂移与位姿图参考（`loop_fusion/src/pose_graph.cpp`） |
| `open_vins/` | MSCKF 官方 C++（firefly-vio* 的移植基准） |
| `iceoryx2/` | IPC 中间件源码 |
| `logforth/`、`fastrace/` | 日志 / tracing 库源码 |
| Cargo registry `purecv-0.7.1/` | 图像金字塔、LK、导数与 FAST（OpenCV 语义镜像；有问题给上游提） |

## 配置（configs/）

- **TOML、一应用一份**：统一放仓库顶层 `configs/`（`sim.toml` / `vio.toml` / `planner.toml`），
  启动时加载（Rust 支持 `--config` 换文件），缺文件即报错。
- **最小化**：只写与代码默认值不同的键，缺失键回落 `*Options::default()`；
  纯数据 Options 直接 `serde::Deserialize` + `#[serde(default)]`，Python 用标准库
  `tomllib`；不引 YAML。

## 技术路线

- 状态估计主线为 VIO + ALIKED + LightGlue + 回环。
- 视觉链路包含离线库图 PnP 定位与会话内 RGB-D 关键帧回环；默认四自由度位姿图
  联合 VIO、地图锚点和回环约束。ESKF 为互斥可选后端，禁止重复融合同一观测。
- 左目路标必须使用注册到左目的深度（重投影、z-buffer、空洞与边缘拒绝），禁止同索引
  读取居中深度相机。回环证据与库图重定位分别验收，模型集成测试不能替代飞行回环验收。
- 在线图和关键帧库默认上限各 512，容量耗尽显式拒绝扩容；只支持同一 VIO 会话，
  不提供跨会话地图恢复、完整 BA 或任意位置重定位。边界与公式见 `docs/loop_closure.md`。
- 深度感知、视觉定位融合、飞控和规划器保持独立职责。

## 真值、初始化与坐标系

- `Firefly/GroundTruth` 供 render 合成传感器图像，GroundTruth 与 PlantState
  均可供评测、可视化对照；禁止进入估计器初始化、在线状态修正、飞控反馈、
  解锁判据与失效回退。
- VIO 从静止 IMU 与图像视差初始化。测量不足或
  运动检查不通过时保持未就绪。飞控必须等待有效 IMU、已初始化且新鲜的里程计。
- 原始里程计使用重力对齐的局部坐标系：初始位置为零、航向为规范自由度。
  静态地图、先验平面和全局目标接入前必须有独立定位或显式固定启动先验得到的
  坐标变换；固定启动先验在 `configs/localization.toml [origin]` 配置，禁止读取在线真值生成变换。
- 轨迹评测允许固定尺度的航向和平移对齐，必须在结果中注明；原始数据保留，
  对齐结果不得回流估计或控制。
- 仿真内部状态用于物理推进、传感器生成、渲染；render 订阅 GroundTruth
  只为生成图像。这不构成估计器与飞控可访问的状态源。

## 坐标基础层

- 跨模块坐标变换统一使用 `firefly-base::{FrameId, RigidTransform, FrameTree}`，
  方向固定为 `T_target_source`；组合检查中间坐标系，点/自由向量/协方差不得混用。
- 坐标树是同一时刻的几何快照，时间同步与新鲜度由消费端负责。
- VIO 空中失联时仅在新鲜 IMU 下提供姿态稳定与标称悬停推力；不承诺保高/定点，
  不用陈旧位置判落地。恢复必须有有效里程计和显式 Hold/Land 指令。
- 飞控反馈和 Hold 锚点始终在 odom 系；map 系规划参考通过坐标树转换，禁止切换反馈源。
- 融合右误差、伴随换基与协方差重置约定见 `docs/frames.md`。算法内部矩阵不携带隐式跨系语义。

## VIO（firefly-vio*）约定

- IMU 白噪声配置为连续时间密度；每采样标准差换算为 `σ_density = σ_sample / √fs`。
  MuJoCo 默认 IMU 100 Hz，对应 gyro `2e-4 rad/s/√Hz`、accel `2e-3 m/s²/√Hz`。
- 高频输出通过独立 IMU 预测缓存生成，不推进相机滤波状态或增广克隆。
  里程计时间戳必须对应状态的 IMU 时刻，速度转换到局部世界系；测量未覆盖时不发布。
- 在线内参更新必须同步投影模型与下一帧跟踪器；锚定表示的 FEJ 由统一表示雅可比处理。

- 标定/噪声等数值参数：改对应 `*Options` 默认值并在 doc 注释标注单位与来源；
  需按部署调整的键同步进 `configs/vio.toml`。
- **进程必须优雅退出**（Ctrl-C → `node.wait` 返回 Err → 端口 Drop）：硬杀（pkill -9/SIGKILL）会留下孤儿内核 shm 对象与幽灵端口注册——后续订阅端会连上死端口的残留连接收不到任何数据，且幽灵占满 `max_publishers` 槽位后新发布器直接创建失败。排障清理：杀干净所有进程后 `rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state`（macOS；须在进程全死后执行）。

## 场地资产

- 统一入口：`uv run --all-packages --extra test python scripts/prepare_rmuc.py <场地.stp>`；
  几何构建器为 `firefly-cad-build`，数值配置在 `configs/cad.toml`。
- GLB、碰撞盒与 FFMap 必须来自同一归一化几何，并通过内容 hash 和坐标/占据验收。
- 离线视觉采集使用 `render --offline` 与 `Firefly/Offline/*`；禁止将摆拍标签发布到在线状态话题。
  离线深度为理想几何标签，在线深度才施加传感器退化。冻结视觉库按场地、标定和
  成像外观契约校验；生成二进制 hash 只作追溯，在线噪声改变不触发重建。
- 资产通过、传感器启动、飞行、在线定位必须分别报告；失败或缺权重不得记为通过。
- 自动任务入口为 `scripts/accept_rmuc.py`，阈值在 `configs/acceptance.toml` 与
  `tests/system/mission.py::Options`；运行前固定，禁止按失败结果放宽。
- 正常任务流转只使用新鲜估计状态和 FC 模式；真值在 RRD 中独立评分，
  估计到达不得记作真实到达。评测碰撞/越界中止须与任务完成分开报告。
- 验收原始证据只进 RRD，JSON/HTML 只保存汇总、配置与内容指纹；局部 VIO 可作
  固定尺度航向/平移对齐，地图定位和跟踪评分禁止事后对齐。

## 运行（MuJoCo 双语言闭环）

仅支持 RMUC2026（`configs/scene.toml`）。缺少 `models/rmuc2026/field.glb`
或 `rmuc2026_collision.json` 必须报错，不得回退场景。最小闭环为
sim / render / vio / fc，另起 firefly-viz 统一记录，iceoryx2 IPC 传递数据与 trace。

```text
sim（MuJoCo 物理） ── IMU ──→ vio ── odom ──→ fc ── 四电机推力 ──→ sim
     └── 仿真位姿 ──→ render ── 双目 ──→ vio
```

停机坪由 `firefly_mujoco.scene.PAD` 与 `PAD_CLEARANCE` 定义。
上电不会自己起飞，等待静止初始化完成后通过 `ffctl fc` 解锁与起飞。
fc 每个 tick 发布 Control（上锁时零推力）；控制指令超过墙钟 50ms 未更新时，
sim 暂停物理和传感器时间。完整步骤见 `docs/how_to_run.md`。

各开一个终端，先启动记录进程：

```bash
uv run firefly-viz --save logs/rmuc_run.rrd
uv run firefly-sim
cargo run --release -p render
cargo run --release -p vio
cargo run --release -p fc
```

sim 发布 IMU 100Hz、仿真位姿 10Hz、PlantState 200Hz、Airframe 1Hz；
render 提供双目与深度。VIO 视觉更新 10Hz、预测里程计 100Hz；飞控 1kHz。
planner 默认读取 `apps/planner/maps/rmuc2026.ffmap`，缺文件报错。
地图系算法必须使用经过固定启动先验或独立定位对齐的状态。planner 订阅 CorrectedOdometry 和 LocalizationStatus；
未就绪不发布参考，500ms 墙钟失联、无效状态或定位质量退化触发停止发布并锁存，须重启恢复。
定位质量默认要求独立视觉测量年龄 ≤3s、融合后同刻位置分歧 ≤0.5m；参数在 localization 的 quality 配置。
FFMap 场地占据是不可被深度空闲射线、衰减或动态移除擦除的先验。
重复/乱序样本不得续期，禁止用参考轨迹代替状态或自动切回局部 VIO。

rerun 可视化约定：vio 写 `vio/odom` + `vio/traj`（估计位姿/轨迹）、
`gt/pose` + `gt/traj`（真值对照）与 `vio/debug/*`（前端健康度），飞控写
`fc/debug/*`（`state` 模式编码 / `altitude` 相对起飞点高度 / 推力与姿态误差），
规划/地图/轨迹由 planner 写入（`plan/*` 前缀）——多进程共用 `sim_time` 时间轴（仿真秒），回放时
跨进程数据按同一时钟对齐。传感器原图（双目/深度）不进 rrd，只在 `render` 进程
自带的调试面板里。默认布局（场景 3D + 前端健康度面板）由进程启动时自动发送，
无需手工配置。

单独启动规划器（仍须提供有效地图系里程计）：

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
  正式链路的录制产物一律落 `logs/`（如 `logs/rmuc_run.rrd`），永远不写 `/tmp`
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

- 数学测试必须同时包含独立公式/解析值、梯度差分和最终输出契约；
  不能只让代价与梯度互相验证。参考源码版本与容差含义见 `docs/testing.md`。
- 确定性任务要求成功；随机安全属性允许显式有界拒绝，但不将拒绝计作任务成功。
  禁止用整体成功率阈值掩盖指定任务失败，不用机器相关耗时作为正确性门槛。

- `cargo test`
- `cargo test -p firefly-planner --test planner_contracts`
- 传感器进程启动：准备 RMUC 资产，先 release 构建 vio / fc / render，确保没有其他闭环进程；运行
  `FIREFLY_RUN_SENSOR_STARTUP=1 uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py -q`。
  检查屏蔽 PlantState、保留 render 生成图像所需的 GroundTruth；需图形会话，录制只进 `logs/*.rrd`，所有子进程通过 SIGINT 退出。

- 双语言 IPC 契约：`uv sync --all-packages --extra test` 后，执行
  `FIREFLY_TEST_PYTHON="$PWD/.venv/bin/python" cargo test -p firefly-pubsub --test python_interop -- --ignored`。
