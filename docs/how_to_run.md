# How to Run（sim → release 全链路）

7 个进程（`sim → vio → gicp → planner` 主链 + `aliked → lightglue → gicp` 视觉支路），
`release` 构建是默认形态（`debug` 重负载下 IMU 断流，只做开发调试）。

| # | 进程 | App | 订阅 | 发布 | 频率 |
|---|---|---|---|---|---|
| 1 | `firefly-sim` | `apps/firefly-sim` | `Firefly/Reference` | `Firefly/Imu` 100Hz / `Firefly/CameraLeft,Right` 10Hz / `Firefly/Depth` 10Hz / `Firefly/GroundTruth` 10Hz | 200Hz 物理 |
| 2 | `vio` | `apps/vio` (MSCKF) | `Imu` + 双目灰度 | `Firefly/Odometry`（100Hz propagation）+ `Firefly/Viz` (10Hz) | 10Hz 视觉修正 / 100Hz 输出 |
| 3 | `gicp` | `apps/gicp` (GICP 全局重定位 + FusionFilter) | `Odometry` (100Hz) + `Depth` + `Firefly/PoseObservation`（视觉观测） | `Firefly/CorrectedOdometry` | 1Hz 重定位 / 100Hz 融合 |
| 4 | `planner` | `apps/planner` (EGO-Planner v2: A* + MINCO) | `Odometry`/`CorrectedOdometry` + `Depth` + `Firefly/Goal` | `Firefly/Reference` + `Firefly/Viz` | 10Hz |
| 5 | `firefly-viz` | `apps/firefly-viz` | `Firefly/Viz` | （写 rerun viewer / rrd） | 消费 10Hz 可视化 |
| 6 | `aliked` | `apps/aliked` (ALIKED-N16 特征提取，`ort`) | `Firefly/CameraLeft`（左目灰度） | `Firefly/Features`（带事件唤醒） | 1Hz 节流推理 |
| 7 | `lightglue` | `apps/lightglue` (LightGlue 匹配 + PnP，`ort`) | `Firefly/Features` + `Firefly/CorrectedOdometry`（先验，优先矫正值）+ 库图 `--map` | `Firefly/PoseObservation`（→ `gicp` 融合） | 特征到即查 |

数据流：`sim → vio → gicp → planner → sim`（PD 闭环跟踪）；视觉支路
`aliked → lightglue → gicp` 与 GICP 共用同一 `FusionFilter`（对照 VINS-Fusion
`loop_fusion` 检环 + 位姿边，见 `crates/firefly-vision-match/src/lib.rs`）。
`sim_time` 为全链路统一时钟，`fastrace` 跨进程续接同一 `trace_id`。
Rust 计算线程零 IO：可视化数据经 `Firefly/Viz` 话题零拷贝发布，由
`firefly-viz` 进程统一写 rerun。

## 0. 前置依赖

```bash
# Rust 1.97+ / Python 3.12+
cargo --version && python3 --version

# 安装 Python 环境（根 workspace 聚合 firefly-mujoco + firefly-sim）
uv sync

# 安装 rerun viewer（可选，但强烈建议先开，7 进程共享同一 viewer）
cargo install rerun-cli   # 或 uv tool install rerun-sdk
```

地图文件：`apps/planner/maps/*.ffmap`（见 `docs/map-format.md`）；
视觉库图：`apps/planner/maps/*.ffvmap`（离线建库产物，见 §2.5）。
配置：`configs/*.toml`（`sim.toml` / `vio.toml` / `gicp.toml` / `planner.toml`），
缺键回落代码默认值。

权重（`models/`，已 ignore，不进 git，离线导出）：

| 文件 | 用途 |
|---|---|
| `aliked-n16-k512.onnx` | `aliked` 在线推理 + 离线建库 |
| `lightglue-aliked-k512.onnx`（+ `.data`） | `lightglue` 图-图匹配 |

## 1. 构建验证（release）

```bash
cargo build --release -p vio -p gicp -p aliked -p lightglue -p planner
cargo test
uv run firefly-sim --help      # 检查 Python 侧可导入
uv run firefly-viz --help      # 检查可视化进程可导入
```

## 2. 启动 — 多终端手动启动（release 全链路）

> 按顺序各开一个终端。**Rust 进程先起、sim 最后起**：`sim --script`
> 的启动互锁等 vio `ready` 电平（`is_initialized=true`）才启动任务时钟，
> sim 先起会 15s 超时退出。`aliked`/`lightglue` 的 ORT 模型加载约 10s，
> 看到"会话就绪"再起下一个。

```bash
# 终端 0 — viewer（多进程共享；或用 `uv run firefly-viz --serve` 起内置 viewer）
rerun

# 终端 1 — 可视化统一写入：订阅 Firefly/Viz，写共享 viewer（或 --save logs/run.rrd）
uv run firefly-viz
# 可选：uv run firefly-viz --save logs/run.rrd   # 离线录制

# 终端 2 — VIO：订阅 IMU/双目，发布 odom + 可视化消息
./target/release/vio
# 可选：./target/release/vio -- --config configs/vio.toml
# 等待日志：VIO 就绪（initialized 连续 10 帧）

# 终端 3 — GICP：订阅 odom+深度+视觉观测，发布校正后里程计
./target/release/gicp -- --map apps/planner/maps/gate.ffmap
# 不指定 --map 时尝试加载 gate.ffmap，不存在则用空地图（GICP 自动禁用，仅透传融合）
# 视觉观测（lightglue → Firefly/PoseObservation）自动融合，无需额外参数

# 终端 4 — 视觉全局定位（需先备好库图与权重，见 §2.5）
./target/release/aliked [--model models/aliked-n16-k512.onnx]   # 左目特征 → Firefly/Features（1Hz）
# 等待日志：aliked 会话就绪 + 已打开特征话题
./target/release/lightglue -- --map apps/planner/maps/straight_forward.ffvmap  # 匹配+PnP → Firefly/PoseObservation

# 终端 5 — 规划器：订阅 odom(优先 CorrectedOdometry) + 深度，发布 Reference + 可视化消息
./target/release/planner -- --map apps/planner/maps/gate.ffmap
# 独立运行不接 sim：同上（无 odom 时回退轨迹推进估计）

# 终端 6 — 物理环境（最后起）：发布传感器，订阅 Reference 做 PD 闭环
uv run firefly-sim --no-trace
# 可选：uv run firefly-sim   # trace 模式，启用 OTel span（约 0.4x real-time）
# 可选：uv run firefly-sim --no-trace --script straight_forward   # 轨迹模式（bench 用）
# 可选：uv run firefly-sim --no-trace --script straight_forward --odom-topic Firefly/VoidOdom  # void 状态源
# 等待日志：状态源就绪（Firefly/Odometry），任务时钟启动
```

### 2.5 视觉库图离线建库与评测

```bash
# 采集（MuJoCo 摆拍，见 scripts/collect_vision_frames.py）
uv run python scripts/collect_vision_frames.py [--traj straight_forward]
# 转 bin
uv run python scripts/gen_vision_eval_data.py [--traj straight_forward]
# 建库
cargo run --release -p aliked -- --build-map logs/bench/vision_eval/frames --out apps/planner/maps/straight_forward.ffvmap
# 离线评测（输出即交付：只统计、不设断言）
cargo test --release -p lightglue --test vision_traj_eval -- --nocapture
# 环境变量覆盖：VISION_ALIKED_MODEL / VISION_LG_MODEL / VISION_MAP / VISION_EVAL_DIR
```

## 3. 发目标点 — 机器人导航

规划器启动后悬停在 `configs/sim.toml: start = [1.0, 4.0, 1.0]`，等待 `Firefly/Goal`：

```bash
# 语法：uv run firefly-goal X Y Z  （地图系，米）
uv run firefly-goal 20 4 1.5        # gate.ffmap 中直线可达点
uv run firefly-goal 25 6 1.2        # 绕柱点
uv run firefly-goal 8.5 4 1.0       # 门前点

# 动态重目标：飞行中可连续发送，最新一条生效（10Hz 轮询，1s 内重算全局路径）
uv run firefly-goal 22 3.6 1.5
```

坐标需落在地图 `ORIGIN + DIMS*RESOLUTION` 范围内（`gate.ffmap`: `0~28 × 0~8 × 0~3.2m`）。不可达目标会被 `PlannerManager::set_goal` 拒绝并 `log::warn`。

可选的 `planner` 启动时指定初始目标：

```bash
./target/release/planner -- --map apps/planner/maps/gate.ffmap --goal 20 4 1.5 --start 1 4 1
```

强制急停（对照官方 `mandatory_stop`）：

```bash
cargo run -p planner -- --mandatory-stop   # 向 Firefly/MandatoryStop 发单帧空消息
```

## 4. 可视化（rerun，统一 `sim_time` 时间轴）

Rust 进程不直接连 rerun：vio/planner 经 `Firefly/Viz` 话题零拷贝发布
`VizMessage`，由 `firefly-viz` 统一写入。启动顺序：先起 viewer（或
`firefly-viz --serve`），再 `uv run firefly-viz` 连共享 viewer；离线录制用
`uv run firefly-viz --save logs/run.rrd`（之后 `rerun logs/run.rrd` 回放）。

* `sensor/stereo_left|right`、`sensor/depth` — MuJoCo 原图
* `vio/odom` (橙) / `gt/pose` (蓝) — VIO 估计 vs 真值
* `plan/map` + `plan/decor` — 静态先验（启动一次性）
* `plan/perceived` — 深度 raycast 在线占据（2.5s 刷新）
* `plan/global_path` (绿)、`plan/local_traj` (蓝+黄速度)、`plan/planes`、`plan/drone`、`plan/motions`
* `vio/debug/track_length` / `db_size` / `track_avg_len` — 前端健康度

## 5. 常用地图与配置

```bash
ls apps/planner/maps/
# gate.ffmap      默认门洞场景（与 MuJoCo scene.py 同构）
# corridor.ffmap  窄走廊
# maze.ffmap      迷宫
# slalom_dyn.ffmap / forest_dyn.ffmap  动态障碍（MOTION 段，见 map-format.md）
# straight_forward.ffvmap  视觉库图（前飞走廊，lissajous 空域无覆盖）

# 换配置（任意 --config）
./target/release/vio -- --config configs/vio.toml
./target/release/gicp -- --config configs/gicp.toml --map apps/planner/maps/maze.ffmap
./target/release/planner -- --config configs/planner.toml --map apps/planner/maps/maze.ffmap
```

## 6. bench 套件（精度评测，5 进程常态）

```bash
# 全套件：15 轨迹 × 34s，每轮自动起 gicp/aliked/lightglue 并采 corrected 对照
uv run python bench/bench_suite.py --duration 34 --turns 1 --tag fleet5
# 单轨迹：uv run python bench/bench_vio.py --duration 34 --trajectory straight_forward --output logs/bench/smoke.json
# 旧语义（仅 sim+vio）：加 --no-fleet；再起 planner：加 --with-planner
# 多轮统计：--turns 3（约 45 分钟）
```

输出（`logs/bench/`，gitignored）：per-turn json（含 `metrics` + `corrected_metrics` +
`ΔRMSE`）、套件 `suite_*.json`、per-turn rrd。库图覆盖面是当前瓶颈：
`ff` 系轨迹有改善（`ff_fast`/`ff_laps` Δ>+0.09），`lissajous` 空域无库图覆盖、
Δ≈0 属预期（见 `logs/bench/suite_fleet5_15x1x34s.json`）。

## 7. 优雅退出与排障

* **必须 `Ctrl+C` 优雅退出**（`node.wait` 返回 `Err` → Drop 端口）。`pkill -9` / `SIGKILL` 会留下孤儿内核 shm 与幽灵端口注册，后续订阅端会连上死端口收不到数据，且占满 `max_publishers` 槽位。
* 清理残留（所有进程杀干净后执行，macOS）：
  ```bash
  rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state
  # Linux: rm -rf /dev/shm/iox2* /tmp/iceoryx2
  ```
* 端口冲突 / `FailedToDeliverSignal` 为良性：订阅端兜底轮询仍可驱动。
* `&` 链式后台启动（`cmd & sleep N && next &` 写在一行）会把子进程挂到奇怪的
  进程组、日志停住——逐条独立启动。

## 8. 最小验证清单

1. `rerun` + `uv run firefly-viz` 已开 → 7 个进程按 `firefly-viz → sim → vio → gicp → aliked → lightglue → planner` 顺序启动（`firefly-viz` 必须最先：它预创建 `Firefly/Log` 服务并定 `max_publishers=10` 上限；Rust 发布端只 `open`，顺序反了即降级纯 stderr）。各进程日志均出现 `日志聚合已挂载（tag=...）` + `已订阅 ...` / `已打开话题`；`firefly-viz` 出现 `已订阅 Firefly/Viz + Firefly/Log`。
2. `uv run firefly-goal 20 4 1.5` → `planner` 日志 `收到新目标` + `目标更新 ... 重新规划中`，`rerun` 中 `plan/local_traj` 出现。
3. `sim` 日志 `收到参考 t=... pos=(...)` 且无人机开始移动。
4. `Ctrl+C` 后 `全部进程已结束` / `优雅退出`，进程组无残留（`ps aux | grep firefly` 为空）。

## 9. 故障速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `planner` 日志 `目标 (...) 不可达` | 目标点在占据体素内或超出地图 | 换 `gate` 范围内空地点，如 `20 4 1.5` |
| `GICP矫正接受` 迟迟不出现 | 空地图或点云 `<30` 点，或 `chi2` 拒收 | 检查 `--map` 路径，近处对墙增加特征 |
| `odom 订阅不可用` | `vio` 未启动或 iceoryx2 幽灵端口 | 重启全栈并清理 `/tmp/iceoryx2` |
| `深度/里程计丢失！进入急停` | 深度或 odom 超时 `>1.0s` | 检查 `sim` 是否卡死，`gicp/planner` 是否正常消费 |
| `sim --script` 15s 超时退出 | vio 未起或未 ready（互锁等不到 `is_initialized`） | 先起 vio，待 `VIO 就绪` 日志再起 sim |
| `lightglue` 无观测输出 | ORT 模型未加载完或库图路径错 | 待 `lightglue 会话就绪` 日志；检查 `--map` |
| `IMU 断流 >200ms` 刷屏 | `debug` 构建算力不足 | 换 `release` 构建 |
| `视觉观测无 odom 内插` 跳过 | 观测先到、odom 后到（事件驱动竞态） | 正常现象，`gicp` 待融合队列会自动追上；频繁出现则检查 vio 是否掉帧 |
| `日志聚合挂载失败（降级纯 stderr）` | `firefly-viz` 未先启动（`Firefly/Log` 服务上限未预创建） | 先起 `firefly-viz`，再起其余 6 进程；重排顺序后重启全栈 |

## 10. 统一日志聚合（`Firefly/Log` → rrd `TextLog`）

散落的终端 stderr 文本行是野生的、不可检索的。本栈的纪律：**所有进程
日志经 `Firefly/Log` 话题向 `firefly-viz` 聚合，统一写 rerun `TextLog`
（`logs/<tag>` 实体），持久进 rrd**——debug 数据一律进 rrd，文本日志只是
`TextLog` 在 viewer 里的一种呈现，不是另一套系统。

链路（对照 `docs/architecture.md` 的 `firefly-pubsub`）：

```text
vio/planner/gicp/aliked/lightglue/sim
  │ log! 宏（调用点零改动）
  ├─→ stderr（原链路保留，双 sink）
  └─→ IpcAppend → 有界队列(128, 满即丢) → 主循环 pump → Firefly/Log (zero-copy)
                                                        │
firefly-viz: 收线程（排空订阅→进队） → channel → 落盘线程（排序→批写 rr.log）
                                                        │
rrd: logs/<tag> TextLog（sim_time 轴；<0 回落 wall_time 轴，可检索）
```

要点（实现见 `crates/firefly-observability/src/lib.rs`）：

- **零改调用点**：`log::info!/warn!/error!` 一行不改；`init()` 常驻 dormant
  的 `IpcAppend`，`init_ipc(&node, tag)` 同线程建端返回 `LogIpc` 句柄，
  主循环每 tick `set_sim_time(t) + pump_log_ipc(&log_ipc)` 两行；
- **线程模型**（对照 logforth 官方 `appenders/async` + `DropIncoming`）：
  `append` 只做纯内存格式化（调用线程快照 sim 时钟/trace 上下文），IPC
  发布由主循环驱动——iceoryx2 发布端不出 `LogIpc` 句柄作用域，无 `unsafe`；
- **背压**：两级满即丢（进程内 128 + viz 侧 1024），日志永不阻塞计算线程；
- **退出**：`pump_log_ipc` 在退出路径调一次（残余落盘）+ viz 侧哨兵排空；
- **检索**：`RrdReader` 按 `logs/<tag>` 实体 + 时间窗取 chunk（见
  `.agents/skills/rerun`），不再翻终端历史。

方法论（为什么值得做）：一次性 `print` 只能解一次性问题；聚合链路是
产品级能力——下一次的 1-2 天从"翻 7 个终端找日志"变成"rrd 里按实体筛
一行"。本节的链路即交付物的一部分，不是调试副产品。
