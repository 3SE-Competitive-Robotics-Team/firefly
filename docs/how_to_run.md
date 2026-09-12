# How to Run（warehouse 全链路，7 进程）

当前场地：**warehouse**（`packages/firefly-mujoco/src/firefly_mujoco/scene.py`
缺省，`models/warehouse/structure.obj` 存在即生效，无需设 `FIREFLY_SCENE`）。
46m × 16m 室内仓库，无人机沿走廊 `+x` 飞，起点 `(2, 0, 1)`（见 `configs/sim.toml`）。
`boxes` / `gate.ffmap` 系旧场景残留，仅回归对照用。

7 个进程（`sim → vio → gicp → planner → sim` 主链 + `aliked → lightglue → gicp` 视觉支路），
`release` 构建是默认形态（`debug` 重负载下 IMU 断流，只做开发调试）。

| # | 进程 | App | 订阅 | 发布 | 频率 |
|---|---|---|---|---|---|
| 1 | `firefly-viz` | `apps/firefly-viz` | `Firefly/Viz` + `Firefly/Log` | （写 viewer / rrd） | 消费落盘 |
| 2 | `vio` | `apps/vio` (MSCKF) | `Imu` + 双目灰度 + 真值（仅初始化对齐） | `Firefly/Odometry`（100Hz propagation）+ `Firefly/Viz` (10Hz) | 10Hz 视觉修正 / 100Hz 输出 |
| 3 | `gicp` | `apps/gicp` (GICP 全局重定位 + FusionFilter) | `Odometry` (100Hz) + `Depth` + `Firefly/PoseObservation` | `Firefly/CorrectedOdometry` | 1Hz 重定位 / 100Hz 融合 |
| 4 | `aliked` | `apps/aliked` (ALIKED-N16 特征提取，`ort`) | `Firefly/CameraLeft`（左目灰度） | `Firefly/Features`（带事件唤醒） | 1Hz 节流推理 |
| 5 | `lightglue` | `apps/lightglue` (LightGlue 匹配 + PnP，`ort`) | `Firefly/Features` + `Firefly/CorrectedOdometry`（先验，优先矫正值）+ 库图 `--map` | `Firefly/PoseObservation`（→ `gicp` 融合） | 特征到即查 |
| 6 | `planner` | `apps/planner` (EGO-Planner v2: A* + MINCO) | `Odometry`/`CorrectedOdometry` + `Depth` + `Firefly/Goal` | `Firefly/Reference` + `Firefly/Viz` | 10Hz |
| 7 | `firefly-sim` | `apps/firefly-sim` | `Firefly/Reference` | `Firefly/Imu` 100Hz / `Firefly/CameraLeft,Right` 10Hz / `Firefly/Depth` 10Hz / `Firefly/GroundTruth` 10Hz | 200Hz 物理 |

数据流：`sim → vio → gicp → planner → sim`（PD 闭环跟踪）；视觉支路
`aliked → lightglue → gicp` 与 GICP 共用同一 `FusionFilter`（对照 VINS-Fusion
`loop_fusion` 检环 + 位姿边，见 `crates/firefly-vision-match/src/lib.rs`）。
`sim_time` 为全链路统一时钟，`fastrace` 跨进程续接同一 `trace_id`。
Rust 计算线程零 IO：可视化数据经 `Firefly/Viz` 话题零拷贝发布，由
`firefly-viz` 进程统一写 rerun。原始双目/深度图像走 IPC 话题直达
vio/aliked/gicp/planner，**不进** rrd（vio 只发位姿/轨迹/健康度瘦版可视化）。

## 0. 前置依赖

```bash
# Rust 1.97+ / Python 3.12+
cargo --version && python3 --version

# 安装 Python 环境（根 workspace 聚合 firefly-mujoco + firefly-sim + firefly-viz）
uv sync

# Rust 日志缺省只打 error；要看就绪/融合日志，每个终端先 export：
export RUST_LOG=info
```

地图与库图（warehouse 链只用这两个；其余 `maps/` 下 `gate/corridor/maze/*_dyn` 均为旧场景）：

| 文件 | 用途 |
|---|---|
| `apps/planner/maps/warehouse.ffmap` | 仓库静态地图（`scripts/gen_warehouse_ffmap.py` 生成，已 ignore；`ORIGIN -2 -9 0`，`DIMS 500 180 52`，分辨率 0.1m → x∈[-2,48]、y∈[-9,9]、z∈[0,5.2]） |
| `apps/planner/maps/wh_corridor.ffvmap` | 视觉库图（走廊 x=2..40，每 2m × 双高度摆拍，40 帧） |

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
uv run python -c "from firefly_sim.trajectories import TRAJECTORIES; print(sorted(TRAJECTORIES))"
uv run firefly-viz --help      # 检查可视化进程可导入（argparse 生效）
```

注意：`firefly-sim` 没有 `--help`（`sys.argv` 只认 `--script/--no-trace/--odom-topic`，
传别的参数不会报错而是直接开跑），别拿它做导入检查。

Rust 二进制的参数一律是**直接跟 flag**（`--flag value`），不要加 `cargo run` 式的
`--` 分隔符（那是 cargo 的，不是程序的）。各进程接受的参数（源码为准）：

- `vio`：仅 `--config <path>`（缺省 `configs/vio.toml`）
- `gicp`：`--map <map.ffmap>` / `--config`（缺省 `configs/gicp.toml`）/ `--odom-topic`
- `aliked`：`--model <onnx>`（缺省 `models/aliked-n16-k512.onnx`）；离线建库见 §2.5
- `lightglue`：`--map <map.ffvmap>`（必填）/ `--model <onnx>`
- `planner`：`--map` / `--config` / `--start X Y Z`（缺省 `2 0 1`）/ `--goal X Y Z`（缺省=起点悬停）/ `--frame-offset` / `--mandatory-stop`

`--map` 缺省行为（两边不一样）：`planner` 不指定时用与 warehouse 碰撞盒同构的
内置静态地图（`apps/planner/src/scene.rs::mujoco_map_file`，走廊全局路径可飞）；
`gicp` 不指定时尝试 `gate.ffmap`（旧场景），不存在则用**空地图**（GICP 自动禁用、
仅透传融合）。warehouse 链两个都显式传 `warehouse.ffmap`。

## 2. 启动 — 多终端手动启动（release 全链路）

> 按顺序各开一个终端，**`firefly-viz` 必须最先、sim 最后**。`firefly-viz` 先创建
> `Firefly/Log` 服务并定 `max_publishers=10` 上限；Rust 发布端只 `open`，顺序反了
> 即降级纯 stderr（rrd 里收不到日志）。`sim --script` 的启动互锁等 vio `ready`
> 电平（`is_initialized=true`）才启动任务时钟，sim 先起会 15s 超时退出。
> `aliked`/`lightglue` 的 ORT 模型加载约 10s，看到"会话就绪"再起下一个。

```bash
# 终端 1 — 可视化统一写入（必须最先）：订阅 Firefly/Viz + Firefly/Log
uv run firefly-viz --save logs/wh_run.rrd   # 离线录制，交付物（见 §4）
# 可选：uv run firefly-viz                   # 无参则自动 spawn viewer 并连接共享 viewer
# 可选：uv run firefly-viz --serve           # 本进程起内置 viewer（与 --save 互斥）

# 终端 2 — VIO：订阅 IMU/双目，发布 odom + 可视化消息
./target/release/vio
# 等待日志：VIO 就绪（initialized 连续 10 帧）

# 终端 3 — GICP：订阅 odom+深度+视觉观测，发布校正后里程计
./target/release/gicp --map apps/planner/maps/warehouse.ffmap
# 等待日志：全局重定位靶图就绪（约 31 万点）

# 终端 4 — 视觉全局定位（需先备好库图与权重，见 §2.5）
./target/release/aliked
# 等待日志：aliked 会话就绪 + 已打开特征话题

# 终端 5 — 匹配 + PnP → Firefly/PoseObservation
./target/release/lightglue --map apps/planner/maps/wh_corridor.ffvmap
# 等待日志：视觉库图就绪（40 帧）+ lightglue 会话就绪 + 冒烟推理通过

# 终端 6 — 规划器：订阅 odom(优先 CorrectedOdometry) + 深度，发布 Reference + 可视化消息
./target/release/planner --map apps/planner/maps/warehouse.ffmap
# 无 --goal 时悬停在 --start（缺省 2 0 1），等待 Firefly/Goal（见 §3）

# 终端 7 — 物理环境（最后起）：发布传感器，订阅 Reference 做 PD 闭环
uv run firefly-sim --no-trace --script wh_corridor
# 可选：uv run firefly-sim --no-trace                          # 无脚本：悬停在起点等 planner 参考（§3 导航用）
# 可选：uv run firefly-sim --no-trace --script wh_corridor     # 轨迹模式：脚本参考驱动（planner 参考被忽略，只验证接线）
# 可选：... --odom-topic Firefly/VoidOdom                      # void 状态源（DIVO A/B 对比用）
# 等待日志：状态源就绪（Firefly/Odometry），任务时钟启动
```

`--script` 可选轨迹（`apps/firefly-sim/src/firefly_sim/trajectories.py: TRAJECTORIES`，
省略 NAME 时为 `lissajous_classic`）：

| 轨迹 | 场景 | 说明 |
|---|---|---|
| `wh_corridor` | warehouse | 走廊前飞：`(2,0,1)` 沿 +x 飞 20m 后折返；`wh_corridor.ffvmap` 覆盖 x=2..40 全程 |
| `straight_forward` 等 15 个 | boxes（旧） | 起点 `(1,4,1)` 系旧场景坐标，在 warehouse 下跑会开局 4m 大机动——别用 |

### 2.5 视觉库图离线建库与评测

```bash
# 采集（MuJoCo 摆拍，见 scripts/collect_vision_frames.py）
uv run python scripts/collect_vision_frames.py --traj wh_corridor
# 转 bin
uv run python scripts/gen_vision_eval_data.py --traj wh_corridor
# 建库（注意：cargo 的 -- 是 cargo 分隔符，程序参数直接跟）
cargo run --release -p aliked -- --build-map logs/bench/vision_eval/frames --out apps/planner/maps/wh_corridor.ffvmap
# 离线评测（输出即交付：只统计、不设断言）
cargo test --release -p lightglue --test vision_traj_eval -- --nocapture
# 环境变量覆盖：VISION_ALIKED_MODEL / VISION_LG_MODEL / VISION_MAP / VISION_EVAL_DIR
```

`vision_traj_eval` 缺省读 `straight_forward.ffvmap`（boxes 旧库图），warehouse 链用
`VISION_MAP=apps/planner/maps/wh_corridor.ffvmap` 覆盖。

## 3. 发目标点 — 机器人导航

先按 §2 起全链路，但终端 7 用无脚本模式（悬停等 planner 参考）：

```bash
uv run firefly-sim --no-trace
```

规划器悬停在 `--start`（缺省 `2 0 1`），等待 `Firefly/Goal`：

```bash
# 语法：uv run firefly-goal X Y Z（warehouse 地图系，米；走廊 y∈[-4,4] 净空）
uv run firefly-goal 30 0 1        # 走廊直线可达点
uv run firefly-goal 22 0 1.5      # 走廊中段高点

# 动态重目标：飞行中可连续发送，最新一条生效（10Hz 轮询，1s 内重算全局路径）
uv run firefly-goal 35 0 1
```

坐标需落在 `warehouse.ffmap` 范围内（`ORIGIN + DIMS*RESOLUTION`：
x∈[-2,48]、y∈[-9,9]、z∈[0,5.2]）。不可达目标会被 `PlannerManager::set_goal`
拒绝并 `log::warn`。

可选的 `planner` 启动时指定初始目标：

```bash
./target/release/planner --map apps/planner/maps/warehouse.ffmap --goal 30 0 1 --start 2 0 1
```

强制急停（对照官方 `mandatory_stop`）：

```bash
./target/release/planner --mandatory-stop   # 向 Firefly/MandatoryStop 发单帧空消息后退出
```

## 4. 数据记录与回读（有且只有 rrd 一种方式）

交付物 = `firefly-viz --save` 落的盘（`logs/` 下，gitignored）。bench 的 stdout 表格
和 `logs/bench/*.json` 只是采样中间量，不做结论载体；bench 自起的 `rerun --save`
收不到数据（`Firefly/Viz` 只有 `firefly-viz` 在消费），落的是空文件。

rrd 实体（`sim_time` 时间轴；`logs/*` 在发布端尚无 sim 时钟时回落 `wall_time` 轴）：

* `vio/odom` (橙) / `gt/pose` (蓝) — VIO 估计 vs 真值位姿；`vio/traj` / `gt/traj` — 轨迹线
* `corr/odom` (绿) / `corr/traj` — GICP+视觉融合后位姿与轨迹（与 vio/gt 同 10Hz，可逐点对比）；`corr/debug/gate`（`[metric,limit,applied,accepted]` 本次判决门与注入量）/ `corr/debug/drift`（`[dx,dy,dz]` 判决时刻累计漂移）— 每次融合尝试即发的诊断标量
* `vio/debug/track_length` / `db_size` / `track_avg_len` — 前端健康度
* `plan/map`（启动一次性）+ `plan/perceived`（深度感知在线占据）+ `plan/global_path`（绿）、`plan/local_traj`（蓝+黄速度）、`plan/planes`、`plan/drone`；`plan/motions` 仅动态地图有 MOTION 段时出现
* `logs/<tag>` (`sim`/`vio`/`gicp`/`aliked`/`lightglue`/`planner`) — 全进程日志聚合为 `TextLog`，可检索

回读：

```bash
rerun logs/wh_run.rrd                                        # 回放
rerun rrd stats logs/wh_run.rrd                             # 实体/行数速览
rerun rrd print --entity vio/odom --entity gt/pose -vvv     # 数值快查
uv run python .agents/skills/rerun/scripts/read_poses.py logs/wh_run.rrd [实体...]
# 输出逐行 sim_time_ns x y z qx qy qz qw（缺省 gt/pose + vio/odom），可管道给 awk/python 算 ATE
```

读法细节见 `.agents/skills/rerun`（entity 前导 `/`、sim_time 转 int64、无 footer 时只用 `stream()`）。

## 5. 换配置（任意 `--config`）

```bash
./target/release/vio --config configs/vio.toml
./target/release/gicp --config configs/gicp.toml --map apps/planner/maps/warehouse.ffmap
./target/release/planner --config configs/planner.toml --map apps/planner/maps/warehouse.ffmap
```

## 6. bench 套件（精度快照，非交付）

```bash
# 全套件：16 轨迹 × 34s，每轮自动起 gicp/aliked/lightglue 并采 corrected 对照
uv run python bench/bench_suite.py --duration 34 --turns 1 --tag fleet5
# 单轨迹（warehouse）：bench/bench_vio.py 暂无 --script 透参，需手动对齐轨迹与库图/地图
# 旧语义（仅 sim+vio）：加 --no-fleet；再起 planner：加 --with-planner
# 多轮统计：--turns 3（约 45 分钟）
```

bench 经 IPC 直采 GT/odom 算 ATE/RPE 打 stdout + `logs/bench/*.json`（含 `metrics` +
`corrected_metrics` + `ΔRMSE`）。注意两条纪律：① json 是中间量，结论只认 §4 的 rrd；
② bench 不起 `firefly-viz`，其 per-turn rrd 为空，要录像用 §2 的 7 终端 + `--save`。
另：仿真噪声（IMU/深度 `np.random`）与任务时钟 latch 时刻（vio 就绪快慢）都未播种，
单轮数值会抖，对比看多轮均值。

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

1. `uv run firefly-viz --save logs/wh_run.rrd` 最先起 → 其余 6 进程按 `vio → gicp → aliked → lightglue → planner → sim` 顺序启动。各进程日志均出现 `日志聚合已挂载（tag=...）` + `已订阅 ...` / `已打开话题`；`firefly-viz` 出现 `已订阅 Firefly/Viz + Firefly/Log`。（Rust 日志需 `RUST_LOG=info`，缺省只打 error。）
2. `uv run firefly-goal 30 0 1` → `planner` 日志 `收到新目标` + `目标更新 ... 重新规划中`，rrd 中 `plan/local_traj` 出现。
3. `sim` 日志 `收到参考 t=... pos=(...)` 且无人机开始移动。
4. `Ctrl+C` 后 `全部进程已结束` / `优雅退出`，进程组无残留（`ps aux | grep firefly` 为空）。

## 9. 故障速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `planner` 日志 `目标 (...) 不可达` | 目标点在占据体素内或超出地图 | 换 warehouse 范围内空地点，如 `30 0 1` |
| `GICP矫正接受` 迟迟不出现 | 空地图或点云 `<30` 点，或 `chi2` 拒收 | 检查 `--map` 路径（warehouse 链用 `warehouse.ffmap`），近处对墙增加特征 |
| `odom 订阅不可用` | `vio` 未启动或 iceoryx2 幽灵端口 | 重启全栈并清理 `/tmp/iceoryx2` |
| `深度/里程计丢失！进入急停` | 深度或 odom 超时 `>1.0s` | 检查 `sim` 是否卡死，`gicp/planner` 是否正常消费 |
| `sim --script` 15s 超时退出 | vio 未起或未 ready（互锁等不到 `is_initialized`） | 先起 vio，待 `VIO 就绪` 日志再起 sim |
| `lightglue` 无观测输出 | ORT 模型未加载完或库图路径错 | 待 `lightglue 会话就绪` 日志；warehouse 链检查 `--map wh_corridor.ffvmap` |
| `IMU 断流 >200ms` 刷屏 | `debug` 构建算力不足 | 换 `release` 构建 |
| `视觉观测无 odom 内插` 跳过 | 观测先到、odom 后到（事件驱动竞态） | 正常现象，`gicp` 待融合队列会自动追上；频繁出现则检查 vio 是否掉帧 |
| `日志聚合挂载失败（降级纯 stderr）` | `firefly-viz` 未先启动（`Firefly/Log` 服务上限未预创建） | 先起 `firefly-viz`，再起其余 6 进程；重排顺序后重启全栈 |
| Rust 进程"没日志" | 缺省级别只打 error | `export RUST_LOG=info` 后重启该进程 |

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
