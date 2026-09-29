# firefly

自主无人机的 Rust / Python monorepo：视觉惯性里程计、深度视觉惯性里程计、
地图定位、轨迹规划、四旋翼飞控、MuJoCo 仿真与 Rerun 可视化。

当前部署以仿真研究与算法验证为主。VIO 和 VOID 从静止传感器观测初始化，
飞控使用 IMU 与里程计反馈。真值仅用于评测与可视化对照，不能提供估计器初始
位置、速度、姿态或飞控反馈。启动时保持机体静止，等待估计就绪后再解锁。

## 系统链路

```text
MuJoCo sim ── IMU / 双目 / 深度 ──→ vio 或 void
     ↑                              │ 里程计
     └──── 四电机推力 ─── fc ←───────┘
                           ↑
                    planner 参考 / ffctl 指令

各进程 ── Firefly/Viz + Firefly/Log ──→ firefly-viz ──→ viewer / logs/*.rrd
```

进程通过 iceoryx2 的 `Firefly/*` 话题通信，消息头携带 fastrace 上下文。
Rust 算法进程不直接写录制文件；`firefly-viz` 统一写入 Rerun。
`sim_time` 对齐消息时间，不能替代空间坐标系的对齐。

## 目录与职责

| 位置 | 内容 |
|---|---|
| `crates/firefly-vio*` | 对照 OpenVINS：类型、KLT 前端、IMU 传播、静态/动态初始化、MSCKF/SLAM 更新 |
| `crates/firefly-void*` | 深度与视觉顺序 ESIKF 更新、体素地图、局部里程计；复用静态 IMU 初始化器 |
| `crates/firefly-voxel-svio` | VIO 路标的体素索引与选点 |
| `crates/firefly-{map,search,trajectory,optimize,cost,obstacle,planner}` | 占据地图、A*、MINCO、L-BFGS、代价、动态障碍与重规划 |
| `crates/firefly-{gicp,localization,vision-map,vision-match}` | 点云配准、定位融合、视觉库图与匹配 |
| `crates/firefly-flight` | 飞行模式、安全条件、姿态/位置控制与电机分配 |
| `crates/firefly-{pubsub,error,observability,render}` | IPC 消息、错误、日志/trace、共享 Bevy 场景与光照 |
| `apps/` | Rust 进程 `vio`、`void`、`fc`、`planner`、`gicp`、`aliked`、`lightglue`、`render`、`quad`、`ffctl`；Python 进程 `firefly-sim`、`firefly-viz` |
| `packages/` | Python 库 `firefly-mujoco` 与 CAD 资产处理 `firefly-cad` |
| `configs/` | 每应用一份 TOML，缺键回落代码默认值，缺文件报错 |
| `bench/`、`docs/`、`thesis/` | 轨迹评测、运行/架构说明与论文材料 |

## 最小闭环

要求 Rust 1.97+、Python 3.12+、uv，以及对应平台的图形运行依赖。
从仓库根安装与构建：

```bash
uv sync
cargo build --release -p vio -p void -p fc -p ffctl
```

分别在终端启动（可视化先启动以接收结构化日志）：

```bash
uv run firefly-viz --save logs/run.rrd
cargo run --release -p vio
cargo run --release -p fc
uv run firefly-sim
```

飞控在未就绪时持续发零推力，使仿真能够推进并产生初始化测量。VIO 使用静止
IMU 窗口与双目视差检查，VOID 使用静止 IMU 窗口。观察就绪日志或里程计的
`is_initialized` 后发送指令：

```bash
./target/release/ffctl fc arm
./target/release/ffctl fc takeoff 1.0
./target/release/ffctl fc land
```

`void` 发布独立的 `Firefly/VoidOdom`，用于里程计对照；当前 `fc` 默认消费
`Firefly/Odometry` / `Firefly/CorrectedOdometry`，不会自动切换到 VOID。
`sim --script` 是估计器评测的运动夹具，其内部真值反馈仅生成被测运动，不能作为
无人机闭环成功的证据。

**坐标约定**：原始里程计在重力对齐的局部系运行，初始位置为零，航向属于规范
自由度。静态场景地图、视觉库图、全局目标需要独立测得的地图对齐关系；不能将
局部里程计直接当成 MuJoCo 世界坐标。VOID 默认不加载仿真世界先验平面。
评测可对轨迹进行固定尺度的航向与平移对齐，变换只作用于评测数据。

更多进程、配置与资源说明见 [运行文档](docs/how_to_run.md) 和
[架构说明](docs/architecture.md)。

## 验证

```bash
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings
cargo test
cargo test --release -p firefly-planner --test random_map_benchmark -- --ignored
uv run --with pytest pytest apps/firefly-sim/tests/ bench/tests/ -q
```

发布构建的无真值启动检查（需 Linux EGL、没有其他闭环进程运行）：

```bash
uv run --no-dev python bench/check_sensor_startup.py
uv run --no-dev python bench/check_sensor_startup.py --estimator void
```

检查会屏蔽仿真的 GroundTruth / PlantState 发布，运行真实传感器、估计器与飞控
进程，录制到 `logs/*_sensor_startup_*.rrd`，并通过 SIGINT 退出。
它验证静止初始化和未解锁零推力，不代表飞行精度验收。

测试包含传感器独立初始化、真值与控制隔离、雅可比/梯度检查及规划/飞控测试。
`synthetic_e2e` 中标记 `ignore` 的场景不属于通过保证；运行与精度结论以对应版本的
录制和评测结果为准。

## 参考实现

- [OpenVINS](https://github.com/rpng/open_vins)：MSCKF 与惯性初始化。
- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)：DIVO 的传播、更新与建图对照。
- [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)：重规划与集群规划。
- [purecv](https://github.com/webarkit/purecv)：视觉前端。
