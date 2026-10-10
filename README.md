# firefly

自主无人机的 Rust / Python monorepo：视觉惯性里程计、视觉重定位、
地图定位、轨迹规划、四旋翼飞控、MuJoCo 仿真与 Rerun 可视化。

当前部署以仿真研究与算法验证为主。VIO 从静止传感器观测初始化，
飞控使用本地姿态 ESKF 与里程计反馈。真值用于仿真传感器生成、评测与可视化对照，不能提供估计器初始
位置、速度、姿态或飞控反馈。启动时保持机体静止，等待估计就绪后再解锁。

## 系统链路

```text
MuJoCo sim ── IMU ──────────────→ vio
     └── 仿真位姿 → render ── 双目 ─→ │
     ↑                              │ 里程计
     └──── 四电机推力 ─── fc ←───────┘
                           ↑
                    planner 参考 / ffctl 指令

各进程 ── Firefly/Viz + Firefly/Log ──→ firefly-viz ──→ viewer / logs/*.rrd
```

进程通过 iceoryx2 的 `Firefly/*` 话题通信，消息头携带 fastrace 上下文。
Rust 算法进程不直接写录制文件；`firefly-viz` 统一写入 Rerun。
`sim_time` 对齐消息时间，不能替代空间坐标系的对齐。
VIO 在相机时刻更新滤波器，使用 IMU 预测输出 100 Hz 里程计；消息时间为
状态对应的 IMU 时钟，位置与速度均在局部世界系。初始化未完成或 IMU
未覆盖目标时刻时不发布里程计。真值可视化保留真值消息自身的采样时间。

FC 的独立 IMU 线程通过 `firefly-imu` 估计姿态和陀螺零偏，角速度反馈使用去偏低通陀螺。
VIO 通过 `Firefly/VioAttitudeAid` 提供带联合协方差的外援；共享 IMU 的相关性用 CI 处理，
加计校正限于上锁静止期。接口、方程与能力限制见 [本地姿态估计](docs/imu.md)。

## 视觉定位与回环路线

采用 **VIO + ALIKED + LightGlue + 回环**：VIO 提供局部运动估计，ALIKED
提取图像特征，LightGlue 匹配候选关键帧，几何验证产生回环约束。

离线库图 PnP 通过 `Firefly/PoseObservation` 提供地图锚点；在线关键帧通过
外观检索、RGB-D 几何验证和连续三帧确认产生 `Firefly/LoopConstraint`。
`localization` 默认联合优化 VIO 相对边、地图锚点和回环边，发布
`Firefly/CorrectedOdometry`；飞控仍使用连续的原始 odom 反馈。

左目路标使用跨相机注册深度，深度另用于近场障碍感知。职责、阈值、参考源码和
验收边界见 [在线回环](docs/loop_closure.md)。在线回环的模型集成测试与实际飞行
回环验收分别报告，不以库图定位成功代替重访验证。

## 目录与职责

| 位置 | 内容 |
|---|---|
| `crates/firefly-vio*` | 对照 OpenVINS：类型、KLT 前端、IMU 传播、静态/动态初始化、MSCKF/SLAM 更新 |
| `crates/firefly-voxel-svio` | VIO 路标的体素索引与选点 |
| `crates/firefly-{map,search,trajectory,optimize,cost,obstacle,planner}` | 占据地图、A*、MINCO、L-BFGS、代价、动态障碍与重规划 |
| `crates/firefly-{localization,vision-map,vision-match}` | 定位融合、视觉库图与匹配 |
| `crates/firefly-flight` | 飞行模式、安全条件、姿态/位置控制与电机分配 |
| `crates/firefly-imu` | 姿态/陀螺零偏 ESKF、静止初始化、加计 NIS 与相关 VIO 外援 |
| `crates/firefly-{pubsub,error,observability,render}` | IPC 消息、错误、日志/trace、共享 Bevy 场景与光照 |
| `apps/` | Rust 进程 `vio`、`fc`、`planner`、`localization`、`aliked`、`lightglue`、`render`、`quad`、`ffctl`；Python 进程 `firefly-sim`、`firefly-viz` |
| `packages/` | Python 库 `firefly-mujoco` 与 CAD 资产处理 `firefly-cad` |
| `configs/` | 每应用一份 TOML，缺键回落代码默认值，缺文件报错 |
| `tests/`、`docs/` | 评测指标、启动检查与运行/架构说明 |

## 场地资产管线

```bash
uv run --all-packages --extra test python scripts/prepare_rmuc.py /path/to/RMUC2026_V2.0.0.stp
```

该入口生成视觉/碰撞/规划地图、执行几何与物理验收，再进行离线采集和系统测试。
阶段状态写入 `models/rmuc2026/preparation_report.json`；缺权重、失败和未执行阶段
明确标记，完整命令在未全部通过时返回非零。只处理资产可加 `--assets-only`。
详细契约见 [CAD 管线](packages/firefly-cad/README.md)。

视觉库按场地、标定和成像外观契约冻结；在线深度噪声调整与普通重编译不触发重建。
资产质量与在线飞行必须分别验收，当前证据见 [视觉资产验收](docs/visual_asset_acceptance.md)。

已有场地和 release 二进制时，自动任务验收可单独执行：

```bash
uv run --all-packages --extra test python scripts/accept_rmuc.py
```

执行初始化、起飞、悬停、往返航点与两种进程失联注入；评分使用记录读回的
定位/跟踪误差、逐物理步碰撞计数及退出状态。每次在 `logs/acceptance/<运行ID>/`
生成 RRD、JSON 与 HTML 报告；任何未完成或失败均返回非零，详见 [验收契约](docs/testing.md)。

## 最小闭环

要求 Rust 1.97+、Python 3.12+、uv，以及对应平台的图形运行依赖。
仅支持 RMUC2026，必须提供 `models/rmuc2026/field.glb` 与
`models/rmuc2026/rmuc2026_collision.json`；缺资产直接报错。
MuJoCo 负责物理与 IMU，Bevy `render` 负责双目与深度。
从仓库根安装与构建：

```bash
uv sync
cargo build --release -p render -p vio -p fc -p ffctl
```

分别在终端启动（可视化先启动以接收结构化日志）：

```bash
uv run firefly-viz --save logs/run.rrd
cargo run --release -p render
cargo run --release -p vio
cargo run --release -p fc
uv run firefly-sim
```

飞控在未就绪时持续发零推力，使仿真能够推进并产生初始化测量。VIO 使用静止
IMU 窗口与双目视差检查。观察就绪日志或里程计的
`is_initialized` 后发送指令：

```bash
./target/release/ffctl fc arm
./target/release/ffctl fc takeoff 1.0
./target/release/ffctl fc land
```

**坐标约定**：原始里程计在重力对齐的局部系运行，初始位置为零，航向属于规范
自由度。地图对齐由 `configs/localization.toml [origin]` 的固定启动先验建立，并由定位修正；不能将
局部里程计直接当成 MuJoCo 世界坐标。
评测可对轨迹进行固定尺度的航向与平移对齐，变换只作用于评测数据。

规划器接收 `Firefly/CorrectedOdometry` 与 `Firefly/LocalizationStatus`，要求地图对齐且视觉质量可用。
等待首个有效状态期间不发布参考；状态/质量流失联 500ms 或定位质量退化后停止发布并锁存，
恢复定位后须重启 planner。不会以轨迹参考代替实测位置。

坐标树、飞控边界与融合公式见 [坐标契约](docs/frames.md)。
流程审查与当前限制见 [可靠性说明](docs/reliability.md)。
更多进程、配置与资源说明见 [运行文档](docs/how_to_run.md) 和
[架构说明](docs/architecture.md)。

## 验证

```bash
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings
cargo test
cargo test -p firefly-planner --test planner_contracts
uv run --all-packages --extra test pytest apps/firefly-sim/tests/ tests/evaluation/ tests/system/ -q -p no:cacheprovider
```

发布构建的传感器启动检查（需 RMUC 资产与图形会话、没有其他闭环进程）：

```bash
FIREFLY_RUN_SENSOR_STARTUP=1 uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py -q
```

检查屏蔽 PlantState 发布，保留供 render 合成图像的 GroundTruth，运行 RMUC
传感器、估计器与飞控进程，录制到 `logs/*_sensor_startup_*.rrd`，并通过 SIGINT 退出。
它验证静止初始化和未解锁零推力，不代表飞行精度验收。

测试设计与验收边界见 [测试契约](docs/testing.md)。

测试包含传感器独立初始化、真值与控制隔离、雅可比/梯度检查及规划/飞控测试。
`synthetic_e2e` 中标记 `ignore` 的场景不属于通过保证；运行与精度结论以对应版本的
录制和评测结果为准。

## 参考实现

- [OpenVINS](https://github.com/rpng/open_vins)：MSCKF 与惯性初始化。
- [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)：重规划与集群规划。
- [purecv](https://github.com/webarkit/purecv)：视觉前端。

运动估计与跟踪验收的已知限制和复测结果见 [VIO 与跟踪审查](docs/vio_tracking_review.md)。
