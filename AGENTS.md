# AGENTS.md

## 规范

- 强迫症：每次改动后审视全仓库一致性，不留冗余、死代码、临时文件。
- 新 crate：`cargo new crates/<name> --lib --edition 2024`（自动纳入 workspace）。
- DDD 拆 crate（firefly-*），依赖自上而下，领域层不依赖应用层。
- 错误用 firefly-error：kind/status 分类，模块边界 .with_context。
- 可观测性：关键函数 `#[fastrace::trace]`，日志用 log 宏，禁 println!。
- 依赖统一在根 `[workspace.dependencies]`。

## VIO（firefly-vio*）约定

- **不接真实驱动**：`apps/vio` 固定接入 `MuJoCo` 物理环境（iceoryx2 订阅
  IMU + 双目灰度，跑完整 MSCKF 视觉闭环），不引入 realsense/串口驱动。
- **不做配置文件系统**：相机内参/外参、IMU 噪声、时间偏移等标定一律硬编码
  在代码里（`VioManagerOptions::default()` / `InitOptions::default()` 即为
  事实配置源），不引入 YAML/JSON/serde 解析。
- 新增标定类数值参数时，直接改对应 `*Options` 的默认值并在 doc 注释标注
  单位与来源。

## 运行（MuJoCo 双语言闭环）

双语言闭环三个进程，iceoryx2 IPC（`Firefly/*` 话题）通信，fastrace
trace 跨进程续接（传感器→vio→demo→参考 单周期 trace）：

```
Python sim（MuJoCo 物理 + 传感器发布）→ vio（MSCKF 位姿估计）→ demo（重规划）
  └──────────── 回传参考 Firefly/Reference（PD 闭环）──────────────┘
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
#    深度感知建图（MuJoCo 闭环下省略 --map），发布参考回传；
#    规划结果写入同一 viewer（与 vio 共用 sim_time 时间轴）
cargo run -p firefly-demo
```

rerun 可视化约定：`sensor/stereo_left|right`、`sensor/depth` 为传感器原图，
`vio/odom` 为估计位姿（3D 变换），规划/地图/轨迹由 demo 写入——多进程
共用 `sim_time` 时间轴（仿真秒），回放时跨进程数据按同一时钟对齐。

独立运行（不依赖闭环）：

```bash
cargo run -p firefly-demo -- --map apps/firefly-demo/maps/gate.ffmap  # 静态地图
```

## 构建

- Rust：`cargo build`（workspace 含 `apps/vio`、`apps/firefly-demo`，排除 `apps/firefly-sim`）。
- Python：`uv sync`（根 workspace 统一管理 `firefly-mujoco` / `firefly-sim`，
  依赖与脚本见各自 `pyproject.toml`）。

## 性能

- 结论只认 trace 实测（ConsoleReporter span 树），不靠读代码推断。
- 主循环建 root span（`Span::root` + `set_local_parent`），结束 `flush()`。
- 热路径禁 `#[logcall]`（无条件格式化大对象，开销爆炸），只留 trace。
- 输出先写清晰，不写解析脚本。

## 验证

- `cargo test`
- `cargo test --release -p firefly-planner --test random_map_benchmark -- --ignored`
- `RUST_LOG=info cargo run -p firefly-planner --example demo`
