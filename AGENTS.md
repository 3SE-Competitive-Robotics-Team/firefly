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
- **进程必须优雅退出**（Ctrl-C → `node.wait` 返回 Err → 端口 Drop）：
  硬杀（pkill -9/SIGKILL）会留下孤儿内核 shm 对象与幽灵端口注册——
  后续订阅端会连上死端口的残留连接收不到任何数据，且幽灵占满
  `max_publishers` 槽位后新发布器直接创建失败。排障清理：杀干净所有
  进程后 `rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes
  /private/tmp/iox2*.shm_state`（macOS；须在进程全死后执行）。

## VIO 调试状态（2026-08）

已修复：sim 轨迹回卷跳变致物理发散（改闭合周期轨迹 + NaN 守卫）、外参
四元数 Hamilton/JPL 约定混用（相机"侧装"、立体基线纵向化→视觉更新全灭，
改用项目 `rot_2_quat`）、SLAM 批量更新死循环（while 内 extend 回填，改为
drain 消费）、IMU 噪声 Q 与 sim 实际注入失配（差 ~1e4 倍致滤波器过信
IMU）、GT 初始化未重写 IMU 协方差块（σ_ba=1mm/s² 零偏不可观测，对照
C++ initialize_with_gt 补 σ=0.02 先验）、场景纹理贫乏（地面棋盘 7m→2m、
立柱贴棋盘材质）、**H_x 列偏移致命缺陷**——`get_feature_jacobian_full`
的 `add_var` 误用变量序号当列偏移，多变量 H_x 列互相覆盖，所有 MSCKF/SLAM
更新自移植以来一直在用坏雅可比（FD 数值验证暴露：修复前最大失配 346，
修复后相对误差 3.5e-4）。合成端到端测试
（`firefly-vio/tests/synthetic_e2e.rs`）零偏下收敛到毫米级；
`jacobian_fd_check.rs` 为雅可比有限差分回归测试（勿删）。

已知遗留（`synthetic_pure_msckf_with_bias`，#[ignore]）：注入加速度计
零偏后仍发散（列偏移修复后从 ~40m 改善到 ~5.9m），ba 学不到。根因分析：
max_clone_size=11（1.1s 窗口）内加速度零偏可观测性弱，且合成测试曾以
单目模式运行（TrackKlt 第 4 参应为 true）掩盖了立体约束。下一步：①检查
`single_gaussnewton` 基线计算（C++ 用 QR 零空间投影 Q.block(0,1,3,2)，
我们用 xy().norm() 近似——已修复，改用闭式垂直平面投影）；②验证 SLAM
特征初始化（Givens 段）与 UpdaterSLAM 更新；③考虑增大 max_clone_size
或依赖 SLAM 特征提供长基线约束使 ba 可观。前端 LK/FAST/NMS 已用
`grid_probe.rs` 回归锚点验证健康。

现场（MuJoCo 闭环）现状：apps/vio 暂禁用 SLAM 特征
（`max_slam_features=0`，消融实验确认 SLAM 更新链路开启后现场发散更快
——SLAM 初始化/Givens 路径待 FD 验证）。纯 MSCKF 含偏+真实纹理下仍
公里级漂移，但 vio 全程存活不再崩溃/卡死（奇异三角化已优雅降级、死循环
已修、H_x 列偏移修复后更新数学已被 FD 验证正确）。合成与现场的共同瓶颈：
跟踪质量与零偏可观测性。

## 运行（MuJoCo 双语言闭环）

双语言闭环三个进程，iceoryx2 IPC（`Firefly/*` 话题）通信，fastrace
trace 跨进程续接（传感器→vio→demo→参考 单周期 trace）：

```
Python sim（MuJoCo 物理 + 传感器发布）→ vio（MSCKF 位姿估计）→ demo（重规划）
  └──────────── 回传参考 Firefly/Reference（PD 闭环）──────────────┘
```

推荐一键启动（后台拉起三进程 + viewer，Ctrl-C 清理）：
```bash
scripts/run_firefly.sh            # 起 viewer + sim + vio + demo
scripts/run_firefly.sh --save task.rrd   # 捕获录制（viewer 后台落盘；--no-viewer 时不产 rrd）
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

## Log / Debug（rerun rrd）

- **debug 数据一律进 rrd，禁止文本日志**：`firefly-vio.log` /
  `firefly-sim.log` 这类文件是违禁品。
- 分工：log 宏（logforth）只做进程级诊断（启动、错误、关键事件）；
  可结构化数据（位姿、图像、标量、轨迹、中间状态）进 rrd——
  viewer 可回放，CLI/RrdReader 可检索。
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
- `RUST_LOG=info cargo run -p firefly-planner --example demo`
