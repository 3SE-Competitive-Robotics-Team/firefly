# firefly

自主无人机的 Rust / Python monorepo：视觉惯性里程计、在线路标回环、深度建图、
轨迹规划、四旋翼飞控、MuJoCo 仿真与 Rerun 可视化。

默认从空会话地图启动，使用 VIO + 持久路标回环 + 在线深度规划，不依赖已知世界起点、
预建地图或神经网络权重。ALIKED/LightGlue 库图定位和静态地图是显式可选增强。
真值只供传感器生成、评测和可视化，不参与估计或控制。
当前部署用于仿真研究与算法验证，能力边界和飞行结果须分别验收。

## 文档

- [开发指南](docs/development.md)：实现约定、构建运行、资产、诊断与测试入口。
- [系统架构](docs/architecture.md)：数据流、坐标、VIO、姿态融合、回环与规划边界。
- [硬件规格](docs/hw/spec.md)：板卡算力、接口与项目约束。
- [协作原则](AGENTS.md)：agent 的工作约定。

参数、测试清单与门槛以代码和配置为准；运行证据保存在 `logs/`。

## 目录

| 位置 | 内容 |
|---|---|
| `crates/firefly-vio*` | 对照 OpenVINS：类型、KLT 前端、IMU 传播、静态/动态初始化、MSCKF/SLAM 更新 |
| `crates/firefly-voxel-svio` | VIO 路标的体素索引与选点 |
| `crates/firefly-{map,search,trajectory,optimize,cost,obstacle,planner}` | 占据地图、A*、MINCO、L-BFGS、代价、动态障碍与重规划 |
| `crates/firefly-{localization,vision-map,vision-match}` | 定位融合、视觉库图与匹配 |
| `crates/firefly-flight` | 飞行模式、安全条件、姿态/位置控制与电机分配 |
| `crates/firefly-imu` | 独立备用姿态/陀螺零偏 ESKF、静止初始化与加计 NIS |
| `crates/firefly-{base,config,landmarks}` | 坐标/刚体变换、可选 TOML 配置、有界会话路标地图 |
| `crates/firefly-{pubsub,error,observability,render}` | IPC 消息、错误、日志/trace、共享 Bevy 场景与光照 |
| `configs/` | 每应用一份 TOML，缺键回落代码默认值 |
| `tests/`、`docs/` | 评测指标、启动检查与运行/架构说明 |
| `apps/`、`packages/` | Rust/Python 进程与 Python 库，逐个职责见[应用与职责](#应用与职责) |

## 应用与职责

### Rust 进程（`apps/`）

| 应用 | 职责 |
|---|---|
| `vio` | MSCKF VIO 估计器：订阅 IMU 与双目灰度，100Hz 发布 `Firefly/Odometry` |
| `fc` | 1kHz 飞控环：飞行模式与安全条件、姿态/位置控制、电机分配 |
| `planner` | 任务执行：全局/局部规划、参考发布与可视化，状态源为已对齐地图系 |
| `localization` | 视觉定位融合：VIO 时间配对、map←odom 修正、发布地图里程计 |
| `slam` | 关键帧驱动的有界会话视觉地图与在线路标回环 |
| `aliked` | ALIKED-N16 特征提取：在线发布特征，`--build-map` 离线建 `ffvmap` 库图 |
| `lightglue` | ALIKED/LightGlue 离线库图定位：`--map <地图.ffvmap>` 解算视觉位姿 |
| `render` | 视觉传感器进程：订阅真值位姿渲染双目与深度并发布 IPC，`--offline` 供离线建库 |
| `quad` | 独立微无人机第三人称飞行游戏，不接 IPC/VIO |
| `ffctl` | 地面站 CLI：把操作员意图发成 IPC 消息，命令为 `<域> <动词>` 两层 |

### Python 进程（`apps/`）

| 应用 | 职责 |
|---|---|
| `firefly-sim` | MuJoCo 物理进程：物理与 IMU 各 1kHz，接收电机推力，无有效控制时暂停 |
| `firefly-viz` | 独立 Rerun 调试进程：标量、H.265 左目视频、采样与事件无损窗口 |

### Python 库（`packages/`）

| 包 | 职责 |
|---|---|
| `firefly-mujoco` | MuJoCo 物理库：RMUC 无人机场景与 IMU 提取 |
| `firefly-cad` | RMUC 场地 STEP→glTF 资产管线：普查/三角化/分组/碰撞聚合 |
| `firefly-evaluation` | 任务验收编排、进程生命周期、RRD 证据评分与报告 |
| `firefly-workspace` | 工作区定位与文件内容指纹，仅依赖标准库 |

## 开始开发

```bash
uv sync --all-packages --extra test
cargo build
cargo test --all-targets
```

Rust 应用的 release 构建统一走 xtask（`.cargo/config.toml` 已把 `cargo xtask` 指向
`xtask` 包）：

```bash
cargo xtask build-all-release
```

`build-all-release` 依次为全部 Rust 应用执行 `cargo build --release -p <app>`。应用名单只在
`xtask/src/main.rs` 维护，不要在脚本、CI 或文档里复制第二份；新增 xtask 子命令时同步
`usage` 文本与本说明。实际运行须准备 RMUC 场地资产、图形会话和 release 构建，按开发指南
检查宿主机、启动与清理进程。CI 绿色不代表真实进程悬停、22m 往返或回环验收通过。
