# firefly

自主无人机的 Rust / Python monorepo：视觉惯性里程计、在线路标回环、深度建图、
轨迹规划、四旋翼飞控、MuJoCo 仿真与 Rerun 可视化。

RootSLAM 默认只提供 VIO；planner 使用机体里程计、深度和相机外参，独立执行 EGO 局部建图与规划。
目标、占据地图和轨迹统一在 odom 系，不依赖已知世界起点、预建地图或神经网络权重。
ALIKED/LightGlue 库图定位保留为独立可选工具，不接入局部规划链路。
真值只供传感器生成、评测和可视化，不参与估计或控制。
稀疏地图优化尚未反馈到在线导航；设备同步、标定、实时性和飞行能力须分别验收。

## 文档

- [开发指南](docs/development.md)：实现约定、构建运行、资产、诊断与测试入口。
- [系统架构](docs/architecture.md)：数据流、坐标、VIO、姿态融合、回环与规划边界。
- [硬件规格](docs/hw/spec.md)：板卡算力、接口与项目约束。
- [协作原则](AGENTS.md)：agent 的工作约定。

参数、测试清单与门槛以代码和配置为准；运行证据保存在 `logs/`。

## 目录

| 位置 | 内容 |
|---|---|
| `apps/rootslam` | RootSLAM 在线双目/IMU、稀疏建图及离线数据集入口 |
| `crates/firefly-{basalt,calibration,opengv,rootslam-mapping}` | 局部 VIO、标定、几何求解与稀疏地图优化 |
| `crates/firefly-ego-map` | EGO 局部概率占据、射线融合与障碍膨胀 |
| `crates/firefly-{map,search,trajectory,optimize,cost,planner}` | 占据地图、动态障碍体素化、A*、MINCO、L-BFGS、代价与重规划 |
| `crates/firefly-cost/src/{swarm,formation}.rs` | 库级集群间距与队形代价；应用集群通信未接入 |
| `crates/firefly-{localization,vision-map,vision-match}` | 定位融合、视觉库图与匹配 |
| `crates/firefly-flight` | 飞行模式、安全条件、姿态/位置控制与电机分配 |
| `crates/firefly-imu` | 独立备用姿态/陀螺零偏 ESKF、静止初始化与加计 NIS |
| `crates/firefly-{base,config}` | 坐标/刚体变换、可选 TOML 配置 |
| `crates/firefly-{pubsub,error,observability,render}` | IPC 消息、错误、日志/trace、共享 Bevy 场景与光照 |
| `configs/` | 每应用一份 TOML，缺键回落代码默认值 |
| `tests/`、`docs/` | 评测指标、启动检查与运行/架构说明 |
| `apps/`、`packages/` | Rust/Python 进程与 Python 库，逐个职责见[应用与职责](#应用与职责) |

## 应用与职责

### Rust 进程（`apps/`）

| 应用 | 职责 |
|---|---|
| `mimir` | RRD 录制：列式数值诊断、左右目无损灰度图、事件与完整性检查 |
| `fc` | 1kHz 飞控环：飞行模式与安全条件、姿态/位置控制、电机分配 |
| `planner` | 独占深度融合与局部避障地图、全局/局部规划、参考发布与可视化 |
| `localization` | 可选外部库图 ESKF：时间配对、map←odom 修正与导航发布，要求显式启动对齐 |
| `firefly-rootslam` | 默认双目/IMU VIO、连续机体里程计与预测姿态；默认关闭建图与回环 |
| `aliked` | ALIKED-N16 特征提取：在线发布特征，`--build-map` 离线建 `ffvmap` 库图 |
| `lightglue` | ALIKED/LightGlue 离线库图定位：`--map <地图.ffvmap>` 解算视觉位姿 |
| `render` | 视觉传感器进程：订阅真值位姿渲染双目与深度并发布 IPC，`--offline` 供离线建库 |
| `quad` | 独立微无人机第三人称飞行游戏，不接 IPC/VIO |
| `ffctl` | 地面站 CLI：把操作员意图发成 IPC 消息，命令为 `<域> <动词>` 两层 |
| `camera`、`imu`、`motor` | 板载 V4L2 相机、SPI BMI088、sysfs PWM；通过标准 IPC 连接算法进程 |

### Python 进程（`apps/`）

| 应用 | 职责 |
|---|---|
| `firefly-sim` | MuJoCo 物理进程：物理与 IMU 各 1kHz，接收电机推力，无有效控制时暂停 |

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

Rust 应用的 release 构建入口：

```bash
cargo xtask release
```

`--target host`（默认）构建桌面应用，`--target board` 在 Apple container 中原生构建
机载进程；应用名单由 `xtask/src/target.rs` 维护。部署、进程管理、场地资产与验收步骤见
[开发指南](docs/development.md#run)。构建与单测通过只覆盖各自验证范围。

## References

- [RootSLAM](https://arxiv.org/abs/2610.15157)
- [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)
- [Basalt](https://gitlab.com/VladyslavUsenko/basalt)
- [OpenGV: A Unified and Generalized Approach to Calibrated Geometric Vision](https://doi.org/10.1109/ICRA.2014.6907291)
- [Ceres Solver](http://ceres-solver.org/)
- [Eigen](https://eigen.tuxfamily.org/)
