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
| `crates/firefly-imu` | 姿态/陀螺零偏 ESKF、静止初始化、加计 NIS 与相关 VIO 外援 |
| `crates/firefly-{pubsub,error,observability,render}` | IPC 消息、错误、日志/trace、共享 Bevy 场景与光照 |
| `apps/` | Rust 进程 `vio`、`fc`、`planner`、`localization`、`slam`、`aliked`、`lightglue`、`render`、`quad`、`ffctl`；Python 进程 `firefly-sim`、`firefly-viz` |
| `packages/` | Python 库 `firefly-mujoco` 与 CAD 资产处理 `firefly-cad` |
| `configs/` | 每应用一份 TOML，缺键回落代码默认值，缺文件报错 |
| `tests/`、`docs/` | 评测指标、启动检查与运行/架构说明 |

## 开始开发

```bash
uv sync --all-packages --extra test
cargo build
cargo test --all-targets
```

实际运行须准备 RMUC2026 资产、图形会话和 release 构建，按开发指南检查宿主机、
启动与清理进程。CI 绿色不代表真实进程悬停、22m 往返或回环验收通过。
