# firefly

Firefly 是用 Rust 和 Python 开发的 RoboMaster 四旋翼自主飞行系统，涵盖视觉惯性定位、轨迹规划与飞行控制，支持在 MuJoCo 中仿真。

机载 Rust 服务 `firefly-daemon` 随系统启动，管理一次上电期间的任务和架次。板上应用或 `ffctl mission` 可以提交任务、查询回执和请求取消。板端部署与 SIL 验收入口见[开发指南](docs/development.md#板上任务服务)。

## 应用目录

Firefly 自有应用位于 [`apps/firefly/`](apps/firefly)。[`apps/third-party/betaflight`](apps/third-party/betaflight) 是 Matrix 飞控固件，[`apps/tools/se3-mono`](apps/tools/se3-mono) 是 3SE 的独立工具仓库。运行 `git submodule update --init` 可取回这两个仓库在当前版本对应的提交。

## 文档

- [系统架构图](docs/architecture.md)
- [开发指南](docs/development.md)
- [硬件规格](docs/hw/spec.md)
- [RootSLAM 公开数据集测评](apps/firefly/rootslam/README.md)
- [可观测性](docs/observability.md)
- [Mimir 录制规则](docs/mimir.md)
- [协作原则](AGENTS.md)
- [许可证与第三方声明](docs/licenses/README.md)
