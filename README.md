# Firefly

Firefly 是 RoboMaster 四旋翼的视觉定位、规划与控制软件。在实机上，Orange Pi 的 `pilot` 通过 USB 接收 Matrix 的 BMI270 样本，RootSLAM 用这些样本和双目图像估计运动。Matrix 负责自稳和电机输出。Mac 仿真用 `fc` 接收相同的四通道杆量，可在 MuJoCo 中验收完整任务。

[浏览或搜索 Firefly 中文文档站](https://3se-competitive-robotics-team.github.io/firefly/)。

## 从哪里开始

| 要做的事 | 入口 |
|---|---|
| 第一次构建，运行 Mac 仿真或 USB 硬件在环任务 | [开发指南](docs/development.md) |
| 连接实板，确认 RootSLAM 收到 IMU | [Matrix 两分钟台架检查](docs/development.md#验证-matrix-imu) |
| 核对板卡批次，构建可刷写的 HEX | [硬件规格](docs/hw/spec.md) · [容器构建命令](apps/third-party/README.md#编译-matrix-bmi270-固件) |
| 查进程职责、坐标系、USB 模式或任务阶段 | [系统架构](docs/architecture.md) |
| 用公开数据集评估定位，或从 STEP 生成仿真场地 | [RootSLAM 测评](apps/firefly/rootslam/README.md) · [场地资产](packages/firefly-cad/README.md) |
| 找一次运行的日志、回放 RRD，或查看 topic 和采样预算 | [运行归档](docs/mimir.md) · [可观测性](docs/observability.md) |

Firefly 应用位于 [apps/firefly](apps/firefly)。首次检出后运行 `git submodule update --init`，取回当前版本固定的 [Betaflight 固件](apps/third-party/betaflight) 和 [se3-mono 工具](apps/tools/se3-mono)。提交改动前可查阅[协作约定](AGENTS.md)及[许可证与第三方声明](docs/licenses/README.md)。
