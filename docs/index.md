# Firefly 文档

Firefly 是 RoboMaster 四旋翼自主飞行系统。开发者可以在 MuJoCo 中验证任务，再到板端联调。

## 开发与设计

- [开发指南](development.md)给出构建、部署、仿真和测试命令。
- [系统架构](architecture.md)用图说明传感器数据如何进入定位、规划和飞控，以及各进程如何执行一次任务。
- [原生占据地图管理设计](map-management.md)说明比赛场地先验、在线障碍和规划器查询的拟议契约。
- [硬件规格](hw/spec.md)列出计算板、飞控电调板、相机和供电接口。

## 运行与测评

- [可观测性](observability.md)列出 IPC topic、观测字段和采集预算。
- [Mimir 录制规则](mimir.md)说明如何找到一次运行的日志、读取 RRD 分段和核对归档容量。
- [RootSLAM 公开数据集测评](rootslam.md#public-dataset-evaluation)给出数据集准备、标定、运行与评分步骤。

## 模块文档

| 主题 | 仓库原文 |
| --- | --- |
| 仿真与场地 | [MuJoCo 物理仿真](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/apps/firefly/firefly-sim/README.md)、[CAD 场地资产](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/packages/firefly-cad/README.md)、[quad 交互演示](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/apps/firefly/quad/README.md) |
| 定位与建图 | [相机标定](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/crates/firefly-calibration/README.md)、[Basalt](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/crates/firefly-basalt/README.md)、[OpenGV](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/crates/firefly-opengv/README.md)、[RootSLAM 建图](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/crates/firefly-rootslam-mapping/README.md) |
| 录制 | [Mimir 应用](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/apps/firefly/mimir/README.md) |

各组件的许可证、文件来源和版权声明见[许可证索引](licenses/README.md)。
