# firefly-okvis-x

OKVIS2-X 的 Rust 实现，单进程承载视觉惯性估计、在线回环和外部米制深度建图。
默认通过 firefly-pubsub / iceoryx2 接收 IMU、双目及独立深度相机，发布里程计、
IMU 时钟上的姿态预测，以及含地图修正版本的原子导航快照。

```bash
cargo run --locked --release -p firefly-okvis-x -- --config configs/okvis2-x.toml
cargo run --locked --release -p firefly-okvis-x -- --help
```

默认标定来自公共 RMUC rig；自己的相机须在 TOML 中覆盖 left/right/depth 的 intrinsics、
extrinsic 及 imu 参数。外参均为 body←sensor，Hamilton xyzw；深度输入单位为米。
默认视觉词典内嵌二进制，`--vocabulary <directory>` 可显式替换。
配置字段、噪声和边界的唯一默认值定义位于 `application/live/config.rs`。

双目按时间配对并等待 IMU 覆盖，IPC loan 释放前复制为算法自有图像；估计、全图优化、
地图融合与传感器接收分别在线程中运行。回环通过共同历史状态锚点更新 map←odom；
局部里程计保持该锚点的坐标连续，导航修正与版本原子发布。姿态协方差来自固定图规范下的
无阻尼边缘协方差及 IMU 噪声传播；无法求得协方差时不伪造姿态置信度。
跟踪丢失、IMU 缺口或后验超时停止预测，重复/倒序输入不刷新测量时间。

可选 ALIKED/LightGlue 外部地图定位使用 `navigation = "external_localization"`，
并显式启动外部定位进程；该模式由外部进程发布地图导航，避免两个修正发布者竞争。
原有离线数据集入口通过 `--dataset` 选择，仍使用完整 VI/地图格式的离线 TOML：

```bash
cargo run --locked --release -p firefly-okvis-x -- --config <offline-config.toml> --dataset <dataset-directory> --output <output-directory>
```

历史源码审计、兼容差异和已知边界保留在 `apps/firefly-okvis-x/README.md`。
SE01 窄窗口、S78 ICP cost-only 及其他既定行为选择保持原处理；地图工作线程的错误向上传播。
当前在线深度后端使用针孔投影，原生窗口、神经网络及硬件 SDK 由外部能力实现。
许可证及版权声明汇总于仓库主 README 末尾，本包 LICENSE 保留。
