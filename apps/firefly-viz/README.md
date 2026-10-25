# firefly-viz

独立 Python 调试数据进程：订阅诊断与双目图像 IPC，保存标量与左右目 H.265 视频，
关键点只叠加在对应相机的录制视图内。Viewer 和 RRD 共用历史录制流，
接收、写入、编码队列有界；叠加信息不烧进视频。

配置入口为 [`configs/viz.toml`](../../configs/viz.toml)。启动、依赖、信号触发与验证见
[开发指南](../../docs/development.md#diagnostics)，数据模型与职责见
[系统架构](../../docs/architecture.md#诊断记录与展示)。
