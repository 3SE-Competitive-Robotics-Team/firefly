# firefly-viz

独立 Python 调试数据进程：订阅诊断与左目图像 IPC，保存标量、H.265 左目视频及
双目关键点，使用 Rerun Viewer 展示。地图实时 static 预览与持久录制分流，
接收、写入、编码队列有界；叠加信息不烧进视频。

配置入口为 [`configs/viz.toml`](../../configs/viz.toml)。启动、依赖、信号触发与验证见
[开发指南](../../docs/development.md#diagnostics)，数据模型与职责见
[系统架构](../../docs/architecture.md#诊断记录与展示)。
