# firefly-sim

`firefly-sim` 在 MuJoCo 中默认以 1 kHz 推进物理仿真并生成 IMU 样本。它发布仿真位姿、评测状态和机体参数，从 `Firefly/Control` 接收四个电机的推力。控制指令过期时，仿真暂停。双目和下视图像由 `render` 进程生成，`depth` 用左目图像估计深度。

必须提供 `configs/scene.toml` 所选场景的 `models/<赛季>/v<版本>/<赛季>_collision.json`。

```bash
uv run firefly-sim
```

单独启动时，物理进程会等待控制输入。要运行包含传感器、pilot、模拟飞控和任务服务的悬停试验，使用[开发指南中的 SIL 命令](../../../docs/development.md#运行)。
