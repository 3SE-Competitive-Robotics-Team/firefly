# firefly-sim

RMUC MuJoCo 物理进程，由根 uv workspace 管理。

物理与 IMU 各 1kHz，发布 IMU、仿真位姿、评测状态与机体参数；接收
`Firefly/Control` 四电机推力，无有效控制指令时物理暂停。
双目、深度与下视图像由独立的 Bevy `render` 进程生成。

必须提供 `configs/scene.toml` 所选场景的 `models/<赛季>/v<版本>/<赛季>_collision.json`。

```bash
uv run firefly-sim
```

单独启动时，物理进程会等待控制输入。要运行包含传感器、pilot、模拟飞控和任务服务的悬停试验，使用[开发指南中的 SIL 命令](../../../docs/development.md#运行)。
