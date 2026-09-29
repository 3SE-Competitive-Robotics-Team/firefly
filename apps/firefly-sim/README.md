# firefly-sim

RMUC MuJoCo 物理进程，由根 uv workspace 管理。

200Hz 物理步进，发布 IMU、仿真位姿、评测状态与机体参数；接收
`Firefly/Control` 四电机推力，无有效控制指令时物理暂停。
双目与深度由独立的 Bevy `render` 进程生成。

必须提供 `models/rmuc2026/rmuc2026_collision.json`。

```bash
uv run firefly-sim
```

上电保持静止，等待传感器初始化完成后用 `ffctl fc` 解锁和起飞。
完整启动说明见 [运行文档](../../docs/how_to_run.md)。
