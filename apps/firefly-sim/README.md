# firefly-sim

MuJoCo 物理环境进程，由根 uv workspace 管理。

运行 200Hz 物理，发布 IMU、双目、深度、真值与机体参数；接收 `Firefly/Control`
的四电机推力。飞控在 `apps/fc`，无有效控制指令时物理暂停。
`--script` 使用内部真值反馈生成评测轨迹，仅作为估计器的运动夹具。

```bash
uv run firefly-sim
```

上电保持静止，等待传感器初始化完成后用 `ffctl fc` 解锁和起飞。
完整启动说明见 [根 README](../../README.md)。
