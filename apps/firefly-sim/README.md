# firefly-sim

MuJoCo 物理环境主循环（Python 应用，`apps/` 下的一个进程）。

职责：运行 MuJoCo 物理（200Hz）→ 发布传感器到 iceoryx2（IMU/双目灰度/深度/真值）→ 订阅 `Firefly/Reference` 参考状态 → PD 闭环控制无人机。

运行（从仓库根）：

```bash
uv run firefly-sim
```

与 Rust 管线（`apps/vio` + `apps/planner`）构成双语言闭环。
