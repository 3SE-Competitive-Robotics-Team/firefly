"""firefly-sim：MuJoCo 物理环境主循环（双语言闭环的 Python 侧）。

职责：
- 运行 MuJoCo 物理（200Hz），PD 跟踪 `Firefly/Reference` 参考状态；
- 发布传感器到 iceoryx2：`Firefly/Imu`（100Hz）、`Firefly/CameraLeft` /
  `Firefly/CameraRight`（双目灰度，10Hz）、`Firefly/Depth`（10Hz）、
  `Firefly/GroundTruth`（真值 odom，10Hz）；
- Rust 侧（vio + firefly-demo）消费传感器 → 估计 → 规划 → 回传参考，闭环。

运行：`uv run python -m firefly_sim`
"""
