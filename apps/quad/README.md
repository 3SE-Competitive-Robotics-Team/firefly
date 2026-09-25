# quad（微无人机第三人称飞行 demo）

250g 级四旋翼的 **6-DOF 动力学 + 第三人称 `WASD` 飞行**，在我们自己的场地里飞。
独立进程，**不接 IPC / VIO / planner**——先把动态做对做顺，作为后续「在真实动态下
调试感知与 VIO」的可玩基座。

## 运行

```sh
# 模型不入 git，先拉一次（Poly Pizza，CC-BY-4.0）
uv run python scripts/fetch_drone_model.py
cargo run --release -p quad
```

## 操作

| 键 | 作用 |
|---|---|
| `W` / `S` | 前倾 / 后倾（松手自动回正） |
| `A` / `D` | 左倾 / 右倾 |
| `Space` / `Shift` | 上升 / 下降（速度控制，松手定高） |
| `Q` / `E` | 左偏航 / 右偏航 |
| `R` | 复位到起点 |

相机为第三人称追踪（机头后方偏上），只用偏航取向，不随俯仰/横滚翻滚。

## 动力学（`src/quad.rs`）

- 刚体：位置/速度 + 四元数姿态/机体系角速度，定步长 240Hz 显式积分。
- 推力沿机体 `+Z`；重力世界 `-Z`；平移阻尼。
- 角度模式控制：`WASD` 给期望俯仰/横滚，姿态误差 → 机体系角速度指令 →
  角加速度（比例）；`Space/Shift` 给升降速度，推力做垂直速度控制并按
  `1/cos(tilt)` 补偿倾斜，保持高度。
- 机体轴：`+X` 前、`+Y` 左、`+Z` 上。

## 参数

全部在 `configs/quad.toml`（质量、推重比、阻尼、控制增益、相机距离、场地路径）。
改完直接重跑，不需要重编译。缺键回落代码默认值。

## 说明

- 场地：`configs/quad.toml` 的 `field`（相对 `models/`），当前指向 RMUC 场地。
- 模型朝向：glb 为 glTF Y-up、机头 `+Z`；挂载时转成世界 Z-up、机头 `+X`
  （`main.rs` 的 `model_rot`）。模型子节点旁挂一个绿色前向标记便于校对。
- 与 `apps/render` 的分工：`render` 是 VIO 的渲染 worker；本 app 是纯手动飞行的
  基座。后续要把动态接进 `firefly-sim` / VIO 时再统一，不在本 app 里做接线。
