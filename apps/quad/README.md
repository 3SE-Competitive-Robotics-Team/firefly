# quad（微无人机第三人称飞行游戏）

250g 级四旋翼的 **6-DOF 动力学 + 第三人称 `WASD` 飞行**，在我们自己的场地里飞。
独立进程，使用本地手动输入与动力学；飞控与 `apps/fc` 使用同一套 PX4 控制核心。

## 运行

```sh
# 下载本地模型资产（Poly Pizza，CC-BY-4.0）
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

相机从机头后方偏上跟踪位置与偏航，保持画面竖直。

## 动力学（`src/quad.rs`）

- 刚体：位置/速度 + 四元数姿态/机体系角速度，定步长 240Hz 显式积分。
- 推力沿机体 `+Z`；重力世界 `-Z`；平移阻尼。
- 控制链与 `apps/fc` 相同：手柄 → 期望姿态 + 归一化集合推力 →
  `AttitudeControl` → `RateControl` → `ControlAllocationSequentialDesaturation`。
  `WASD` 给期望倾角，`Space/Shift` 给升降速度，推力做垂直速度控制并按
  `1/cos(tilt)` 补偿倾斜，保持高度。
- 机体轴：`+X` 前、`+Y` 左、`+Z` 上。

## 参数

`configs/quad.toml` 的 `[control]` 是 PX4 控制参数，`[manual]` 是手柄映射；
`[drone]` 配置质量、惯量与阻尼，`[airframe]` 配置旋翼与推力上限，`field` 和 `start`
指定场地及起点。默认值来自配置类型；改 TOML 后重新运行即可。

## 说明

- 场地：`configs/quad.toml` 的 `field`（相对 `models/`），当前指向 RMUC 场地。
- 模型朝向：glb 为 glTF Y-up、机头 `+Z`；挂载时转成世界 Z-up、机头 `+X`
  （`main.rs` 的 `model_rot`）。模型子节点旁挂一个绿色前向标记便于校对。
- 与 `apps/render` 的分工：`render` 是 VIO 的渲染 worker；本 app 是纯手动飞行。
