# quad（微无人机第三人称飞行游戏）

用键盘在 RMUC 场地中驾驶 250 g 级四旋翼。quad 在本机运行六自由度动力学，与 Mac 的模拟飞控共享姿态控制、角速度控制和推力分配。

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

## 参数和坐标

`configs/quad.toml` 当前覆盖了姿态和角速度控制增益。`[manual]` 可调整按键映射参数，`[drone]` 和 `[airframe]` 可调整质量、惯量和旋翼。`field` 选择 `models/` 内的场地，`start` 设定起点。[安装配置](../../../configs/quad.toml) · [可选字段与默认值](src/config.rs)

动力学以 240 Hz 积分，机体系为 +X 向前、+Y 向左、+Z 向上。模型挂载时从 glTF 的 Y-up 坐标转换到世界 Z-up。要运行带传感器、定位和任务服务的场景，使用[完整 SIL](../../../docs/development.md#运行)。
