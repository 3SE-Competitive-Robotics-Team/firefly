# RMUC 运行指南

从仓库根执行命令。运行场景只支持 `rmuc2026`，由 `configs/scene.toml`
校验；缺少资产或地图时直接报错。

## 1. 资产与构建

必须准备以下本地资产（`models/` 不进 Git）：

| 文件 | 使用方 |
|---|---|
| `models/rmuc2026/rmuc2026_collision.json` | MuJoCo 碰撞体，非空 `boxes` 集合 |
| `models/rmuc2026/field.glb` | render 相机画面与 Rerun 静态场景 |
| `models/drone/drone.glb` | render 无人机外观 |

碰撞与视觉资产必须共用世界坐标系，单位米。CAD 转换方法见
[firefly-cad](../packages/firefly-cad/README.md)，视觉导出脚本为
`scripts/blender_field.py`。停机坪表面在 `(-13, 0, 0.375)`，机体原点增加
`0.03 m` 余量。具体场地接触与视觉质量需用实际资产验证。

```bash
uv sync
cargo build --release -p render -p vio -p fc -p ffctl
```

运行要求 Python 3.12+、Rust 1.97+、uv，以及能运行 Bevy 的 GPU 和图形会话。
参数位于 `configs/`，缺键使用代码默认值，缺配置文件报错。

## 2. 最小闭环与记录

分别开终端启动：

```bash
# 先启动统一记录进程；离线录制只写 logs/ 下的 rrd
uv run firefly-viz --save logs/rmuc_run.rrd

# MuJoCo 物理与 IMU；没有飞控指令时仿真时间暂停
uv run firefly-sim

# Bevy 窗口与传感器：订阅仿真位姿，发布双目和深度
cargo run --release -p render

# IMU + 双目估计；静止初始化完成前不发布里程计
cargo run --release -p vio

# 飞控：未解锁时持续发零推力，允许仿真推进并产生初始化测量
cargo run --release -p fc
```

实时查看时先启动 `rerun`，将第一条命令换为 `uv run firefly-viz`，
默认连接 `127.0.0.1:9876`。离线录制用 `rerun logs/rmuc_run.rrd` 回放。
`--save` 使用离线模式。传感器原图在 render 调试面板中查看。

物理为 200Hz、IMU 为 100Hz，仿真位姿默认 10Hz；render 按 10Hz 窗口生成
双目和深度。VIO 在图像时刻更新滤波器，通过 IMU 预测输出 100Hz 里程计。
飞控为 1kHz，仿真取最新电机指令；指令超过墙钟 50ms 未更新时物理暂停。

`Firefly/GroundTruth` 用于仿真图像生成、评测和可视化；
`Firefly/PlantState` 只作评测。估计器和飞控反馈不得消费真值。

### 2.1 起飞、悬停、降落

上电保持静止，观察 `logs/vio` 的初始化日志或里程计 `is_initialized`，
就绪后执行：

```bash
./target/release/ffctl fc arm
./target/release/ffctl fc takeoff 1.0
./target/release/ffctl fc hold
./target/release/ffctl fc land
./target/release/ffctl fc disarm
```

高度相对于起飞点。拒绝指令的原因在 `logs/fc` 中；未就绪时不能跳过初始化。
各进程通过 Ctrl-C 优雅退出，确保 IPC 端口释放和 rrd 完成写入。

## 3. 视觉定位与规划

VIO 输出重力对齐的局部坐标：初始位置为零，航向为规范自由度。
RMUC 静态地图和视觉库图在场地坐标系；`configs/gicp.toml [origin]` 配置固定启动位置与航向，
由定位进程建立 map←odom 变换。改变启动点必须同步配置。
不能直接把原始里程计当成场地坐标，也不能用真值提供在线对齐。

可选资产：

| 文件 | 用途 |
|---|---|
| `apps/planner/maps/rmuc2026.ffmap` | planner 与 gicp 的默认静态地图 |
| `apps/planner/maps/rmuc2026.ffvmap` | LightGlue 视觉库图 |
| `models/aliked-n16-k512.onnx` | ALIKED 特征提取权重，可用 `--model` 指定 |
| `models/lightglue-aliked-k512.onnx` | LightGlue 匹配权重，可用 `--model` 指定 |

`.ffmap` 必须由对应 RMUC 资产生成并核验坐标；缺失时 planner / gicp 启动失败。
`--map` 可指定其他路径，但内容必须对应当前 RMUC 场地。

```bash
cargo run --release -p aliked
cargo run --release -p lightglue -- --map apps/planner/maps/rmuc2026.ffvmap
cargo run --release -p gicp
```

当前视觉链路是特征提取 → 库图匹配 → PnP 位姿观测 → 定位融合 →
`Firefly/CorrectedOdometry`。在线关键帧回环与位姿图优化尚未实现。
完成地图对齐后，可运行 planner 接收 `Firefly/CorrectedOdometry` 并发布飞控参考；
通过 `ffctl fc track` 进入跟踪模式，`ffctl planner goal X Y Z` 发布地图系目标。
planner 不订阅原始局部 VIO，不提供平移偏置捷径。未收到有效初始化状态时不发布
参考；状态失联 500ms、变为未初始化或出现非法数值时停止发布并锁存，恢复定位后
须重启 planner。飞控按自己的参考流超时策略处置，不会收到伪造的位置反馈。
飞控反馈与 Hold 锚点保持在 odom 系，仅将地图目标转换到 odom；详情见 [坐标契约](frames.md)。
深度帧按同源状态历史插值，每帧只融合一次；历史不覆盖时跳过该帧。

`scripts/collect_vision_map.py` 从 render 采集 RMUC 左图、深度和离线位姿标签，
再交给 `aliked --build-map`。该工具要求外部飞控、估计器和有效地图对齐，
自行启动 sim / render；不应用于验证传感器闭环是否成立。

## 4. 诊断与验证

所有运行日志与调试数据通过 `Firefly/Log` / `Firefly/Viz` 汇入 firefly-viz，
正式记录只写 `logs/*.rrd`。`sim_time` 对齐时间，不能替代空间对齐。

常用实体：`vio/odom`、`vio/traj`、`vio/debug/*`、`gt/pose`、`gt/traj`、
`fc/debug/*`、`plan/*`、`world/rmuc2026`、`logs/<app>`。
原始 VIO 与真值的空间对齐只在评测中进行，固定尺度，结果不得回流算法。
通用指标在 `bench/metrics.py`。

```bash
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings
cargo test
cargo test --release -p firefly-planner --test random_map_benchmark -- --ignored
uv run --with pytest pytest apps/firefly-sim/tests/ bench/tests/ -q
```

准备真实 RMUC 资产并构建 release 版 vio / fc / render 后，在没有其他闭环进程
运行的图形会话中执行：

```bash
uv run --no-dev python bench/check_sensor_startup.py
```

检查屏蔽 PlantState，保留供 render 合成图像的 GroundTruth，验证静止初始化、
局部原点、时间戳递增与未解锁零推力。记录写入 `logs/`，子进程通过 SIGINT 退出。
资产缺失时检查在启动子进程前报错；单元测试不能替代真实 RMUC 飞行验证。
