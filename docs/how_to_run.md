# How to Run

目前一共 5 个进程：

| # | 进程 | App | 订阅 | 发布 | 频率 |
|---|---|---|---|---|---|
| 1 | `firefly-sim` | `apps/firefly-sim` | `Firefly/Reference` | `Firefly/Imu` 100Hz / `Firefly/CameraLeft,Right` 10Hz / `Firefly/Depth` 10Hz / `Firefly/GroundTruth` 10Hz | 200Hz 物理 |
| 2 | `vio` | `apps/vio` (MSCKF) | `Imu` + 双目灰度 | `Firefly/Odometry` (估计位姿，100Hz propagation) + `Firefly/Viz` (10Hz) | 10Hz 视觉修正 / 100Hz 输出 |
| 3 | `gicp` | `apps/gicp` (GICP 全局重定位 + FusionFilter) | `Odometry` (100Hz) + `Depth` | `Firefly/CorrectedOdometry` | 1Hz 重定位 / 100Hz 融合 |
| 4 | `planner` | `apps/planner` (EGO-Planner v2: A* + MINCO) | `Odometry`/`CorrectedOdometry` + `Depth` + `Firefly/Goal` | `Firefly/Reference` + `Firefly/Viz` | 10Hz |
| 5 | `firefly-viz` | `apps/firefly-viz` | `Firefly/Viz` | （写 rerun viewer / rrd） | 消费 10Hz 可视化 |

数据流：`sim → vio → gicp → planner → sim`（PD 闭环跟踪）。`sim_time` 为全链路统一时钟，`fastrace` 跨进程续接同一 `trace_id`。Rust 计算线程零 IO：可视化数据经 `Firefly/Viz` 话题零拷贝发布，由 `firefly-viz` 进程统一写 rerun。

## 0. 前置依赖

```bash
# Rust 1.97+ / Python 3.12+
cargo --version && python3 --version

# 安装 Python 环境（根 workspace 聚合 firefly-mujoco + firefly-sim）
uv sync

# 安装 rerun viewer（可选，但强烈建议先开，4 进程共享同一 viewer）
cargo install rerun-cli   # 或 uv tool install rerun-sdk
```

地图文件：`apps/planner/maps/*.ffmap`（见 `docs/map-format.md`）
配置：`configs/*.toml`（`sim.toml` / `vio.toml` / `gicp.toml` / `planner.toml`），缺键回落代码默认值。

## 1. 构建验证

```bash
cargo build                    # workspace 构建（apps/vio, planner, gicp）
cargo test
uv run firefly-sim --help      # 检查 Python 侧可导入
uv run firefly-viz --help      # 检查可视化进程可导入
```

## 2. 启动 — 多终端手动启动

> 按顺序各开一个终端，**先开 viewer + firefly-viz**。

```bash
# 终端 0 — viewer（多进程共享；或用 `uv run firefly-viz --serve` 起内置 viewer）
rerun

# 终端 1 — 可视化统一写入：订阅 Firefly/Viz，写共享 viewer（或 --save logs/run.rrd）
uv run firefly-viz
# 可选：uv run firefly-viz --save logs/run.rrd   # 离线录制

# 终端 2 — 物理环境：发布传感器，订阅 Reference 做 PD 闭环
uv run firefly-sim --no-trace
# 可选：uv run firefly-sim   # trace 模式，启用 OTel span

# 终端 3 — VIO：订阅 IMU/双目，发布 odom + 可视化消息
cargo run -p vio
# 可选：cargo run -p vio -- --config configs/vio.toml

# 终端 4 — GICP：订阅 odom+深度，以静态先验为靶图做配准，发布校正后里程计
cargo run -p gicp -- --map apps/planner/maps/gate.ffmap
# 不指定 --map 时尝试加载 gate.ffmap，不存在则用空地图（GICP 自动禁用，仅透传融合）

# 终端 5 — 规划器：订阅 odom(优先 CorrectedOdometry) + 深度，发布 Reference + 可视化消息
cargo run -p planner -- --map apps/planner/maps/gate.ffmap
# 独立运行不接 sim：cargo run -p planner -- --map apps/planner/maps/gate.ffmap
```

## 3. 发目标点 — 机器人导航

规划器启动后悬停在 `configs/sim.toml: start = [1.0, 4.0, 1.0]`，等待 `Firefly/Goal`：

```bash
# 语法：uv run firefly-goal X Y Z  （地图系，米）
uv run firefly-goal 20 4 1.5        # gate.ffmap 中直线可达点
uv run firefly-goal 25 6 1.2        # 绕柱点
uv run firefly-goal 8.5 4 1.0       # 门前点

# 动态重目标：飞行中可连续发送，最新一条生效（10Hz 轮询，1s 内重算全局路径）
uv run firefly-goal 22 3.6 1.5
```

坐标需落在地图 `ORIGIN + DIMS*RESOLUTION` 范围内（`gate.ffmap`: `0~28 × 0~8 × 0~3.2m`）。不可达目标会被 `PlannerManager::set_goal` 拒绝并 `log::warn`。

可选的 `planner` 启动时指定初始目标：

```bash
cargo run -p planner -- --map apps/planner/maps/gate.ffmap --goal 20 4 1.5 --start 1 4 1
```

强制急停（对照官方 `mandatory_stop`）：

```bash
cargo run -p planner -- --mandatory-stop   # 向 Firefly/MandatoryStop 发单帧空消息
```

## 4. 可视化（rerun，统一 `sim_time` 时间轴）

Rust 进程不直接连 rerun：vio/planner 经 `Firefly/Viz` 话题零拷贝发布
`VizMessage`，由 `firefly-viz` 统一写入。启动顺序：先起 viewer（或
`firefly-viz --serve`），再 `uv run firefly-viz` 连共享 viewer；离线录制用
`uv run firefly-viz --save logs/run.rrd`（之后 `rerun logs/run.rrd` 回放）。

* `sensor/stereo_left|right`、`sensor/depth` — MuJoCo 原图
* `vio/odom` (橙) / `gt/pose` (蓝) — VIO 估计 vs 真值
* `plan/map` + `plan/decor` — 静态先验（启动一次性）
* `plan/perceived` — 深度 raycast 在线占据（2.5s 刷新）
* `plan/global_path` (绿)、`plan/local_traj` (蓝+黄速度)、`plan/planes`、`plan/drone`、`plan/motions`
* `vio/debug/track_length` / `db_size` / `track_avg_len` — 前端健康度

## 5. 常用地图与配置

```bash
ls apps/planner/maps/
# gate.ffmap      默认门洞场景（与 MuJoCo scene.py 同构）
# corridor.ffmap  窄走廊
# maze.ffmap      迷宫
# slalom_dyn.ffmap / forest_dyn.ffmap  动态障碍（MOTION 段，见 map-format.md）

# 换配置（任意 --config）
cargo run -p vio -- --config configs/vio.toml
cargo run -p gicp -- --config configs/gicp.toml --map apps/planner/maps/maze.ffmap
cargo run -p planner -- --config configs/planner.toml --map apps/planner/maps/maze.ffmap
```

## 6. 优雅退出与排障

* **必须 `Ctrl+C` 优雅退出**（`node.wait` 返回 `Err` → Drop 端口）。`pkill -9` / `SIGKILL` 会留下孤儿内核 shm 与幽灵端口注册，后续订阅端会连上死端口收不到数据，且占满 `max_publishers` 槽位。
* 清理残留（所有进程杀干净后执行，macOS）：
  ```bash
  rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state
  # Linux: rm -rf /dev/shm/iox2* /tmp/iceoryx2
  ```
* 端口冲突 / `FailedToDeliverSignal` 为良性：订阅端兜底轮询仍可驱动。

## 7. 最小验证清单

1. `rerun` + `uv run firefly-viz` 已开 → 5 个进程按 `sim → vio → gicp → planner` 顺序启动，日志均出现 `已订阅 ...` / `已打开话题`；`firefly-viz` 出现 `已订阅 Firefly/Viz`。
2. `uv run firefly-goal 20 4 1.5` → `planner` 日志 `收到新目标` + `目标更新 ... 重新规划中`，`rerun` 中 `plan/local_traj` 出现。
3. `sim` 日志 `收到参考 t=... pos=(...)` 且无人机开始移动。
4. `Ctrl+C` 后 `全部进程已结束` / `优雅退出`，`cargo run` 进程组无残留（`ps aux | grep firefly` 为空）。

## 8. 故障速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `planner` 日志 `目标 (...) 不可达` | 目标点在占据体素内或超出地图 | 换 `gate` 范围内空地点，如 `20 4 1.5` |
| `GICP矫正接受` 迟迟不出现 | 空地图或点云 `<30` 点，或 `chi2` 拒收 | 检查 `--map` 路径，近处对墙增加特征 |
| `odom 订阅不可用` | `vio` 未启动或 iceoryx2 幽灵端口 | 重启全栈并清理 `/tmp/iceoryx2` |
| `深度/里程计丢失！进入急停` | 深度或 odom 超时 `>1.0s` | 检查 `sim` 是否卡死，`gicp/planner` 是否正常消费 |
