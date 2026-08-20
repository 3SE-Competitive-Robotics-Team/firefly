# VIO 简易验证流程（估计 odom vs 真值）

> 目标：**只验证 VIO 估计是否能跟真值**，不涉及 planner/任务规划。
> 用 `firefly-sim --script` 给双目无人机一段**平滑脚本轨迹**（无 planner），
> 跑 `vio`，在 rerun 里同轴对比 `vio/odom/traj`（橙）与 `gt/traj`（蓝）。

## 为什么不需要 planner

MuJoCo 的 `firefly-sim` 靠 PD 跟踪 `Firefly/Reference` 参考来飞，**没有参考就悬停**。
`firefly-demo`（planner）只是"恰好发布这个参考"的那个进程——我们在调试 VIO 时
并不关心它的规划结果。`--script` 让 sim 自己按 `_scripted_ref(t)` 生成平滑参考
（前向 0.8 m/s + 横向/高度正弦，提供 3D 视差），从而把 planner 从验证里剔除，
只留"相机运动 → MSCKF → odom"。想要任务轨迹（绕障）再用
`scripts/run_firefly.sh`（含 demo）。

## 启动（三个终端，按序）

```bash
# 0. 可选：先开 rerun viewer（多进程共享；不开则 vio 自动起一个）
rerun

# 1. 物理环境：MuJoCo 200Hz 物理 + 传感器（IMU/双目/深度/真值），
#    发布 Firefly/Reference 的脚本参考（`--script`），无需 planner
uv run firefly-sim --script

# 2. Rust VIO：订阅 IMU/双目 → MSCKF → 发布 odom（10Hz）
#    同时订阅真值并写入 rerun：vio/odom/traj(橙) vs gt/traj(蓝)
RUST_LOG=info cargo run -p vio
```

- 停：逐终端 Ctrl-C；若有残留清理 `pkill -f firefly_sim; pkill -f target/debug/vio`。
- 需要带工具链日志跑 `cargo build -p vio` 后直接 `./target/debug/vio`（快些）。

## rerun 里看什么（可视对比）

统一 `sim_time`（仿真秒）时间轴，双进程数据按同一时钟对齐回放：

| 实体 | 含义 | 颜色 |
|---|---|---|
| `sensor/stereo_left` / `sensor/stereo_right` | 双目灰度（KLT 输入） | 图 |
| `sensor/depth` | 深度（仅可视化） | Turbo |
| `vio/odom` / `vio/odom/traj` | **估计位姿(刚体) / 估计轨迹(折线)** | 橙 |
| `gt/pose` / `gt/traj` | **真值位姿(刚体) / 真值轨迹(折线)** | 蓝 |

**判断：**播放时看橙线是否贴合蓝线。贴合 → 估计准；分叉 → 该轴漂移。
打开两线的 3D 视图，逐轴看偏差。

## 关键参数（看哪些差异 / 日志）

### 1. 轨迹误差（最直接）
vio 的 `odom t=.. p=(x,y,z) v=..` 与真值 GT 逐时刻对比（rerun 里或日志）：
- **X（前向）**：应精确跟踪（脚本前向为主，估计应几乎贴合）。
- **Y（横向）/ Z（高度）**：短基线(0.05m)+前向运动的约束弱区，**允许中等漂移**
  （实测 ~0.5-1.5m 量级）。若之前"发散到数十米"，说明有 bug；若只差零点几米，
  属正常量级。

### 2. 视觉约束是否真的进滤波器（`RUST_LOG=debug`）
更新漏斗 `crates/firefly-vio/src/updater.rs` 的 `MSCKF 漏斗: 候选 X 三角化存活 Y 组装行 Z`：
- `候选`：本帧终结/边缘化的特征数（≈ 3-5 属当前量级，见下文）；
- `三角化存活`：通过 DLT+LM 的；
- `组装行`：最终并入 EKF 的测量行。**`组装行>0` 有值时视觉才算真正约束了**
  （修复 nullspace/FEJ 前 ~25px 残差 → 全被 chi2 拒 → 组装 0）。

### 3. 测量残差（判断"拒因"）
`RUST_LOG=firefly_vio=debug` 下 `chi2 / res_px` 应 ~1px 量级。若仍见到 ~20-30px，
说明又有残差缩放/子空间类问题（正常修复后是亚像素）。

### 4. 循环吞吐（当前已知限制）
闭环主循环含 `thread::sleep(IMU_PERIOD)` + 各步骤，**实际只跑到 ~1Hz 左右的
sim 时间、CPU 常 96%**（odom 常落后实时好几秒）。机器是慢在循环体（tracker/传播/
rerun），不是特征不够。这是当前 VIO 验证的一个主要待优化点：
- 观察：`odom t=` 到的仿真秒 vs 墙钟推进 —— 差距大 = 吞吐不足；
- 影响：即便每帧 5 个特征，实际每秒进滤波器只有 ~2-5 个测量。

### 5. 特征供给（"每帧 1-5 个"是为什么）
MSCKF 的 update 只处理**本帧被终结/边缘化的特征**（`feats_lost`+`feats_marg`），
数量 ≈ 特征库存量 ÷ 平均轨迹长。当前库稳态 ~56、窗口 11 帧 → ~4-5/帧，观测吻合。
要增强 6DOF 约束（尤其 Y/Z）需把库总量提到 ~100+：
- 增大检测净增（放宽 close-grid/min_px、抬高 `num_features`、改善特征散布），
  对应 `apps/vio/src/main.rs` 的 `TrackKlt::new(...)`。

## 判定标准（从"发散"到"可用"）

| 状态 | 表现 | 含义 |
|---|---|---|
| 严重 bug | 残差 ~25px、组装行~0、odom 跑到几十米外 | 有算法错（如本次 nullspace/FEJ） |
| 正常但欠约束 | X 贴合、Y/Z 漂 ~0.5-1.5m、组装行>0 但少 | 短基线/机动下约束偏弱，待提升特征供给 |
| 理想 | 三轴都贴合、漂移 < 0.1m、组装行稳定>0 | 可作状态源 |

## 参考

- 修复记录：`git log --oneline`（`8ac8a60` 性能根因、`9e38f72` 双目耦合、
  `ff55e70` nullspace/FEJ —— 残差 25px→0.8px）。
- 脚本轨迹：`apps/firefly-sim/src/firefly_sim/main.py::_scripted_ref`。
- 轨迹可视化：`crates/firefly-rerun/src/lib.rs::log_line_strip`。
