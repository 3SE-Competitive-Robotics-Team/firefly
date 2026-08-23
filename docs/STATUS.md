# firefly 系统状态（2026 多轮迭代后）

本文档诚实记录经多轮模块化迭代后的系统状态：已完成、已知限制、运行/验证方式。
目标读者：后续继续开发的工程师/模型。

## 架构（MuJoCo 双语言闭环）

Python sim（MuJoCo 物理 + 传感器发布 200Hz，订阅 `Firefly/Reference` PD 闭环）
→ Rust vio（MSCKF 位姿估计，iceoryx2 订阅 IMU/双目）
→ Rust planner（重规划 10Hz，`Firefly/Odometry` 状态源 + 回传参考）
多进程共享 `Firefly/*` 话题 / fastrace 跨进程 trace / rerun `sim_time` 时间轴。

## 已完成并验证（全部有回归单测 / 实跑数值证据）

### 任务/导航层
- **simplify_path 膨胀检查**（`firefly-search` `line_is_clear` 改 `is_occupied_inflated`）：
  全局路径不再直线擦边穿越膨胀层 → **planner 任务此前贴柱卡死，现可完整走到目标**。
  回归单测 `line_clipping_inflation_is_blocked`（旧代码 FAIL/新代码 PASS）。
- **touch_goal 物理到达判定**：轨迹耗尽仅当真实位置距目标 < ARRIVE_DIST 才完成，
  避免提前结束。
- **planner 重启上限 3→6** + e2e 矮墙适配（高墙翻越时 MINCO 收敛不稳）。
- **进退振荡**：PD KD 10→22（ζ 0.37→0.82）、replan 起点接续旧轨迹、target 净距重试+冷却+退化轨迹防护。
- **动态障碍**：`update_motion` 每 tick 把运动体素写入规划图（感知建图+动力学回避）。

### VIO 层（MSCKF，对照 OpenVINS）
- **外参镜像修复**：`q_ito_c` w 反号导致视线镜像（-2858m 爆炸）→ 修正 + 射线回归单测。
- **双传播发散根治**：round3 的每相机双传播（propagate_to 预推进 + feed 内
  propagate_and_clone）重叠积分同一段 IMU → 指数漂移；改为 open_vins 单传播模型。
- **相机时序**：state 绝不越过相机帧（无"图像乱序"丢帧）。
- **mask 空**：tracker 校验拒收 → 修复后特征真正跟踪。
- **克隆窗无限增长**：odom 传播误增广克隆 → 修复后窗口稳定。
- **残差双重缩放回归修复**：`CamRadtan::distort_f` 本就把归一化→像素；round4 误加
  `fx·uv_dist+cx` 与 `pix_scale` 双重缩放 → SLAM 残差 -25 万级、landmark 被毁。
  回退后 SLAM 3 测试由失败/ignore → **全部通过（45 绿、0 ignore）**。
- **chi2 补测量噪声 R**、**MSCKF 硬残差门（>40px 拒收）**：止住协方差膨胀下
  垃圾特征穿透 chi2 主动加剧发散（x 跟踪从灾难 -144 恢复）。
- 立体基线几何修正：scene 相机沿前向分开（射线近共线、无侧向视差）→ 改横向基线。

## 已知限制（诚实）

1. **VIO 运行时视觉参与未收敛**（VIO 测试层全绿，但运行时不可用于状态源）：
   - 强机动下死航位推算姿态漂移→重力泄漏→odom 发散（对话/更新数学均正确，已证）。
   - 视觉无法提供姿态约束：**候选特征少**（feats_lost 0-3/帧）+
     **三角化全部拒绝**（低视差单目 cond 30k~5900 万）+ 立体耦合两种尝试
     （最近邻/SAD，均加单测）实跑均产生垃圾立体特征回归。
   - 结论：当前"斜视低纹理棋盘 + 0.1m 小基线 + 强机动"场景下需要专业立体匹配
     （对极约束+逆深度初始化）的大规模投入，非增量修补可成。
   - **planner 以 odom 为状态源**（`--script` 场景 34s ATE_RMSE 0.6m，
     见 `logs/bench/summary_*.json`；新鲜度超时回退轨迹推进）。
2. **cargo test --workspace 全绿**（firefly-vio 45、firefly-vio-core 104、search/map/
   planner 等），clippy 0；`--ignored` 随机地图基准确定性 0.87（LCG 固定种子，
   无 flaky；剩 4 败为硬图优化器收敛限，阈值 0.8 已达成，安全优先不求强推优化器）。
3. 已 root-cause 但按"未用路径"处置：无（SLAM 已修好）。
4. **PLANNER 暖启动已实现**（对照 `planner_manager.cpp computeInitState`
   case 2）：非首帧重规划用上一条最优轨迹作初始 MINCO（剩余段采样 + 全局路径
   延续），失败自动降级冷启动。任务执行 FSM 同步下沉至 `firefly-planner`
   crate（对照 `ego_replan_fsm.cpp`），apps/planner 只留 IPC/viewer 壳。
5. **约束点生成已按官方重建（完成）**：原"build_plane 射线步进"曾因连续
   重规划链上偶发 L-BFGS 发散样本（单轴 1e12 级）无限循环（旧代码已加
   步数上限 + 非有限守卫防挂死）。现按官方 `finelyCheckAndSetConstraintPoints` /
   `roughlyCheckConstraintPoints` 重建（`obstacles.rs`）：稠密采样（`computePointsToCheck`）
   → 占据分段 → **约束点数组内 in/out 自由点搜索 + A* 绕障 + 交点平面**，
   不再做射线步进（天然有界，末端无自由点即报错/放弃）。同时对齐：
   L-BFGS 内循环检测激活条件（iter>3 且平滑度/piece<10）、五项代价公式
   （K=5 梯形采样、2/3 截断、官方 CLEARANCE=Cw×1.5 等）、rebound 全局
   迭代上限。对照清单与逐文件映射见 `docs/v2_alignment.md`。

## 建议下一步

- A. 若要让 VIO 真正进入闭环：增大立体基线（0.1→0.3m）+ 专业立体匹配 + 逆深度
  初始化（对照 open_vins `FeatureInitializer`/`TrackKLT`），工作量大且场景需调。
- B. 若 VIO 作已知限制：planner 以 odom 为状态源的可运行基线（现状），转向新功能/其它模块。
- C. 换场景增可观测性（提高纹理 / 大基线 / 减机动）再评估 VIO。

## 验证

```bash
cargo test                # 全 workspace 单测
cargo clippy --workspace  # 0 警告
uv run firefly-sim &      # 0. viewer: rerun
cargo run -p vio &
cargo run -p planner # 任务跑通（odom 状态源）
```
