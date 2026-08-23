# firefly-planner ↔ EGO-Planner-v2 逐文件对照

> 对照基线:`/Users/flamingo/Projects/EGO-Planner-v2/swarm-playground/formation_ws/src/planner/`
> 结论分级:**✅ 一致 / 🟡 等价(数值或结构不同但数学等价)/ 🔴 偏离(语义不同,需忠实移植)/ 📋 未实现(官方有、我们没有)**
>
> 本表按"官方文件 → 我们文件"逐模块记录。**🔴/📋 是后续移植工作的范围**;🟡 属已文档化的等价偏离,一般不动。

## 0. 总览映射

| 官方文件 | 我们 | 状态 |
|---|---|---|
| `path_searching/dyn_a_star.cpp/.h` | `crates/firefly-search/src/astar.rs` | 🟡 |
| `plan_env/grid_map.cpp/.h` | `crates/firefly-map/src/grid.rs` + `depth.rs` | 🟡 |
| `plan_env/raycast.cpp/.h` | `crates/firefly-map/src/depth.rs`(感知) | 🟡 |
| `traj_opt/poly_traj_utils.hpp` (MinJerkOpt) | `crates/firefly-trajectory/src/{minco,banded}.rs` | ✅ |
| `traj_opt/lbfgs.hpp` | `crates/firefly-optimize/src/lbfgs.rs` | 🟡 |
| `traj_opt/poly_traj_optimizer.cpp/.h` | `crates/firefly-planner/src/{planner,obstacles,objective}.rs` | 🔴 |
| `plan_manage/planner_manager.cpp/.h` | `crates/firefly-planner/src/{init,manager}.rs` | 🔴 |
| `plan_manage/ego_replan_fsm.cpp/.h` | `crates/firefly-planner/src/manager.rs` + `apps/planner/src/main.rs` | 🔴 |
| `traj_utils/plan_container.hpp` | `manager.rs`(GlobalTrajData/LocalTrajData) | 📋 |
| `plan_manage/launch/advanced_param.xml` | `crates/firefly-planner/src/config.rs` | 🔴 |

## 1. A* (`dyn_a_star` ↔ `firefly-search/src/astar.rs`) 🟡

一致:26 邻域、代价 √(dx²+dy²+dz²)、世代计数 `rounds_` 复用节点池、8-邻域启发(官方 `getDiagHeu` 3D 对角;我们用欧氏距离,对网格 A* 一致且更快)、`simplify_path` 字符串拉直(我们的膨胀检查是修复,官方没有拉直但官方全局是 MINCO 不用网格路径)。

偏离:
- 官方 `ConvertToIndexAndAdjustStartEndPoints`:起点/终点在障碍内时**沿连线方向逐步挪出障碍**(不失败);我们直接报错 `goal is occupied`。对我们影响小:局部绕障 A* 的 in/out 点按构造都是自由点。🟡
- 官方搜索窗口是以中点为心的 100³ 局部池;我们全图。结果等价。🟡

## 2. 地图 (`grid_map` ↔ `firefly-map`) 🟡

一致:占据体素 + 独立膨胀层 `occupancy_buffer_inflate_` ↔ `inflate`、`getInflateOccupancy` ↔ `is_occupied_inflated`(越界即占据)、深度 raycast 更新(Amanatides & Woo 体素遍历,我们 `depth.rs` 同算法)、`clearAndInflateLocalMap` ↔ `inflate_obstacles`。

偏离:
- 官方 `obstacles_inflation=0.0` 但 `inf_step = ceil(0 − 0.001/res)+1 = 1` → 有效膨胀恒 ≥1 体素(0.1m);我们 `obstacle_inflation=0.2` @ res 0.1 → 2 体素(0.2m)。🟡 安全侧但偏保守。
- 官方有 `getLessInflateOccupancy`(膨胀减一档)供部分检查;我们没有。影响小。🟡

## 3. MINCO (`poly_traj_utils.hpp` MinJerkOpt ↔ `firefly-trajectory`) ✅

逐项核对一致:
- 带形系统布局:行序 起点PVA / 中间块{jerk连续,snap连续,位置,位置连续,速度连续,加速度连续} / 终点PVA,带宽 (6,6),**无部分主元 LU**(官方 "NO PIVOT for efficiency")——`build_banded` 与官方逐元素一致。
- `getGrad2TP`(solveAdjGradC → addPropCtoT → addPropCtoP)↔ `propagate_gradient`(solve_transpose → dq 取 6i+5 行 → dT 减 ∂M/∂Tᵢ·c 项)一致。
- `getTrajJerkCost`(闭式 ∫‖p‴‖²dt)↔ `SmoothnessPenalty` Gram 矩阵一致(M5 数值梯度验证过)。
- 5 次多项式、C² 连续、时间正性:官方虚拟时间 τ(T 映射)↔ 我们 `τ=ln T`。🟡 数值等价,已文档化。

## 4. L-BFGS (`lbfgs.hpp` ↔ `firefly-optimize`) 🟡

一致:两循环递归、BB 初始 Hessian、强 Wolfe、`past/delta` 相对改进收敛、`g_epsilon`=1e-5、mem=16、max_iter=200。

偏离(官方 `optimizeTrajectory` 覆盖值):
- `delta`:**官方 1e-3**,我们 1e-2(差 10×,收敛判据更松)。🔴
- `max_linesearch`:官方 40,我们 200;`min_step` 官方 1e-32,我们软失败阈值 1e-14。🟡
- 线搜索算法:官方 liblbfgs 三次/二分(强 Wolfe),我们 Lewis-Overton 弱 Wolfe 二分。数学等价但数值轨迹不同。🟡

## 5. 优化器主循环 (`poly_traj_optimizer.cpp` ↔ `planner.rs`/`objective.rs`) 🔴

官方 `optimizeTrajectory` 结构:init → `finelyCheckAndSetConstraintPoints(initMJO, first_init=true)` → L-BFGS(代价回调内 `iter_num_>3 && smoo_cost/piece<10` 时 `roughlyCheckConstraintPoints`) → LBFGSERR_CANCELED=REBOUND(rebound≤20)→ fine check 碰撞 → restart(restart<3)→ 成功条件 `min_ellip_dist2² > (Cw·1.25)² && finely==OBS_FREE`。

我们的结构一致(rebound 循环 + 内循环检测 + fine check + restart),但:
- **🔴 内循环检测实现不同**:官方 `roughlyCheckConstraintPoints` 在**约束点数组上**检测(稠密采样点),发现新穿入后**沿数组向前/向后找最近自由点**(in/out,天然有界;末端无自由点 → STOP_FOR_ERROR 放弃),再 A* 绕障 + 交点平面;我们 `ReboundDetector` 是分段 + 最近引导点 + 射线步进(自创过渡实现,obstacles.rs 里留了 TODO)。**这是本次移植的核心。**
- **🔴 激活条件**:官方 `iter_num_>3 && smoo_cost/piece<10`;我们仅 `eval_count>=3`,缺平滑度条件。
- 🔴 约束点**布局**:官方 `getInitConstraintPoints(K)` = **N·K+1** 点(K 个/段,边界不重复);我们 `piece·(κ+1)+j`(κ+1 个/段,边界重复)。影响平面索引与 uniform 代价。
- 🔴 `constraint_points_per_piece`:官方 **5**,我们 12。
- 📋 `distinctiveTrajs`(多拓扑优化,`use_distinctive_trajs=false` 默认关闭)——未实现,符合默认。
- 🟡 `roughlyCheck` 的"覆盖判据" `(p−base)·dir < res` 我们有一致的等价实现。

## 6. 代价函数 (`costFunctionCallback`/`addPVAGradCost2CT` ↔ `firefly-cost`) 🔴

| 项 | 官方 | 我们 | 状态 |
|---|---|---|---|
| 平滑 Js | MINCO 目标本身(权重 1) | `SmoothnessPenalty` Gram 矩阵 | ✅ |
| 总时间 Jt | `wei_time·ΣT`(τ 虚拟时间传播) | `TimePenalty`(ln T 传播) | 🟡 |
| 可行 Jd | `wei_feas·max{(v²−vm²),0}³` + **加速度**(**无 jerk 项**) | v/a/**jerk** 三项 | 🔴 |
| 障碍 Jo | 硬 `wei_obs·err³` + 软 `wei_obs_soft·r²(√(1+err²/r²)−1)` | 同 | ✅ |
| 障碍采样 | **前 2/3 约束点**(`two_thirds_id`) | 全段 | 🔴 |
| 集群 Jw | 椭球 E=diag(1/4,1/4,1) `(Cw·1.5)²`,**前 2/3** | 同公式,全段,`(Cw+peer)·1.5` | 🔴 |
| 队形 Jf | `wei_f·‖p−tar‖²`,**前 2/3** | 同公式,全段 | 🔴 |
| 均匀 Ju | `wei_sqrvar·ΣR²/N`(R=相邻约束点距离²,全数组,**非中心化**) | 中心化方差 mean(R²)−mean(R)²,按段内对 | 🔴 |
| 采样权重 | 梯形 `omg=(j==0‖j==K)?0.5:1`,权重 `omg·T/K` | 均匀 `T/κ`(端点不折半) | 🔴 |
| 采样密度 | 所有代价 K=5/段 | obstacle κ=12,feas/swarm 20 | 🔴 |

注:官方 `swarmGradCostP`/`formationGradCostP`/`obstacleGradCostP` 的**时间梯度**含 `gradt += dJ·(v−swarm_v)` 等;我们 `Accumulator::add_absolute` 用相对速度实现,公式等价 ✅。

## 7. 初始化 (`planner_manager.cpp computeInitState` ↔ `init.rs`) 🔴

- **冷启动 case 1**:官方**不用 A***——先构造 start→local_target 的 2s 单段最小 jerk"init-of-init",再按 `piece_nums=round(dist/piece_length)`,`ts=piece_length/max_vel` 均匀分段采样出中间点;我们 `init_from_path` 用 A* 引导路径拐点 + 弧长分配时长(更接近 v1 路线)。🔴
- **暖启动 case 2**:官方 `piece_nums=ceil(dist/piece_length)`、**每段时长均匀** `t_to_lc_tgt/piece_nums`、中间点 = 旧局部轨迹剩余段 + **全局 MINCO 轨迹**采样;我们 `init_warm_start` 剩余轨迹细采样 + 引导路径延续段 + 弧长 waypoint + `allocate_time` 按速度分配。🔴
- 官方 case 1 失败可走 `flag_randomPolyTraj`(随机扰动中间点重试,失败次数自适应);我们没有。📋
- `init_of_init_totaldur=2.0` 等常量见官方。

## 8. 全局轨迹 (`planGlobalTrajWaypoints` ↔ 我们的全局路径) 🔴

官方全局轨迹是 **MINCO 最小 snap 平滑轨迹**(过航点,时间 = 距离/(max_vel/1.5) 迭代降速至可行);`GlobalTrajData` 带时间轴,`getLocalTarget` 沿它按时间步进找 planning_horizon 处局部目标,暖启动也采样它。
我们全局路径是 **A* 简化折线**(无时间轴),局部目标按弧长截取,暖启动延续段 = 折线尾段。**结构级差异**,影响 getLocalTarget 语义与暖启动,列为下一阶段移植。

## 9. 任务执行 FSM (`ego_replan_fsm.cpp` ↔ `manager.rs` + `apps/planner`) 🔴

官方:
- **100Hz exec 定时器 + 20Hz safety 定时器**;我们 10Hz 单环。🔴
- 状态机 INIT→WAIT_TARGET→GEN_NEW_TRAJ→REPLAN_TRAJ→EXEC_TRAJ→EMERGENCY_STOP;我们只有"初始规划/重规划/到达"。📋 EMERGENCY_STOP 缺失。
- **20Hz 安全检查**:用稠密 `pts_chk`(每条局部轨迹保存)扫描执行中轨迹前方,发现碰撞立即 `planFromLocalTraj()`,失败且 <emergency_time(1.0s)→ EMERGENCY_STOP;我们无此环,只有重规划触发。📋
- 重规划触发:`t_cur>replan_thresh(1.0s)` **或** 接近已检段末端;我们 `t_cur>thresh || 轨迹耗尽`,且无 `pts_chk` 末端触发。🔴
- `planFromLocalTraj` 降级链:暖启动→冷启动→**随机多边形重试**;我们只有暖→冷。📋
- 航点序列推进(`wpt_id_`,`no_replan_thresh`=1.0m 提前切点);我们单目标 + 全局折线,无航点。📋
- `thresh_no_replan_meter=1.0`、`emergency_time=1.0`、`planning_horizon=7.5`、`fail_safe` 等参数未对齐。🔴

## 10. 参数 (`advanced_param.xml`/`run_in_sim.launch` ↔ `config.rs`) 🔴

| 参数 | 官方 | 我们 | 状态 |
|---|---|---|---|
| constraint_points_perPiece | 5 | 12 | 🔴 |
| weight_obstacle / _soft | 1e4 / 5e3 | 同 | ✅ |
| weight_swarm / _feas / _sqrvar | 1e4×3 | swarm/feas 1e4,sqrvar(=uniform)1e4 | ✅(uniform 公式待改) |
| weight_time | 10 | 10 | ✅ |
| weight_formation | 100 | 100 | ✅ |
| obstacle_clearance / _soft | 0.1 / 0.5 | 同 | ✅ |
| swarm_clearance | 0.5 | 0.5 | ✅ |
| max_vel / max_acc | 1.5 / 6.0 | 同 | ✅ |
| max_jer | 20(仅读取,代价未用) | 10(仅用于我们自加的 post-check) | 🟡 |
| grid resolution / inflation | 0.1 / 0.0(+1 体素) | 0.1 / 0.2 | 🟡 |
| polyTraj_piece_length | launch 未显式给(default 见 manager) | 1.5 | 🟡 |
| planning_horizon | 7.5 | 7.5(planning_distance) | ✅ |

## 11. 我们的自加项(官方没有,保留为文档化偏离)

- `ensure_feasible` 时间等比缩放 post-check(v1 概念;官方 v2 只靠惩罚,无 post-check)——保留(安全兜底)。
- `REBOUND_MAX_ITERATIONS=40` 全局迭代预算、restart 上限 6(官方 3,注释说明放宽原因)、`simplify_path` 膨胀检查、脱困回退、odom 新鲜度回退。

## 移植优先级

1. **🔴 约束点生成**(finely/roughly check:稠密采样 + in/out 自由点搜索 + A* 绕障 + 交点平面)+ 约束点布局 N·K+1 + κ=5 —— 本次已实现。
2. **🔴 cost 对齐**(梯形权重、2/3 截断、uniform 官方公式、feasibility 去 jerk)。
3. **🔴 冷/暖启动**(case 1 最小 jerk 采样、case 2 均匀时长 + 全局 MINCO 轨迹)+ 全局 MINCO 轨迹取代折线。
4. **🔴 FSM**(100Hz+20Hz 双环、稠密 pts_chk 安全检查、EMERGENCY_STOP、航点序列、随机重试)。
5. 🟡 L-BFGS delta 1e-3;参数全表对齐。
