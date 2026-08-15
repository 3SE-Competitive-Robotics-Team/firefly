# EGO-Planner 实现计划（MINCO 后端，无 v1 概念）

论文：
- MINCO: Geometrically Constrained Trajectory Optimization for Multicopters (arXiv:2103.00190)
- Swarm of micro flying robots in the wild (Sci. Robot. 2022, eabm5954)

## 算法核心（全部来自 v2 / MINCO 路线）

1. 轨迹表示：MINCO，参数 {q, T}
   - q ∈ R^{3×(M−1)} 中间点，T ∈ R_{>0}^M 分段时长，M = 5
   - 每段 2s−1 次多项式（s = 3 → 5 次），整体 C^{s−1}
   - 映射 M(T)c = b(q)，带形矩阵，banded PLU 分解 O(M)
   - 梯度传播：∂H/∂q、∂H/∂T 线性复杂度
2. 前端：A* 在占据栅格上搜索无碰撞引导路径
3. 障碍：平面模型 (x−s)ᵀv = 0，d_o = (p−s)ᵀv，生成约 0.1ms
4. 惩罚（约束转惩罚 + 均匀采样离散化，每段 κ = 5 采样点）：
   - Js 平滑：∫||p^(s)||²dt
   - Jt 总时间：sum(T)
   - Jd 可行性：max{(v²−vm²),0}³ + 同加速度/加加速度
   - Jo 障碍：Σ max{(Co−d_o),0}³
   - Jw 集群避碰：椭球距离 E=diag(1,1,1/c)，Cw 安全距离
   - Jf 队形：Σ||p(t)−T_S·S·P_prf||²
5. 求解：L-BFGS（两循环递归 + BB 初始 Hessian + 强 Wolfe 线搜索）
6. 参数（Table S6）：M=5, κ=5, λs=1, λt=10, λd=λo=λw=10000, λf=100
   Co=0.3m(仿真)/0.2m(实飞), Cw=0.5m/0.4m, vm=1.5m/s, am=6m/s², jm=10m/s³

## crate 结构（DDD，能力模块化）

```
crates/
├── firefly-error/         错误设计：kind（调用者能做什么）+ status（重试语义）
│                           + 扁平 context + #[track_caller] 位置捕获
├── firefly-observability/ 可观测性：logforth（RUST_LOG + FastraceDiagnostic
│                          + FastraceEvent）+ fastrace（ConsoleReporter）
├── firefly-trajectory/    MINCO 领域：{q,T} 参数化、求值、梯度传播
├── firefly-map/           环境领域：占据栅格 + {s,v} 平面障碍
├── firefly-search/        A* 前端：6 邻域网格搜索
├── firefly-optimize/      L-BFGS：两循环递归 + BB 初始 + 强 Wolfe
└── firefly-planner/       编排：config（Table S6 参数）+ 领域组合
```

依赖方向（自上而下）：planner → {trajectory, map, search, optimize} → error
fastrace 库模式（默认零开销），planner/observability 启用 enable。

## 里程碑

- M1 ✅ spline/MINCO 数据结构（firefly-trajectory 骨架 + 校验）
- M2 ✅ map + A*（firefly-map 占据栅格/平面障碍 + firefly-search 6 邻域 A*）
- M3 ✅ optimize：L-BFGS 完整实现（二次/rosenbrock 收敛测试）
- M4 ✅ MINCO 核心：M 矩阵构造 + LU 求解 + 求值 + 梯度传播
  - 验证：边界条件 / 中间点穿越 / C² 连续 / 单段闭式解 / 数值梯度
  - 端到端：MINCO + L-BFGS 时空联合优化收敛（planner 集成测试）
  - 时间正性：对数参数化 T = exp(τ)
- M5 ✅ cost：Js/Jt/Jd/Jo 及解析梯度（firefly-cost crate）
  - Smoothness（闭式 Gram）/ Time / Feasibility / Obstacle（平面距离）
  - 验证：逐项 dF/dc + 端到端 (dq, dt) vs 数值差分
- M6 ✅ planner：Rebound 主循环（A* → MINCO → L-BFGS → post-check）
  - 论文 v1 Alg.2 OneStepOptimize：每轮少量迭代 + 新障碍信息
  - 每个约束点私有 {s,v}（补充材料 S6），s 为障碍表面点（Fig.3）
  - 可行性 post-check：时间等比缩放（v1 时间重分配思想）
  - 验证：穿墙轨迹逃逸 / 安全轨迹直返 / e2e 绕墙+可行
- M7 ✅ swarm：Jw 集群避碰 + Jf 队形 + Ju 均匀分布 + 多机接口
  - SwarmPenalty：椭球距离 E=diag(1,1,1/c)，绝对时间采样（Eq. S28–S33）
  - FormationPenalty：Jf = ‖p(t)−g(t)‖²（引导轨迹跟随）
  - UniformPenalty：约束点均匀分布（Eq. S34–S36，防段时长消失）
  - plan_in_swarm：分布式接口（接收 peer 轨迹，只规划自己）
  - 验证：绝对时间梯度端到端 / 避让静止 peer / 对飞迭代收敛
- M8 ✅ 验证：随机地图 benchmark（论文 v1 Sec. VI-B 方法）
  - 30 随机圆柱障碍场景：成功率 97%（29/30）
  - 规划耗时（release）：avg 9.0ms / max 70ms（论文同量级）
  - vmax avg 1.06 ≤ 1.5（可行），能量 27.4
  - 运行：cargo test --release -p firefly-planner --test random_map_benchmark -- --ignored

## 验证方式

- 单元测试：数值梯度 vs 解析梯度（每个 cost 项）
- L-BFGS：二次函数 + Rosenbrock 收敛
- MINCO：与论文 MINCO 参考实现对比轨迹
- 随机障碍场景：成功率、计算耗时
