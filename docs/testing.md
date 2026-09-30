# 测试契约

## 数学依据

规划器对照 EGO-Planner-v2 `9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`
的 `swarm-playground/main_ws/src/planner/`。参考源码可放在
`~/Projects/EGO-Planner-v2`；CI 不下载源码，不需要 ROS 或 RMUC 资产。

| 层级 | 验证内容 | 验收依据 |
|---|---|---|
| 公式 | 障碍硬/软权重、双方净距、对端时间偏移 | 手算解析值与官方公式，包含实际代价组装 |
| 微分 | 各代价、MINCO 伴随传播、时间梯度 | 中心差分，不把实现中的梯度复制成期望值 |
| 反例 | 非零端点再分配时间、Rebound 采样点、再分配后的对端冲突 | 固定输入；边界保持、真实极值与拒绝语义 |
| 输出 | 完整起终点、PVA 连续、速度/加速度/jerk、占据碰撞 | 独立极值求根与稠密采样 |
| 管理器 | 目标速度、对端时钟、监控触发、不可恢复状态 | 状态机行为与输出契约 |

`planner_regression_tests.rs` 检查公式组装和修复对应的数学边界。
`tests/support` 的极值判定使用“递归求导划分单调区间 + 二分”，独立于生产代码
的 Sturm/四次方程实现。位置、速度、加速度边界误差要求小于 `1e-6`；独立极值
验收允许 `1e-6` 相对数值误差，生产代码后检查只允许 `1e-9`。

几何验收的时间步对应最多 `resolution/32` 的位移，并覆盖末端。
这是一项数值回归检查，不是连续时间无碰撞证明。生产代码的最终静态验收使用
Bernstein 凸包递归覆盖曲线；测试以独立采样检验输出，另有毫米级擦角、弯曲穿越、
末端碰撞和虚拟墙的解析几何反例。

## 确定性任务与随机安全属性

`planner_contracts` 纳入普通 `cargo test`，没有 ignore 或机器相关耗时门槛。

- `mandatory_*` 与 `planner_e2e` 中指定的可完成任务必须成功；墙绕行、非零端点
  状态、冷/暖启动、对向避让均有输出检查。不可达目标及固定边界冲突要求明确拒绝。
- `safety_seed_*` 使用固定种子的 30 张中央障碍地图，端点与绕行空间由构造保证。
  每例独立执行完整起终点规划；成功返回必须通过输出契约，失败只接受显式临时
  `Convergence`。非凸求解器的拒绝不计作任务成功，本组不宣称求解完备性或成功率。
- 必须成功的用例与随机安全属性互补：后者不能防止“全部拒绝”的退化，前者负责
  约束任务完成能力。不能用随机属性通过数代替规划成功数。
- 新增数学缺陷时先保存最小反例；不得靠提高容差或降低整体成功率门槛让测试通过。

性能结论需要相同构建、硬件与输入上的 trace；单测墙钟不承担性能验收。

## Python 评测与系统测试

`tests/evaluation/metrics.py` 只处理评测副本，固定尺度，提供位置 ATE 和一秒位置
增量 RPE。RPE 按 `t` 与 `t+1s` 配对，支持异频/不规则采样；不足一秒返回 `None`
并报告零配对数。时间必须严格递增，非法数值与形状报错。评测结果不得回流估计器。

`tests/system/test_sensor_startup.py` 默认显式 skip，须设置
`FIREFLY_RUN_SENSOR_STARTUP=1`。启用后缺资产、缺 release 二进制或启动失败均报错。
它需要 RMUC 资产、图形会话及空闲 IPC 环境，录制只写 `logs/*.rrd`，子进程经
SIGINT 退出；只验证传感器初始化和上锁零推力，不等于实际飞行验收。

```bash
cargo test --workspace
cargo test -p firefly-planner --test planner_contracts
uv run --all-packages --extra test pytest tests/evaluation/ tests/system/ -q -p no:cacheprovider
```

CI 的 Rust job 执行全部普通测试；Python job 执行评测测试并报告系统测试的 skip。
手动系统验收命令见 `how_to_run.md`。
