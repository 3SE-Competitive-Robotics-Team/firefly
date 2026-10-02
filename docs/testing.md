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
SIGINT 退出；未启用 `FIREFLY_RUN_FLIGHT` 时只验证传感器初始化和上锁零推力。

```bash
cargo test --workspace
cargo test -p firefly-planner --test planner_contracts
uv run --all-packages --extra test pytest -q -p no:cacheprovider
```

CI 的 Rust job 执行全部普通测试；Python job 执行 `tests/`、`apps/`、`packages/`
内的普通测试并报告系统测试的 skip。测试发现范围不包含生成资产与 Blender 的隔离依赖。
手动系统验收命令见 `how_to_run.md`。

## CAD 与真实资产

`packages/firefly-cad/tests` 验证解析长方体外法线、负坐标格心、空腔分离、
碰撞体积、失配资产拒绝，以及输入/产物变化后的缓存失效。
离线采集测试验证同帧配对、二进制格式、逐帧内容校验与不覆盖失败产物的重试。
`FIREFLY_RUN_RMUC_ASSETS=1` 启用 `tests/system/test_rmuc_assets.py`，检查真实
资产 hash、出生点穿透与无推力静置。

`cargo test --release -p firefly-planner --test rmuc_asset -- --ignored`
读取实际导出地图，执行停机坪前方爬升任务并验收完整轨迹契约。

启动测试为大型场地加载与 GPU 管线初始化留 90s 墙钟预算；算法仍必须通过
静止测量与局部原点验收。额外设置 `FIREFLY_RUN_FLIGHT=1` 后执行起飞 1m、
持续两秒稳定悬停和降落：水平偏移 <0.5m，高度误差 <0.15m，悬停速度 <0.2m/s。
真值仅用于评测，飞控仍只消费传感器估计。失败保留 RRD，不放宽门槛。

## 自动任务验收

```bash
cargo build --release -j 2 -p vio -p render -p fc -p ffctl -p localization -p planner -p aliked -p lightglue
uv run --all-packages --extra test python scripts/accept_rmuc.py
```

配置为 `configs/acceptance.toml`（`--config` 可覆盖），缺键采用
`tests/system/mission.py::Options`。`--case nominal|reference_loss|estimator_loss`
可单独复现。要求独占本仓库闭环进程与图形会话；检测到其他进程时拒绝运行，
不终止其他任务。命令默认不重建二进制；完整场地准备入口会先 release 构建再调用它。

| 验收项 | 固定契约 |
|---|---|
| 初始化 | 真实 IMU/图像，屏蔽 PlantState；静止窗口至少 1s，局部原点误差 <0.1m，上锁电机为零 |
| 起飞 / 悬停 | 起飞 1m；高度误差 <0.15m，水平偏移 <0.5m，速度 <0.2m/s，连续保持 3s |
| 路径跟踪 | 真实视觉定位与 planner，地图系航点 `(-11,0,2)` → `(-13,0,1.405)`；逐点到达误差 <0.35m、速度 <0.3m/s 持续 1s |
| 跟踪评分 | 同时刻 GT 与实际 Reference，禁止对齐；RMSE ≤0.35m，最大误差 ≤0.8m |
| 局部定位 | 固定尺度航向/平移对齐；VIO ATE RMSE ≤0.2m，1s 位置增量 RPE RMSE ≤0.15m |
| 地图定位 | 至少两次已接受的视觉融合；禁止对齐；位置 RMSE ≤0.25m，航向 RMSE ≤15° |
| 碰撞 | MuJoCo 每物理步累计；只豁免停机坪半径 0.5m、高差 0.06m、速度 <0.5m/s 且法线竖直余弦 ≥0.9 的接触；其他接触必须为零 |
| 参考失联 | SIGINT 停止 planner；2.5s 墙钟内进入 Hold，在失联点 0.5m 内连续保持 2s |
| 估计失联 | SIGINT 停止 VIO；0.8s 内进入姿态降级（编码 6），连续 3s 保持非零推力且无碰撞；安全落地明确标为 not_supported，不计作任务成功 |
| 退出 | 只向本次子进程发 SIGINT，15s 内退出；不得遗留进程或用 SIGKILL 掩盖退出失败 |

`tests/system/mission_sim.py` 只加评测观测：固定 IMU 噪声种子、读取接触、拒绝
静默物理重置。控制律、传感器生成与物理步进仍调用生产实现；噪声种子不控制
OS/GPU 调度。地面真值仅用于验收判定，不生成目标坐标或估计器修正。

数据映射：`acceptance/{gt,odom,corrected,reference}` 为各自原坐标系的
`Transform3D`，`*_velocity` 与 `motors/physics/penetration` 为 `Scalars`，
均为 `sim_time` 上的实时原始观测，经 Firefly/Viz 聚合；阶段、指令和故障注入
写入 `logs/acceptance` 的 `TextLog`。位置评分只读取这些 RRD，不向控制链反馈。
接触计数以 200Hz 物理步统计并以 20Hz 发布累计值，位姿评分约 10Hz；记录缺失、
回退时钟或超过 0.5s 的评分数据断流不能计为通过。

每次报告目录 `logs/acceptance/<UTC时间-随机ID>/` 包含每用例 RRD、`report.json`
与 `report.html`。JSON 保留 Git 提交及脏状态、源码/二进制/资产/RRD SHA-256、
完整配置、运行依赖版本、阶段时间窗、实际指标与退出码。HTML 为可读汇总。
中途失败后的依赖阶段标为 blocked；碰撞通过只覆盖报告中实际观测到的时间，
不能代替尚未完成的整段任务。该入口验证 ALIKED/LightGlue 库图重定位，不验证在线回环。

## 深度注册与在线回环

公式、固定参数、参考源码版本和实现边界见 [在线回环](loop_closure.md)。
普通测试包含：

- 深度基线解析位移与导数、倾斜平面、z-buffer、空洞、深度边缘和非法标定。
- RGB-D 已知刚体运动、25% 外点、共线拒绝、平移导数。
- 图的四自由度残差解析值、雅可比中心差分（`1e-6` 步长，`1e-6` 误差）、
  鲁棒目标梯度（`1e-5` 误差）、位置/航向闭合与原始 odom 不变。
- 未覆盖时间不外推、乱序拒绝、初始化丢失锁存、连续确认与错误深度拒绝。

```bash
cargo test -p firefly-vision-match -p firefly-localization -p lightglue -p localization
```

真实资产测试显式启用，`FIREFLY_LOOP_CAPTURES` 指向视觉地图清单中
`capture_manifest` 所在目录，模型和地图需先按标准 pipeline 构建：

```bash
FIREFLY_LOOP_CAPTURES=models/rmuc2026/derived/vision/170f31928eb82cd9 \
  cargo test --release -p lightglue -- --ignored
```

`real_model_revisit_contract` 必须生成经过三次确认的回环，注入 0.4m 漂移后，图优化
输出误差小于 0.2m，并拒绝低质量回环。它重访同一采集图，不代表跨视角飞行成功。
`real_model_insufficient_overlap_rejection_contract` 固定使用 `[-13,-1,1.2]` 与
`[-12,-1,1.2]`、同航向的两帧 RMUC 库图，检查不足 30 对应时明确拒绝；该拒绝
不计为定位或回环任务成功。不能把场景中的一米位置差直接等价为足够视觉重叠。

普通 `accept_rmuc.py` 验证真实进程闭环、地图定位和故障处置；报告使用
`not_scored_in_this_mission` 标注在线回环，不能用单测或库图更新次数代替长路线重访验收。
