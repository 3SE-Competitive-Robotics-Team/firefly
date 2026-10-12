# 测试契约

## VIO 分层回归

从仓库根目录执行；无需图形会话、参考项目源码或场地资产：

```bash
cargo test-vio
cargo test-vio -- --list
cargo test-vio-synthetic
```

`test-vio` 覆盖四个 VIO 库和 `vio` 应用的所有测试 target；`--list` 列出当前用例，
避免以文档中的静态计数作为覆盖依据。普通 CI 的 `cargo test --all-targets` 包含同一组测试。
`test-vio-synthetic` 额外执行显式忽略的图像运动场景；它是诊断入口，不保证通过。

| 层级 / 入口 | 约束 |
|---|---|
| `firefly-vio-types` 单测 | JPL 四元数、姿态扰动、变量与雅可比基础 |
| `firefly-vio-core` 单测、`frontend_contracts` | 相机模型、IMU 传播、特征数据库、双目补点、FAST/NMS 与 LK 输出 |
| `firefly-vio-init` 单测 | 静止初始化、动态初始化求解与边缘协方差 |
| `firefly-vio` 单测 | 状态增广、MSCKF/SLAM、FEJ、ZUPT 与预测时序 |
| `jacobian_fd_check`、`tests/support` | 共用相机/克隆夹具；独立解析值、有限差分及当前像素残差 |
| `vio_manager::analytic_contract` | 解析 IMU 与特征轨迹的尺度、速度、倾角和零偏最终输出 |
| `synthetic_e2e` | 像素图像经完整前端与滤波器；静止启动及运动场景 |
| `vio` 应用单测 | 输入、发布与诊断消息契约 |

移植基准为 OpenVINS `69488123ed9362dd44b6f28e7f4680abbff1442b`。
图像语义对照 OpenCV `d7b17c1bbc361f608768a44d9cd89458fe70bea8`，固定像素/鱼眼
输出夹具由 OpenCV CPU 4.12.0 生成；测试运行时无需安装 OpenCV。

重点契约各自保留最小反例，不以飞行一次成功替代：

- 双目补点覆盖旧点裁剪后的起始下标、已配对右点保留和网格总预算。
- FEJ 外参导数使用冻结几何，残差使用当前状态；锚定特征检查各表示的自观测抵消。
- 直方图检查常量图、非整除网格与裁剪/插值；鱼眼检查解析边缘射线、差分及失败哨兵。
- ZUPT 视差跳过不完整观测对，使用样本标准差；初始化协方差检查完整系统的边缘分布。
- LK 对固定整数位移逐点检查跟踪成功和像素误差，不能只打印均值或忽略丢失点。

雅可比相对误差与差分步长在各测试中固定；LK 含噪整数位移误差预算为 0.1 像素。
合成纯 MSCKF 无偏场景的 3m 阈值只拦截严重发散，不代表精度达标。
`synthetic_pure_msckf_with_bias` 和 `synthetic_slam_zero_bias` 保留显式 `ignore`，
分别存在位置/零偏与末端速度的验收缺口；不得把它们计作 CI 通过的用例。

真实进程的悬停、十轮扰动悬停、22m 往返入口与固定门槛见 [IMU 契约](imu.md)。
这些用例依赖 release 构建、RMUC 资产和图形会话，普通 CI 的 skip 不计作飞行通过。

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

## A* 与全局引导契约

参考 EGO-Planner-v2 `9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`：
`dyn_a_star.h/.cpp` 的膨胀占据、26 邻域和 Closed 节点跳过；
`ego_replan_fsm.cpp::planNextWaypoint` 的单任务航点全局引导。
全局引导不拼接 A* 拐点，不承诺无碰撞；只通过局部轨迹验收后才发布可执行参考。

- `single_waypoint_matches_analytic_quintic_and_valid_22m_horizon`：22m 任务输入，
  用 `10u³−15u⁴+6u⁵` 独立解析位置/速度、速度中心差分和地图/高度边界检验
  全局引导及局部目标。解析误差 <1e-9，差分误差 <1e-7，均为数值容差。
- `firefly-search::segment`：跨越全部体素面，闭集接触也拒绝；毫米级擦角、
  沿面、同点、端点、反向、非法数值和虚拟墙。随机固定输入与独立线段/AABB
  slab 公式比较，不用步进实现自身作为预言机。
- A* 路径输出：膨胀占据外推的确切端点、同格内不同端点、连续连边净距；
  解析擦角布局必须绕行两步，总长度 2m，不能取长度 √2 的穿角连边。
- 最终多项式检查：即使两端在地图内，曲线中间越界仍拒绝。

连续连边检查是本项目的保守安全扩展，官方只查邻居节点。搜索仍绑定地图索引，
没有移植官方随起终点中点移动的独立节点池；不能把这些行为称作官方实现逐项等价。
局部冷启动对照 `planner_manager.cpp::computeInitState`：2s 多项式种子、
`max(2, round(distance/piece_length))` 段、等时重采样及统一初始段时长
`piece_length/max_velocity`。A* 用于碰撞段的 Rebound 约束生成。
解析五次多项式、速度公式及位置中心差分同时验证种子采样与边界条件。

真实 RMUC 资产的 22m 初始局部目标、轨迹优化与连续碰撞验收：

```bash
cargo test --release -p firefly-planner rmuc_22m_initial_trajectory_is_executable -- --ignored
```

该用例不替代飞行验收。缺资产必须失败，不能跳过后记作成功。
LightGlue 固定查询的采样方式、原始录制及结果见
[性能测量](performance.md)。性能采样显式启用，不设置机器相关耗时门槛。

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
cargo build --release -j 2 -p vio -p render -p fc -p ffctl -p localization -p planner -p slam
uv run --all-packages --extra test python scripts/accept_rmuc.py
```

22m 去程到会话地图坐标 `(22, 0, 1.595)m`，到位后返回启动点上方并降落：

```bash
uv run pytest tests/system/test_roundtrip.py
```

该无 mock 用例直接调用正式验收器 `scripts/accept_rmuc.py`，使用
`configs/acceptance_22m.toml`，默认空地图、在线回环和深度建图；不加载离线地图。
运行前须准备 RMUC 资产及图形会话，并构建运行程序：

```bash
cargo build --release -p render -p vio -p fc -p ffctl -p localization -p planner -p slam
```

初始化、往返真值精度、轨迹跟踪、在线回环、降落、碰撞和进程退出沿用正式验收阈值，
任何一项失败均使 pytest 失败；JSON/HTML 汇总与原始 RRD 位于 `logs/test_roundtrip_22m_*/`。
`tests/system/conftest.py` 共用运行条件检查、证据目录与退出核对；显式指定文件即执行，
普通全仓测试收集时跳过真实飞行。悬停入口仍为
`uv run pytest tests/system/test_imu_flight.py -k imu_hover`。

该配置只指定航点，飞行、定位、碰撞及时间预算均使用默认验收条件。
运行前完成全部编译，避免编译负载与传感器进程争抢资源。

纯 VIO 的相同路线对照：

```bash
uv run --all-packages --extra test python scripts/accept_rmuc.py --case nominal --config configs/acceptance_22m.toml --vio-only
```

该模式不启动 slam 或 ALIKED/LightGlue，不要求模型或冻结视觉库；localization 与 planner 使用
和完整链路相同的默认入口。报告将视觉前端与回环标为关闭，真值评分仍使用相同的飞行、
地图误差、碰撞、时间预算阈值。默认在线 SLAM 任务额外从 RRD 检查已接受回环约束；
外部地图模式另检查至少两次已接受地图观测，二者不阻塞规划启动。
VIO 对照不能替代完整视觉链路的验收。


配置为 `configs/acceptance.toml`（`--config` 可覆盖），缺键采用
`tests/system/mission.py::Options`。`--case nominal|reference_loss|estimator_loss`
可单独复现。要求独占本仓库闭环进程与图形会话；检测到其他进程时拒绝运行，
不终止其他任务。命令默认不重建二进制；完整场地准备入口会先 release 构建再调用它。

单程任务可配置 `waypoints = [[22.0, 0.0, 1.595]]` 与 `finish_action = "hold"`：
到达后在终点持续悬停，再停止仿真。悬停由新鲜估计状态与 FC 模式判定，RRD 中
真值独立评分 `terminal_hold_accuracy`；不执行或声称通过降落验收。
`finish_action` 缺省为 `land`，用于返回停机坪的任务。

`--prior-maps --localization-config <含显式 origin 的配置>` 启用外部地图模式；
航点与规划高度边界必须使用该外部地图坐标。默认在线模式无需两份预建地图。
`--allow-map-rig-translation` 仅用于外部地图模式，是显式的冻结视觉库复用例外：只允许左目和深度相机
发生相同的机体内平移，其他几何、内参、相对标定与外观契约必须相同。三维路标
保留地图坐标；采集时机体位置仍用于候选检索，位置偏移和例外状态写入报告的
`provenance.visual_map_compatibility`。该开关不修改冻结地图或其生成清单。

10m 去程并返回的四次独立进程验收：

```bash
uv run --all-packages --extra test python scripts/accept_rmuc.py --case nominal --config configs/acceptance_10m.toml --repeat 4
```

该配置使用会话高度 1.795m、航点 `(10,6,1.795)` → `(0,0,1.795)`，每段预算
90s 仿真时间；到达与误差阈值采用相同默认值。未通过估计状态的去程到达判定时不会进入返航。
各次使用相同噪声种子，不能视为独立随机样本；RRD 分别保存在 `attempt_NN/`。
单次任务失败仍继续下一次，进程未退出或用户中断则停止。

| 验收项 | 固定契约 |
|---|---|
| 初始化 | 真实 IMU/图像，屏蔽 PlantState；静止窗口至少 1s，局部原点误差 <0.1m，上锁电机为零 |
| 起飞 / 悬停 | 起飞 1m；高度误差 <0.15m，水平偏移 <0.5m，速度 <0.2m/s，连续保持 3s |
| 路径跟踪 | VIO、在线回环与 planner，会话航点 `(2,0,1.595)` → `(0,0,1)`；逐点到达误差 <0.35m、速度 <0.3m/s 持续 1s |
| 跟踪评分 | 同时刻 GT 与实际 Reference，禁止轨迹拟合；RMSE ≤0.35m，最大误差 ≤0.8m |
| 局部定位 | 固定尺度航向/平移对齐；VIO ATE RMSE ≤0.2m，1s 位置增量 RPE RMSE ≤0.15m |
| 地图定位 | 禁止轨迹拟合；位置 RMSE ≤0.25m，航向 RMSE ≤15°；外部地图模式另要求至少两次已接受库图观测 |
| 在线回环 | 默认在线 SLAM 往返至少一条已接受回环约束；库图观测不能替代 |
| 碰撞 | MuJoCo 每物理步累计；只豁免停机坪半径 0.5m、高差 0.06m、速度 <0.5m/s 且法线竖直余弦 ≥0.9 的接触；其他接触必须为零 |
| 参考失联 | SIGINT 停止 planner；2.5s 墙钟内进入 Hold，在失联点 0.5m 内连续保持 2s |
| 估计失联 | SIGINT 停止 VIO；0.8s 内进入姿态降级（编码 6），连续 3s 保持非零推力且无碰撞；安全落地明确标为 not_supported，不计作任务成功 |
| 退出 | 只向本次子进程发 SIGINT，15s 内退出；不得遗留进程或用 SIGKILL 掩盖退出失败 |

`tests/system/mission_sim.py` 只加评测观测：固定 IMU 噪声种子、读取接触、拒绝
静默物理重置。控制律、传感器生成与物理步进仍调用生产实现；噪声种子不控制
OS/GPU 调度。正常起飞、悬停、到达和返航由新鲜估计状态驱动，降落结束依据 FC 上锁。
真值在 RRD 读回后独立评分悬停、航点和着陆；估计到达不等于真实到达。评测器的
碰撞/越界终止仅是实验保护，不是实机任务逻辑。schema 2 报告区分
`estimated_waypoints_completed` 与 `all_waypoints_reached`，与真值驱动流程的历史结果
不可直接比较任务完成率。

数据映射：`acceptance/{gt,odom,corrected,reference}` 为各自原坐标系的
`Transform3D`，`*_velocity` 与 `motors/physics/penetration` 为 `Scalars`，
均为 `sim_time` 上的实时原始观测，经 Firefly/Viz 聚合；阶段、指令和故障注入
写入 `logs/acceptance` 的 `TextLog`。会话地图评分用首个真值位置/航向确定固定换基，
不拟合估计轨迹，不向控制链反馈；原始坐标保留在 RRD，换基参数写入报告。
接触计数以 200Hz 物理步统计并以 20Hz 发布累计值，位姿评分约 10Hz；记录缺失、
回退时钟或超过 0.5s 的评分数据断流不能计为通过。

每次报告目录 `logs/acceptance/<UTC时间-随机ID>/` 包含每用例 RRD、`report.json`
与 `report.html`。JSON 保留 Git 提交及脏状态、源码/二进制/资产/RRD SHA-256、
完整配置、运行依赖版本、阶段时间窗、实际指标与退出码。HTML 为可读汇总。
中途失败后的依赖阶段标为 blocked；碰撞通过只覆盖报告中实际观测到的时间，
不能代替尚未完成的整段任务。默认入口验证在线回环，库图重定位仅在外部地图模式验收。

## 深度注册与在线回环

公式、固定参数、参考源码版本和实现边界见 [在线回环](loop_closure.md)。
普通测试包含：

- 深度基线解析位移与导数、倾斜平面、z-buffer、空洞、深度边缘和非法标定。
- RGB-D 已知刚体运动、25% 外点、共线拒绝、平移导数。
- 图的四自由度残差解析值、雅可比中心差分（`1e-6` 步长，`1e-6` 误差）、
  鲁棒目标梯度（`1e-5` 误差）、位置/航向闭合与原始 odom 不变。
- 未覆盖时间不外推、乱序拒绝、初始化丢失锁存与错误几何拒绝。

```bash
cargo test -p firefly-landmarks -p slam -p firefly-vio -p firefly-vision-match -p firefly-localization -p lightglue -p localization
```

`firefly-landmarks` 检查解析投影、雅可比差分、已知三维点与位姿恢复、
双侧几何外点拒绝、无当前深度时的持久几何、真实观测三角化与共视关系。
`slam` 使用校准的合成双目图检查图像到三维点的完整输入输出，并验证后端确认的
端点匹配、缺失快照拒绝、原始位姿不变和离线锚点修正同步。
这些检查不能代替以下无 mock 飞行用例：

```bash
uv run pytest tests/system/test_imu_flight.py -q -k imu_hover
uv run pytest tests/system/test_roundtrip.py -q
```

`accept_rmuc.py` 验证真实进程闭环、地图定位和故障处置；报告单独汇总在线
关键帧、候选与已接受回环约束。未完成返航重访时，零回环不能证明回环能力通过或失败；
不能用单测或库图更新次数代替长路线重访验收。


## 冻结视觉资产的独立视角验收

采集帧先检查纹理与有效深度，建库再要求每帧至少 64 个有效路标且覆盖至少 6/16
图像分区。几何标签不施加在线深度噪声。训练帧自匹配只验证接口，不证明定位泛化。

`lightglue::validation::held_out_map_localization_contract` 使用独立采集的查询库：
查询只提供二维特征与描述子，查询位姿仅作已知的离线测试标签。每个查询位置必须距
所有训练位置超过 0.1m；先验叠加 `[0.3,-0.2,0.1]m` 偏差，不作事后对齐。
要求每个指定查询均得到至少 30 内点、内点率 ≥0.3、重投影 ≤3px，位置误差 ≤0.25m、
旋转误差 ≤15°。拒绝或缺失的查询均不能计为通过。

先启动 `firefly-viz --save logs/map_validation.rrd`，再执行：

```bash
RUST_LOG=info FIREFLY_QUERY_MAP=<独立查询.ffvmap> FIREFLY_QUERY_COUNT=20 \
  FIREFLY_MAP_VALIDATION_REPORT="$PWD/logs/map_validation.json" \
  cargo test --release -p lightglue held_out_map_localization_contract -- --ignored --nocapture
```

该用例只评测库图定位，不替代在线状态融合、飞行或回环验收。大型地图使用缓冲 I/O，
内容与 FFVMAP v2 格式保持一致。库图清单保存模型与生成来源的 hash；兼容性只检查
场地、标定和成像外观。`build_vision_map.py` 在契约一致时复用库，在替换前归档原资产。

## VIO 与地图融合的数值契约

持续悬停回归入口与阈值见 [本地姿态门控](imu.md#参考与测试)。新增门控须用当前固定
配置独立启动三次，三次全部通过才建立基线，失败不得用重试后的通过覆盖。
降落的触地安全与上锁后的静止状态分别评分：上锁后固定等待 1 秒，再检查连续 1 秒
静止；全过程接触/重置判定保持有效，时间窗口不能依真值自适应后移。

- `track` LK 偏置测试：已知整数平移的平均误差 <0.25px，保留点数 ≥90%；
  遮挡夹具同时要求拒绝消失的纹理、保留仍可见点的正确位移。双向检查对照
  VINS-Fusion `FLOW_BACK`，不能排除所有重复纹理误匹配。
- `vio_manager/analytic_contract.rs`：10m 加速、非恒定航向、20° 相机下倾的双目
  解析投影，传播、三角化和 SLAM 更新全部执行；全程位置与最终速度误差 <0.01。
  这隔离滤波数学，不替代真实图像前端或真实飞行验收。
- SLAM 观测消费：体素未选中的已有路标不能进入 MSCKF；重新选中后的协方差
  必须等于单次 SLAM 更新，不能重复增益。同一批未选中观测不能被提前清理。
- 静止陀螺初始化：独立白噪声/游走闭式值、均值的有限差分以及完整初始化输出
  共同验证零偏协方差；噪声由部署配置同步到初始化器与传播器。
- 位姿图：一秒运动拆成 1/2/4/10 段，对等方差下的解析融合位置均为 1.05m，
  容差 1e-6m；另有四自由度残差与整体目标函数的差分检查。
- 占据图：重复像素每帧只累积一次证据；先验经空闲射线、3000 次衰减、动态移除、
  窗口移出/移入后仍占据。飞控测试要求起飞完成容差不改变指令的 Hold 位置与航向。
- 仿真步进：发布的位姿、IMU、接触缓存与积分后的 `data.time` 同刻；常速水平
  运动、自由落体和旋转分别校验位置、速度与角速度。验收噪声使用显式 Generator 种子。
- 状态输入：重复、乱序、无穷/NaN、未初始化及 500ms 墙钟断流不能维持规划；
  锁存后不能被后续好消息自动解除。默认定位后端在没有库图观测、观测超期时仍输出
  新鲜 VIO 的地图变换；未对齐或估计器重置不得输出。库图观测质量只用于诊断。

- 图像时刻先验：90° 转向和平移的解析坐标组合，时间窗口外必须拒绝。
- 电机分配：偏航饱和时保留总推力与滚转/俯仰力矩；总推力对请求值导数为一。
  参考流失效的首拍就锁定当前位置，模式超时不重置锚点。

数值审查与实际飞行证据见 [VIO 与跟踪审查](vio_tracking_review.md)。

## IMU 姿态估计

`firefly-imu` 的解析、差分与输出契约，以及 VIO 外援的时间/协方差接口测试，
见 [本地姿态估计](imu.md#参考与测试)。数学测试通过不代表实际 IMU 标定或飞行验收通过。
