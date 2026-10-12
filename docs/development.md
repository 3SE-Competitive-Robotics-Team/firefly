# 开发指南

开发、运行、记录和验证统一在本文；系统职责、坐标、算法及能力边界见
[architecture.md](architecture.md)。路径和命令均相对仓库根目录。
跨模块约定只维护这两份文档；README 只提供介绍和入口。参数以代码默认值和配置为准，测试用例与门槛以测试源码为准。
历史排查和试飞结果留在 Git 历史及 `logs/` 证据中，不另建平行规范。

## 实现约定

- 新 crate：`cargo new crates/<name> --lib --edition 2024`（自动纳入 workspace）。
- DDD 拆 crate（firefly-*），依赖自上而下，领域层不依赖应用层。
- 错误用 firefly-error：kind/status 分类，模块边界 .with_context。
- 可观测性：关键函数 `#[fastrace::trace]`，日志用 log 宏，禁 println!。
- 场景光照唯一一份（`firefly-render::lighting`），所有相机共用；视角差异只写相机
  配置（曝光/色调映射），禁止给单个相机加专属可聚簇光源（`PointLight`/`SpotLight`）
  ——Bevy GPU 聚簇在多视图同帧渲染时不按 `RenderLayers` 隔离，会渗进别的视角。
- 依赖统一在根 `[workspace.dependencies]`。

### 注释规范

- **禁止过程叙事**：注释只描述现在的设计与约束，不写「曾/旧版/之前/原来/不再/
  修复了/从 X 改为 Y/当时」等演变史——历史归 git log 与 commit message。
- **禁止 bug 战争故事**：现象、复现步骤、排查过程、修复日期、新旧实测对比一律不进
  注释。背后有仍成立的陷阱时，改写成祈使句约束（「必须 X，否则 Y」），只留结论。
- **禁止阶段/计划叙事**：「wave N」「待移植」「尚未接入」「后续将」等不进代码——
  代码只陈述已存在的事实，计划放 issue。
- doc 注释（///、//!）：写用途、单位、不变量、边界条件；与参考实现的对应关系
  （如「对照 OpenVINS xxx」）保留。
- 行内注释（//）：只在代码无法自解释时解释「为什么」，不复述代码；注释掉的死代码
  直接删除。

### 配置

- **TOML、一应用一份**：统一放仓库顶层 `configs/`（`sim.toml` / `vio.toml` / `planner.toml`），
  启动时加载（Rust 支持 `--config` 换文件），缺文件即报错。
- **最小化**：只写与代码默认值不同的键，缺失键回落 `*Options::default()`；
  纯数据 Options 直接 `serde::Deserialize` + `#[serde(default)]`，Python 用标准库
  `tomllib`；不引 YAML。

标定与噪声改对应 Options 默认值，doc 注释写单位和来源；部署覆盖同步 `configs/`。
升级视觉依赖必须运行 LK 位移回归，不放宽容差；依赖统一在根 Cargo/uv workspace 和锁文件。
改行为前阅读对应官方源码，固定版本与差异见架构文档。参考仓库位于 `~/Projects/`：
`open_vins`、`ORB_SLAM3`、`VINS-Fusion`、`EGO-Planner-v2`、`iceoryx2`、`logforth`、`fastrace`；
purecv 使用 `XiaoPengYouCode/purecv` fork，锁定版本见 Cargo.lock，发现上游问题应提 issue。

<a id="run"></a>

## 构建与运行

只支持 `configs/scene.toml` 指定的 RMUC2026。必须有 `models/rmuc2026/field.glb`、
`rmuc2026_collision.json` 和 `models/drone/drone.glb`，缺失报错，不回退场景。
停机坪定义来自 `firefly_mujoco.scene.PAD` / `PAD_CLEARANCE`，不在文档复制坐标。
运行需图形会话；Rust/Python 版本和依赖以 workspace 配置为准。

```bash
uv sync --all-packages --extra test
cargo build --release -p render -p vio -p fc -p ffctl -p localization -p planner -p slam
```

运行进程一律 release；普通 `cargo test` 用 debug。源码改动后先完成构建，再做运行验收。
分别在终端启动，记录进程最先启动、最后退出：

```bash
uv run firefly-viz --save logs/rmuc_run.rrd
uv run firefly-sim
cargo run --release -p render
cargo run --release -p vio
cargo run --release -p fc
cargo run --release -p localization
cargo run --release -p slam
cargo run --release -p planner
```

最小闭环只需 viz/sim/render/vio/fc。纯 VIO 规划对照不启动 slam，保留 localization/planner
的相同入口。离线库图显式启动 aliked 和 `lightglue --map <地图.ffvmap>`；静态规划图用
`planner --map <地图.ffmap>`，两者均要求 `configs/localization.toml [origin]` 声明外部对齐。
ESKF 对照设置 `backend="eskf"`，不启动 slam。禁止把场地真值当作默认定位或规划先验。

静止初始化就绪后，按任务顺序发送指令（不要一次执行整组）：

```bash
./target/release/ffctl fc arm
./target/release/ffctl fc takeoff 1.0
./target/release/ffctl fc hold
./target/release/ffctl fc track
./target/release/ffctl planner goal 22 0 1
./target/release/ffctl planner goal 0 0 1
./target/release/ffctl fc land
```

起飞高度相对起飞点，planner 目标属于当前地图坐标。拒绝原因查看 RRD 的 `logs/fc`。
上电不自动起飞，未就绪不能绕过初始化。飞控失联行为见架构文档。

### 进程与宿主机

- 启动前核对 PID、可执行文件和工作目录，确认没有其他闭环、采集、录制或验收任务占用。
  只管理确认属于本任务的进程，禁止按模糊名称批量杀进程。
- 编译结束后采样实际 CPU/GPU、温度和降频计数增量；资源稳定再运行。`ps %CPU` 的生命周期
  均值不替代采样。后台负载或残留未排除时不作性能归因，不改验收阈值，证据进 RRD。
- 所有成功、失败、超时和中断路径均发 SIGINT、等待子进程退出、回收线程与管道、完成 RRD
  flush；核对所启动 PID 及子进程。残留须显式报错并阻止下一轮，超时不算清理完成。
- iceoryx2 终止信号读取会消费事件；接收线程退出后设置停止标志，主循环同步检查，线程
  错误传播并回收。不能假设所有节点各自收到 SIGINT。
- 禁止用 SIGKILL 掩盖退出错误：它可能遗留 shm 和幽灵端口。macOS 排障清理仅在所有相关
  进程已退出后删除 `/tmp/iceoryx2/services` 与 `/tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state`。

<a id="assets"></a>

## 场地与可选离线资产

```bash
uv run --all-packages --extra test python scripts/prepare_rmuc.py <场地.stp>
```

`--assets-only` 仅构建验收几何；入口按阶段报告，不保证飞行通过。
构建器为 `firefly-cad-build`，配置为 `configs/cad.toml`。GLB、碰撞盒、FFMap 必须来自
同一归一化几何，通过内容 hash、坐标和占据检查；细节见 `packages/firefly-cad/README.md`。

`scripts/collect_vision_map.py` 使用 `render --offline` 和隔离的 `Firefly/Offline/*`，
按同一时间戳配对左图、理想深度与位姿标签，不启动在线估计器/飞控，不把标签发布到在线话题。
无同步帧不得建库；图像纹理、路标覆盖与内容指纹由采集和构建器检查。
库图兼容性只依赖场地、标定、光照和曝光；源码/二进制 hash 用于追溯，普通重编译或在线
噪声变化不要求重建。不能改历史 manifest 冒充原采集；替换资产前归档。

官方预训练模型的可复现导出（CPU，独立环境，不影响运行环境）：

```bash
uv venv models/vision-export/.venv --python 3.12
uv pip install --python models/vision-export/.venv/bin/python torch==2.14.1 torchvision==0.29.1 --index-url https://download.pytorch.org/whl/cpu
uv pip install --python models/vision-export/.venv/bin/python onnxscript==0.7.2 onnx==1.23.1 onnxruntime==1.30.0 kornia==0.8.3 opencv-python==5.0.0.93 matplotlib==3.11.2
models/vision-export/.venv/bin/python scripts/export_vision_models.py
uv run --all-packages python scripts/build_vision_map.py <通过内容校验的采集目录>
```

使用官方 ALIKED-N16（128 维）与匹配的 LightGlue 权重，320×240、512 点。
T16 为 64 维，无法直接使用官方 ALIKED LightGlue 权重。导出保留全部匹配层，
关闭依赖输入数据的动态裁剪和提前退出；不将未训练的截断网络作为轻量模型。
导出器校验可变形卷积与 torchvision 的一致性、ONNX 与 PyTorch 输出一致性，
`models/vision_models.json` 保存源码版本、权重及模型 hash、数值检查结果。
视觉地图旁的 `.manifest.json` 关联模型、构建器、采集清单和场地指纹。
模型与地图是本地产物，不进入 Git；上游源码及权重的许可见固定版本的 LightGlue 仓库。

<a id="diagnostics"></a>

## 记录与性能

正式运行原始证据只写 `logs/*.rrd`，不写文本日志或 `/tmp`；JSON/HTML 只存配置、
内容指纹和汇总。Rust 计算线程经 `Firefly/Viz` 发数值，经 `Firefly/Log` 发 log 宏诊断，
由 firefly-viz 统一落盘；stderr 仅作进程诊断的双 sink，不作第二份录制。

各进程以测量时刻设置 `sim_time`；无仿真时刻的进程日志回落 `wall_time`。
常用实体为 `vio/odom`、`vio/traj`、`gt/pose`、`gt/traj`、`vio/debug/*`、`fc/debug/*`、
`plan/*`、`logs/<app>`。传感器原图只在 render 面板，不进 RRD。
render 默认按原图时间配对叠加 VIO 点；图像标记和估计/真值影子只改调试显示，不进传感器。

传感器延迟按原始采集 `sim_time` 记录墙钟耗时（秒）：
`render/capture/stages_s` 为请求→提交、提交→回读、回读→装配、装配→处理；
`render/capture/publish_s` 为像素处理、处理结束→发布、请求→发布、帧序号。
这些诊断不改图像时间戳；`vio/debug/camera_age` 是消费时仿真时刻减采集时刻，
`fc/imu/health` 的外援年龄还包含视觉更新与等待下一拍的时间。

实时查看先启动 rerun，再运行不带 `--save` 的 firefly-viz，默认连接 `127.0.0.1:9876`。
离线回放 `rerun logs/rmuc_run.rrd`；数值可用 `rerun rrd print`、`rerun rrd stats` 或
Python RrdReader，读取方法见仓库 `.agents/skills/rerun/SKILL.md`。

性能只认相同构建、硬件、输入与环境下的 trace 实测。主循环建 root span、设置 local parent、
结束 flush；热路径不使用无条件格式化大对象的 `#[logcall]`。ConsoleReporter span 树
经记录进程进入 RRD；先直接读输出，不为简单查看另造解析脚本。推理微基准不代表全栈吞吐或功耗。

### 姿态与 VIO 诊断

- `fc/imu/quaternion_xyzw`：控制实际使用的 Hamilton body→odom 姿态（10Hz）。
- `fc/debug/position_target_odom` / `position_error_odom`：odom 系目标与位置误差（10Hz）。
- `fc/imu/health`：本地可用、odom 已对齐、最后接受外援的测量年龄（秒）。
- `fc/imu/accel_gate`：NIS、是否实际应用加计校正。
- `fc/imu/gyro_bias`：控制输出采用的三轴零偏，rad/s。
- `fc/imu/aid_gate`：最近一次 CI 的接受标记、六维 NIS、本地权重；未计算为 -1。
- `fc/imu/attitude_variance` / `bias_variance`：输出协方差对角，rad² / (rad/s)²。
- `fc/debug/tilt_deg` 与 `fc/debug/tilt_err_deg`：估计倾角与仅供评测的真值倾角误差。

运行记录由 `firefly-viz` 写入 `logs/*.rrd`。

#### VIO 更新诊断

所有诊断只写入 RRD，不回流估计器或控制器，时间轴为 `sim_time`。
`vio/input/gyro` 与 `vio/input/accel` 保留送入 VIO 的原始 IMU 采样。
`vio/debug/keypoints/{left,right}` 的 `track_id` 将像素轨迹与滤波路标关联。

`vio/update/{msckf,slam,landmark_init,zero_velocity}` 按相机帧汇总：

- `innovation`：残差维数之和、RMS、各次 NIS 之和、更新次数。
- `correction`：各次局部切空间的 15 维修正之和，顺序为 θ、p、v、bg、ba。
- `{before,after}/state`：首个更新前、最后更新后的 q_GtoI（JPL xyzw）、p、v、bg、ba。
- `{before,after}/variance`：15 维误差状态对角方差。
- `{before,after}/attitude_bias_covariance`：θ、bg、ba 的 9×9 联合边缘协方差。

标量每四项拆成一个实体，后续块使用 `/1`、`/2`、`/3`。
`vio/update/slam/features/{batch}` 为逐路标矩阵：列依次为 id、残差维数、RMS、NIS、
是否接受，以及该路标对 θ、bg、ba 的九维修正贡献；拒绝项的贡献为零。
各已接受路标贡献之和等于该批 SLAM 更新实际施加的修正。
姿态单位 rad、速度 m/s、陀螺零偏 rad/s、加计零偏 m/s²；协方差使用相应单位的乘积。

回环查看 `loop/debug/frontend`（关键帧、候选、内点、连续确认）、`corr/debug/loops`
（已接受约束）、`corr/debug/gate`、`logs/slam` / `logs/localization`（拒收、容量和同步）。
关键帧产生或候选匹配成功不等于后端接受。

<a id="tests"></a>

## 验证

```bash
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
cargo test -p firefly-planner --test planner_contracts
uv run --all-packages --extra test pytest -q -p no:cacheprovider
FIREFLY_TEST_PYTHON="$PWD/.venv/bin/python" cargo test -p firefly-pubsub --test python_interop -- --ignored
```

数学测试同时包含独立公式/解析值、中心差分和最终输出契约，不能只让代价与梯度互证。
确定性任务必须成功；随机安全属性允许明确有界拒绝，但拒绝不计成功，不用整体成功率
掩盖指定任务失败，不用机器相关耗时作正确性门槛。参考版本见架构文档。

- 规划检查端点 PVA、导数极值、地图/对端净距、时钟与输出轨迹一致性；测试侧独立极值
  求根和几何采样，不用生产实现验证自身。A* 连边与 Bernstein 检查不等于动态安全证明。
- 坐标、姿态、回环检查解析变换、残差/重置差分、相关信息不重复计权、会话与时间拒绝。
- 前端检查已知位移、丢点与双目约束；解析轨迹隔离滤波数学，不能代替实际纹理和飞行。
- CAD、深度、IPC 分别验证资产一致性、噪声/投票/先验保护和双语言布局、头部、事件往返。

### VIO 专项

```bash
cargo test --all-targets \
  -p firefly-vio-types -p firefly-vio-core -p firefly-vio-init -p firefly-vio -p vio
cargo test -p firefly-vio --test synthetic_e2e -- --include-ignored
```

第一条覆盖四个库与应用，末尾加 `-- --list` 枚举当前用例。`frontend_contracts` 约束 FAST/LK
输出，`jacobian_fd_check` 使用共享夹具做测量差分，`vio_manager::analytic_contract` 检查
完整传播/更新后的尺度、倾角和零偏。具体输入、误差预算和断言以测试源码为准。
第二条包含已知失败的 `synthetic_pure_msckf_with_bias`、`synthetic_slam_zero_bias`；
ignore 不计通过，合成无偏场景的严重发散检查也不代表定位精度验收。

<a id="system-tests"></a>

### 无 mock 系统验收

先完成 release 构建，准备 RMUC 资产与图形会话，按运行章节确认宿主机和 IPC 干净。
以下用例自行启动生产进程、用本次 RRD 评分并清理退出，不依赖聊天会话或手动启动的进程：

```bash
FIREFLY_RUN_SENSOR_STARTUP=1 uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py -q
uv run pytest tests/system/test_imu_flight.py -q
uv run pytest tests/system/test_imu_flight.py -q -k imu_hover
uv run pytest tests/system/test_hover_disturbance.py::test_hover_disturbance -q
uv run pytest tests/system/test_roundtrip.py -q
```

- 启动检查屏蔽 PlantState，保留 render 所需 GroundTruth；未启用飞行时只证明初始化与零推力。
- IMU 三例分别检查飞行、持续悬停与 VIO 失联后的有限姿态支持。持续悬停保持原目标 20 秒，
  独立评分整段真值、姿态与外援健康；上锁后固定等待再评分固定静止窗口，不能按真值自适应
  推迟窗口。全过程碰撞、重置和退出检查始终有效。
- 十轮扰动每次恢复后连续悬停，逐轮真值评分通过才发下一轮，不重发 Hold 改目标。
  失败后的未执行轮次不能记通过。
- 22m 往返使用 `configs/acceptance_22m.toml`，默认空地图、在线回环与深度建图；初始化、
  真实到点、跟踪、定位、回环、降落、碰撞和退出必须分别通过。

门槛唯一来源为 `tests/system/mission.py::Options`、`tests/evaluation/hover_metrics.py::HoverLimits`、
`tests/evaluation/disturbance_metrics.py::DisturbanceLimits` 及 `configs/acceptance*.toml`；
运行前固定并随报告保存，不能按失败结果放宽。新增门控须用当前固定配置独立启动三次，
三次全通过才建立基线，不以重试通过覆盖失败。

姿态融合、初始化、坐标、视觉前端或仿真传感器改动后，必须运行 `imu_hover`；
悬停不替代往返/回环。显式指定系统用例缺运行条件时报错；普通 CI 全仓收集中的 skip
不计飞行通过。`tests/evaluation/test_hover_metrics.py` 仅检验评分规则，不启动仿真。

### 任务验收与可选地图

```bash
uv run --all-packages --extra test python scripts/accept_rmuc.py
uv run --all-packages --extra test python scripts/accept_rmuc.py --case nominal --config configs/acceptance_22m.toml --vio-only
```

`--case nominal|reference_loss|estimator_loss` 选择任务；纯 VIO 仅改变进程部署和回环证据项，
不改变飞行误差、碰撞或时间阈值。`--prior-maps --localization-config <含 origin 的配置>`
启用外部地图，航点和高度边界须使用外部坐标。`--allow-map-rig-translation` 仅允许库图
左目/深度相机相同机体内平移，其余契约必须相同；例外记入报告，不改历史清单。

正常阶段流转只用新鲜估计状态和 FC 模式；真值仅独立评分及碰撞/越界中止，不能驱动
到点或恢复。局部 VIO 可固定尺度航向/平移对齐，地图定位与参考跟踪禁止事后拟合。
会话地图评分以首个真值样本确定固定换基，保留原始数据且不回流在线链路。
缺失、断流、未完成、碰撞、重置、退出残留分别报告，不用平均误差或一项通过替代完整任务。

证据落 `logs/`：RRD 保存原始观测/事件，JSON/HTML 保存配置、源码/二进制/资产指纹、
阶段、实际指标与退出码。失败依赖阶段标 blocked，不把未执行阶段记成功；原始证据不删除。

### 可选资产检查

```bash
FIREFLY_RUN_RMUC_ASSETS=1 uv run --all-packages --extra test pytest tests/system/test_rmuc_assets.py -q
cargo test --release -p firefly-planner --test rmuc_asset -- --ignored
cargo test --release -p firefly-planner rmuc_22m_initial_trajectory_is_executable -- --ignored
```

库图独立视角验收先启动 `uv run firefly-viz --save logs/map_validation.rrd`，再执行：

```bash
RUST_LOG=info FIREFLY_QUERY_MAP=<独立查询.ffvmap> FIREFLY_QUERY_COUNT=20 \
  FIREFLY_MAP_VALIDATION_REPORT="$PWD/logs/map_validation.json" \
  cargo test --release -p lightglue held_out_map_localization_contract -- --ignored --nocapture
```

独立查询不能来自建库位置，查询位姿只作评分标签，先验加入固定偏差；每个指定查询
必须通过几何和误差断言，拒绝不能计通过。模型导出、资产质量、库图定位、飞行与在线回环
分别验收；自匹配、几何单测或库图观测次数不能替代真实重访的后端接受证据。
