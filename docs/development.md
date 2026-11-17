# 开发指南

开发、运行、记录和验证统一在本文；系统职责、坐标、算法及能力边界见
[architecture.md](architecture.md)。路径和命令均相对仓库根目录。
观测契约统一见 [observability.md](observability.md)；README 提供介绍和入口。

## 维护与收敛

- 文档描述当前职责、契约、操作入口和验收边界；同一规则只维护一处，其余位置链接到该处。
- 调整职责时，同步清理旧接口、配置、依赖与文档。删除前全仓追踪生产、测试、工具和部署
  消费者，确认保留路径仍可构建、运行和验证。
- 参数、默认值与单位归对应类型及配置，测试条件与门槛归测试源码；文档只解释来源和使用边界。
- 文档与注释用正向陈述描述当前用途、合法输入、状态转换、错误条件和使用步骤。
  过程记录、修复对比和计划分别归 Git 历史、运行证据与 issue。
- 生命周期由拥有资源的类型维护：用具名状态表达互斥阶段，转换函数检查前置条件，
  完成、取消、失败及清理分别返回明确结果。测试覆盖正常转换、重复事件和退出路径。
- 可复现运行固定源码、二进制、配置、输入与标定指纹，为每轮选择新的产物路径。
  运行日志与原始验证证据统一归 [Mimir 的 `log.rrd`](mimir.md)，命令结果通过终端与退出码返回。
- 编译、类型检查、单测、真实 IPC、数据集与飞行按各自原定门槛验证，分别报告通过、
  未运行、跳过和失败。

## 实现约定

- 新 crate：`cargo new crates/<name> --lib --edition 2024`（自动纳入 workspace）。
- DDD 拆 crate（firefly-*），应用层依赖领域层，领域层保持独立。
- 错误用 firefly-error：kind/status 分类，操作名统一 `.with_operation(...)`，其他诊断字段
  用 `.with_context(...)`。`Permanent` 结束本次重试，`Temporary` 允许按策略重试。
- 运行诊断：关键函数 `#[fastrace::trace]`，日志用 log 宏；CLI 的帮助、结果和导出内容可写 stdout。
- 场景光照由 `firefly-render::lighting` 统一维护，所有相机共用。视角差异通过相机的
  曝光与色调映射表达；Bevy 的聚簇光源跨多视图共享，因此光源属于场景级资源。
- 依赖统一在根 `[workspace.dependencies]`。
- Rust API 遵循 [Rust API Guidelines 的命名规则](https://rust-lang.github.io/api-guidelines/naming.html#c-getter)：
  属性读取用 `state()`，可变借用用 `state_mut()`，属性设置用 `set_state(...)`。
  构造、采样、读取外部资源与填充缓冲区使用 `create`、`draw_*`、`read_*`、`write_*`
  等明确操作名。上游移植保留数学符号与数值契约，公开 Rust 接口及全部调用方采用本仓库命名。
  命名前核对实现与消费者：区分借用、分配、组装和状态推进，名称表达实际返回内容与副作用。
  查询可能缺失的数据使用 `Option` 或 `Result`；调用方明确选择缺失分支。
- 集合读取使用共享借用与切片；修改入口显式命名为 `*_mut`，按实际操作提供元素修改或
  容器修改权限。仅遍历的数据优先返回借用迭代器；排序、随机访问、跨锁或跨线程快照
  按消费者需要保留集合所有权。固定长度的临时输入优先使用数组。
- IPC 发布/订阅使用 `firefly-pubsub` 的 `Publisher<T>` / `Subscriber<T>`，话题名与容量
  由消息模块导出；独立接收深度和服务容量用 `SubscriberOptions`。需要事件唤醒的发布端
  显式使用 `with_topic_notify`，trace 用户头由泛型端口统一处理。

### 地面站 CLI

`ffctl` 用 `usage-rs`，固定 `<域> <动词>` 两层，命令声明在 `apps/ffctl/src/cli/`。新增子命令：
在 `cli/<域>/` 加一个文件，`#[derive(Args)]` 与 `impl Run` 写在一起，再在该域的动词枚举里
挂上；新增域则在 `cli/<域>/mod.rs` 建容器、在顶层 `Command` 挂上并给 `display_order`。
取值与约束写成 `usage-rs` 的声明式属性（`value_enum`、`validate` 等），由统一 `main.rs` 执行。
框架用法见官方 [Rust 框架文档](https://usage.jdx.dev/rust/) 的 quickstart、subcommands、
args-and-flags、validation。

### 配置

- **TOML、一应用一份**：统一放仓库顶层 `configs/`（`sim.toml` / `firefly-rootslam.toml` / `planner.toml`），
  启动时加载（Rust 支持 `--config` 换文件）。配置文件可选：缺文件记录一行 INFO 并使用
  代码默认值；存在的文件经过读取、解析和参数校验，失败时返回具体错误。
- **字段校验**：配置解析核对所有嵌套字段；拼写错误、未知字段和非法组合返回错误。
- **最小化**：保留必填键，只对有默认值的键写差异覆盖；无覆盖项时保留仅含用途和
  默认值来源注释的配置文件。缺失键使用代码默认值；
  纯数据 Options 直接 `serde::Deserialize` + `#[serde(default)]`，Python 用标准库
  `tomllib`。数据集适配器读取官方原始传感器格式，如 Kalibr YAML，
  转换到统一标定类型后使用。

#### 应用启动入口

固定顺序：**解析命令行 → 初始化基础日志 → 读取、转换并校验配置 → 报告配置来源
→ 创建运行资源 → 运行应用**。命令行帮助与参数错误在资源创建前完成处理。

- Rust 主配置通过 `firefly_config::startup(path, init_logging, validate)` 加载。
  普通进程传入 `firefly_observability::init`；校验函数返回 `firefly_error::Result<()>`。
  无额外参数约束时传入 `|_| Ok(())`。配置类型只描述字段、默认值和参数约束，
  文件读取、日志安装和退出处理分别由公共入口及 `main` 负责。
- Python 主配置通过 `firefly_workspace.startup(path, decode)` 加载。
  `decode` 负责字典转换与配置校验；默认安装标准库 INFO 日志。
  宿主需要特殊日志格式时通过 `init_logging` 参数提供初始化函数。
- 公共入口成功返回后才创建 IPC、模型、工作线程、窗口等资源。辅助配置在此之后使用
  `firefly_config::load` / `firefly_workspace.load_toml`，沿用已安装的进程日志。
  领域对象构造时仍保留自己的不变量校验。
- 配置来源由公共加载模块统一记录。缺文件与空文件
  使用相同默认值；缺文件仅在转换、校验成功后提示回落。INFO 的显示尊重日志过滤设置，
  Rust 可用 `RUST_LOG=info` 查看。
- Bevy 先独立安装日志设施，在配置通过后安装其余插件。使用 `LogPlugin` 时，
  在随后的 `DefaultPlugins` 中禁用它，避免重复安装；自定义日志宿主沿用自己的桥接。

新应用以可编译的 [Rust 最小应用](../crates/firefly-config/examples/minimal.rs) 和
[Python 最小应用](../packages/firefly-workspace/examples/minimal.py) 为入口示例；
Bevy 日志接线参考 [quad](../apps/quad/src/main.rs)。Rust 示例随
`cargo check --workspace --all-targets` 编译。

标定与噪声改对应 Options 默认值，doc 注释写单位和来源；部署覆盖同步 `configs/`。
升级视觉依赖按既有容差运行相关前端、投影及匹配回归；依赖统一在根 Cargo/uv workspace 和锁文件。
改行为前阅读对应官方源码，固定版本与差异见架构文档。参考仓库位于 `~/Projects/`：
`open_vins`、`ORB_SLAM3`、`VINS-Fusion`、`EGO-Planner-v2`、`iceoryx2`、`logforth`、`fastrace`；
purecv 的来源与固定版本以 workspace 依赖及 Cargo.lock 为准，发现上游问题应提 issue。

## 提交与 PR

commit message 使用 `<type>(<scope>): <摘要>`；PR 标题使用 `<type>: <中文摘要>`，
摘要直接描述整个 PR 的最终结果。补充信息用逗号分隔，
例如 `fix: 修正悬停参考的速度前馈`。PR 说明交代行为变化、依据和实际验证结果。

<a id="run"></a>

## 构建与运行

场景由 `configs/scene.toml` 的 `scene = "<赛季>@<版本>"` 唯一选择，已注册
`rmuc2027@1.0.0`；资产在 `models/<赛季>/v<版本>/`（`field.glb`、
`<赛季>_collision.json`），另需 `models/drone/drone.glb`。启动时检查版本注册和资产完整性，错误指向具体场景。
`configs/game.toml` 的 `team = "red"` / `"blue"` 选择己方机库；默认 red。
停机坪定义由 `firefly_mujoco.scene.SCENES[].hangars` 与 `PAD_CLEARANCE` 提供，出生点在
所选 H 标记中心正上方。物理代理按 CAD 甲板的八边形轮廓切分凸棱柱，保留轮廓外的碰撞体积，支撑面高度与图案一致。
render 使用无窗口 GPU 渲染；Rerun 原生 Viewer 需要图形会话。Rust/Python 版本和依赖
以 workspace 配置为准。mimir 直接将左右目和下视无损灰度图和结构化诊断写入 RRD，配置见 `configs/mimir.toml`。

```bash
uv sync --all-packages --extra test
cargo xtask release
```

运行进程一律 release；普通 `cargo test` 用 debug。源码改动后先完成构建，再做运行验收。
RootSLAM 的 `firefly-calibration`、`firefly-basalt`、`firefly-rootslam-mapping` 和
`firefly-rootslam` 在 release 下使用单个代码生成单元，让 LLVM 在整个 crate 内内联
逐帧数值循环；这四个包的 release 构建时间相应增加，其他应用保持默认并行代码生成。
分别在终端启动，记录进程最先启动、最后退出：

```bash
./target/release/mimir
uv run firefly-sim
cargo run --release -p render
cargo run --release -p firefly-rootslam
cargo run --release -p fc
cargo run --release -p traj
cargo run --release -p planner
```

最小闭环只需 mimir/sim/render/firefly-rootslam/fc；规划同时启动 planner 与 traj
两个独立应用，前者规划并发布心跳，后者采样并发布飞控参考。
planner 使用 `configs/planner.toml`，traj 使用 `configs/traj.toml`；轨迹心跳失联门限由 traj 配置。
RootSLAM 默认关闭建图与回环，作为局部 VIO 使用。
可选库图定位先填写 `configs/localization.toml [origin]`，并用
`firefly-rootslam --external-localization`，再启动 localization、aliked 和
`lightglue --map <地图.ffvmap>`。这些可选服务由操作员显式启动。
planner 使用机体 Odometry、深度与配置中的相机外参；定位与规划从传感器观测建立状态。
场地真值用于传感器生成、独立评分与可视化。

### 机载部署

板载平台为 Orange Pi Zero 3W，系统与硬件见 [硬件规格](hw/spec.md)。
`--target board` 在 Apple container 的 Debian trixie 镜像中原生构建；镜像 digest、
工具链及 bindgen 依赖由 [`xtask/src/container.rs`](../xtask/src/container.rs) 固定，
产物在 `target/container/aarch64/release`。首次运行前启动 `container system start`；
xtask 自动构建派生镜像并复用缓存。

产物固定放 `/opt/firefly/{bin,configs,models}`：

```bash
cargo xtask release --target board
cargo xtask deploy
```

板载应用名单由 [`xtask/src/target.rs`](../xtask/src/target.rs) 维护。
`deploy` 把二进制、`configs/*.toml` 与默认视觉模型打成
`dist/firefly-aarch64.zst`（xtask 定义的 zstd 容器），推到设备后由同一 xtask 的 `unpack`
解包，`--device` 默认 `firefly-0001`。解包前会停掉 `firefly.service` 并清空
`/opt/firefly/{bin,configs,models}`，用本次发布替换应用产物；`logs` 与镜像
预置的 `systemd` 保留。解包后把每个二进制软链到 `/usr/local/bin`，因此 `ffctl`、
`fc` 等在任意目录、非交互 `adb shell` 下都可直接调用。
最后重新 `systemctl start firefly`，以 `ffctl process status` 在超时内应答为就绪判据。
systemd active 只表示服务已启动；socket 应答才证明 supervisor 可管理进程。
启动失败或解包缺文件时 `deploy` 非零退出。板载 adbd 使用 legacy shell 协议，
远端退出结果通过输出判定，本地 `adb shell` 退出码描述传输状态。

板端构建按机载应用名单编译，依赖的 feature 只在这些包之间统一。
上板前执行 `cargo xtask release --target board`，独立验证 Linux 代码和实际部署的
feature 组合。依赖被门控的 API 时，在 workspace 依赖中显式声明 feature；例如
`camera --hil` 使用 `nix::fcntl` 设置 O_NONBLOCK，需要 `nix` 的 `fs` feature。

二进制按安装布局定位配置与模型，与启动目录无关；解析顺序（越显式越优先）：

1. `FIREFLY_ROOT` 环境变量；
2. 可执行文件的上两级（部署布局下 `bin/<app>` → 安装根；仅当该前缀下存在 `configs/`）；
3. 编译期默认：板载 `/opt/firefly`，开发机构建为仓库根。

解析由 [`firefly-config::paths`](../crates/firefly-config/src/lib.rs) 实现。
`--config <path>` 可覆盖单个配置文件。非标准安装前缀使用 `FIREFLY_ROOT`。
supervisor 的相对命令 `bin/<app>` 和 `logs/process` 仍依赖工作目录；运行
`ffctl process supervise` 以安装根为 cwd。
可复现输入为镜像 digest、Rust 工具链、`--locked` 和 `Cargo.lock`。
升级基础镜像时同步更新容器配置，派生镜像按工具链与 digest 生成标签。

板侧传感器与执行器由三个独占进程接替仿真侧的 `render` 与 `firefly-sim`：

| 进程 | 设备 | 话题 |
|---|---|---|
| `camera` | `V4L2` 节点 | 发布 `Firefly/CameraLeft`、`CameraRight`、可选 `Depth` 和 `CameraDown` |
| `imu` | `spidev`（`BMI088` 两片） | 发布 `Firefly/Imu` |
| `motor` | `sysfs` PWM | 订阅 `Firefly/Control`，发布 `Firefly/Airframe` |

三者由 xtask 的 board release 清单构建。上板前按配置核对：

- `configs/camera.toml`：`/dev/videoN` 编号随内核与接线变化，先用
  `v4l2-ctl --list-devices` 核对。分辨率由 IPC 定长消息定死（320×240），
  双目和深度使用该组合；`[down]` 独立固定为 352×288，驱动必须原样接受格式。
  `[depth]` 控制深度发布，下视使用独立工作线程、V4L2 单调采集时间戳与行跨度。
  下视 `rate_hz` 只接受 15/30，上界与任务租约取较小值；先用 S_PARM 请求驱动帧间隔，
  发布前再按采集时间限频。未持有租约时释放 mmap 流并 STREAMOFF。
  具体相机的格式、同步、时间戳和标定按配置指定的 `V4L2` 流做实机验证。
- `configs/imu.toml`：实现 `BMI088`，启动通过 `WHO_AM_I` 核验设备身份。
  寄存器初始化值取自数据手册，首次上板须核对
  `Firefly/Imu` 的读数是否落在量程内的合理重力/静止零偏。
- `configs/motor.toml`：`sysfs` PWM 只适用于 PWM 电调。`[airframe]`
  （质量/惯量/旋翼几何/推力上限）在这里维护一份，真机装配后填写实测值。
  `Control` 断流时四路回到零推力脉宽。`pwm-sunxi` 使能通道需要对应的设备树 DT 节点；
  上板时先核对 DT，再验证周期、占空比和 `enable`。

三个驱动默认 `autostart = false`。先核对设备节点与设备树，再逐个
`ffctl process enable camera`、`imu`、`motor` 并独立验证。`processes.toml`
随部署覆盖，持久配置更新写回部署源文件。

`camera` 与 `imu` 的时间戳都用 `CLOCK_MONOTONIC` 秒，与仿真侧 `sim_time`
语义各自独立，每份录制使用对应时钟。三个进程都可以单独起停：`bin/camera`、
`bin/imu`、`bin/motor` 接 `--config <path>`，`Ctrl-C` 正常退出。

`firefly.service` 由镜像构建 [`orangepi-build`](https://github.com/3SE-Competitive-Robotics-Team/orangepi-build)
预置并默认启动；服务执行 `ffctl process supervise`。supervisor 按 `configs/processes.toml`
的数组顺序拉起 `autostart = true` 的进程。`processes.toml` 被 `deploy` 覆盖；
`lightglue` 在生产 `.ffvmap` 部署前保持 `autostart = false`。运行期管理：

```bash
cd /opt/firefly
./bin/ffctl process status                      # 一次性，含 CPU%/MEM
./bin/ffctl process status --watch --interval 1 # top 式刷新，Ctrl-C 退出
./bin/ffctl process status --json      # 单次 JSON 数组，供脚本与 agent 读取
./bin/ffctl process start [name...]     # 省略 = 所有 autostart；显式名字忽略 autostart
./bin/ffctl process stop [name...]      # 省略 = 全部，逆序
./bin/ffctl process restart [name...]
./bin/ffctl process enable <name...>    # 改 processes.toml，下次 supervisor 启动生效
./bin/ffctl process disable <name...>
systemctl stop firefly                  # supervisor 逆序 SIGINT 所有进程
```

supervisor 对异常退出退避重启；停止使用 SIGINT，超时返回错误并保留进程状态。具体时限由
[`supervisor.rs`](../apps/ffctl/src/process/supervisor.rs) 维护。
`status` 的 `STATE` 列显示 `stopped / running / backoff / stopping`；`RUNNING` 和 PID
仍表示 supervisor 持有的子进程。`--json` 输出相同进程的 `name / running / pid / restarts / state`，
成功退出码为 0，查询失败为 1，`--json --watch` 参数冲突为 2。JSON 使用 supervisor 的进程
快照，CPU/MEM 与本地清单的 AUTOSTART 在表格模式显示。连接旧 supervisor 时 `state`
可能缺失；完整转换与兼容契约见[进程状态契约](architecture.md#进程状态契约)。
子进程 stdout/stderr 在 `logs/process/<name>.log`，运行诊断的归档证据仍以 RRD 为准。
`camera`、`imu`、`motor` 已实现标准 IPC 并纳入板载构建；设备同步、标定、实时性和飞行
均须实机验收。部署成功只证明发布和 supervisor 应答。

静止初始化就绪后，逐阶段确认模式与任务状态，再发送下一条指令：

```bash
./target/release/ffctl fc arm
./target/release/ffctl fc takeoff 1.0
./target/release/ffctl fc hold
./target/release/ffctl fc track
./target/release/ffctl planner goal 22 0 1
./target/release/ffctl planner goal 0 0 1
./target/release/ffctl fc land
```

起飞高度相对起飞点，planner 目标属于当前 odom 坐标。拒绝原因查看 RRD 的 `logs/fc`。
`ffctl planner goal X Y Z --yaw 90` 可指定终点航向（度，odom +X 为零、逆时针为正）；
省略时随规划路径定向。roll/pitch 由飞控实现推力方向。
上电后先完成静止初始化与解锁检查，再接受起飞指令。飞控失联行为见架构文档。

### 下视精确降落

模型目录使用版本化名称，由 manifest 固定模型、源权重、校准集摘要及输入输出形状。
训练权重来自 `se3_yolo/runs/h_marker/yolo26n_sim/weights/best.pt`；用训练仓库的
导出环境运行（依赖由该仓库锁定）：

```bash
../se3_yolo/.venv/bin/python scripts/export_landing.py \
  --checkpoint ../se3_yolo/runs/h_marker/yolo26n_sim/weights/best.pt \
  --dataset ../se3_yolo/datasets/h_marker --output models/h-marker-yolo26n-cif-v1
```

相机输出 352×288；默认网络 NCHW 为 `[1,3,288,352]`。静态 INT8 QDQ 使用 train
分组校准，val/test 保持独立。灰度复制为三通道，原分辨率角点用于米制定位。
现有合成校准先重采样为 CIF；装机后使用真实镜头、曝光和运动图重新校准与测评。
`configs/landing.toml` 配置内参、畸变、H 尺寸、安装位置与触地原点偏移；默认几何匹配仿真 CAD，
实机参数须由标定/测量替换。下视光心位于机体后方 3cm。

生产 `landing` 通过 ort-rs 的 `VSINPU` EP 运行，加载
`models/npu/libonnxruntime.so`，禁止 CPU 回退。原生库必须提供 C API 31 和
`OrtSessionOptionsAppendExecutionProvider_VSINPU`；标准 CPU 发行包不提供该 EP。
官方 [ORT VSINPU 实现](https://github.com/microsoft/onnxruntime/tree/main/onnxruntime/core/providers/vsinpu)
依赖 TIM-VX；TIM-VX 与 A733 SDK/内核驱动的兼容性、INT8 算子放置和时延由实机启动验证。

在 aarch64 Linux 上，准备与板端驱动匹配的 SDK、已安装的 TIM-VX 和固定到完整 commit
的干净 ORT 源码后，使用 `scripts/build_npu_runtime.py --help` 所列参数构建运行库。
脚本验证源码 C API、构建 VSINPU、打包共享库、设置同目录 RPATH 并记录指纹；
构建完成不代表模型已经通过 NPU 验收。运行库目录、INT8 ONNX 和模型 manifest 随
`xtask deploy` 部署；发布缺少运行库时明确失败。可选 ALIKED/LightGlue 共用该 Linux 原生库。
普通与 HIL 清单均自启动轻量的 landing 宿主，固定 CPU 4；FC 固定 CPU 5，VIO 在 CPU 6、7。
A733 官方 SDK 的运行接口与版本见
[厂商 NPU 开发文档](https://docs.radxa.com/en/cubie/a7a/app-dev/npu-dev/cubie-acuity-sdk)，
VSINPU/TIM-VX 与该板 SDK 的兼容性仍须通过实际运行库和设备验证。

```bash
cargo xtask release
./target/release/landing --simulation
./target/release/ffctl fc land --precision
uv run python scripts/validate_landing.py --output target/landing-validation/cif-v1
uv run python scripts/flight.py --mission hover --hover-seconds 3 --precision-land
```

`--simulation` 显式选择 CPU，用于桌面仿真；生产默认后端始终为 NPU。
在线进程按 `Firefly/Activity` 租约在 Dormant/Active 之间切换，Dormant 释放模型会话、图像订阅和里程计线程。
任务每 200ms 续租，2s 未续租自动休眠；多个拥有者只释放自己的请求，LANDING 同时请求下视相机。
默认下视 15 FPS，`flight.py --down-rate 30` 可选择 30 FPS，推理另受 `landing.rate_hz` 上界控制，默认保持 10 Hz。
任务通过统一资源 API 续租和释放请求；FC 独立判定精降命令资格，推理失败通过正常进程错误路径报告。
`landing/runtime` 在 RRD 中记录实际活跃状态、累计推理数、请求帧率和 NPU 标志。
离线 `landing --image <352x288.png> --simulation` 输出检测、优化角点、米制平移和重投影残差。
图像验收在两队机库上方 1m/1.5m、中心及偏置位置检验识别和米制误差，并导出 PNG；
运行与飞行原始证据由最先启动、最后退出的 Mimir 保存。初次识别建议在 1m 高度完成，
近距离由连续 H 角点跟踪维持观测；超时后重新经过 YOLO 确认。

### 下视训练数据整理

`scripts/collect_landing_dataset.py` 保留过程 PNG 和 RRD 的原始下视像素，按像素去重，
从同时间的 GroundTruth 获取标签相机位姿，再用固定 CAD 的语义发光掩码独立生成
完整 H 平台的 YOLO 框；检测模型的预测不作为标签。原始截图路径、录制指纹、
采集时刻、位姿及插值标记写入 manifest。语义掩码保留其他几何的遮挡。

```bash
uv run python scripts/collect_landing_dataset.py --images target/landing-validation \
  --recordings logs/<录制目录>/log.rrd logs/<另一录制目录>/log.rrd logs/<第三录制目录>/log.rrd \
  --output datasets/h_marker_cif_v1 --sample-period 1 --max-samples 24
```

每个飞行录制归属同一划分，重复出现在不同划分的相同图像进入 quarantine；
相同静态相机验收属于固定姿态组。微小、无 H 涂层或位姿缺失的样本保持待复核状态，
按 `images/quarantine` 隔离，避免生成错误负样本。输出包含 `data.yaml`、
images/labels/masks 的 train/val/test 目录及带框 review 图片；标注契约与
`se3_yolo` 的完整平台可见范围一致。使用新的输出目录保留每版数据，
检查 review 后再交给训练仓库；实际 OV2640 图像另行采集与标注。

### HIL 部署与运行

HIL 用 USB 把电脑上的 MuJoCo 物理仿真与板端生产计算连接起来。电脑运行
`mimir`、`render`、`hil` 和仿真；板端运行 `camera --hil`、
`motor --hil`、RootSLAM、planner、traj 与 FC。电机指令经 USB 返回仿真，HIL 驱动访问
FunctionFS 端点，不访问实体相机、IMU 或 PWM。

先构建主机验收程序，再部署板端，最后立即运行验收：

```bash
cargo build --locked --release -p render -p mimir -p hil -p ffctl
cargo xtask deploy-hil
uv run python scripts/hil_hover.py
```

`deploy-hil` 沿用普通 `deploy` 的构建、替换、服务就绪检查，然后以
`configs/processes-hil.toml` 启动完整进程集并切换 USB gadget；`hil --check` 成功认领接口
和两个数据端点后才报告链路就绪。板端与主机使用同次构建的协议版本。

`configs/rootslam-hil.toml` 使用顶层 `precision = "float32"` 和
`[vio] optical_flow_detection_grid_size = 20`，关闭建图与回环。
SIL 的 `configs/firefly-rootslam.toml` 使用 FP64 后端；两条路径的光流补丁均为 FP32。
FC 使用 `configs/fc-hil.toml`：水平速度环增益与正常部署一致，
`control.allocation.airmode = 0` 使用集合推力优先分配，其余字段沿用代码默认值。
普通部署的 airmode 默认值由 `firefly-flight::AllocationParams` 定义。

`configs/hil.toml` 管理 USB 身份、队列与送图节拍。`imu_batch = 1`；
双目按采集时间配对，图像分为最多 16 KiB 的块，块间先发送 IMU 与指令。
`image_queue` 是待发图像预算，至少为 2；双目对原子入队，每个队列位置最多占两张图，
深度图和 CIF 下视图各独占一个位置；下视按独立资源租约启停和节流，
`down_rate_hz` 为下视 15/30 FPS 上界，双目/深度继续沿用 `image_period_s`。上行每条遥测和日志发送前优先排空控制队列。
`image_period_s` 是板端视觉负载预算，图像、控制和后验的实际频率分别验收。
HIL 默认送图预算为 30 Hz。`configs/processes-hil.toml` 为 A733 设置以下亲和性：

| CPU | HIL 应用 | 调度约束 |
|---|---|---|
| 6、7（A76） | RootSLAM | 两个 Rayon worker，所有线程在这两个大核内调度 |
| 5（A55） | FC | 主循环和 IMU worker 固定在同一个小核 |
| 4（A55） | landing | INT8 NPU 会话、预处理和角点；巡航释放工作资源 |
| 0–4（A55） | camera、motor、planner 与 traj | 共享小核池，按运行负载调度 |

亲和性由 `taskset` 在应用启动前设置，新线程继承该集合；Rayon 并行度由
同一命令中的 `RAYON_NUM_THREADS` 设置。普通板端清单沿用相同核分配；桌面 SIL 直接运行二进制，使用宿主机调度。
HIL 亲和性约束应用线程，内核中断、系统服务和 supervisor 仍按系统策略调度。
更换 HIL 板型时按实际 CPU 拓扑调整清单，并重新验证控制周期和完整悬停验收。
用 `taskset -apc <pid>` 核对所有线程；修改清单后重启 supervisor 才会重新读取。

硬件约束：

- 单个厂商接口使用一个 OUT 与一个 IN；板载 UDC 的可用端点由实际设备决定。
  HIL 期间 ADB 不可用，配置和二进制在切换前部署。
- `hil-gadget up` 重建 gadget，写完 FunctionFS 描述符和 strings 后再绑定；
  只有 OTG 角色不匹配时才切换角色，绑定后执行 soft-connect。
- 60 秒未建立绑定或收到主机心跳时，看门狗先停 HIL 进程集，恢复正常清单，
  再重建 ADB gadget 和正常服务。主机退出后等待恢复，并用
  `adb -s firefly-0001 shell ffctl process status --json` 核对收尾。
- gadget 重建期间 IPC 尚不可用，设备启动诊断由既有
  `/opt/firefly/logs/hil/up.log` 保留；运行遥测与板端日志经 USB 进入主机 RRD。
- USB 模式转换通过 `/opt/firefly/logs/hil/transition.lock` 跨进程互斥，覆盖服务启动、
  完成标记与落盘；ADB 刚枚举时发起的新会话会等待上一轮恢复完成。

scripts/hil_hover.py 发送起飞、20 秒 Hold 和降落命令，记录 FC 原生模式。
scripts/hil_hover_monitor.py 可独立查看状态，不发送控制命令。
命令流程完成、录制完整性、飞行精度和实时性分别报告。

往返任务使用板端 RootSLAM、planner、traj 与 FC，视觉降落同时使用 landing；主机下发双目、IMU、深度与下视：

```bash
cargo build --locked --release -p render -p mimir -p hil -p ffctl
cargo xtask deploy-hil
uv run python scripts/hil_roundtrip.py --precision-land
```

悬停和往返使用同一份 `processes-hil.toml` 与 `hil.toml`，全部进程自启动并持续下发深度。
主机/板端驱动共享 `firefly-hil::topics` 路由表和 `open / drain / publish` API；
`SensorIngress::accept_raw` 统一解码、图像重组、校验和与 IMU 唤醒。
新增载荷维护协议编码和该路由表，应用层保持通用转发。协议 v7 的资源请求走高优先队列，
下视只在有效租约期传输；控制优先级、诊断连续性与队列预算在共享实现中维护。
`cargo test -p firefly-hil --test bridge --release` 用真实 IPC 和字节流验证全部上行路由、
下行请求、CIF 分片以及穿插 IMU；USB 实机验收仍由同一生产驱动执行。
部署仅选择设备；任务脚本决定发送哪些飞行指令。HIL 与 SIL 共用 `flight.py` 的
`--goal DX DY`、`--altitude`、`--hover-seconds`、`--return-offset DX DY` 和 `--seed`。
目标相对解锁前保存的 odom 起飞点生成，以当前目标身份、原生轨迹完成、实际里程计到位和低速保持共同切换航段。
飞控自身保护保持启用；两端使用同次构建并部署。
`hil --check` 核对 USB 接口和端点；实际传输及协议一致性由运行验证。
帧版本不匹配会结束会话，错误包含收到版本和当前版本；重新部署两端后再运行。

### 进程与宿主机

- 启动前核对 PID、可执行文件和工作目录，确认闭环、采集、录制与验收资源可用。
  进程操作使用本任务已确认的 PID。
- 编译结束后采样实际 CPU/GPU、温度和降频计数增量，资源稳定后运行。`ps %CPU` 的生命周期
  均值用于概览，性能归因使用实际采样；负载、残留与原定验收阈值一并记录到 RRD。
- 成功、失败、超时和中断均进入 SIGINT、等待退出、回收线程与管道、RRD flush 的清理流程。
  核对所有已启动 PID 及子进程后才开始下一轮；残留和超时保留为清理失败。
- iceoryx2 终止信号读取会消费事件；接收线程退出后设置共享停止标志，主循环同步检查，
  传播线程错误并回收资源。
- 退出流程只使用 SIGINT 和等待回收，退出错误保留进程及共享内存现场供排障。
  macOS 排障清理在所有相关进程退出后，才删除 `/tmp/iceoryx2/services` 与
  `/tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state`。

<a id="assets"></a>

## 场地与可选离线资产

脚本、CAD 与验收共用 `firefly-workspace` 的工作区定位和流式 SHA-256；该包只依赖标准库。

```bash
uv run --package firefly-cad firefly-cad-build <场地.stp> --scene <赛季>@<版本>
```

`firefly-cad-build` 构建并验收几何资产，配置为 `configs/cad.toml`。GLB、碰撞盒、FFMap 必须来自
同一归一化几何，通过内容 hash、坐标和占据检查；细节见
[CAD README](../packages/firefly-cad/README.md)。在线运行、离线采集、建库与验收资产指纹
共同使用 `configs/scene.toml` 所选场景；采集与库图默认输出位于该版本的资产目录及地图路径。

`scripts/collect_vision_map.py` 使用 `render --offline` 和隔离的 `Firefly/Offline/*`，
按同一时间戳配对左图、理想深度与位姿标签，使用独立离线通道。
同步帧、图像纹理、路标覆盖与内容指纹由采集和构建器检查，通过后建库。
库图兼容性只依赖场地、标定、光照和曝光；源码/二进制 hash 用于追溯，普通重编译或在线
噪声变化可沿用兼容库图。每次采集保留原 manifest，替换资产前归档。

官方预训练模型的可复现导出使用独立 CPU 环境：

```bash
uv venv models/vision-export/.venv --python 3.12
uv pip install --python models/vision-export/.venv/bin/python -e packages/firefly-workspace
uv pip install --python models/vision-export/.venv/bin/python torch==2.15.0 torchvision==0.29.2 --index-url https://download.pytorch.org/whl/cpu
uv pip install --python models/vision-export/.venv/bin/python onnxscript==0.7.2 onnx==1.23.2 onnxruntime==1.31.1 kornia==0.9.0 opencv-python==5.0.0.93 matplotlib==3.11.2
models/vision-export/.venv/bin/python scripts/export_vision_models.py
uv run --all-packages python scripts/build_vision_map.py <通过内容校验的采集目录>
```

使用官方 ALIKED-N16（128 维）与匹配的 128 维 LightGlue 权重，320×240、512 点。
导出保留全部匹配层，以固定执行路径完成推理。
导出器校验可变形卷积与 torchvision 的一致性、ONNX 与 PyTorch 输出一致性，
`models/vision_models.json` 保存源码版本、权重及模型 hash、数值检查结果。
视觉地图旁的 `.manifest.json` 关联模型、构建器、采集清单和场地指纹。
模型与地图保存在本地产物目录；上游源码及权重的许可见固定版本的 LightGlue 仓库。

<a id="diagnostics"></a>

## 记录与性能

观测登记、topic 清单、开关、录制及完整性验收统一见 [可观测性](observability.md)。
新增或修改诊断必须先遵循该文档的登记与添加流程。

<a id="tests"></a>

## 验证

测试依赖通过构造函数或参数注入，替身只覆盖外部交互边界，执行与评分使用生产实现。
数据夹具使用真实 dataclass/消息；行为替身实现生产接口，或使用
`create_autospec(..., spec_set=True)`。生产执行通过公开入口及显式参数配置；
进程环境可用 monkeypatch 隔离恢复。

`tests/system` 的 session fixture 先执行 `cargo xtask release`，再启动真实进程。
构建并行度使用 Cargo 默认值或 `CARGO_BUILD_JOBS`，Python 依赖通过 `uv sync` 安装。

firefly_workspace.process 提供子进程回收、输出排空和运行互斥；
firefly_mujoco.messages 统一 Python IPC 定义，firefly_mujoco.vision_assets 维护建图库图的资产契约。
对应单元测试随各自包运行。

```bash
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
cargo test -p firefly-planner --test planner_contracts
uv run --all-packages --extra test pytest -q -p no:cacheprovider
FIREFLY_TEST_PYTHON="$PWD/.venv/bin/python" cargo test -p firefly-pubsub --test python_interop -- --ignored
FIREFLY_RUN_TELEMETRY_LOAD=1 uv run --all-packages --extra test pytest tests/system/test_mimir.py -q
```

mimir 的真实 IPC 与 RRD 读回要求见 [可观测性验证](observability.md#validation)。
数学回归以独立公式/解析值、中心差分和最终输出契约作为依据：

- 规划：端点 PVA、C2 连续、避障/集群场景、轨迹时钟与软惩罚梯度；输出能力边界见架构文档。
- 坐标/姿态/回环：解析变换、残差与重置差分、相关信息计权、会话与时间拒绝。
- 前端/资产/IPC：已知位移与双目约束、几何一致性、深度投票与先验保护、双语言布局及事件往返。

指定的确定性任务以完成为成功条件；随机安全属性分别报告成功与有界拒绝。
正确性使用机器无关门槛，性能单独计时。解析、采样与性质回归各自覆盖明确契约。

### Basalt 与 RootSLAM

在线与数据集共用 `firefly-rootslam --config <path>`；默认读取 IPC，数据集示例为
`configs/firefly-rootslam-dataset.toml`。相对路径以进程工作目录为基准，示例从 workspace 根运行。
运行前准备配置指定的数据和标定。公开数据集准备、单双精度运行、评分与结果说明统一见
[RootSLAM 测评指南](../apps/rootslam/README.md)，
职责及飞控接线边界见 [系统架构](architecture.md#basalt-与-rootslam-独立应用)。

```bash
cargo run --release -p firefly-rootslam -- --config configs/firefly-rootslam-dataset.toml
cargo test -p firefly-calibration -p firefly-basalt -p firefly-rootslam-mapping -p firefly-rootslam --all-targets
cargo clippy -p firefly-calibration -p firefly-basalt -p firefly-rootslam-mapping -p firefly-rootslam --all-targets -- -D warnings
uv run python -B -m unittest discover -s apps/rootslam/tests -p test_export_rrd.py
```

默认测试使用独立 C++ 参考夹具；外部数据集、完整上游程序与设备用例须满足各自运行条件。
局部单测、数据集轨迹等价、论文性能复现与整机验收分别报告。

### 规划与深度建图专项

`cargo test -p firefly-planner --lib worker` 检查取消、过期、地图/窗口变化与阻塞求解时的安全决策。
实时性基准源码为 [`replanning.rs`](../apps/planner/benches/replanning.rs)：
先 `cargo bench -p planner --bench replanning --no-run`，启动 RRD 并等待宿主稳定，
再运行生成的可执行文件并传 `--bench`；加 `--updates` 测地图写入及快照同步开销。
按源码场景交替同步/异步顺序，证据写入 `logs/replanning-bench`，无入库结果即失败。
该基准覆盖进程内规划与调度，闭环 IPC、渲染、飞控和飞行另行验收。

本地深度输入、地图新鲜度与坐标修订回归运行 `cargo test -p planner --bin planner`。
真实 IPC 回归须独占话题，先完成 release 构建，再执行：

```bash
cargo test -p planner --bin planner -- --ignored --test-threads=1
cargo test -p traj --test trajectory_ipc -- --ignored --test-threads=1
```

IPC 用例独占 Odometry/Depth/Trajectory/Reference，覆盖完整多项式、心跳超时发布顺序、
里程计失联锁存，以及局部地图和参考对 Navigation 跳变的独立性。

<a id="system-tests"></a>

### 真实进程系统验收

本地深度融合的数值对照使用固定上游 C++ 函数，生成器校验 checkout 修订号。
`--eigen` 指向包含 `Eigen/` 的头文件目录；生成产物位于 `target/ego-map-reference`：

```bash
uv run python crates/firefly-ego-map/tests/reference/generate.py --upstream ~/Projects/EGO-Planner-v2 --eigen /path/to/eigen
FIREFLY_EGO_MAP_REFERENCE="$PWD/target/ego-map-reference/reference" cargo test -p firefly-ego-map --test cpp_reference -- --ignored
```

先完成 release 构建，准备 RMUC 资产与图形会话，按运行章节确认宿主机和 IPC 干净。
真实 IPC 启动检查独立运行：

~~~bash
FIREFLY_RUN_SENSOR_STARTUP=1 uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py -q
~~~

启动检查屏蔽 PlantState，保留 render 所需 GroundTruth；未启用飞行时只证明初始化与零推力。

### 飞行指令脚本

`scripts/flight.py` 启动 release 进程、编排起降/航点并回收进程。
完整任务使用 `--mission roundtrip --precision-land`：从所选机库起飞，飞往航点并悬停，
返回起飞点附近，等待新的下视识别，最后精确降落、上锁并记录静止保持。
多途经点沿用同一任务入口：每点完成规划、到位停稳和悬停，再提交下一点，最终返航精降。

~~~bash
uv run python scripts/flight.py --mission roundtrip --precision-land
uv run python scripts/flight.py --mission roundtrip --precision-land --goal 4 0 --hover-seconds 3
uv run python scripts/flight.py --shape square --side 2 --precision-land
uv run python scripts/flight.py --shape triangle --side 2 --precision-land
uv run python scripts/flight.py --waypoint 2 0 --waypoint 3 1 --waypoint 2 0 --precision-land
uv run python scripts/flight.py --mission hover --precision-land
uv run python scripts/hil_roundtrip.py --precision-land
FIREFLY_RUN_PRECISION_MISSION=1 uv run pytest tests/system/test_flight_mission.py -q
uv run python scripts/verify_flight.py logs/<本轮目录>/log.rrd
~~~

完整往返使用 `configs/sim-mission.toml`，使用 30Hz 双目/深度供给；下视独立按租约选入 15/30 FPS，
物理和 IMU 保持 1000Hz；HIL 的同周期送图预算见前文。悬停沿用 `configs/sim.toml`。
`--goal DX DY` 相对解锁前保存的 odom 起飞点；RMUC 示例默认朝场地中部，red 为 (4,0)m、blue 为 (-4,0)m；
`--shape square|triangle` 先向场地飞一个边长，再从该入口飞闭合的正方形或等边三角形，
回到入口后返航；`--side` 指定边长，blue 将预设航线绕起点旋转 180°。
重复 `--waypoint DX DY` 按输入顺序设置自定义航线，每点均相对同一起飞坐标，适合任意折线或多边形。
`--goal`、`--shape`、`--waypoint` 三者互斥；多点和预设参数同样可用于 `scripts/hil_roundtrip.py`。
途经点数量、离起点半径、相邻航段长度与参数范围在入口检查；实际通行由原生 planner 的地图和碰撞门控判定。
`--altitude` 为相对起飞高度，往返缺省 2m、悬停缺省 1m。机库附近存在立柱，
往返高度需要同时满足已知障碍净距和配置的规划高度边界；planner 保留原有碰撞门控。
`--hover-seconds` 为每个航点的停留时长，往返缺省 3s、悬停缺省 20s。
`--return-offset DX DY` 缺省 (0.12,-0.08)m，限定在起飞点 0.3m 内，作为视觉接近点。
航段同时检查当前 PlannerStatus/Reference 目标身份、原生完成状态、里程计距离和低速保持。
返航后的 acquire 阶段要求至少三幅新的有效视觉观测，采集时间跨度至少 0.15s；
旧观测、异队伍/会话和过期输入不能启动精降。`--acquire-timeout` 提供有界识别等待。
任务异常时请求 HOLD 并结束本轮仿真，原始失败保持非零结果。
返航引导依赖连续 VIO；累计漂移使平台超出可识别邻域时，本入口以识别超时结束，平台重捕获范围由相机视场与 landing 门控决定。
省略 `--precision-land` 使用原有起飞点高度降落；真机接入仍遵循硬件与 NPU 验收边界。

Mimir 先启动、最后关闭；任务设置、阶段、起飞坐标锚点和所用输入的指纹进入 `logs/flight`。
精降任务关闭 RRD 后自动运行 `scripts/verify_flight.py`，也可以用上面的独立命令复核。
评分器要求完整 footer、时序覆盖、原生模式与精降阶段顺序，以及激活窗口的 352×288 下视灰度图像；
同时检查下视帧率上界（含 1ms 采集时钟量化和长期频率约束）、空闲期无送图/推理，以及任务结束后的资源释放；旧录制沿用其原有全程图像契约。
视觉闭环验收须保持 `mimir.images = true`。真值仅在评分阶段使用，
以起飞时刻的 GT/odom 姿态固定坐标对齐，不使用全程拟合抵消漂移。
航点停留按声明顺序逐点检查目标身份、原生 TRACK/HOLD 模式、实际位置、速度与持续时间；
每个点均须合格，评分分别报告各点及全程最大误差。返航分别检查水平/高度误差和机库邻域；
落地检查实际落点、原停机坪高度、上锁时机和上锁后的静止保持。误差门槛与失败原因由
评分源码和测试维护；碰撞安全仍由 planner/FC 执行，实时性能另按原始观测核对。
`commands_completed` 仅描述任务阶段执行完成；验证失败或进程回收失败均使入口非零退出。
运行产物遵循 [Mimir 录制规则](mimir.md)。

### 可选资产检查

```bash
FIREFLY_RUN_RMUC_ASSETS=1 uv run --all-packages --extra test pytest tests/system/test_rmuc_assets.py -q
cargo test --release -p firefly-planner --test rmuc_asset -- --ignored
cargo test --release -p firefly-planner rmuc_22m_initial_trajectory_is_executable -- --ignored
```

库图独立视角验收先启动 `./target/release/mimir`，再执行：

```bash
RUST_LOG=info FIREFLY_QUERY_MAP=<独立查询.ffvmap> FIREFLY_QUERY_COUNT=20 \
  FIREFLY_MAP_VALIDATION_REPORT="$PWD/logs/map_validation.json" \
  cargo test --release -p lightglue held_out_map_localization_contract -- --ignored --nocapture
```

独立查询使用建库位置之外的视角，查询位姿作为评分标签，先验加入固定偏差；每个指定查询
分别报告几何与误差断言，以及接受或拒绝结果。模型导出、资产质量、库图定位、飞行和
在线回环分别验收；在线回环以真实重访被后端接受的记录为证据。
