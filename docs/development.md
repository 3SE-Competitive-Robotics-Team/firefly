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
  运行诊断归 RRD，配置、指纹和评分摘要归 `logs/` 的 JSON/HTML。
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
停机坪定义统一由 `firefly_mujoco.scene.SCENES[].pad` 与 `PAD_CLEARANCE` 提供。
render 使用无窗口 GPU 渲染；Rerun 原生 Viewer 需要图形会话。Rust/Python 版本和依赖
以 workspace 配置为准。mimir 直接将左右目无损灰度图和结构化诊断写入 RRD，配置见 `configs/mimir.toml`。

```bash
uv sync --all-packages --extra test
cargo xtask release
```

运行进程一律 release；普通 `cargo test` 用 debug。源码改动后先完成构建，再做运行验收。
分别在终端启动，记录进程最先启动、最后退出：

```bash
./target/release/mimir --save logs/rmuc_run.rrd
uv run firefly-sim
cargo run --release -p render
cargo run --release -p firefly-rootslam
cargo run --release -p fc
cargo run --release -p planner -- --traj-server
cargo run --release -p planner
```

最小闭环只需 mimir/sim/render/firefly-rootslam/fc；规划同时启动 planner 与 `planner --traj-server`
两个独立进程，前者规划并发布心跳，后者采样并发布飞控参考。
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

板端构建按机载应用名单逐包编译，依赖的 feature 只在这些包之间统一；全 workspace
构建（`cargo test --all-targets --release`）把所有成员拉进来，一个包启用的 feature
会掩盖另一个包缺失的声明。所以**全 workspace 编译通过不代表板端构建通过**：`nix` 的
`fs` feature 就是这样暴露的——板端 `camera --hil` 用 `nix::fcntl` 设 O_NONBLOCK，
全 workspace 构建时 feature 被别的包统一打开，板端逐包构建却报找不到该函数。依赖被
门控的 feature 时要在 workspace 依赖里显式列出。

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
| `camera` | `V4L2` 节点 | 发布 `Firefly/CameraLeft`、`CameraRight`、可选 `Depth` |
| `imu` | `spidev`（`BMI088` 两片） | 发布 `Firefly/Imu` |
| `motor` | `sysfs` PWM | 订阅 `Firefly/Control`，发布 `Firefly/Airframe` |

三者由 xtask 的 board release 清单构建。上板前按配置核对：

- `configs/camera.toml`：`/dev/videoN` 编号随内核与接线变化，先用
  `v4l2-ctl --list-devices` 核对。分辨率由 IPC 定长消息定死（320×240），
  驱动按该组合启动；格式协商失败时返回错误。`[depth]` 控制深度发布。
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

### HIL 部署与运行

HIL 用同一根 USB 线把电脑仿真与板端生产进程接起来。构建、部署与链路自检：

```bash
cargo xtask deploy-hil [--device firefly-0001]
```

它先执行普通 `deploy`（构建、打包、解包、服务就绪），再把
`configs/processes-hil.toml` 切为活动进程清单，由板端 `hil-gadget up` 重建
gadget，最后用 `hil --check` 要求厂商接口与两个端点都在位才算就绪。`--probe`
只打印 USB 描述符。

板端 gadget 约束（UDC 事实见 [spec.md](hw/spec.md)）：

- 每次 `up`/`down` 整块删除并重建 `g1`；configfs 在绑定失败后会保留
  `udc_name`，重试写 UDC 只会得到 `EBUSY`，反复失败还会推进接口计数。
- FunctionFS 描述符写完后必须写 strings 块，内核才创建 `epN`；`fs_count`/
  `hs_count` 是描述符个数，不是字节数；`ep0` 事件结构是 12 字节。
- 解绑后 OTG 角色可能退回 host；写回 `usb_device` 会触发厂商驱动的
  `rmmod/insmod`，仅在角色不对时写。
- 绑定后通过 UDC `soft_connect` 断开再接入，避免主机在同一 serial 上快速
  重枚举时拒绝注册新配置。
- 镜像 `/etc/adb_conf.sh` 的后台 `enable_udc` 循环会抢绑 UDC，重建前必须
  让它退出（`/etc/.disable_udc` + `pkill -x adb_conf.sh`）。

HIL 期间 `adb` 不可用，`hil-gadget` 的每一步写入
`/opt/firefly/logs/hil/up.log` 并 `fsync`（TF 卡 `commit=600`，断电会丢未落盘
写入）。`hil-gadget watchdog` 在 60 s 内没等到绑定完成或主机心跳时恢复 adb：
先停 HIL 进程集、写回 `processes-normal.toml`，再重建 adb-only gadget 并启动正常
`firefly`——不能只换 gadget，否则 camera/motor 仍持有 ffs 端点，adb 会时有时无。
`up` 启动前按 pidfile 结束旧的 `serve`/`watchdog`，多次部署不会留下互相抢 UDC
的看门狗。板端 HIL 应用收到任意下行帧会刷新
`/opt/firefly/logs/hil/host_seen`。

主机侧依次启动 `firefly-viz`、`firefly-sim`、`render` 与 `hil`；`configs/hil.toml`
的 `[link]` 控制 IMU 批大小与图像节流。`imu_batch` 必须为 1：板端估计器的 IMU
队列有界，成批突发会整批拒绝。`image_period_s` 按板端单帧耗时设置（当前 0.167 s）；
双目只在时间戳一致时成对下发，否则 okvis 的双目容差会拒绝整帧。桥每秒发布
`hil/link`、`hil/board` 计数标量并写入 RRD，同时打印控制老化（控制所用 IMU
时刻相对主机最新下行 IMU 的滞后）。

PoC 步骤（`ARMED_GROUNDED` 不保高，解锁后必须尽快给起飞目标）：

```bash
./target/release/ffctl fc arm
./target/release/ffctl fc takeoff 1.0
./target/release/ffctl fc hold
uv run python scripts/hil_hover_monitor.py --seconds 40
```

`scripts/hil_hover_monitor.py` 打印真值高度、里程计高度、位置误差、最大推力
与模式，只订阅不参与控制。验收仍以本文的任务门槛为准，HIL 不得降低算法门槛。

已验证：USB 传输用 `hil/camera/{left,right}` 回传像素和，与主机下发一致
（`hil` 日志的“校验ok/校验不一致”，当前 0 不一致）；1 kHz IMU/控制、~1.3 kHz
里程计上行稳定，控制老化均值 2.5 ms、峰值 8–10 ms；关掉深度并按 1 ms 时间戳
成对下发双目后，VIO 可连续 55 s 保持新鲜。

**闭环结果**：`configs/okvis2-x-hil.toml` 用 200 特征、送图 6 Hz（飞行时单帧约
128 ms、后验 0.28–0.40 s），配合 `bin/fc --airmode-off`：解锁 → 起飞 2 s 到 1.40 m
→ HOLD 保持 20 s（1.39–1.42 m，`gt_vz≈0`）→ LAND（模式 5，0.4 m/s 下降）→ 落地
自动上锁，记录在 `logs/hil_6hz.rrd`。8 Hz 版本在约 16 s 时 okvis 图像队列溢出、
里程计断流，FC 判状态估计丢失进入 AttitudeFallback；6 Hz 后不再出现，说明板端
VIO 的余量只够 6 Hz 量级。`--airmode-off` 只用于 HIL 验证：板端 VIO 后验延迟让
速率环在低集合推力下饱和，`airmode=1` 会为保姿态抬高总推力（实测 `thrust_z≈-0.5`
而垂直环在刹车），形成爬升；生产 `configs/fc.toml` 仍保持 `airmode=1`，不带该开关。
这条偏差必须在结论里单独标注；真正的修法是降低速率环饱和/时延，而不是改生产
配置。

**30 Hz 供给未达标**：`render` 的采集周期是固定常量
（`apps/render/src/link.rs` 的 `CAPTURE_PERIOD = 0.1`），仿真位姿也是 10 Hz，所以
主机当前最多供 10 Hz 图像；HIL 桥又按 6 Hz 节流给板端 VIO 留余量。要满足 30 Hz
需同时改 render/sim 的节拍并重新压测 USB/板端队列，本次没有做，不能用 6 Hz 的
悬停结论冒充 30 Hz 实时性。

**遗留**：降级状态（AttitudeFallback）仍没有地面上锁——8 Hz 那次降级后飞机穿过
停机坪继续下落。正常 LAND 已通过，但估计丢失后的地面保护仍是安全遗留项，需要
可靠的落地判据。

悬停时分配长期接近力矩上限：`hil_6hz.rrd` 的 HOLD 段（18.3 s）里，
`torque_sp_frd - allocated_control` 的前三轴范数均值 0.035、p95 0.106、最大
0.295（归一化），约 60% 的 tick 超过 0.01；逐轴看几乎全在 pitch（std 0.052），
对应的实测 pitch 角速度 std 0.62、到 ±2.3 rad/s，而 pitch 速率设定只有 ±0.15，
是 pitch 速率环的极限环。这不是 1e-4 的饱和阈值噪声。

对照与排除：
- **本地同源悬停**（sim + render + okvis + fc，直连 IPC）`saturated_ticks=0`，
  而 HIL 是 700–850/1000，说明差异来自 HIL 的 IMU/控制通路，不是 FC 算法本身。
- 桥的上行优先（控制先于图像下行）把控制老化从均值 2.5 ms、峰值 26 ms 降到
  均值 1.4 ms、峰值 10 ms，但 pitch 极限环不变（std 0.620 vs 0.618），排除
  控制回路时延为主因。
- 内环速率改取 FC 独立 ESKF 的速率后饱和反而升到约 850，因为 ESKF 的 `rate` 是
  滤波状态、比 VIO 预测的陀螺更滞后；该改动已撤回。
- `airmode=1` + 独立 ESKF 姿态、以及 `airmode=1` + 上述内环速率改动，都仍在起飞
  瞬间满推力发散，说明触发在起飞瞬态（HIL 的 IMU 交付抖动），不是姿态源。

结论：HIL 验证仍用 `--airmode-off`；生产 `airmode=1` 在 HIL 下的差异应归因于
USB 交付的 IMU 时序（真实机体 SPI IMU 不经过这段通路），后续要把 HIL 的 IMU
通路时延/抖动压到本地水平，或让 FC 内环直接使用最新原始陀螺样本，再验证生产
分配。

**独立端点的实测结果**：把 HIL 功能扩成「OUT 0x03 IMU/指令 + OUT 0x05 图像 +
IN 0x84 控制」后，这颗 `sunxi_usb_udc` **能枚举、能绑定**（内核把地址重排成
0x01/0x02/0x81），但**第二个 OUT 的数据通路不工作**：camera 打开 ep1/ep2 后
收不到任何 IMU/图像（fc 一直 `IMU 0 Hz`、桥的 `校验ok=0`），同时 IN 的控制
回传正常。说明厂商 UDC 的多 OUT 端点只有描述符、没有实际投递，方案 2 在这颗
UDC 上不可用；已回退到单 OUT，下一步只能走「同一 OUT 端点、写线程在图像分片
之间插 IMU」的软件兜底，或先修厂商 UDC 驱动。

### 图像分块与 IMU 插队（HIL 协议 v2）

单 OUT 端点上，整帧 150 KB 图像会把 1 kHz 的 IMU 压在后面约 5 ms。协议 v2 新增
`ImageChunk`：每张图按 16 KiB 分块（块头 + 像素，单块不超过一个帧），主机写线程
先排空 IMU/指令高优先队列，再写一块图像，块间继续插高优先消息；板端 `camera`
按 `image_id` 重组，收齐后 `publish_with` 直接填进 iceoryx2 共享内存，校验和在同
一次拷贝里算掉。主机侧不再把整帧 encode 成中间 `Vec`，板端不再 decode 出中间消息。

2026-10-27 实测（firefly-0001，6.7 Hz 双目）：

- 链路：control 1001/s、odom 1.28–1.29 kHz、`校验ok=12/s` 且不一致 0、控制老化
  均值 1.5–1.6 ms、最大 5–6 ms；
- `--airmode-off`：起飞 1.0 m、悬停 1.40–1.43 m、自动降落；
- **速率环饱和仍为 693–779 次/秒（与分块前相同）**；FC 侧 `imu_age` 均值 2.9 ms；
- 生产 `airmode=1`（不带开关）：起飞瞬间 z=0.84 m、vz=1.71 m/s，仍发散。

结论：分块解决的是**最大阻塞**（整帧 5 ms → 单块约 0.4 ms），没有降低**平均
延迟**（FC 侧约 2.9 ms），而 airmode 放大的是持续饱和。HIL 仍用 `--airmode-off`；
要让生产 `airmode=1` 在 HIL 下成立，下一步必须压均值延迟（FC 内环直接用最新
原始陀螺并按时间戳外推，或换更低延迟的传输），而不是继续减小块。

### RootSLAM 的 HIL 配置

板端 HIL 同时只跑一个估计器：两个估计器都订阅同一批 IMU 与双目话题，同时跑
会抢队列。`cargo xtask deploy-hil --estimator` 选估计器，`okvis2-x` 用
`configs/processes-hil.toml`，`rootslam` 用 `configs/processes-hil-rootslam.toml`；
板端 `hil-gadget up [manifest]` 接受同一份清单名，省略时用默认清单。

RootSLAM 侧与 OKVIS2-X 的差别：

- 估计器用 `bin/firefly-rootslam` 的 live 入口，`--no-loop-closures
  --no-mapping` 关掉回环与建图，理由与 OKVIS2-X 相同：板端初始化不能阻塞
  IMU 队列。
- RootSLAM 只用双目 + IMU，不消费深度，与 HIL 的 `depth = false` 一致。
- 话题层不需要适配：RootSLAM 订阅 `Firefly/Camera/Left`、
  `Firefly/Camera/Right` 与 `Firefly/Imu`，与 `camera --hil` 发布的完全相同；
  它向 FC 发布的 `Firefly/Odometry`、`Firefly/Navigation`、
  `Firefly/VioAttitude` 也与 FC 订阅的话题一致。
- VIO 参数用 `VioPreset::Rig` 预设：Basalt 自带的默认参数是按 512×512 TUM-VI
  调的，在 320×240 的 rig 上光流网格只剩 6×4 = 24 个单元（约 24 个特征点）；
  预设把网格边长改成 20 px，得到 16×12 = 192 个单元，特征点数与 OKVIS2-X 在
  板端实测可用的 200 点对齐。`configs/rootslam-hil.toml` 里用
  `vio_preset = "rig"` 显式声明；`vio_config` 指向的文件在预设之后整体覆盖。

**未验证**：板端没有 RootSLAM 的实测数据。网格边长 20 px 是按特征点数对齐推
出来的，不是按板端单帧耗时或跟踪质量实测；板端的单帧耗时、核分配与送图节流
都还没有数据。首次上板需要记录 `rootslam/frame_processing_ms`、里程计连续性与
悬停高度，再回头调 `vio_preset` 与 `image_period_s`。

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

`packages/firefly-evaluation` 提供任务编排、进程管理、物理观测、RRD 评分与报告。
CLI 和系统测试都通过 uv workspace 导入该包；`tests/` 仅保留测试和 pytest fixtures。
包从当前目录向上定位工作区，CLI 可用 `--workspace` 指定目录。
纯评分、报告与进程管理单元测试：`uv run pytest tests/evaluation -q`。

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
cargo test -p planner --test trajectory_ipc -- --ignored --test-threads=1
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
以下用例自行启动生产进程、用本次 RRD 评分并清理退出：

```bash
FIREFLY_RUN_SENSOR_STARTUP=1 uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py -q
uv run pytest tests/system/test_imu_flight.py -q
uv run pytest tests/system/test_imu_flight.py -q -k imu_hover
uv run pytest tests/system/test_hover_disturbance.py::test_hover_disturbance -q
uv run pytest tests/system/test_roundtrip.py -q
```

- 启动检查屏蔽 PlantState，保留 render 所需 GroundTruth；未启用飞行时只证明初始化与零推力。
- IMU 用例分别检查飞行、持续悬停与 VIO 失联后的有限姿态支持。保持原目标，独立评分整段
  真值、姿态和主链路健康；静止评分使用固定等待与窗口，全程检查碰撞、重置和退出。
- 扰动用例保持同一个 Hold 目标，逐轮恢复并持续悬停，评分通过后发送下一轮，按实际执行轮次报告。
- 估计器为 `firefly-rootslam`，处理时延证据来自 RRD 的 `rootslam/frame_processing_ms`。
- 22m 往返使用 `configs/acceptance_22m.toml`，默认空局部地图与 RootSLAM VIO；
  分别验证初始化、真实到点、跟踪、定位、降落、碰撞和退出；在线回环使用独立验收。

任务配置和门槛由 `firefly_evaluation` 的 `mission.Options`、`hover_metrics.HoverLimits`、
`disturbance_metrics.DisturbanceLimits` 及 `configs/acceptance*.toml` 维护，运行前固定并存入报告。
新增门控须以固定配置独立启动三次、全部通过才建立基线，保留每次失败。

姿态融合、初始化、坐标、视觉前端或仿真传感器改动后，必须运行 `imu_hover`；
悬停、往返与回环分别验收。显式指定系统用例缺运行条件时报错；普通 CI 的 skip
单独列出。`tests/evaluation/test_hover_metrics.py` 使用纯数据检验评分规则。

### 任务验收与可选地图

CLI 使用已构建的 release 程序，运行前执行统一构建命令：

```bash
cargo xtask release
uv run firefly-evaluate
uv run firefly-evaluate --case nominal --config configs/acceptance_22m.toml
```

`--case nominal|reference_loss|estimator_loss` 选择任务。所有任务使用 RootSLAM VIO 与从空地图
启动的局部深度规划；飞行误差、碰撞和时间门槛由验收配置固定。

正常阶段流转使用新鲜估计状态和 FC 模式；真值用于独立评分及碰撞/越界中止。
局部 VIO 采用固定尺度的航向/平移对齐，地图定位与参考跟踪直接使用各自坐标契约评分。
会话地图评分以首个真值样本确定固定换基，原始数据保留在独立评估链路。
完整任务报告逐项列出覆盖、断流、完成状态、碰撞、重置和退出清理结果。

证据落 `logs/`：RRD 保存原始观测/事件，JSON/HTML 保存配置、源码/二进制/资产指纹、
阶段、实际指标与退出码。依赖失败的阶段标 blocked，未执行阶段单列，所有原始证据保留。

### 可选资产检查

```bash
FIREFLY_RUN_RMUC_ASSETS=1 uv run --all-packages --extra test pytest tests/system/test_rmuc_assets.py -q
cargo test --release -p firefly-planner --test rmuc_asset -- --ignored
cargo test --release -p firefly-planner rmuc_22m_initial_trajectory_is_executable -- --ignored
```

库图独立视角验收先启动 `./target/release/mimir --save logs/map_validation.rrd`，再执行：

```bash
RUST_LOG=info FIREFLY_QUERY_MAP=<独立查询.ffvmap> FIREFLY_QUERY_COUNT=20 \
  FIREFLY_MAP_VALIDATION_REPORT="$PWD/logs/map_validation.json" \
  cargo test --release -p lightglue held_out_map_localization_contract -- --ignored --nocapture
```

独立查询使用建库位置之外的视角，查询位姿作为评分标签，先验加入固定偏差；每个指定查询
分别报告几何与误差断言，以及接受或拒绝结果。模型导出、资产质量、库图定位、飞行和
在线回环分别验收；在线回环以真实重访被后端接受的记录为证据。
