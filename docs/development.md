# 开发指南

开发、运行、记录和验证统一在本文；系统职责、坐标、算法及能力边界见
[architecture.md](architecture.md)。路径和命令均相对仓库根目录。
观测契约统一见 [observability.md](observability.md)；README 提供介绍和入口。

## 维护与收敛

- 文档描述当前职责、契约、操作入口和验收边界；同一规则只维护一处，其余位置链接到该处。
- 调整职责时，同步清理旧接口、配置、依赖与文档。删除前全仓追踪生产、测试、工具和部署
  消费者，确认保留路径仍可构建、运行和验证。
- 参数、默认值与单位归对应类型及配置，测试条件与门槛归测试源码；文档只解释来源和使用边界。
- 注释写用途、不变量、边界条件和仍成立的原因；过程、修复对比和计划分别归 Git 历史、
  运行证据与 issue。运行诊断归 RRD，配置、指纹和评分摘要归 `logs/` 的 JSON/HTML。
- 编译、类型检查、单测、真实 IPC、数据集与飞行分别验证和报告；未运行、跳过、失败分别说明，
  不放宽门槛，也不把局部通过扩展为整机能力。

## 实现约定

- 新 crate：`cargo new crates/<name> --lib --edition 2024`（自动纳入 workspace）。
- DDD 拆 crate（firefly-*），依赖自上而下，领域层不依赖应用层。
- 错误用 firefly-error：kind/status 分类，操作名统一 `.with_operation(...)`，其他诊断字段
  用 `.with_context(...)`；不得以 `"operation"` 上下文键绕过结构化字段。
  `Permanent` 表示不可重试，`Temporary` 表示允许重试。
- 运行诊断：关键函数 `#[fastrace::trace]`，日志用 log 宏；CLI 的帮助、结果和导出内容可写 stdout。
- 场景光照唯一一份（`firefly-render::lighting`），所有相机共用；视角差异只写相机
  配置（曝光/色调映射），禁止给单个相机加专属可聚簇光源（`PointLight`/`SpotLight`）
  ——Bevy GPU 聚簇在多视图同帧渲染时不按 `RenderLayers` 隔离，会渗进别的视角。
- 依赖统一在根 `[workspace.dependencies]`。
- IPC 发布/订阅使用 `firefly-pubsub` 的 `Publisher<T>` / `Subscriber<T>`，话题名与容量
  由消息模块导出；独立接收深度和服务容量用 `SubscriberOptions`。需要事件唤醒的发布端
  显式使用 `with_topic_notify`，trace 用户头由泛型端口统一处理。

### 地面站 CLI

`ffctl` 用 `usage-rs`，固定 `<域> <动词>` 两层，命令声明在 `apps/ffctl/src/cli/`。新增子命令：
在 `cli/<域>/` 加一个文件，`#[derive(Args)]` 与 `impl Run` 写在一起，再在该域的动词枚举里
挂上；新增域则在 `cli/<域>/mod.rs` 建容器、在顶层 `Command` 挂上并给 `display_order`。
取值与约束写成 `usage-rs` 的声明式属性（`value_enum`、`validate` 等），`main.rs` 通常不动。
框架用法见官方 [Rust 框架文档](https://usage.jdx.dev/rust/) 的 quickstart、subcommands、
args-and-flags、validation。

### 配置

- **TOML、一应用一份**：统一放仓库顶层 `configs/`（`sim.toml` / `firefly-rootslam.toml` / `planner.toml`），
  启动时加载（Rust 支持 `--config` 换文件）。配置文件可选：缺文件记录一行 INFO 并使用
  代码默认值；文件存在但不可读、格式或参数无效仍报错，不静默回落。
- **拒绝未知键**：配置解析检查嵌套字段，避免拼写错误静默使用默认值；复合配置保留各部分的校验。
- **最小化**：保留必填键，只对有默认值的键写差异覆盖；无覆盖项时保留仅含用途和
  默认值来源注释的配置文件。缺失键使用代码默认值；
  纯数据 Options 直接 `serde::Deserialize` + `#[serde(default)]`，Python 用标准库
  `tomllib`；应用配置不引 YAML。数据集适配器可读取官方原始传感器格式，如 Kalibr YAML，
  转换到统一标定类型后使用。

#### 应用启动入口

固定顺序：**解析命令行 → 初始化基础日志 → 读取、转换并校验配置 → 报告配置来源
→ 创建运行资源 → 运行应用**。命令行帮助与参数错误不创建运行资源。

- Rust 主配置通过 `firefly_config::startup(path, init_logging, validate)` 加载。
  普通进程传入 `firefly_observability::init`；校验函数返回 `firefly_error::Result<()>`。
  无额外参数约束时传入 `|_| Ok(())`。配置类型只描述字段、默认值和参数约束，
  文件读取、日志安装和退出处理分别由公共入口及 `main` 负责。
- Python 主配置通过 `firefly_workspace.startup(path, decode)` 加载。
  `decode` 将字典转换为应用配置并校验，不创建运行资源；默认安装标准库 INFO 日志。
  宿主需要特殊日志格式时通过 `init_logging` 参数提供初始化函数。
- 公共入口成功返回后才创建 IPC、模型、工作线程、窗口等资源。辅助配置在此之后使用
  `firefly_config::load` / `firefly_workspace.load_toml`，不重复初始化进程日志。
  领域对象构造时仍保留自己的不变量校验。
- 配置来源由公共加载模块记录，调用方不重复打印。缺文件与空文件
  使用相同默认值；缺文件仅在转换、校验成功后提示回落。INFO 的显示尊重日志过滤设置，
  Rust 可用 `RUST_LOG=info` 查看。
- Bevy 先独立安装日志设施，在配置通过后安装其余插件。使用 `LogPlugin` 时，
  在随后的 `DefaultPlugins` 中禁用它，避免重复安装；自定义日志宿主沿用自己的桥接。

新应用以可编译的 [Rust 最小应用](../crates/firefly-config/examples/minimal.rs) 和
[Python 最小应用](../packages/firefly-workspace/examples/minimal.py) 为入口示例；
Bevy 日志接线参考 [quad](../apps/quad/src/main.rs)。Rust 示例随
`cargo check --workspace --all-targets` 编译。

标定与噪声改对应 Options 默认值，doc 注释写单位和来源；部署覆盖同步 `configs/`。
升级视觉依赖必须运行相关前端、投影及匹配回归，不放宽容差；依赖统一在根 Cargo/uv workspace 和锁文件。
改行为前阅读对应官方源码，固定版本与差异见架构文档。参考仓库位于 `~/Projects/`：
`open_vins`、`ORB_SLAM3`、`VINS-Fusion`、`EGO-Planner-v2`、`iceoryx2`、`logforth`、`fastrace`；
purecv 的来源与固定版本以 workspace 依赖及 Cargo.lock 为准，发现上游问题应提 issue。

## 提交与 PR

commit message 使用 `<type>(<scope>): <摘要>`；PR 标题使用 `<type>: <中文摘要>`，
描述整个 PR 的最终结果，不写中英文括号、WIP 或阶段编号。补充信息用逗号分隔，
例如 `fix: 修正悬停参考的速度前馈`。PR 说明交代行为变化、依据和实际验证结果。

<a id="run"></a>

## 构建与运行

场景由 `configs/scene.toml` 的 `scene = "<赛季>@<版本>"` 唯一选择，已注册
`rmuc2027@1.0.0`；资产在 `models/<赛季>/v<版本>/`（`field.glb`、
`<赛季>_collision.json`），另需 `models/drone/drone.glb`。未注册版本或缺失资产报错，不回退场景。
停机坪定义来自 `firefly_mujoco.scene.SCENES[].pad` 与 `PAD_CLEARANCE`，不在文档复制坐标。
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
`lightglue --map <地图.ffvmap>`。默认进程清单不自动启动这些可选服务。
planner 只消费机体 Odometry 与深度；相机外参见 planner 配置，不消费可选库图定位输出。
禁止把场地真值作为默认定位或规划先验。

### 机载部署

板载平台为 Orange Pi Zero 3W，系统与硬件见 [硬件规格](hw/spec.md)。
`--target board` 在 Apple container 的 Debian trixie 镜像中原生构建；镜像 digest、
工具链及 bindgen 依赖由 [`xtask/src/container.rs`](../xtask/src/container.rs) 固定，
产物在 `target/container/aarch64/release`。首次运行前启动 `container system start`；
xtask 自动构建派生镜像并复用缓存，不维护交叉 sysroot。

产物固定放 `/opt/firefly/{bin,configs,models}`：

```bash
cargo xtask release --target board
cargo xtask deploy
```

板载应用名单由 [`xtask/src/target.rs`](../xtask/src/target.rs) 维护。
`deploy` 把二进制、`configs/*.toml` 与默认视觉模型打成
`dist/firefly-aarch64.zst`（zstd 容器，不是 tar），推到设备后由同一 xtask 的 `unpack`
解包，`--device` 默认 `firefly-0001`。解包前会停掉 `firefly.service` 并清空
`/opt/firefly/{bin,configs,models}`，退役的进程与配置不会残留在板上；`logs` 与镜像
预置的 `systemd` 保留。解包后把每个二进制软链到 `/usr/local/bin`，因此 `ffctl`、
`fc` 等在任意目录、非交互 `adb shell` 下都可直接调用，不需要改镜像的 shell 配置。
最后重新 `systemctl start firefly`，以 `ffctl process status` 在超时内应答为就绪判据。
systemd active 只表示服务已启动；socket 应答才证明 supervisor 可管理进程。
启动失败或解包缺文件时 `deploy` 非零退出。板载 adbd 使用 legacy shell 协议，
远端退出结果通过输出判定，不能依赖 `adb shell` 的本地退出码。

二进制按安装布局定位配置与模型，与启动目录无关；解析顺序（越显式越优先）：

1. `FIREFLY_ROOT` 环境变量；
2. 可执行文件的上两级（部署布局下 `bin/<app>` → 安装根；仅当该前缀下存在 `configs/`）；
3. 编译期默认：板载 `/opt/firefly`，开发机构建为仓库根。

解析由 [`firefly-config::paths`](../crates/firefly-config/src/lib.rs) 实现。
`--config <path>` 可覆盖单个配置文件。非标准安装前缀使用 `FIREFLY_ROOT`。
supervisor 的相对命令 `bin/<app>` 和 `logs/process` 仍依赖工作目录；运行
`ffctl process supervise` 必须以安装根为 cwd。安装根定位不替代这项进程运行约束。
可复现输入为镜像 digest、Rust 工具链、`--locked` 和 `Cargo.lock`。
升级基础镜像时同步更新容器配置，派生镜像按工具链与 digest 生成标签。

板侧传感器与执行器由三个独占进程接替仿真侧的 `render` 与 `firefly-sim`：

| 进程 | 设备 | 话题 |
|---|---|---|
| `camera` | `V4L2` 节点 | 发布 `Firefly/CameraLeft`、`CameraRight`、可选 `Depth` |
| `imu` | `spidev`（`BMI088` 两片） | 发布 `Firefly/Imu` |
| `motor` | `sysfs` PWM | 订阅 `Firefly/Control`，发布 `Firefly/Airframe` |

三者纳入 xtask 的 board release 清单，host release 清单不包含它们。上板前按配置核对：

- `configs/camera.toml`：`/dev/videoN` 编号随内核与接线变化，先用
  `v4l2-ctl --list-devices` 核对。分辨率由 IPC 定长消息定死（320×240），
  驱动不接受该组合时进程直接报错，不静默换成别的分辨率。删掉 `[depth]`
  就不发布深度。当前实现只覆盖配置指定的 `V4L2` 流，具体相机的格式、同步、
  时间戳和标定须在实机验证，不能从设备名称推断兼容性。
- `configs/imu.toml`：目前只实现 `BMI088`，`WHO_AM_I` 不匹配直接退出，
  不会在错型号上继续跑。寄存器初始化值取自数据手册，首次上板须核对
  `Firefly/Imu` 的读数是否落在量程内的合理重力/静止零偏。
- `configs/motor.toml`：`sysfs` PWM 只适用于 PWM 电调。`[airframe]`
  （质量/惯量/旋翼几何/推力上限）在这里维护一份，真机装配后必须换成实测值，
  否则飞控会拿仿真机体的参数去分配推力。`Control` 断流时四路回到零推力脉宽。
  当前内核的 `pwm-sunxi` 拒绝使能**没有设备树描述**的通道（dmesg 报
  `can't parse pwm device`），所以上板前必须先补 PWM 通道的 DT 节点，否则
  `sysfs` 只能写周期和占空比、`enable` 永远失败。

三个驱动默认 `autostart = false`。先核对设备节点与设备树，再逐个
`ffctl process enable camera`、`imu`、`motor` 并独立验证。`processes.toml`
随部署覆盖，设备上的手动修改不会跨部署保留。

`camera` 与 `imu` 的时间戳都用 `CLOCK_MONOTONIC` 秒，与仿真侧 `sim_time`
语义不同，同一份录制里不要混用。三个进程都可以单独起停：`bin/camera`、
`bin/imu`、`bin/motor` 接 `--config <path>`，`Ctrl-C` 正常退出。

`firefly.service` 由镜像构建 [`orangepi-build`](https://github.com/3SE-Competitive-Robotics-Team/orangepi-build)
预置并默认启动；服务执行 `ffctl process supervise`。supervisor 按 `configs/processes.toml`
的数组顺序拉起 `autostart = true` 的进程。`processes.toml` 被 `deploy` 覆盖；
`lightglue` 在生产 `.ffvmap` 部署前保持 `autostart = false`。运行期管理：

```bash
cd /opt/firefly
./bin/ffctl process status                      # 一次性，含 CPU%/MEM
./bin/ffctl process status --watch --interval 1 # top 式刷新，Ctrl-C 退出
./bin/ffctl process start [name...]     # 省略 = 所有 autostart；显式名字忽略 autostart
./bin/ffctl process stop [name...]      # 省略 = 全部，逆序
./bin/ffctl process restart [name...]
./bin/ffctl process enable <name...>    # 改 processes.toml，下次 supervisor 启动生效
./bin/ffctl process disable <name...>
systemctl stop firefly                  # supervisor 逆序 SIGINT 所有进程
```

supervisor 对异常退出退避重启，停止超时只报错、不 SIGKILL；具体时限由
[`supervisor.rs`](../apps/ffctl/src/process/supervisor.rs) 维护。
子进程 stdout/stderr 在 `logs/process/<name>.log`，运行诊断的归档证据仍以 RRD 为准。
`camera`、`imu`、`motor` 已实现标准 IPC 并纳入板载构建；设备同步、标定、实时性和飞行
均须实机验收。部署成功只证明发布和 supervisor 应答。

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

起飞高度相对起飞点，planner 目标属于当前 odom 坐标。拒绝原因查看 RRD 的 `logs/fc`。
`ffctl planner goal X Y Z --yaw 90` 可指定终点航向（度，odom +X 为零、逆时针为正）；
省略时随规划路径定向。目标不包含独立 roll/pitch，这两个轴由飞控实现推力方向。
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

脚本、CAD 与验收共用 `firefly-workspace` 的工作区定位和流式 SHA-256；该包只依赖标准库。

```bash
uv run --package firefly-cad firefly-cad-build <场地.stp> --scene <赛季>@<版本>
```

`firefly-cad-build` 构建并验收几何资产，配置为 `configs/cad.toml`。GLB、碰撞盒、FFMap 必须来自
同一归一化几何，通过内容 hash、坐标和占据检查；细节见
[CAD README](../packages/firefly-cad/README.md)。在线运行、离线采集、建库与验收资产指纹
共同使用 `configs/scene.toml` 所选场景；采集与库图默认输出位于该版本的资产目录及地图路径。

`scripts/collect_vision_map.py` 使用 `render --offline` 和隔离的 `Firefly/Offline/*`，
按同一时间戳配对左图、理想深度与位姿标签，不启动在线估计器/飞控，不把标签发布到在线话题。
无同步帧不得建库；图像纹理、路标覆盖与内容指纹由采集和构建器检查。
库图兼容性只依赖场地、标定、光照和曝光；源码/二进制 hash 用于追溯，普通重编译或在线
噪声变化不要求重建。不能改历史 manifest 冒充原采集；替换资产前归档。

官方预训练模型的可复现导出（CPU，独立环境，不影响运行环境）：

```bash
uv venv models/vision-export/.venv --python 3.12
uv pip install --python models/vision-export/.venv/bin/python -e packages/firefly-workspace
uv pip install --python models/vision-export/.venv/bin/python torch==2.15.0 torchvision==0.29.2 --index-url https://download.pytorch.org/whl/cpu
uv pip install --python models/vision-export/.venv/bin/python onnxscript==0.7.2 onnx==1.23.2 onnxruntime==1.31.1 kornia==0.9.0 opencv-python==5.0.0.93 matplotlib==3.11.2
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

观测登记、topic 清单、开关、录制及完整性验收统一见 [可观测性](observability.md)。
新增或修改诊断必须先遵循该文档的登记与添加流程。

<a id="tests"></a>

## 验证

测试依赖通过构造函数或参数注入，替身只覆盖外部交互边界，执行与评分使用生产实现。
数据夹具使用真实 dataclass/消息；行为替身实现生产接口，或使用
`create_autospec(..., spec_set=True)`。禁止通过改模块全局变量、类或 `sys.argv` 接管生产
执行，禁止 `SimpleNamespace` 和无规格 mock 拼接口；进程环境可用 monkeypatch 隔离恢复。

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
数学回归使用独立公式/解析值、中心差分和最终输出契约，避免生产实现自证：

- 规划：端点 PVA、C2 连续、避障/集群场景、轨迹时钟与软惩罚梯度；输出能力边界见架构文档。
- 坐标/姿态/回环：解析变换、残差与重置差分、相关信息计权、会话与时间拒绝。
- 前端/资产/IPC：已知位移与双目约束、几何一致性、深度投票与先验保护、双语言布局及事件往返。

指定的确定性任务必须成功；随机安全属性可明确有界拒绝，拒绝不计成功。
正确性门槛不使用机器相关耗时。解析、采样与性质回归只证明测试覆盖的契约。

### Basalt 与 RootSLAM

在线与数据集共用 `firefly-rootslam --config <path>`；默认读取 IPC，数据集示例为
`configs/firefly-rootslam-dataset.toml`。相对路径以进程工作目录为基准，示例从 workspace 根运行。
运行前准备配置指定的数据和标定。输入契约、LaMAria 下载及评分边界见
[RootSLAM README](../apps/rootslam/README.md)，
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
里程计失联锁存及 Navigation 跳变不影响局部地图与参考。

<a id="system-tests"></a>

### 无 mock 系统验收

本地深度融合的数值对照使用固定上游 C++ 函数，生成器校验 checkout 修订号。
`--eigen` 指向包含 `Eigen/` 的头文件目录；生成产物位于 `target/ego-map-reference`：

```bash
uv run python crates/firefly-ego-map/tests/reference/generate.py --upstream ~/Projects/EGO-Planner-v2 --eigen /path/to/eigen
FIREFLY_EGO_MAP_REFERENCE="$PWD/target/ego-map-reference/reference" cargo test -p firefly-ego-map --test cpp_reference -- --ignored
```

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
- IMU 用例分别检查飞行、持续悬停与 VIO 失联后的有限姿态支持。保持原目标，独立评分整段
  真值、姿态和主链路健康；静止评分使用固定等待与窗口，全程检查碰撞、重置和退出。
- 扰动用例逐轮恢复并持续悬停，评分通过才发下一轮；禁止重发 Hold 改目标或将未执行轮次计通过。
- 估计器为 `firefly-rootslam`，处理时延证据来自 RRD 的 `rootslam/frame_processing_ms`。
- 22m 往返使用 `configs/acceptance_22m.toml`，默认空局部地图与 RootSLAM VIO；
  初始化、真实到点、跟踪、定位、降落、碰撞和退出必须分别通过。在线回环不属于该任务链路。

任务配置和门槛由 `firefly_evaluation` 的 `mission.Options`、`hover_metrics.HoverLimits`、
`disturbance_metrics.DisturbanceLimits` 及 `configs/acceptance*.toml` 维护，运行前固定并存入报告。
新增门控须以固定配置独立启动三次、全部通过才建立基线，保留每次失败。

姿态融合、初始化、坐标、视觉前端或仿真传感器改动后，必须运行 `imu_hover`；
悬停不替代往返/回环。显式指定系统用例缺运行条件时报错；普通 CI 全仓收集中的 skip
不计飞行通过。`tests/evaluation/test_hover_metrics.py` 仅检验评分规则，不启动仿真。

### 任务验收与可选地图

CLI 使用已构建的 release 程序，运行前执行统一构建命令：

```bash
cargo xtask release
uv run firefly-evaluate
uv run firefly-evaluate --case nominal --config configs/acceptance_22m.toml
```

`--case nominal|reference_loss|estimator_loss` 选择任务。所有任务使用 RootSLAM VIO 与局部深度规划，
不加载场地先验或库图定位；飞行误差、碰撞和时间门槛由验收配置固定。

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

库图独立视角验收先启动 `./target/release/mimir --save logs/map_validation.rrd`，再执行：

```bash
RUST_LOG=info FIREFLY_QUERY_MAP=<独立查询.ffvmap> FIREFLY_QUERY_COUNT=20 \
  FIREFLY_MAP_VALIDATION_REPORT="$PWD/logs/map_validation.json" \
  cargo test --release -p lightglue held_out_map_localization_contract -- --ignored --nocapture
```

独立查询不能来自建库位置，查询位姿只作评分标签，先验加入固定偏差；每个指定查询
必须通过几何和误差断言，拒绝不能计通过。模型导出、资产质量、库图定位、飞行与在线回环
分别验收；自匹配、几何单测或库图观测次数不能替代真实重访的后端接受证据。
