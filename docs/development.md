# 开发指南

开发、运行、记录和验证统一在本文；系统职责、坐标、算法及能力边界见
[architecture.md](architecture.md)。路径和命令均相对仓库根目录。
跨模块约定只维护这两份文档；README 只提供介绍和入口。参数以代码默认值和配置为准，测试用例与门槛以测试源码为准。
历史排查和试飞结果留在 Git 历史及 `logs/` 证据中，不另建平行规范。

## 实现约定

- 新 crate：`cargo new crates/<name> --lib --edition 2024`（自动纳入 workspace）。
- DDD 拆 crate（firefly-*），依赖自上而下，领域层不依赖应用层。
- 错误用 firefly-error：kind/status 分类，操作名统一 `.with_operation(...)`，其他诊断字段
  用 `.with_context(...)`；不得以 `"operation"` 上下文键绕过结构化字段。
  `Permanent` 表示不可重试，`Temporary` 表示允许重试。
- 可观测性：关键函数 `#[fastrace::trace]`，日志用 log 宏，禁 println!。
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

- **TOML、一应用一份**：统一放仓库顶层 `configs/`（`sim.toml` / `okvis2-x.toml` / `planner.toml`），
  启动时加载（Rust 支持 `--config` 换文件）。配置文件可选：缺文件记录一行 INFO 并使用
  代码默认值；文件存在但不可读、格式或参数无效仍报错，不静默回落。
- **最小化**：保留必填键，只对有默认值的键写差异覆盖；无覆盖项时保留仅含用途和
  默认值来源注释的配置文件。缺失键使用代码默认值；
  纯数据 Options 直接 `serde::Deserialize` + `#[serde(default)]`，Python 用标准库
  `tomllib`；不引 YAML。

#### 应用启动入口

固定顺序：**解析命令行 → 初始化基础日志 → 读取、转换并校验配置 → 报告配置来源
→ 创建运行资源 → 运行应用**。命令行帮助与参数错误不创建运行资源。

- Rust 主配置通过 `firefly_config::startup(path, init_logging, validate)` 加载。
  普通进程传入 `firefly_observability::init`；校验函数返回 `firefly_error::Result<()>`。
  无额外参数约束时传入 `|_| Ok(())`。配置类型只描述字段、默认值和参数约束，
  不再包装文件读取、安装日志或调用 `process::exit`；启动错误交给 `main` 返回或处理。
- Python 主配置通过 `firefly_workspace.startup(path, decode)` 加载。
  `decode` 将字典转换为应用配置并校验，不创建运行资源；默认安装标准库 INFO 日志。
  宿主需要特殊日志格式时通过 `init_logging` 参数提供初始化函数。
- 公共入口成功返回后才创建 IPC、模型、工作线程、窗口等资源。辅助配置在此之后使用
  `firefly_config::load` / `firefly_workspace.load_toml`，不重复初始化进程日志。
  领域对象构造时仍保留自己的不变量校验。
- 配置来源由公共加载模块记录，调用方不再重复打印“已加载配置”。缺文件与空文件
  使用相同默认值；缺文件仅在转换、校验成功后提示回落。INFO 的显示尊重日志过滤设置，
  Rust 可用 `RUST_LOG=info` 查看。
- Bevy 先独立安装日志设施，在配置通过后安装其余插件。使用 `LogPlugin` 时，
  在随后的 `DefaultPlugins` 中禁用它，避免重复安装；自定义日志宿主沿用自己的桥接。

新应用以可编译的 [Rust 最小应用](../crates/firefly-config/examples/minimal.rs) 和
[Python 最小应用](../packages/firefly-workspace/examples/minimal.py) 为入口示例；
Bevy 日志接线参考 [quad](../apps/quad/src/main.rs)。示例调用公共实现，配置默认值
仍归各应用自己的类型。Rust 示例随 `cargo check --workspace --all-targets` 编译，
不维护第二份示例代码或独立应用名单。

标定与噪声改对应 Options 默认值，doc 注释写单位和来源；部署覆盖同步 `configs/`。
升级视觉依赖必须运行相关前端、投影及匹配回归，不放宽容差；依赖统一在根 Cargo/uv workspace 和锁文件。
改行为前阅读对应官方源码，固定版本与差异见架构文档。参考仓库位于 `~/Projects/`：
`open_vins`、`ORB_SLAM3`、`VINS-Fusion`、`EGO-Planner-v2`、`iceoryx2`、`logforth`、`fastrace`；
purecv 的来源与固定版本以 workspace 依赖及 Cargo.lock 为准，发现上游问题应提 issue。

## 提交与 PR

commit message 沿用 `<type>(<scope>): <摘要>`。**PR 标题用 `<type>: <中文摘要>`，不写括号**
（中英文括号都不用）：括号里的补充说明改成逗号分隔的短句，保证在 PR 列表和合并提交里
一行可读。

- `feat: 切换到 PX4 控制核心，补起降平滑、安全标志与 VIO 验收`
- `fix: 修正悬停参考的速度前馈`
- `perf: 共用约束点采样，单次规划 2.08→0.98ms`
- `docs: 补充 PR 标题约定`
- `refactor: 删除旧角度模式与 mixer，quad 切到 PX4 链`

反例：

- `feat(fc): 切换到 PX4 控制核心（起降平滑）`：PR 标题不写 scope 括号
- `feat: 切换到 PX4 控制核心 (VIO 验收)`：不写英文括号
- `feat: 切换到 PX4 控制核心（WIP，后续补安全标志）`：不写阶段或计划叙事

PR 标题描述整个 PR 的最终结果，不写 WIP、阶段编号或“后续再做”；同一条规则只维护在本节。

<a id="run"></a>

## 构建与运行

场景由 `configs/scene.toml` 的 `scene = "<赛季>@<版本>"` 唯一选择，已注册
`rmuc2026@2.0.0` / `rmuc2027@1.0.0`；资产在 `models/<赛季>/v<版本>/`（`field.glb`、
`<赛季>_collision.json`），另需 `models/drone/drone.glb`。未注册版本或缺失资产报错，不回退场景。
停机坪定义来自 `firefly_mujoco.scene.SCENES[].pad` 与 `PAD_CLEARANCE`，不在文档复制坐标。
render 使用无窗口 GPU 渲染；Rerun 原生 Viewer 需要图形会话。Rust/Python 版本和依赖
以 workspace 配置为准。视频记录依赖外部 FFmpeg、所选编码器和 GPU 用户态驱动；
编码器按平台选择（macOS 用系统 VideoToolbox，其余用 Intel VA-API），设备与编码选项见 `configs/viz.toml`。仅列出编码器并不证明驱动可用。

```bash
uv sync --all-packages --extra test
cargo xtask release
```

运行进程一律 release；普通 `cargo test` 用 debug。源码改动后先完成构建，再做运行验收。
分别在终端启动，记录进程最先启动、最后退出：

```bash
uv run firefly-viz --save logs/rmuc_run.rrd
uv run firefly-sim
cargo run --release -p render
cargo run --release -p firefly-okvis-x
cargo run --release -p fc
cargo run --release -p planner
```

最小闭环只需 viz/sim/render/firefly-okvis-x/fc；规划增加 planner。
纯 VIO 对照用 `firefly-okvis-x --no-loop-closures --no-mapping`。
可选库图定位先填写 `configs/localization.toml [origin]`，并用
`firefly-okvis-x --external-localization`，再启动 localization、aliked 和
`lightglue --map <地图.ffvmap>`。默认进程清单不自动启动这些可选服务。
静态规划图用 `planner --map <地图.ffmap>`，同样要求明确外部对齐。
禁止把场地真值作为默认定位或规划先验。

### 机载部署

板载计算是 Orange Pi Zero 3W（aarch64 Debian 13，glibc 2.41）。`--target board` 在
Apple container 的 `rust:1.98.1-trixie` 里原生构建，镜像按 digest 固定，产物在
`target/container/aarch64/release`；不维护交叉 sysroot。首次构建前运行
`container system start`，需要拉取镜像并编译依赖。构建镜像是在基础镜像上
加装 `libclang-dev` 的派生镜像（`v4l2-sys-mit` 用 bindgen 生成 V4L2 绑定），
`xtask` 首次运行会 `container build` 并复用缓存。

产物固定放 `/opt/firefly/{bin,configs,models}`，进程必须从 `/opt/firefly` 启动，
默认的相对配置和模型路径才成立：

```bash
cargo xtask release --target board
cargo xtask deploy
adb -s firefly-0001 shell 'systemctl start firefly'
```

`release --target board` 只构建机载应用（不含 render/quad），并在容器内构建设备端
`xtask` 解包器；`deploy` 把二进制、`configs/*.toml` 与默认视觉模型打成
`dist/firefly-aarch64.zst`（zstd 容器，不是 tar），推到设备后由同一 xtask 的 `unpack`
解包，`--device` 默认 `firefly-0001`。解包前会停掉 `firefly.service` 并清空
`/opt/firefly/{bin,configs,models}`，退役的进程与配置不会残留在板上；`logs` 与镜像
预置的 `systemd` 保留。因此部署后要重新 `systemctl start firefly`。可复现性由固定镜像 digest、Rust 1.98.1、`--locked`
和 `Cargo.lock` 保证；升级 Rust 或 Debian 时只需更新 `xtask/src/container.rs` 的
基础镜像 digest，派生构建镜像的标签由工具链与 digest 前缀拼出，会自动重建。
干净构建先删除 `target/container/`，再执行上面的命令。

板侧传感器与执行器由三个独占进程接替仿真侧的 `render` 与 `firefly-sim`：

| 进程 | 设备 | 话题 |
|---|---|---|
| `camera` | `V4L2` 节点（MIPI-CSI） | 发布 `Firefly/CameraLeft`、`CameraRight`、可选 `Depth` |
| `imu` | `spidev`（`BMI088` 两片） | 发布 `Firefly/Imu` |
| `motor` | `sysfs` PWM | 订阅 `Firefly/Control`，发布 `Firefly/Airframe` |

三者都只在板侧构建，宿主机上不参与构建矩阵。上板前按配置文件核对：

- `configs/camera.toml`：`/dev/videoN` 编号随内核与接线变化，先用
  `v4l2-ctl --list-devices` 核对。分辨率由 IPC 定长消息定死（320×240），
  驱动不接受该组合时进程直接报错，不静默换成别的分辨率。删掉 `[depth]`
  就不发布深度。**USB `D430` 不是 `V4L2` 设备**（需 librealsense），
  当前实现只覆盖走内核 `V4L2` 的相机。
- `configs/imu.toml`：目前只实现 `BMI088`，`WHO_AM_I` 不匹配直接退出，
  不会在错型号上继续跑。寄存器初始化值取自数据手册，首次上板须核对
  `Firefly/Imu` 的读数是否落在量程内的合理重力/静止零偏。
- `configs/motor.toml`：`sysfs` PWM 只适用于 PWM 电调。`[airframe]`
  （质量/惯量/旋翼几何/推力上限）在这里维护一份，真机装配后必须换成实测值，
  否则飞控会拿仿真机体的参数去分配推力。`Control` 断流时四路回到零推力脉宽。
  当前内核的 `pwm-sunxi` 拒绝使能**没有设备树描述**的通道（dmesg 报
  `can't parse pwm device`），所以上板前必须先补 PWM 通道的 DT 节点，否则
  `sysfs` 只能写周期和占空比、`enable` 永远失败。

三个驱动默认 `autostart = false`：板上还没有相机与 IMU，SPI 控制器也需要先在设备树
启用才有 `/dev/spidev*`。硬件到位后逐个 `ffctl process enable camera`、`imu`、`motor`
打开并单独验证，不要一次性全开——失败原因会互相掩盖。`processes.toml` 是烧录语义，
`deploy` 会覆盖它，所以在设备上手动改的结果不持久。

`camera` 与 `imu` 的时间戳都用 `CLOCK_MONOTONIC` 秒，与仿真侧 `sim_time`
语义不同，同一份录制里不要混用。三个进程都可以单独起停：`bin/camera`、
`bin/imu`、`bin/motor` 接 `--config <path>`，`Ctrl-C` 正常退出。

`firefly.service` 由镜像构建 [`orangepi-build`](https://github.com/3SE-Competitive-Robotics-Team/orangepi-build)
预置并默认启动；`deploy` 只发布二进制、配置和模型。systemd 按 `configs/processes.toml`
的数组顺序拉起 `autostart = true` 的进程。`processes.toml` 被 `deploy` 覆盖（烧录语义）；
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

supervisor 对异常退出按 1s→30s 退避重启；停止等待 5s，超时只报错、不 SIGKILL；
进程日志在 `logs/process/<name>.log`。板载的 MIPI 相机、IMU 和飞控串口数据入口尚未接入
生产二进制，当前部署只验证启动、链接、配置加载和 IPC 节点创建，不能当作飞行验收。

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

脚本、CAD 与验收共用 `firefly-workspace` 的工作区定位和流式 SHA-256；该包只依赖标准库。

```bash
uv run --package firefly-cad firefly-cad-build <场地.stp>
```

`firefly-cad-build` 构建并验收几何资产，配置为 `configs/cad.toml`。GLB、碰撞盒、FFMap 必须来自
同一归一化几何，通过内容 hash、坐标和占据检查；细节见 `packages/firefly-cad/README.md`。

`scripts/collect_vision_map.py` 使用 `render --offline` 和隔离的 `Firefly/Offline/*`，
按同一时间戳配对左图、理想深度与位姿标签，不启动在线估计器/飞控，不把标签发布到在线话题。
无同步帧不得建库；图像纹理、路标覆盖与内容指纹由采集和构建器检查。
库图兼容性只依赖场地、标定、光照和曝光；源码/二进制 hash 用于追溯，普通重编译或在线
噪声变化不要求重建。不能改历史 manifest 冒充原采集；替换资产前归档。

官方预训练模型的可复现导出（CPU，独立环境，不影响运行环境）：

```bash
uv venv models/vision-export/.venv --python 3.12
uv pip install --python models/vision-export/.venv/bin/python -e packages/firefly-workspace
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
内容指纹和汇总。Rust 计算线程经 `Firefly/VizScalars` 发标量，经 `Firefly/Viz` 发空间诊断，
planner 经 `Firefly/VizVoxels` 发地图更新，经 `Firefly/Log` 发 log 宏诊断，
由 firefly-viz 统一落盘；stderr 仅作进程诊断的双 sink，不作第二份录制。

各进程以测量时刻设置 `sim_time`；无仿真时刻的进程日志回落 `wall_time`。
常用实体为 `vio/odom`、`vio/traj`、`gt/pose`、`gt/traj`、`vio/debug/*`、`fc/debug/*`、
`plan/*`、`logs/<app>`。左目 H.265 视频、可选右目/深度采样及触发无损窗口进入 RRD；
Rerun 按原图时间配对叠加 VIO 点，不把标记烧进视频。`/live/**` 是独立 gRPC stream
发送的 static 最新预览，不写入 RRD，也不代表时间光标处的历史数据。

`configs/viz.toml` 控制视频、预览频率、图像采样、触发窗口与容量；未知配置键拒绝启动。
SIGUSR1 请求保存前后无损帧窗口，ERROR 日志可自动触发，窗口与冷却时间遵循该配置。
所有输入先复制到自有内存再释放 IPC loan，队列与环形缓存有界。体素分块完整后才更新地图；
标量不携带体素数组。队列丢帧、编码耗时、输出 sample 大小与退出状态在 `viz/**` 查看。

传感器延迟按原始采集 `sim_time` 记录墙钟耗时（秒）：
`render/capture/stages_s` 为请求→提交、提交→回读、回读→装配、装配→处理；
`render/capture/publish_s` 为像素处理、处理结束→发布、请求→发布、帧序号。
这些诊断不改图像时间戳；`fc/imu/health` 的后验年龄还包含视觉更新与等待下一拍的时间。

实时查看先启动 `rerun`，再运行 `uv run firefly-viz`：默认连接 `127.0.0.1:9876` 并创建
`logs/` 下唯一 RRD。`--save logs/run.rrd` 单独使用为离线录制；加 `--serve` 同时启动原生
Viewer，或加 `--connect rerun+http://127.0.0.1:9876/proxy` 连接已有 Viewer。仅实时查看须
显式加 `--no-save`。既有 RRD 路径拒绝覆盖，recording ID 每次运行独立。
离线回放 `rerun logs/rmuc_run.rrd`；数值可用 `rerun rrd print`、`rerun rrd stats` 或
Python RrdReader，读取方法见仓库 `.agents/skills/rerun/SKILL.md`。

H.265 走当前 workspace Rerun SDK 的 `VideoStream` API，原生 Viewer 解码也需要 FFmpeg；
浏览器端的 HEVC 能力不能替代原生回放验收。macOS 的 VideoToolbox 允许使用系统软件实现；
编码器初始化失败会结束录制并保留错误证据。IPC 布局升级后须同批重建、重启所有生产者
与消费者，不能混用不同版本的消息类型。

文件录制固定启用完整 footer/manifest，正常关闭后供 Viewer 与随机访问工具直接打开；
不提供关闭索引的配置项。索引内存随 chunk 数增长，独立于 Viewer、图像缓存和编码队列预算。
异常中断或历史文件缺少 footer 时，可用标准 `rerun rrd optimize` 生成带索引副本。

性能只认相同构建、硬件、输入与环境下的 trace 实测。主循环建 root span、设置 local parent、
结束 flush；热路径不使用无条件格式化大对象的 `#[logcall]`。ConsoleReporter span 树
经记录进程进入 RRD；先直接读输出，不为简单查看另造解析脚本。推理微基准不代表全栈吞吐或功耗。

### 姿态与 VIO 诊断

- `fc/imu/quaternion_xyzw`：控制实际使用的 Hamilton 姿态（主链路为 body→odom，备用为 body→本地重力系）（10Hz）。
- `fc/debug/position_target_odom` / `position_error_odom`：odom 系目标与位置误差（10Hz）。
- `fc/imu/health`：所选姿态可用、VIO 主链路新鲜、VIO 后验测量年龄（秒）。
- `fc/imu/source`：0 未就绪、1 VIO 预测、2 独立 ESKF 热备（10Hz）。
- `fc/standby/health`：独立 ESKF 可用标记、IMU 墙钟年龄（秒）。
- `fc/standby/accel_gate`：NIS、是否实际应用地面加计校正。
- `fc/standby/gyro_bias`、`quaternion_xyzw`、`attitude_variance`、`bias_variance`：
  独立热备状态；姿态属于备用本地重力系，方差单位 rad² / (rad/s)²。
- `fc/vio/gyro_bias`：主链路采用的 VIO 陀螺零偏；主链路不发布姿态方差。
- `fc/debug/tilt_deg` 与 `fc/debug/tilt_err_deg`：估计倾角与仅供评测的真值倾角误差。
- `fc/debug/rate_sp_frd` / `rate_flu`：PX4 角速度设定值（FRD）与实测角速度（FLU）。
- `fc/debug/torque_sp_frd` / `allocated_control`：归一化力矩设定值与分配实际达到的控制量。
- `fc/debug/saturated` / `hover_thrust`：分配饱和标志与归一化悬停推力。
- `fc/arming/failed`：未解锁时首个失败解锁检查的位掩码（定义见 `firefly-flight::ArmingCheck`）。
  VIO 已初始化后位 1 仍可能置位，表示独立 ESKF 热备尚未完成静止初始化；等待清零后重试
  `ffctl fc arm`。
- `fc/failsafe/flags` / `fc/failsafe/action`：失效条件位掩码与动作编码（定义见
  `firefly-flight::safety`）；1Hz 日志同时输出名字。

运行记录由 `firefly-viz` 写入 `logs/*.rrd`。

#### 估计与回环诊断

OKVIS 的位姿使用 `vio/odom` 实体。
`okvis/loop_commits` 是后端已提交全图修订次数，任务评分记录为 `acceptance/loop_backend`；
候选检索和几何验证不单独计为提交。外部库图模式的融合诊断位于 `corr/debug/*`。
通讯测试和修订计数不替代完整轨迹精度验证。

<a id="tests"></a>

## 验证

测试依赖通过构造函数或函数参数显式注入（环境工厂、时钟、证据读取器、进程/录制接口），
不得通过 importlib 后修改生产模块全局变量、替换类或改写 `sys.argv` 接管运行。
数据夹具使用真实 dataclass/消息类型；行为替身使用实现生产接口的测试类，或
`create_autospec(..., spec_set=True)` 检查成员和调用签名。禁止 `SimpleNamespace` 和无规格
mock 拼接接口。测试子类只覆盖外部交互边界，执行和评分逻辑仍使用生产实现；环境变量、
工作目录等进程环境可用 pytest 的 monkeypatch 夹具隔离并恢复。

运行 `pytest tests/system` 时，session fixture 会在系统用例启动前执行
`cargo xtask release`，重建所有 Rust 应用的 release 二进制，避免端到端测试使用旧产物。
该命令是全部 Rust 应用 release 构建的统一入口，应用列表只在 xtask 维护；
并行度使用 Cargo 默认值或 `CARGO_BUILD_JOBS`。Python 依赖通过 `uv sync` 安装。

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
FIREFLY_RUN_VIDEO_ENCODER=1 uv run --all-packages --extra test pytest apps/firefly-viz/tests/test_video_integration.py -q
```

视频集成用例使用当前 `configs/viz.toml` 指定的真实编码器，检查帧数、采集时间关联和
FFmpeg 解码后的画面；默认跳过不代表通过。Rerun 回放还需检查视频跳转、逐帧点集匹配、
static 预览不进入文件和正常/异常退出时的完整性。构建、单测与实际 Viewer 回放分别记录结果。

数学测试同时包含独立公式/解析值、中心差分和最终输出契约，不能只让代价与梯度互证。
确定性任务必须成功；随机安全属性允许明确有界拒绝，但拒绝不计成功，不用整体成功率
掩盖指定任务失败，不用机器相关耗时作正确性门槛。参考版本见架构文档。

- 规划检查端点 PVA、导数极值、地图/对端净距、时钟与输出轨迹一致性；测试侧独立极值
  求根和几何采样，不用生产实现验证自身。A* 连边与 Bernstein 检查不等于动态安全证明。
- 坐标、姿态、回环检查解析变换、残差/重置差分、相关信息不重复计权、会话与时间拒绝。
- 前端检查已知位移、丢点与双目约束；解析轨迹隔离滤波数学，不能代替实际纹理和飞行。
- CAD、深度、IPC 分别验证资产一致性、噪声/投票/先验保护和双语言布局、头部、事件往返。

### OKVIS 算法与通讯专项

```bash
cargo test --all-targets -p firefly-okvis-x -p firefly-brisk -p firefly-dbow2 \
  -p firefly-opengv -p firefly-srl-projection -p firefly-supereight2
cargo test -p firefly-okvis-x application::live::tests
```

算法测试覆盖独立解析值、几何退化、差分与数值边界；通讯测试覆盖配对、测量覆盖、
丢失与过期状态、回环坐标修订、协方差与真实 iceoryx2 消息往返。
夹具回归、Rust 自身基线和独立算法性质测试分别说明来源，不将自身基线当作跨语言等价证明。
无输入启动与退出、编译和单测不等于实际数据集或飞行验收。

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
  独立评分整段真值、姿态与主链路健康；上锁后固定等待再评分固定静止窗口，不能按真值自适应
  推迟窗口。全过程碰撞、重置和退出检查始终有效。
- 十轮扰动每次恢复后连续悬停，逐轮真值评分通过才发下一轮，不重发 Hold 改目标。
  失败后的未执行轮次不能记通过。
- 22m 往返使用 `configs/acceptance_22m.toml`，默认空地图、在线回环与深度建图；初始化、
  真实到点、跟踪、定位、回环、降落、碰撞和退出必须分别通过。

门槛唯一来源为 `firefly_evaluation.mission.Options`、`firefly_evaluation.hover_metrics.HoverLimits`、
`firefly_evaluation.disturbance_metrics.DisturbanceLimits` 及 `configs/acceptance*.toml`；
运行前固定并随报告保存，不能按失败结果放宽。新增门控须用当前固定配置独立启动三次，
三次全通过才建立基线，不以重试通过覆盖失败。

姿态融合、初始化、坐标、视觉前端或仿真传感器改动后，必须运行 `imu_hover`；
悬停不替代往返/回环。显式指定系统用例缺运行条件时报错；普通 CI 全仓收集中的 skip
不计飞行通过。`tests/evaluation/test_hover_metrics.py` 仅检验评分规则，不启动仿真。

### 任务验收与可选地图

CLI 使用已构建的 release 程序，运行前执行统一构建命令：

```bash
cargo xtask release
uv run firefly-evaluate
uv run firefly-evaluate --case nominal --config configs/acceptance_22m.toml --vio-only
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
