# 开发指南

以下构建、仿真和部署命令都在 Mac 的仓库根目录执行。标有 Orange Pi 的命令在板上执行。

## 构建

```bash
git submodule update --init
uv sync --locked --all-packages --extra test
cargo xtask release
```

Orange Pi 程序用 Apple `container` 构建。只需生成板端二进制时运行以下命令。部署命令也会执行这一步。

```bash
container system start
cargo xtask release --target board
```

Matrix BMI270 固件使用单独的 `linux/amd64` 镜像。容器构建说明给出了镜像、可刷写 HEX 的位置和协议测试入口。[Matrix 固件构建](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/apps/third-party/README.md#编译-matrix-bmi270-固件)

## 运行

### Mac 软件在环（SIL）

准备 GPU 图形会话、[场地资产](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/packages/firefly-cad/README.md)和任务所需模型。从一次悬停任务开始。

```bash
cargo xtask sil configs/mission/hover.toml
```

可在同一次调用中依次验收多个计划，例如 `cargo xtask sil configs/mission/square.toml configs/mission/cuboid.toml`。各架次共享一个 VIO 坐标系。每次落稳上锁后，xtask 再提交下一项任务。若已有飞行进程占用 IPC，用 `cargo xtask processes .` 查看 PID。

`configs/mission/` 存放任务计划。棒棒糖形水平圈使用 `circle.toml`，绕机体前后轴的竖圈使用 `vertical-circle.toml`，平面和三维李萨如分别使用 `lissajous.toml` 和 `lissajous-3d.toml`。三维李萨如的最高航点是 3.2 m，复现该 HIL 任务时，板端规划器使用试飞时的 3.7 m 高度上界。

### USB 硬件在环（HIL）

Mac 运行 MuJoCo、render、模拟飞控和 Mimir。Orange Pi 运行任务和在线算法。Mac 与 Orange Pi 之间的 USB 连接传输仿真测量、杆量和遥测。[部署分工](architecture.md#部署与验证边界)

```bash
cargo xtask deploy-hil
cargo xtask hil configs/mission/square.toml
```

#### 回放 H 坪起飞观测

选取已封口归档和采集时钟上的起止秒数。窗口可以跨多趟任务。导出器复制下视原图，配对各帧之前的传感器样本，并提取截至窗口结束的任务 home、阶段事件。回放按开机、任务 ID、估计会话归属观测。一次旧版帧列表仍可作为单架次实验输入。

```bash
uv run python scripts/export_takeoff_replay.py logs/runs/log_0161 1.3 7 logs/algorithm-perf/pad-launch-1
cargo run --release -p landing -- --config configs/landing.toml --takeoff-replay logs/algorithm-perf/pad-launch-1/frames.json
```

同一个目录可复制到 Orange Pi，用板端 `landing --config configs/landing.toml --takeoff-replay <目录>/frames.json` 回放。每行 JSON 输出一帧的时间、H 坪坐标系下的机体位置（米）、状态和首次接管候选的坐标变换。坐标原点是停机坪中心处的触地机体原点，竖直轴向上，水平朝向取自独立 IMU。H 高度使用已标定的 H 尺寸、相机内外参和独立姿态。归档中的该姿态约 1 Hz，适于评估平缓直升。

任务进入 TAKEOFF 后，须收到本阶段起点之后、年龄至多 0.25 秒的飞控 TAKEOFF 回报；此后 TAKEOFF 和 HOLD 均可维持窗口。连续三帧确认 H。H 高度进入 0.9～1.1 米、VIO 后验年龄在 150 ms 内、至少 8 个双目匹配和 6 个地标、估计速度至多 0.25 m/s 时开始比较。窗口开始时固定 VIO 到局部水平系的变换和 H 在 VIO 系中的位姿。连续 0.5 秒内，位置差须在 6 cm 内，完整三维旋转差须在 5° 内，按 H 的半周对称性比较。纯视觉法向与独立重力夹角超过 25° 时暂停交接。JSON 分别给出原始视觉和重力辅助的机体 ← H 变换；其中平移是机体系米数，旋转是 xyzw 四元数。当前飞行控制仍执行普通起飞。

任务感知回放从持久回执中的 TAKEOFF 阶段事件计时。1.5 秒内未确认 H、跟踪后超过 0.25 秒没有连续有效观测，或 6 秒内未形成接管候选，进入相应的中止状态。导航、精降、恢复和取消关闭本次交接窗口，新的任务重新确认。回放按独立截止时刻输出一次 kind=timeout，即使窗口结束前所有图像均已断流。在线飞控接入时由单调时钟驱动同一判据。

#### USB 看门狗

板端部署会安装并启用 `firefly-usb-watchdog.service`。开机时，原厂 `adbd.service` 准备 ADB 和 FunctionFS。随后 systemd 停止原厂的 UDC 轮询脚本，在 CPU 0 上启动一个 `hil-gadget watchdog`。用 `systemctl status firefly-usb-watchdog` 查看服务，用 `journalctl -b -t firefly-hil-gadget` 查看切换和重绑记录。

看门狗每 5 秒尝试取得 USB 切换锁。锁忙时等下一轮。持锁时按以下规则操作。

| 板端状态 | 看门狗动作 |
|---|---|
| HIL 已配置 | 进入 HIL 或看门狗重新启动后有 60 秒宽限。观察到新的主机心跳后重新计时。连续 60 秒没有更新，恢复实机进程清单和 ADB。 |
| 本次开机的显式切换中断 | 取得锁后观察 60 秒，仍有未完成标记就执行同一恢复流程。 |
| 唯一配置仅链接 `ffs.adb`，`ep1` 和 `ep2` 就绪，`UDC` 明确为空，唯一控制器报告 `not attached` 或 `configured` | 将控制器名称写入 `UDC`，回读确认。A733 在解绑后可能保留上次的 `configured` 链路状态。飞行进程和 `adbd` 持续运行。 |
| ADB 仍绑定、USB 挂起、配置混合、读取失败或端点未就绪 | 保持现状。ADB 可以长期空闲。 |

SIL 和 HIL 都在录制结束后逐架次评分。要重新评分，向 `target/release/firefly-verify --transport sil` 或 `--transport hil` 传入对应的运行目录。评分器将 JSON 写到标准输出，通过时返回 0。[运行归档与回放](mimir.md#录制与读取)

### 板上任务服务

用 `adb devices` 核对 Orange Pi 的设备号。在仓库的 `models/` 下备齐精降的 `h-marker.nb`、`viplite.json`、单目深度模型 `yolo26n-depth-320-int8.nb` 和 HIL 双目深度模型 `zipdepth/relative-320-int8.nb`。部署会停止服务，更新 `/opt/firefly` 的程序、配置和模型，再启动服务。[打包文件清单](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/xtask/src/deploy.rs) · [板端模型准备](#板端模型)

```bash
container system start
cargo xtask deploy --device firefly-0001
adb -s firefly-0001 shell
```

在 Orange Pi 上运行 `ffctl process status` 检查 `pilot` 和 `firefly-rootslam`，用 `ffctl mission status` 查询任务服务。实机尚缺任务所需的机体参数和飞控解锁状态反馈。HIL 提供这些反馈，可以用 `ffctl mission submit inspection-001 configs/mission/square.toml` 提交计划，再用 `ffctl mission status inspection-001` 查询回执。相同 ID 和计划可重试，取消用 `ffctl mission cancel inspection-001`。[任务阶段和回执](architecture.md#系统运行架次和任务)

进程启用项、命令和 CPU 亲和性以[实机清单](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/configs/processes.toml)和 [HIL 清单](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/configs/processes-hil.toml)为准。装好相机后用 `ffctl process enable camera` 启用。板端需要 RRD 时手动启动 `firefly-mimir.service`，HIL 由 Mac 录制。

每份计划最多 16 个航点，起飞和返航高度为 0.5–2 米。航点水平半径最多 10 米，首点距起飞点的水平距离至少 1 米。每段三维距离为 0.5–10 米，航点高度为 0.5–3.3 米。停留时间大于 0 秒且不超过 600 秒，返航平面偏移不超过 0.3 米。[计划校验](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/crates/firefly-mission/src/lib.rs)

### 验证 Matrix IMU

这项检查只需 Matrix 和 Orange Pi。拆桨并让飞控保持上锁，Matrix 的 USB 数据口接 Orange Pi 的 Host 口，Mac 通过另一根 USB 线连 Orange Pi。在板上执行：

1. 运行 `ls -l /dev/serial/by-id/`，将实际设备路径与 `/opt/firefly/configs/pilot.toml` 的 `device` 对照。
2. 在空闲台架上用 `ffctl process stop pilot` 停止采集。用 `ffctl process restart firefly-rootslam` 开始一轮独立计数，再用 `ffctl process start pilot` 建立会话。
3. 查看本次开机运行目录的 `text/pilot.*.log` 和 `text/firefly-rootslam.*.log`。等本轮出现 `Matrix USB session ready` 和 `gravity initialized`，让板子静置至少 120 秒。
4. 停止 pilot，再重启 RootSLAM。查看刚退出实例的 `IMU accepted=` 汇总和 pilot 的 `stick frames sent`。核对整个窗口只有一次会话，按样本数和实际采集时长估算频率。纯接收测试应有 0 帧杆量。用 `ffctl process start pilot` 恢复采集。[运行目录](mimir.md) · [本批次台架实测](hw/spec.md#飞控电调板)

要用 Mac 上的 Betaflight Configurator，先停止 pilot 并等一秒多，再把 Matrix 的 USB 数据线接到 Mac。配置完成后接回 Orange Pi 并启动 pilot。[USB 会话状态图](architecture.md#matrix-usb-会话)

## 板端模型

### A733 精降模型

`scripts/export_landing_viplite.py` 从现有 FP32 ONNX 拆出检测框和置信度两个输出，让两者分别量化。使用 `se3_yolo` 的导出环境和训练时的标定集准备工作目录，然后在全志的 x86 Linux ACUITY 镜像 `ubuntu-npu:v2.0.10.2` 内编译。下例中的 `/work` 是该工作目录在容器内的挂载位置。

```bash
python scripts/export_landing_viplite.py --dataset ../se3_yolo/datasets/h_marker --output target/npu-runtime/h-marker
cp scripts/compile_landing_viplite.py target/npu-runtime/h-marker/
# 在配有 ACUITY_PATH 和 VIV_SDK 的容器 shell 中运行
cd /work
python3 compile_landing_viplite.py
```

标定集的 `manifest.json` 用 `frames[].image` 指定图片路径，两个导出脚本都使用 `split` 为 `train` 的帧。VIPLite 准备阶段读取模型目录中的 `fp32.onnx` 和 `manifest.json`。

将工作目录 `bundle/` 下的 `h-marker.nb` 与 `viplite.json` 一起放入 `models/h-marker-yolo26n-cif-v1/`。板端 `landing --image` 可用 352×288 图像验证 VIPLite 推理。桌面 `landing --simulation --image` 使用同一检测器的 ONNX 版本进行对照。

### 单张 RGB 图像的深度验证

`scripts/depth_image.py` 使用 ONNX Runtime CPU 后端读取一张图片，按原图高、宽写出以米为单位的 `float32` 数组。模型目录是 `models/yolo26n-depth/`。输入是 Ultralytics 发布的 nano 权重，SHA-256 为 `befed1b8561d8b2eaa66274b070cdfc9c44853bda6d6d62a04759f9383af74e9`。[官方权重](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n-depth.pt) · [模型和距离标定说明](https://docs.ultralytics.com/tasks/depth/)

用 Ultralytics 8.4.216 从权重导出固定尺寸的 FP32 ONNX。脚本接受该 ONNX，也接受由 ONNX Runtime 1.31.0 的 CPU 后端以 `ORT_ENABLE_EXTENDED` 保存的优化图 `yolo26n-depth.ort`。

```bash
yolo export task=depth model=models/yolo26n-depth/yolo26n-depth.pt format=onnx imgsz=768 opset=17 simplify=False dynamic=False device=cpu
python scripts/depth_image.py models/yolo26n-depth/yolo26n-depth.ort input.jpg depth.npy
```

图片推理需要 Python、NumPy、OpenCV 和 ONNX Runtime。脚本读取模型输入的固定高宽，支持仓库里的 320 和 768 两种导出，再按官方深度预测器的双线性插值方式将输出恢复到原图尺寸。用同图对照 PyTorch 输出可验证导出和预后处理。现场距离精度需用相机拍摄的画面和对应距离真值评估。

### HIL 的 ZipDepth 双目深度

将 A733 VIPLite 模型放入 `models/zipdepth/relative-320-int8.nb`。当前部署产物 SHA-256 为 `27258bda43f37516e97037fc5241d7264fe2b987ef8de6863fb0b2010d09f190`。源权重为 [ZipDepth 的 zipdepth_base_npu.pth](https://github.com/fabiotosi92/ZipDepth/tree/91f3fd21e131641f51e8d35736d1958350180e3a/checkpoints)。NBG 输入为 320×240 左右目依次存放的两个原始 UINT8 平面，图中取左目预测，输出一个同尺寸 FP32 相对视差平面。板端运行时使用 Debian trixie 的 OpenCV core、imgproc 和 video 库。

HIL 清单使用 `depth --zipdepth`。左右目的 sensor ID 和采集时刻必须匹配已校正的 HIL 相机组。VIO 接收 30 Hz 双目图像，深度进程按采集时间从中选取 15 Hz 同步帧对，再提交 NPU 推理。NPU 执行时，CPU 3 从双向光流和水平极线匹配中提取锚点。全图拟合后沿左图边缘平滑局部视差残差，按 HIL 相机的焦距和基线换算成轴向米数。有效锚点不足时发布零深度，仍使用该帧采集时钟。运行 `cargo x deploy-hil`，再运行 `cargo x hil configs/mission/square.toml`。

### 单目飞行链路的 320 深度模型

`depth` 订阅最新一张 320×240 左目灰度图，全视场缩放至 320×320，按 RGB 三通道输入。SIL 的 `depth --simulation` 加载 `models/yolo26n-depth/yolo26n-depth-320.onnx`。按上节从同一权重导出，使用 `imgsz=320`。板端加载同目录的 `yolo26n-depth-320-int8.nb`，输入为有符号 INT8，输出为 80×80 FP16 米制深度。部署包随程序安装该 NBG。两个后端按端点对齐插值恢复为 320×240，并保留原图的采集时间戳。仅离线数据集导出使用渲染几何标签。

在同一工作目录放置 320 ONNX 和 `scripts/compile_depth_viplite.py`。把选定的 JPEG 或 PNG 标定图片打成 ZIP，用导出环境运行 `python compile_depth_viplite.py --prepare /path/to/calibration.zip`，再在上述 ACUITY 镜像中运行 `python3 compile_depth_viplite.py`。量化迭代次数取自生成的图片清单。将 ONNX 和生成的 NBG 安装到模型目录。本次板测 NBG 的 SHA-256 为 `4dbadb624f071d7a15f5df059a5a39df0bfcf4fe84f1ec2e4ddfe20367677a08`。

## 测试

```bash
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
uv run --all-packages --extra test pytest -q -p no:cacheprovider
cargo test -p firefly-pubsub --test python_interop -- --ignored
```

常规 pytest 收集单元测试。已有桌面 release 后，可运行 `uv run pytest tests/system/test_matrix_pilot.py -q`，用伪终端检查 USB 握手、心跳、有效杆量和过期杆量。系统验收按 `tests/system` 下的文件或用例名显式运行。
有场地资产和图形会话时，先退出其他仿真进程。以下两项分别验证传感器启动和仿真起飞、悬停、降落：

```bash
uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py::test_sensor_startup -q
uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py::test_takeoff_hover_and_land -q
```

完整精降验收需要 GPU 和量化模型，包含任务往返和推理资源回收两组用例。

```bash
uv run --all-packages --extra test pytest tests/system/test_flight_mission.py tests/system/test_landing_lifecycle.py -q
```

以下 Rust 专项回归从仓库根下的固定位置读取外部数据。准备好对应输入后，用 `cargo test` 选择该用例并传 `-- --ignored`。

| 用例 | 输入及来源 |
|---|---|
| `frozen_original_tracks_match_setup_and_optimizer_stages` | `datasets/basalt-reference/mapper/` 内放同一批次的边缘化缓存 `cache/`、Basalt 的 `tumvi_512_config.json` 和 `tumvi_512_ds_calib.json`，以及用这三项输入运行上游 Mapper 对照程序 `--freeze` 得到的 `frozen.json`。 |
| `fixed_input_patch_intermediates` | `datasets/basalt-reference/optical_flow_patch_trace.json`，由 C++ 光流探针读取 TUM VI room1 双目数据和对应配置生成。Mapper 与光流探针源码见本仓库提交 `7ffdb4d` 的 `crates/firefly-basalt/tests/reference/{mapper,optical_flow_patch_trace}.cpp`。 |
| `frozen_single_precision_system_meets_unchanged_gradient_criterion` | `datasets/rootslam-reference/linear_system.json`，使用单精度 LSQR 求解失败时由 `linear_system_archive` 导出的原始矩阵和残差。 |

## 预览和构建文档站

安装 [uv](https://docs.astral.sh/uv/getting-started/installation/) 后，在干净检出中运行以下命令。uv 会按锁文件安装文档工具，Zensical 会在终端打印本地预览地址。修改 `docs/` 下的 Markdown 即可查看更新。

```bash
uv run --locked --only-group docs zensical serve
```

按 Ctrl+C 结束预览。运行与 CI 相同的严格构建，静态站点会写入 `site/`。

```bash
uv run --locked --only-group docs zensical build --clean --strict
```

新增页面时在 `zensical.toml` 的 `nav` 中登记。站内页面使用相对链接，RootSLAM 测评页在构建时读取 `apps/firefly/rootslam/README.md`，更新正文时编辑这份 README。源码链接使用 GitHub 文件地址。文档 PR 会自动构建，合入 `main` 后发布到 GitHub Pages。首次发布时，在仓库 **Settings → Pages → Source** 选择 **GitHub Actions**。

## 错误处理

Rust 应用和跨 crate 的业务接口用 `firefly_error::Result<T>` 传播失败。实现 `Read`、`Write`、Serde 或设备库接口时，遵循该接口的错误类型。在了解操作含义的一层将外部错误转为 `firefly_error::Error`。[共享错误 API](https://github.com/3SE-Competitive-Robotics-Team/firefly/tree/main/crates/firefly-error/src)

1. 按调用方的处理方式选 `ErrorKind`。例如，配置值无效用 `InvalidArgument`，输入数据损坏用 `InvalidData`，设备正忙用 `Unavailable`。只有调用方可以安全地重复该操作时才用 `Error::temporary`。重试次数和期限由调用方决定。
2. 在知道路径、设备名或话题名的地方补充上下文。文件操作可写成 `File::open(path).at_path("open recording", path)?`，需要引入 `firefly_error::IoResultExt`。其他外部错误用 `Error::caused_by(kind, message, source)` 保留原始错误。传递已有的 Firefly 错误时直接用 `?`，或用 `with_context` 补充信息。
3. 用线程的返回值或通道传递错误。线程退出后，由所有者收回线程并完成清理。需要多个读者查看同一次失败时，可克隆 `Error`。测试应断言分类、重试状态和关键上下文。
4. 在进程入口调用 `firefly_error::report("应用名", result)`。成功返回 0，运行失败把诊断写入 stderr 并返回 1。已经接入进程日志的入口可用 `report_with` 写日志，并在退出前排空队列。命令行解析器负责用法错误。Python 仿真保留原生异常，在 CLI 入口报告运行失败。

调用方读取任务回执或评分 JSON 时，沿用已发布的字段和退出码。只在生成协议响应、评分报告或日志时将错误转成文本。对协议的回归测试应从实际命令读取 stdout、stderr 和退出码。

## Rust 命令行

所有 Rust 应用、`xtask`、示例和自定义基准程序都用 `usage-rs` 解析命令行参数。在所属 crate 的 `[dependencies]` 中写入 `usage-rs.workspace = true`。如果只有示例或基准程序需要解析参数，将这行放在 `[dev-dependencies]`。

在参数结构体上派生 `usage_rs::Cli`，有子命令时在枚举上派生 `usage_rs::Subcommands`。在 `main` 中先调用 `Arguments::parse()`，再读取配置、连接 IPC 或打开设备。这样用户运行 `--help` 时就能直接看到用法。文件路径用 `PathBuf`，默认配置路径用 `firefly_config::paths::config` 取得。可参考[最小示例](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/crates/firefly-config/examples/minimal.rs)和 [xtask 的子命令](https://github.com/3SE-Competitive-Robotics-Team/firefly/blob/main/xtask/src/main.rs)。

修改 CLI 后，运行对应命令的 `--help` 核对用法，用 `__usage_spec__` 查看参数声明。测试中调用 `parse_from`，检查必填参数、互斥选项和每个选项接受几个值。声明语法见 [usage-rs 参数指南](https://usage.jdx.dev/rust/args-and-flags)。
