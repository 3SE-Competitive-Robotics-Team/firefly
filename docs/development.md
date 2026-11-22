# 开发指南

以下命令从仓库根目录执行。

## Rust 命令行统一使用 usage-rs

所有 Rust 应用、`xtask`、示例和自定义基准程序都用 `usage-rs` 解析命令行参数。在所属 crate 的 `[dependencies]` 中写入 `usage-rs.workspace = true`。如果只有示例或基准程序需要解析参数，将这行放在 `[dev-dependencies]`。

在参数结构体上派生 `usage_rs::Cli`，有子命令时在枚举上派生 `usage_rs::Subcommands`。在 `main` 中先调用 `Arguments::parse()`，再读取配置、连接 IPC 或打开设备。这样用户运行 `--help` 时就能直接看到用法。文件路径用 `PathBuf`，默认配置路径用 `firefly_config::paths::config` 取得。可参考[最小示例](../crates/firefly-config/examples/minimal.rs)和 [xtask 的子命令](../xtask/src/main.rs)。

修改 CLI 后，运行对应命令的 `--help` 核对用法，用 `__usage_spec__` 查看参数声明。测试中调用 `parse_from`，检查必填参数、互斥选项和每个选项接受几个值。声明语法见 [usage-rs 参数指南](https://usage.jdx.dev/rust/args-and-flags)。

## 构建

同步 Python 环境，构建桌面 release 程序。

```bash
uv sync --all-packages --extra test
cargo xtask release
```

构建板端程序需先启动 Apple container。

```bash
container system start
cargo xtask release --target board
```

### 板上任务服务

安装包含 `firefly-daemon` 和 `ffctl` 的 board release。Orange Pi BSP 随开机启动 `firefly.service` 并读取 `/opt/firefly/configs/processes.toml`。整机装配时核对相机设备、IMU 和 PWM 通道，在板上用 `ffctl process enable camera imu motor` 为已装配的设备保存开机配置。部署会更新进程清单，现场交付时需核对设备启用项与实测机体参数。

```bash
cd /opt/firefly
ffctl mission status
ffctl mission submit inspection-001 configs/mission-square.toml
ffctl mission status inspection-001
```

同 ID 重试会返回该任务的当前或最终回执。需要中断时使用 `ffctl mission cancel inspection-001`。任务进入 `complete` 或 `interrupted` 后，可查询运行就绪状态，再提交新的任务 ID。板上应用可直接使用 `firefly_mission::call`。当前 socket 允许服务所属用户访问。现场任务通过计划指定本架次的起飞高度、以起飞点为原点的三维航点、停留时间、返航偏移和精降方式。普通返航偏移默认为零。示例长方体上框高 3.3 米，SIL 配套场地的规划上界为 3.7 米。现场应根据实测空间配置规划边界。

每份计划最多 16 点。起飞和返航高度为 0.5–2 米，停留时间为 (0, 600] 秒，返航平面偏移不超过 0.3 米。航点距起点水平半径不超过 10 米，首点水平距离至少 1 米，各段三维长度为 0.5–10 米。航点高度为 0.5–3.3 米，以支持长方体竖边。`systemctl cat firefly-daemon.service firefly-mimir.service` 可查看部署的两个单元。部署会关闭板端 Mimir 自启动。普通板端运行如需本地录制，可手动执行 `systemctl start firefly-mimir.service`。USB HIL 由宿主机 Mimir 同步录制板端遥测和仿真真值。

### A733 进程绑核

普通模式和 USB HIL 使用相同的板端 CPU 分配。planner 的规划和建图线程可在 CPU 2、4 并行，RootSLAM 的两个 Rayon 工作线程可在 CPU 6、7 并行。其余常驻进程各自限制到一个核心。

| CPU | 进程 |
|---|---|
| 0 | ffctl supervisor、firefly-daemon、板端 Mimir、HIL gadget 的 up、serve 和 watchdog |
| 1 | camera、imu、motor |
| 2、4 | planner（含深度建图线程） |
| 3 | depth |
| 4 | traj、landing、localization |
| 5 | fc |
| 6、7 | firefly-rootslam |

板上用 `ffctl process status` 查看进程 PID，再用 `taskset -apc <PID>` 核对所有线程的有效亲和性。

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

### 飞行链路的 320 深度模型

`depth` 订阅最新一张 320×240 左目灰度图，全视场缩放至 320×320，按 RGB 三通道输入。SIL 的 `depth --simulation` 加载 `models/yolo26n-depth/yolo26n-depth-320.onnx`。按上节从同一权重导出，使用 `imgsz=320`。板端加载同目录的 `yolo26n-depth-320-int8.nb`，输入为有符号 INT8，输出为 80×80 FP16 米制深度。部署包随程序安装该 NBG。两个后端按端点对齐插值恢复为 320×240，并保留原图的采集时间戳。仅离线数据集导出使用渲染几何标签。

在同一工作目录放置 320 ONNX 和 `scripts/compile_depth_viplite.py`。把选定的 JPEG 或 PNG 标定图片打成 ZIP，用导出环境运行 `python compile_depth_viplite.py --prepare /path/to/calibration.zip`，再在上述 ACUITY 镜像中运行 `python3 compile_depth_viplite.py`。量化迭代次数取自生成的图片清单。将 ONNX 和生成的 NBG 安装到模型目录。本次板测 NBG 的 SHA-256 为 `4dbadb624f071d7a15f5df059a5a39df0bfcf4fe84f1ec2e4ddfe20367677a08`。

## 运行

SIL 和 USB HIL 使用同一份任务计划和 Rust daemon。启动前，xtask 取得运行锁并检查当前工作区是否有占用 IPC 的飞行进程。`cargo xtask processes .` 可以查看匹配的 PID。SIL 在本机启动隔离的任务服务，HIL 通过 USB 请求板上正在运行的 daemon。MuJoCo 物理进程使用 Python。

xtask 顺序提交计划，录制结束后逐架次评分。HIL 使用主机 Mimir，将板端遥测和仿真真值写入同一运行目录下的 RRD 分段。桌面 release 包含独立的 `firefly-verify`，可复制到相同平台的机器上使用。评分器将 JSON 报告写入标准输出，验收通过时返回 0，验收失败或录制无效时返回 1。

```bash
cargo xtask sil configs/mission-hover.toml
cargo xtask sil configs/mission-square.toml configs/mission-cuboid.toml
cargo xtask deploy-hil
cargo xtask hil configs/mission-square.toml
target/release/firefly-verify --transport sil <录制路径>
target/release/firefly-verify --transport board <板上录制路径>
```

板上录制按运行、任务和架次核对状态、FC 实测模式、架次间上锁区间与 VIO 会话。SIL/HIL 另用仿真真值核对高度、航点和落点。SIL 需要 GPU 与场地资产。同一次调用保持一个 VIO 坐标系。长方体 3 × 2 × 2 米，从 A′ 起笔，最高航点为 3.3 米。SIL 使用 3.7 米顶板，板端使用现场规划配置。计划偏移直接沿 odom 轴声明。首架次完成且落稳上锁后才提交下一架次。

## 测试

```bash
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
uv run --all-packages --extra test pytest -q -p no:cacheprovider
cargo test -p firefly-pubsub --test python_interop -- --ignored
```

常规 pytest 收集单元测试。系统验收按 `tests/system` 下的文件或用例名显式运行。
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
