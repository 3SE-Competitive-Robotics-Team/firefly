# 开发指南

以下命令从仓库根目录执行。

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

每份计划最多 16 点。起飞和返航高度为 0.5–2 米，停留时间为 (0, 600] 秒，返航平面偏移不超过 0.3 米。航点距起点水平半径不超过 10 米，首点水平距离至少 1 米，各段三维长度为 0.5–10 米。航点高度为 0.5–3.3 米，以支持长方体竖边。`systemctl cat firefly-daemon.service firefly-mimir.service` 可查看部署的两个单元。普通板端运行先启动独立的录制服务，停机时最后排空录制。USB HIL 切换清单后由主机录制同一条真值时间线。

### A733 精降模型

`scripts/export_landing_viplite.py` 从现有 FP32 ONNX 拆出检测框和置信度两个输出，让两者分别量化。使用 `se3_yolo` 的导出环境和训练时的标定集准备工作目录，然后在全志的 x86 Linux ACUITY 镜像 `ubuntu-npu:v2.0.10.2` 内编译。下例中的 `/work` 是该工作目录在容器内的挂载位置。

```bash
python scripts/export_landing_viplite.py --dataset ../se3_yolo/datasets/h_marker --output target/npu-runtime/h-marker
cp scripts/compile_landing_viplite.py target/npu-runtime/h-marker/
# 在配有 ACUITY_PATH 和 VIV_SDK 的容器 shell 中运行
cd /work
python3 compile_landing_viplite.py
```

将工作目录 `bundle/` 下的 `h-marker.nb` 与 `viplite.json` 一起放入 `models/h-marker-yolo26n-cif-v1/`。板端 `landing --image` 可用 352×288 图像验证 VIPLite 推理。桌面 `landing --simulation --image` 使用同一检测器的 ONNX 版本进行对照。

## 运行

SIL 和 USB HIL 使用同一份任务计划和 Rust daemon。xtask 管理仿真进程、顺序提交计划，并用 Mimir 返回的 RRD 逐架次评分。SIL 在本机启动隔离的任务服务，HIL 通过 USB 请求板上正在运行的 daemon。HIL 使用主机 Mimir 录制包含真值的验收数据。MuJoCo 物理进程和离线 RRD 评分使用 Python。

```bash
cargo xtask sil configs/mission-hover.toml
cargo xtask sil configs/mission-square.toml configs/mission-cuboid.toml
cargo xtask deploy-hil
cargo xtask hil configs/mission-square.toml
uv run python scripts/verify_flight.py <录制路径>
uv run python scripts/verify_flight.py --transport board <板上录制路径>
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
