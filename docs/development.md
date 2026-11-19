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

桌面悬停仿真需要 GPU 和 [scene.toml](../configs/scene.toml) 对应的[场地资产](../packages/firefly-cad/README.md)。

```bash
uv run python scripts/flight.py --mission hover
```

脚本会启动并回收仿真进程。其他运行参数见 `uv run python scripts/flight.py --help`，录制结果见 [Mimir 录制规则](mimir.md)。

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
