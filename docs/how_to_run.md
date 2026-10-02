# RMUC 运行指南

从仓库根执行命令。运行场景只支持 `rmuc2026`，由 `configs/scene.toml`
校验；缺少资产或地图时直接报错。

## 1. 资产与构建

可先运行 `uv run --all-packages --extra test python scripts/prepare_rmuc.py <场地.stp>`。
只生成并验收几何可加 `--assets-only`；完整入口按阶段报告结果，不保证飞行必然通过。

必须准备以下本地资产（`models/` 不进 Git）：

| 文件 | 使用方 |
|---|---|
| `models/rmuc2026/rmuc2026_collision.json` | MuJoCo 碰撞体，非空 `boxes` 集合 |
| `models/rmuc2026/field.glb` | render 相机画面与 Rerun 静态场景 |
| `models/drone/drone.glb` | render 无人机外观 |

碰撞与视觉资产必须共用世界坐标系，单位米。CAD 转换方法见
[firefly-cad](../packages/firefly-cad/README.md)，视觉导出脚本为
`scripts/blender_field.py`。停机坪表面在 `(-13, 0, 0.375)`，机体原点增加
`0.03 m` 余量。具体场地接触与视觉质量需用实际资产验证。

```bash
uv sync
cargo build --release -p render -p vio -p fc -p ffctl
```

运行要求 Python 3.12+、Rust 1.97+、uv，以及能运行 Bevy 的 GPU 和图形会话。
参数位于 `configs/`，缺键使用代码默认值，缺配置文件报错。
深度图在 Bevy 回读后施加双目退化模型；参数、量程、复现方式与占据地图关系见
[深度传感器模型](depth_sensor.md)。

## 2. 最小闭环与记录

分别开终端启动：

```bash
# 先启动统一记录进程；离线录制只写 logs/ 下的 rrd
uv run firefly-viz --save logs/rmuc_run.rrd

# MuJoCo 物理与 IMU；没有飞控指令时仿真时间暂停
uv run firefly-sim

# Bevy 窗口与传感器：订阅仿真位姿，发布双目和深度
cargo run --release -p render

# IMU + 双目估计；静止初始化完成前不发布里程计
cargo run --release -p vio

# 飞控：未解锁时持续发零推力，允许仿真推进并产生初始化测量
cargo run --release -p fc
cargo run --release -p localization
```

实时查看时先启动 `rerun`，将第一条命令换为 `uv run firefly-viz`，
默认连接 `127.0.0.1:9876`。离线录制用 `rerun logs/rmuc_run.rrd` 回放。
`--save` 使用离线模式。传感器原图在 render 调试面板中查看。

物理为 200Hz、IMU 为 100Hz，仿真位姿默认 10Hz；render 按 10Hz 窗口生成
双目和深度。VIO 在图像时刻更新滤波器，通过 IMU 预测输出 100Hz 里程计。
飞控为 1kHz，仿真取最新电机指令；指令超过墙钟 50ms 未更新时物理暂停。

`Firefly/GroundTruth` 用于仿真图像生成、评测和可视化；
`Firefly/PlantState` 只作评测。估计器和飞控反馈不得消费真值。

### 2.1 起飞、悬停、降落

上电保持静止，观察 `logs/vio` 的初始化日志或里程计 `is_initialized`，
就绪后执行：

```bash
./target/release/ffctl fc arm
./target/release/ffctl fc takeoff 1.0
./target/release/ffctl fc hold
./target/release/ffctl fc land
./target/release/ffctl fc disarm
```

高度相对于起飞点。拒绝指令的原因在 `logs/fc` 中；未就绪时不能跳过初始化。
各进程通过 Ctrl-C 优雅退出，确保 IPC 端口释放和 rrd 完成写入。

## 3. 视觉定位与规划

VIO 输出重力对齐的局部坐标：初始位置为零，航向为规范自由度。
RMUC 静态地图和视觉库图在场地坐标系；`configs/localization.toml [origin]` 配置固定启动位置与航向，
由定位进程建立 map←odom 变换。改变启动点必须同步配置。
不能直接把原始里程计当成场地坐标，也不能用真值提供在线对齐。

可选资产：

| 文件 | 用途 |
|---|---|
| `apps/planner/maps/rmuc2026.ffmap` | planner 的默认静态地图 |
| `apps/planner/maps/rmuc2026.ffvmap` | LightGlue 视觉库图 |
| `models/aliked-n16-k512.onnx` | ALIKED 特征提取权重，可用 `--model` 指定 |
| `models/lightglue-aliked-k512.onnx` | LightGlue 匹配权重，可用 `--model` 指定 |

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

`firefly-cad-build` 从碰撞体的相同体素格导出 `.ffmap` 并核验坐标；
缺失时 planner 启动失败。
`--map` 可指定其他路径，但内容必须对应当前 RMUC 场地。

```bash
cargo run --release -p aliked
cargo run --release -p lightglue -- --map apps/planner/maps/rmuc2026.ffvmap
```

视觉进程同时执行库图 PnP 定位和在线 RGB-D 关键帧回环；`localization` 默认
采用四自由度位姿图发布 `Firefly/CorrectedOdometry`。在线采集需要同步的
`Firefly/Depth`、`Firefly/Odometry` 和 `Firefly/Features`，配置在
`configs/lightglue.toml`（支持 `--config`）。回环阈值、容量和验收边界见
[在线回环](loop_closure.md)。如需 ESKF 对照，设置 `localization.toml` 顶层
`backend = "eskf"`，并在 `lightglue.toml` 设置 `enabled = false` 关闭在线回环。
完成地图对齐后，可运行 planner 接收 `Firefly/CorrectedOdometry` 并发布飞控参考；
通过 `ffctl fc track` 进入跟踪模式，`ffctl planner goal X Y Z` 发布地图系目标。
planner 不订阅原始局部 VIO，不提供平移偏置捷径。未收到有效初始化状态时不发布
参考；状态失联 500ms、变为未初始化或出现非法数值时停止发布并锁存，恢复定位后
须重启 planner。飞控按自己的参考流超时策略处置，不会收到伪造的位置反馈。
飞控反馈与 Hold 锚点保持在 odom 系，仅将地图目标转换到 odom；详情见 [坐标契约](frames.md)。

`scripts/collect_vision_map.py` 独立启动 `render --offline`，在隔离的
`Firefly/Offline/*` 话题设置已知相机位姿，严格按同一时间戳配对左图、深度与标签。
它不启动 sim、估计器或飞控；采集资产供 `aliked --build-map` 使用，
不能作为传感器闭环成立的证据。未得到可用同步帧不得写入库图。
采集网格读取 `configs/vision_map.toml`：全场 2m 网格、两层高度、八个朝向与作业通道
1m 网格合并。碰撞盒射线预筛朝外视角，实际图像再按纹理分区与有效深度拒绝空白视角。
离线深度是理想几何标签，在线传感器仍施加噪声。采集配置、逐帧位姿、质量与内容指纹
保存在清单中；更换作业区应验证相应覆盖。冻结库图只在场地、相机标定或成像外观改变时
失效，普通重编译与在线噪声修改不触发重建。`render --export-asset-contract <path>`
导出实际兼容契约；历史二进制 hash 只用于追溯生成来源。

## 4. 诊断与验证

所有运行日志与调试数据通过 `Firefly/Log` / `Firefly/Viz` 汇入 firefly-viz，
正式记录只写 `logs/*.rrd`。`sim_time` 对齐时间，不能替代空间对齐。

常用实体：`vio/odom`、`vio/traj`、`vio/debug/*`、`gt/pose`、`gt/traj`、
`fc/debug/*`、`plan/*`、`world/rmuc2026`、`logs/<app>`。
原始 VIO 与真值的空间对齐只在评测中进行，固定尺度，结果不得回流算法。
通用指标在 `tests/evaluation/metrics.py`。

```bash
cargo fmt --all --check
cargo clippy --all-targets -- -D warnings
cargo test
cargo test -p firefly-planner --test planner_contracts
uv run --all-packages --extra test pytest apps/firefly-sim/tests/ tests/evaluation/ tests/system/ -q -p no:cacheprovider
```

准备真实 RMUC 资产并构建 release 版 vio / fc / render 后，在没有其他闭环进程
运行的图形会话中执行：

```bash
FIREFLY_RUN_SENSOR_STARTUP=1 uv run --all-packages --extra test pytest tests/system/test_sensor_startup.py -q
```

检查屏蔽 PlantState，保留供 render 合成图像的 GroundTruth，验证静止初始化、
局部原点、时间戳递增与未解锁零推力。记录写入 `logs/`，子进程通过 SIGINT 退出。
资产缺失时检查在启动子进程前报错；单元测试不能替代真实 RMUC 飞行验证。

## 位置估计失联

VIO 失联超过墙钟 500ms 或变为未初始化时，空中进入 `ATTITUDE_FALLBACK`（模式编码 6）。
停止位置/速度反馈与轨迹跟踪，仅由新鲜 IMU 稳定水平姿态，输出带有限倾斜补偿的
标称 `m·g` 推力。IMU 逐样本按测量时间积分；重复样本不续期、不重复积分。
此模式不保证定点、保高或安全落地；水平速度、风、质量误差和 IMU 漂移仍会造成位移。
没有独立高度或接地传感器，不能按陈旧位置自动上锁，也不定时切断推力。
里程计恢复不会自动回到轨迹：需有效里程计和 IMU 后显式 `ffctl fc hold` 或 `land`。
定位进程不支持空中重启后的 odom 坐标重置；恢复样本必须仍属于原来的 odom 系。
地面未起飞时失联则上锁；IMU 也失联时没有可维持姿态的反馈，推力保护仍会归零。
该行为只是一种能力受限的降级，不作为自主安全降落验收通过的依据。
