# 性能测量

## 2026-10-03：LightGlue CPU 推理

环境：Intel Core Ultra 5 336H、12 个逻辑 CPU、release 构建、ORT 2.0.0-rc.13，
其余飞行进程未启动。基线为 `bac1126`，使用相同的查询 root span 插桩。
计时取 RRD 中 ConsoleReporter 的 `lightglue::query_once`，包含候选选择、
匹配与 PnP，不含模型加载及地图加载。

输入固定为 ALIKED-N16 / LightGlue K512 与 20 个独立查询。
地图为 `apps/planner/maps/rmuc2026.ffvmap`，SHA-256：
`de4aec777cb6a3bc59a2aa669097ea8ef5320197388d0e5fb7453dc8dbd56180`。
查询库为 `models/rmuc2026/derived/vision_validation/c703dc6e50a84dec/queries.ffvmap`。
没有重新生成模型、库图或查询，没有修改特征数量、定位或飞行验收阈值。

| 配置 | 查询数 | 平均耗时 | 中位数 | 最大值 | RRD（`logs/fastwin/`） |
|---|---:|---:|---:|---:|---|
| 2 线程，默认自旋 | 20 | 294.237 ms | 265.849 ms | 790.557 ms | `r1_before.rrd` |
| 4 线程，禁用自旋 | 20 | 251.389 ms | 227.425 ms | 672.910 ms | `r1_after.rrd` |
| 4 线程，禁用自旋，复测 | 20 | 242.891 ms | 220.728 ms | 666.415 ms | `r1_after_repeat.rrd` |
| 2 线程，禁用自旋（未采用） | 20 | 323.260 ms | 297.940 ms | 869.333 ms | `r1_two_sleep.rrd` |

采用四线程且禁用自旋。两次实测平均耗时比基线少 14.6% / 17.5%。
全部配置的 20/20 查询通过；每个查询的位置误差、角度误差、内点数、对应点数和
重投影误差完全一致。位置 RMSE 为 0.051643m，最大误差 0.121318m。
在线回环仍用独立单线程会话。

四线程增加推理阶段的 CPU 并发；禁用自旋让等待阶段休眠。这些数据仅证明固定输入
下的定位查询耗时，不证明全栈吞吐、功耗、VIO 精度或往返任务成功率。完整飞行中的
CPU 竞争与视觉观测总延迟仍须单独测量。

复现入口见 [独立视角验收](testing.md#冻结视觉资产的独立视角验收)。
每个查询都有独立 root span，汇总 JSON 的 `query_wall_s` 仅用于便捷查看，
性能结论以原始 trace 为准。ConsoleReporter 输出经
`tests/system/mission_io.py::Recorder.event` 汇入 `logs/acceptance` TextLog；
`firefly-viz --save logs/fastwin/<tag>.rrd` 统一落盘，结束时使用 SIGINT。

## 2026-10-03：双向 LK 金字塔复用

对照 OpenVINS `69488123ed9362dd44b6f28e7f4680abbff1442b` 的
`ov_core/src/track/TrackKLT.cpp`：预处理阶段构建图像金字塔，匹配阶段复用。
本次仅在一次双向跟踪调用内复用两幅图像的金字塔；构建次数由 4 次降为 2 次。
两个方向仍分别计算各自模板的 Scharr 导数；梯度归一化、插值、迭代、阈值和
双向检查保持一致。没有引入跨帧缓存或更改内参同步方式。

固定 320×240 量化光滑纹理、300 点、已知 `[0.25,-0.4]px` 平移。
预热 10 次后测量 100 次，默认 Rayon 线程池；其余飞行进程未启动。
取 `firefly_vio_core::track::lk::optical_flow` 的 trace，包含全部金字塔、导数、
正向/反向求解和一致性检查，不含输入生成、输出断言和 trace 落盘。
100 次基线与复用版本使用相同插桩，只改变金字塔构建的位置。

| 配置 | 样本数 | 平均耗时 | 中位数 | 最大值 | RRD（`logs/fastwin/`） |
|---|---:|---:|---:|---:|---|
| 双向各自构建 | 100 | 5.303 ms | 5.148 ms | 8.261 ms | `r2_before_100.rrd` |
| 双向复用 | 100 | 5.258 ms | 4.786 ms | 10.719 ms | `r2_after_100.rrd` |
| 双向复用，复测 | 100 | 4.972 ms | 4.692 ms | 9.525 ms | `r2_after_100_repeat.rrd` |

中位数下降 7.0% / 8.9%，平均值下降 0.8% / 6.2%。最大值没有改善，
因此只报告小幅典型耗时收益，不宣称尾延迟改善或系统实时性保证。
最初 30 次试测也保留在 `r2_before.rrd` / `r2_after.rrd`：平均
5.105 / 4.940ms；后者额外记录金字塔子 span，正式比较采用上表一致插桩。

全部 300 点有效，每点相对解析平移的误差 <0.08px；全部采样的输出坐标位模式
及有效状态的 FNV 风格指纹均为 `11122838785378531661`，前后相同。
指纹用于本机 A/B 诊断，不能代替解析平移断言或要求不同平台的浮点结果逐位相同。
普通测试另覆盖单位斜坡导数、中心差分、单步位移以及正负亚像素/大位移。

显式运行性能采样用例（普通 `cargo test` 跳过；没有机器相关耗时通过门槛）：

```bash
cargo test --release -p firefly-vio-core profile_bidirectional_tracking -- --ignored --nocapture
```

捕获方式同上，所有 ConsoleReporter 输出经 Recorder 进入 RRD。
这是现有 LK 测试模块内的固定输入用例，不提供独立 bench 程序。

### 固定输入指纹

| 输入 | SHA-256 |
|---|---|
| `lightglue-aliked-k512.onnx` | `df3694994422ceb0c9df8e390c99edc4dd03778044c646c1d01722c7252994ed` |
| `aliked-n16-k512.onnx` | `4d4264f61795909f84bddc0dcf10b5ed1252012a8b0cffdc38983f4bd90b0c7e` |
| 独立查询 `queries.ffvmap` | `25bc496c47b6c217e0992b6077f78838f746c1192b36a6e3596164f2decfdf7c` |

### 回归范围

`cargo test --workspace` 全部普通测试通过；显式运行的 20 查询定位与 300 点 LK
采样也通过。依赖真实资产或手动启用的其他 ignored 用例不包含在普通测试通过结论中。
本轮没有重新进行 10m 往返飞行验收。
