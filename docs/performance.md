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

## 2026-10-09：LightGlue 查询改 ORT `IoBinding`（无收益，代码保留）

环境同上（Intel Core Ultra 5 336H、release、ORT 2.0.0-rc.13）。问题：`match_points`
每次调用都走 `Session::run(inputs)`——输入拷进会话缓冲、输出新建张量；改用 `IoBinding`
（输入张量复用 + 输出绑定到预分配张量 + 写入后重绑）是否能降低查询耗时。

同机、同查询集（20 个留出查询）、同模型，先后各跑一次；`query_once` span 取
`logs/bench/*.rrd`：

| 配置 | 查询数 | 中位 | 均值 | 最小 | 最大 | RRD（`logs/bench/`） |
|---|---:|---:|---:|---:|---:|---|
| 无 io-binding（`a00d79c`） | 20 | 185.0 ms | 202.8 ms | 175.4 ms | 562.2 ms | `iob_binding_B.rrd` |
| io-binding（`b620ab6`） | 20 | 185.7 ms | 203.9 ms | 180.2 ms | 553.1 ms | `iob_binding_A.rrd` |

两侧均 20/20 通过冻结库定位契约（≥30 内点、内点率 ≥0.3、重投影 ≤3px、位置 ≤0.25m、
旋转 ≤15°），即 io-binding 不改变输出。

**结论：无收益（中位差 0.4%，在噪声内）。** `Session::run` 的输入拷贝（~256KB）与输出
分配（~6KB）相对每次 ~185ms 的 ORT 推理可忽略。曾有一次飞行观察到「查询中位
462.6ms → 226.2ms」——那是**抢核噪声**（同配置在飞行里出现过 266/462ms 的波动），
不是该改动带来的收益。飞行中的查询耗时不能代替固定输入的重复查询。

顺带量化（同一基准）：最大 553–562ms ≈ 中位 185ms × 3 —— **查询耗时与候选帧数线性相关**
（1 个候选 ~185ms，3 个 ~560ms）。中位 185ms 说明多数查询在第 1 个候选即通过早退；
尾延迟由「跑了多个候选」的那部分决定，所以候选侧优化的目标是**尾延迟**，不是中位。
