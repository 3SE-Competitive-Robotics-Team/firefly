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
