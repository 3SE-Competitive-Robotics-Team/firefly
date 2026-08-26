---
name: bench
description: Run VIO GT-vs-odom bench as a suite over trajectory generator instances (bench/bench_suite.py) or single runs (bench/bench_vio.py). Use when you need 34s accuracy, multi-trajectory comparison, or per-turn isolated rrds.
---

# Bench — VIO GT vs Odom 套件

轨迹定义单一来源：`apps/firefly-sim/src/firefly_sim/trajectories.py` 的
`TRAJECTORIES` 注册表（当前 4 个李萨如实例）。套件自动枚举全部实例；
新增曲线生成器实例后无需改 bench 代码。

## Run

```bash
# 套件：全部轨迹实例各 1 轮 × 34s，汇总表 + summary json
uv run python bench/bench_suite.py

# 每实例 3 轮（统计均值）
uv run python bench/bench_suite.py --turns 3

# 只跑指定实例
uv run python bench/bench_suite.py --only lissajous_classic,lissajous_tight

# 单引擎（一条轨迹一轮，viewer-only 不落盘 rrd）
uv run python bench/bench_vio.py --duration 34
uv run python bench/bench_vio.py --duration 34 --trajectory lissajous_wide
```

- `--duration` 秒/轮（默认 34）。
- 套件每轮独立 rrd：`logs/bench/<traj>_turn_NN_*.rrd`；单轮默认 viewer-only。
- 输出均 repo-local（`logs/bench/`），never /tmp。

## Where to look

- `logs/bench/<traj>_turn_NN.json` — per-turn metrics
- `logs/bench/suite_<n>x<turns>x<dur>s.json` — 套件汇总
- `logs/bench/sim.log` / `vio.log` — 最后一轮日志
- 汇总表 frames 列 < duration×10×0.9 时标注「物理可能发散」（QACC NaN）

## 当前实例（峰值速度 m/s）

| 实例 | 形状特点 | peak \|v\| |
|---|---|---|
| lissajous_classic | 历史基线，z 频率最高 | ~1.2 |
| lissajous_wide | 大水平范围、低动态 | ~1.3 |
| lissajous_tight | 小振幅高频，快速运动压力测试 | ~2.1 |
| lissajous_vertical | z 主导 | ~1.1 |

新实例约束：整倍频保证周期闭合连续；z 下限 > 0（防触地）；峰值速度/
加速度在 PD 可跟踪范围内（超限会物理发散）。

## Report

套件末尾打印汇总表（每轮一行 + 多轮时每实例 avg 行），列：
`ATE_RMSE/mean/max/final`（m）、`RPE_1s`（m）、`frames`（10Hz）。

## Notes

- Rerun viewer kept; turns never share one rrd — per-turn isolated files.
- Bench uses numpy linear interp (no scipy), sim `--script [NAME]`, 1x realtime.
