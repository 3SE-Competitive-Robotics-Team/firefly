---
name: bench
description: Run VIO GT-vs-odom bench (bench/bench_vio.py) and report a clean table. Use when you need 34s accuracy, multi-turn stats, or per-turn isolated rrds.
---

# Bench — VIO 34s GT vs Odom

Bench keeps rerun enabled with per-turn isolated rrds (never single shared file). Outputs are repo-local (`logs/bench/`), never `/tmp`.

## Run

```bash
# single 34s turn (viewer-only, no file)
uv run python bench/bench_vio.py --duration 34

# 10 turns, auto per-turn rrds in logs/bench/turn_01_34s_*.rrd ...
uv run python bench/bench_vio.py --turns 10 --duration 34

# custom output / per-turn rrds dir
uv run python bench/bench_vio.py --turns 10 --duration 34 --save-dir logs/bench --output logs/bench/vio_bench.json
```

- `--turns N` runs N isolated turns sequentially (default 1). For N>1 bench auto-creates per-turn rrds in `logs/bench/` even if `--save-dir` not set — never writes many turns into one `task.rrd`.
- `--duration` seconds per turn (default 34).
- `--save-dir` per-turn isolated `*.rrd` dir (default `logs/bench` when turns>1).
- `--output` per-turn json (`logs/bench/turn_XX.json` when turns>1) + `summary_NxDs.json`.

## Where to look

- `logs/bench/turn_*.json` — per-turn metrics
- `logs/bench/summary_10x34s.json` — aggregated
- `logs/bench/*.rrd` — one file per turn (4–5M each), isolated
- `logs/bench/sim.log` / `vio.log` — last turn logs

## Report (already printed)

Bench prints a clean table at end of multi-turn run:

```
turn  ATE_RMSE  ATE_mean   ATE_max  ATE_final  RPE_1s  err@34s frames
   1     762.7     496.4    2025.4     2025.4    80.5   2025.4    340
 ...
 avg      803.6     560.5               1985.5    73.7
 std      510.3                         1244.0
```

Columns: `ATE_RMSE/mean/max/final` (m, km-scale right now expected), `RPE_1s` (delta 1s), `err@34s` (snapshot at 34s), `frames` (10Hz).

## Read summary programmatically

```bash
uv run python -c "import json; j=json.load(open('logs/bench/summary_10x34s.json')); [print(f\"{i+1}: {m['ate_rmse']:.0f} {m['ate_final']:.0f}\") for i,m in enumerate([r['metrics'] for r in j['results']])]"
```

## Notes

- Rerun viewer is kept (shared when single turn, dedicated per-turn when N>1). `bench` never writes many turns into one rrd — each turn gets `turn_XX_34s_<ts>.rrd`.
- Bench uses numpy linear interp (no scipy), sim `--script` Lissajous (`period 20s`), 1x realtime.
