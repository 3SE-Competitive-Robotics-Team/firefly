#!/usr/bin/env python3
"""VIO bench 套件：遍历轨迹生成器实例逐个评测并汇总。

轻量编排层——轨迹定义单一来源在 `firefly_sim.trajectories.TRAJECTORIES`，
本文件只做枚举 × 调 `bench_vio.run_bench` × 汇总表格；新增曲线生成器
实例后自动纳入套件，无需改这里。

Usage:
  uv run python bench/bench_suite.py                                    # 全部实例各 1 轮
  uv run python bench/bench_suite.py --turns 3                          # 每实例 3 轮
  uv run python bench/bench_suite.py --only lissajous_classic,lissajous_tight
  uv run python bench/bench_suite.py --duration 40

Outputs (repo-local, never /tmp):
  logs/bench/<traj>_turn_NN.json   per-turn metrics
  logs/bench/<traj>_turn_NN_*.rrd  per-turn isolated rerun file
  logs/bench/suite_<...>.json      aggregated summary
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "apps" / "firefly-sim" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from bench_vio import cleanup_iceoryx, run_bench  # noqa: E402
from firefly_sim.trajectories import TRAJECTORIES  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description="VIO bench suite: run all trajectory instances and summarize")
    ap.add_argument("--duration", type=float, default=34.0, help="sim duration seconds per turn (default 34)")
    ap.add_argument("--turns", type=int, default=1, help="turns per trajectory (default 1)")
    ap.add_argument("--only", type=str, default=None, help="comma-separated instance names; default all")
    args = ap.parse_args()

    names = sorted(TRAJECTORIES)
    if args.only:
        requested = [s.strip() for s in args.only.split(",") if s.strip()]
        unknown = [n for n in requested if n not in TRAJECTORIES]
        if unknown:
            sys.exit(f"[suite] 未知轨迹实例: {unknown}（可用：{names}）")
        names = requested

    save_dir = REPO_ROOT / "logs" / "bench"
    print(f"[suite] {len(names)} trajectories x {args.turns} turn(s) x {args.duration:.0f}s: {names}")

    results: list[dict] = []
    try:
        for name in names:
            for turn in range(1, args.turns + 1):
                print(f"\n{'=' * 20} {name} turn {turn}/{args.turns} {'=' * 20}")
                out = save_dir / f"{name}_turn_{turn:02d}.json"
                payload = run_bench(float(args.duration), save_dir, out, trajectory=name)
                payload["turn"] = turn
                results.append(payload)
                time.sleep(1)
    except KeyboardInterrupt:
        print("\n[suite] interrupted, cleaning up...")
        cleanup_iceoryx()
        sys.exit(130)

    # 汇总表：每轮一行 + 每实例均值行
    dur_key = str(int(args.duration))
    print("\n" + "=" * 96)
    print(f"suite summary: {len(names)} trajectories x {args.turns} turn(s) x {args.duration:.0f}s")
    print("=" * 96)
    header = f"{'trajectory':<22} {'turn':>4} {'ATE_RMSE':>9} {'ATE_mean':>9} {'ATE_max':>8} {'ATE_final':>10} {'RPE_1s':>7} {'frames':>7}"
    print(header)
    print("-" * 96)
    for name in names:
        turns = [r for r in results if r["trajectory"] == name]
        for r in turns:
            m = r["metrics"]
            snap = m["snapshots"].get(dur_key) or {}
            err_end = snap.get("norm", m["ate_final"])
            warn = " <-- 帧数不足，物理可能发散" if m["num_frames"] < args.duration * 10 * 0.9 else ""
            print(
                f"{name:<22} {r['turn']:>4} {m['ate_rmse']:9.3f} {m['ate_mean']:9.3f} "
                f"{m['ate_max']:8.3f} {m['ate_final']:10.3f} {m['rpe_rmse_1s']:7.3f} {m['num_frames']:7d}{warn}"
            )
        if len(turns) > 1:
            ate = np.array([r["metrics"]["ate_rmse"] for r in turns])
            fin = np.array([r["metrics"]["ate_final"] for r in turns])
            rpe = np.array([r["metrics"]["rpe_rmse_1s"] for r in turns])
            print(
                f"{name + ' avg':<22} {'':>4} {ate.mean():9.3f} {'':>9} {'':>8} {fin.mean():10.3f} {rpe.mean():7.3f}"
            )
    print("=" * 96)

    summary_path = save_dir / f"suite_{len(names)}x{args.turns}x{int(args.duration)}s.json"
    with open(summary_path, "w") as f:
        json.dump(
            {
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "duration_s": float(args.duration),
                "trajectories": names,
                "results": results,
            },
            f,
            indent=2,
        )
    print(f"[suite] summary -> {summary_path}")


if __name__ == "__main__":
    main()
