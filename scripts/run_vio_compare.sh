#!/usr/bin/env bash
# VIO 对比采集：与 run_void_e2e.sh 同口径（GT 起点初始化 + 首点对齐 ATE），
# 订阅 Firefly/Odometry，输出 npy 轨迹到 logs/p10_compare/vio/runN/。
# 仅采集 + 算 ATE，不改任何代码。
#
# 用法：scripts/run_vio_compare.sh [秒数] [--runs N] [--trajectory NAME]
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

T=60
RUNS=1
TRAJECTORY=""
while [ $# -gt 0 ]; do
  case "$1" in
    --runs) RUNS=${2:-1}; shift 2 ;;
    --trajectory) TRAJECTORY=${2:-}; shift 2 ;;
    *) T=$1; shift ;;
  esac
done

# 场景子目录：悬停 = hover，轨迹 = 轨迹名（与 logs/p10_compare/void/ 对齐）
SCENE="hover"
if [ -n "$TRAJECTORY" ]; then
  SCENE="$TRAJECTORY"
fi
OUT="$ROOT/logs/p10_compare/vio/$SCENE"
mkdir -p "$OUT"

cleanup() {
  pkill -9 -f "firefly-sim --no-trace" 2>/dev/null
  pkill -9 -f "target/release/vio" 2>/dev/null
  sleep 1
  rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state 2>/dev/null
}

run_one_round() {
  local i=$1
  local run_dir="$OUT/run$i"
  mkdir -p "$run_dir"
  local sim_pid vio_pid rec_ec ate_ec sim_cmd

  cleanup

  if [ -n "$TRAJECTORY" ]; then
    sim_cmd="uv run firefly-sim --no-trace --script $TRAJECTORY"
  else
    sim_cmd="uv run firefly-sim --no-trace"
  fi

  echo "[compare][run $i] 启动 sim（${T}s 闭环）"
  nohup $sim_cmd > "$run_dir/sim.log" 2>&1 &
  sim_pid=$!

  for _ in $(seq 1 50); do
    grep -q "已就绪" "$run_dir/sim.log" && break
    if ! kill -0 "$sim_pid" 2>/dev/null; then
      echo "[compare][run $i] sim 启动失败"; tail -5 "$run_dir/sim.log"; return 1
    fi
    sleep 0.2
  done

  echo "[compare][run $i] 启动 vio"
  sleep 3
  nohup env RUST_LOG=info "$ROOT/target/release/vio" > "$run_dir/vio.log" 2>&1 &
  vio_pid=$!

  echo "[compare][run $i] 采集 ${T}s"
  uv run python - "$T" "$run_dir" <<'PY'
"""订阅 Firefly/GroundTruth 与 Firefly/Odometry，记录轨迹（VIO 版 recorder）。"""
import sys
import time

import iceoryx2 as iox2
import numpy as np

from firefly_mujoco import OdomMessage, TraceContext

T = float(sys.argv[1])
WORK = sys.argv[2]
ODOM_TOPIC = "Firefly/Odometry"
GT_TOPIC = "Firefly/GroundTruth"


def subscriber(node, topic: str):
    service = (
        node.service_builder(iox2.ServiceName.new(topic))
        .publish_subscribe(OdomMessage)
        .user_header(TraceContext)
        .open_or_create()
    )
    return service.subscriber_builder().buffer_size(20).create()


def main() -> None:
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    gt_sub = subscriber(node, GT_TOPIC)
    odom_sub = subscriber(node, ODOM_TOPIC)
    gt: list[tuple[float, np.ndarray]] = []
    odom: list[tuple[float, np.ndarray]] = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < T:
        while (s := gt_sub.receive()) is not None:
            m = s.payload().contents
            gt.append((m.timestamp, np.array([m.position_x, m.position_y, m.position_z])))
        while (s := odom_sub.receive()) is not None:
            m = s.payload().contents
            pos = np.array([m.position_x, m.position_y, m.position_z])
            if bool(m.is_initialized) and np.all(np.isfinite(pos)):
                odom.append((m.timestamp, pos))
        time.sleep(0.005)
    print(f"[recorder] gt={len(gt)} odom={len(odom)} 样本")
    gt_dict = {t: p for t, p in gt}
    odom_dict = {t: p for t, p in odom}
    gt = sorted(gt_dict.items())
    odom = sorted(odom_dict.items())
    with open(f"{WORK}/gt.npy", "wb") as f:
        np.save(f, np.array([p for _, p in gt]))
    with open(f"{WORK}/gt_t.npy", "wb") as f:
        np.save(f, np.array([t for t, _ in gt]))
    with open(f"{WORK}/odom.npy", "wb") as f:
        np.save(f, np.array([p for _, p in odom]))
    with open(f"{WORK}/odom_t.npy", "wb") as f:
        np.save(f, np.array([t for t, _ in odom]))
    with open(f"{WORK}/record_end.txt", "w") as f:
        f.write(str(gt[-1][0] if gt else 0.0))


if __name__ == "__main__":
    main()
PY
  rec_ec=$?

  echo "[compare][run $i] 停止 sim/vio（SIGINT 优雅退出）"
  pkill -INT -f "firefly-sim --no-trace" 2>/dev/null
  pkill -INT -f "target/release/vio" 2>/dev/null
  for _ in $(seq 1 50); do
    if ! pgrep -f "firefly-sim --no-trace" >/dev/null 2>&1 \
      && ! pgrep -f "target/release/vio" >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
  pkill -9 -f "firefly-sim --no-trace" 2>/dev/null
  pkill -9 -f "target/release/vio" 2>/dev/null
  sleep 1

  echo "=== [run $i] vio 日志尾部 ==="
  tail -8 "$run_dir/vio.log"
  echo "=== [run $i] sim 日志尾部 ==="
  tail -5 "$run_dir/sim.log"

  echo "=== [run $i] ATE 计算（首点对齐）==="
  uv run python - "$run_dir" <<'PY'
"""首点对齐 ATE（与 run_void_e2e.sh 完全同口径）。"""
import sys

import numpy as np

WORK = sys.argv[1]

gt = np.load(f"{WORK}/gt.npy")
gt_t = np.load(f"{WORK}/gt_t.npy")
odom = np.load(f"{WORK}/odom.npy")
odom_t = np.load(f"{WORK}/odom_t.npy")

print(f"[ate] GT {len(gt)} 样本（t {gt_t.min():.2f}~{gt_t.max():.2f}），"
      f"Odom {len(odom)} 样本（t {odom_t.min():.2f}~{odom_t.max():.2f}）")

if len(gt) < 10 or len(odom) < 10:
    print("[ate] 样本不足")
    sys.exit(1)

t0 = max(gt_t.min(), odom_t.min()) + 0.1
t_end_rec = 0.0
try:
    with open(f"{WORK}/record_end.txt") as f:
        t_end_rec = float(f.read().strip())
except FileNotFoundError:
    pass
t1 = min(gt_t.max(), odom_t.max(), t_end_rec if t_end_rec > 0 else float("inf")) - 0.1
if t1 <= t0:
    print("[ate] 无共同时间窗")
    sys.exit(1)
mask = (gt_t >= t0) & (gt_t <= t1)
gt_w = gt[mask]
gt_tw = gt_t[mask]
odom_w = np.stack([
    np.interp(gt_tw, odom_t, odom[:, 0]),
    np.interp(gt_tw, odom_t, odom[:, 1]),
    np.interp(gt_tw, odom_t, odom[:, 2]),
], axis=1)

gt_span = float(np.ptp(gt_w, axis=0).max())
odom_span = float(np.ptp(odom_w, axis=0).max())

print("=== vio 健康统计（perf-diag）===")
try:
    with open(f"{WORK}/vio.log", "r") as f:
        for line in f:
            if "[perf-diag]" in line:
                print(line.strip())
except FileNotFoundError:
    print("(无 vio.log)")

if gt_span < 0.05 or odom_span < 0.05:
    err = np.linalg.norm(odom_w - gt_w, axis=1)
    ate_rms = float(np.sqrt(np.mean(err**2)))
    ate_mean = float(np.mean(err))
    ate_max = float(np.max(err))
    print(f"[ate] 轨迹退化（GT span={gt_span:.3f}m），直接绝对误差：")
else:
    A = odom_w.T
    B = gt_w.T
    aligned = A - A[:, :1] + B[:, :1]
    err = np.linalg.norm(aligned - B, axis=0)
    ate_rms = float(np.sqrt(np.mean(err**2)))
    ate_mean = float(np.mean(err))
    ate_max = float(np.max(err))
    print(f"[ate] 首点对齐：")
ate_rms = float(np.sqrt(np.mean(err**2)))
ate_mean = float(np.mean(err))
ate_max = float(np.max(err))
print(f"[ate] ATE-RMS  = {ate_rms:.4f} m")
print(f"[ate] ATE-mean = {ate_mean:.4f} m")
print(f"[ate] ATE-max  = {ate_max:.4f} m")
ok = ate_rms < 0.3
print(f"[ate] {'PASS' if ok else 'FAIL'}（阈值 ATE-RMS < 0.3m）")

with open(f"{WORK}/ate_meta.txt", "w") as f:
    f.write(f"{ate_rms} {ate_mean} {ate_max} {1 if ok else 0}\n")
sys.exit(0 if ok else 2)
PY
  ate_ec=$?

  echo "=== [run $i] 结束 (recorder=${rec_ec} ate=${ate_ec}) ==="
  [ "$rec_ec" -eq 0 ] && [ "$ate_ec" -eq 0 ]
}

ALL_OK=1
for i in $(seq 1 "$RUNS"); do
  if ! run_one_round "$i"; then
    ALL_OK=0
  fi
done

echo
echo "=========================================================="
echo "=== vio compare 汇总（$RUNS 轮 × ${T}s）==="
echo "=========================================================="
uv run python - "$OUT" "$RUNS" <<'PY'
"""汇总多轮 ATE：逐轮明细 + mean±std。"""
import sys

import numpy as np

WORK = sys.argv[1]
RUNS = int(sys.argv[2])

rows = []
for i in range(1, RUNS + 1):
    try:
        with open(f"{WORK}/run{i}/ate_meta.txt") as f:
            rms, mean, mx, ok = map(float, f.read().split())
    except FileNotFoundError:
        print(f"run {i}: 缺 ate_meta.txt（该轮失败）")
        rows.append((i, float("nan"), float("nan"), float("nan"), False))
        continue
    rows.append((i, rms, mean, mx, bool(ok)))

print("--- 逐轮明细 ---")
print(f"{'run':>3} {'ATE-RMS':>9} {'ATE-mean':>9} {'ATE-max':>9} {'verdict':>6}")
all_ok = True
for i, rms, mean, mx, ok in rows:
    print(f"{i:>3} {rms:>9.4f} {mean:>9.4f} {mx:>9.4f} {'PASS' if ok else 'FAIL':>6}")
    all_ok &= ok

rms = np.array([r[1] for r in rows])
rms = rms[np.isfinite(rms)]
if len(rms) == 0:
    print("无有效轮次数据")
    sys.exit(1)
std = float(np.std(rms, ddof=1)) if len(rms) > 1 else 0.0
print(f"--- 汇总 ---")
print(f"ATE-RMS mean±std = {np.mean(rms):.4f} ± {std:.4f} m（{len(rms)} 轮）")
sys.exit(0 if all_ok else 1)
PY

cleanup
