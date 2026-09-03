#!/usr/bin/env bash
# firefly-void 三进程 e2e 闭环验证：sim（MuJoCo 物理）→ void（DIVO 里程计）。
#
# 用法：scripts/run_void_e2e.sh [秒数] [--runs N] [--trajectory NAME]
#   [秒数]        每轮时长（缺省 60s）
#   --runs N      重复 N 轮（缺省 1）：每轮独立起停 sim+void，轮间清理进程与
#                 iceoryx2 残留；末尾输出逐轮 ATE 明细与 mean±std。
#                 sim 噪声无种子，多轮统计反映启动期偏置注入的随机性。
#   --trajectory NAME  透传给 `firefly-sim --script NAME`（具名轨迹驱动运动，
#                 跳过 planner；缺省不传 = 悬停闭环）。李萨如 bench 用
#                 `--trajectory lissajous_classic`。
#
# 流程（每轮）：
#   1. 清理残留（进程 + iceoryx2 shm/port_tag）
#   2. 后台起 `uv run firefly-sim --no-trace [--script NAME]`（物理 + 传感器发布）
#   3. 后台起 `cargo run -p void`（估计，发布 Firefly/VoidOdom）
#   4. Python recorder 同步采集 GT 与 VoidOdom（期间两进程在跑）
#   5. 到点优雅 kill（SIGINT → 端口 Drop，无 iceoryx 幽灵残留）
#   6. 离线对齐（首点对齐：odom 首点平移到 GT 首点，保留绝对漂移）算 ATE，
#      输出 RMS/mean/max 与健康统计。Umeyama 相似变换会吸收全局平移/旋转/
#      尺度（"形状误差"），把整体偏移虚低——机器人比赛要绝对定位精度，
#      口径改为首点对齐（仅移除初始位置偏置，保留漂移与尺度）。
#
# 退出码：0 = 全部轮 PASS（ATE-RMS < 0.3m）；1 = 任一轮失败。

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK=/tmp/void_e2e
mkdir -p "$WORK"

cd "$ROOT"

# 参数：位置参数 = 秒数；--runs N = 轮数；--trajectory NAME = 轨迹脚本
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

# 清理可能的历史残留（进程全死后才安全；iceoryx2 服务注册文件是持久
# 缓存可复用，幽灵端口 = node 目录里的 port_tag 残留）
cleanup() {
  pkill -9 -f "firefly-sim --no-trace" 2>/dev/null
  pkill -9 -f "target/release/void" 2>/dev/null
  sleep 1
  rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state 2>/dev/null
}

run_one_round() {
  local i=$1
  local run_dir="$WORK/run$i"
  mkdir -p "$run_dir"
  local sim_pid void_pid rec_ec ate_ec leftover sim_cmd

  cleanup

  if [ -n "$TRAJECTORY" ]; then
    sim_cmd="uv run firefly-sim --no-trace --script $TRAJECTORY"
  else
    sim_cmd="uv run firefly-sim --no-trace"
  fi

  echo "[e2e][run $i] 启动 sim（--no-trace${TRAJECTORY:+ --script $TRAJECTORY}，${T}s 闭环）"
  nohup $sim_cmd > "$run_dir/sim.log" 2>&1 &
  sim_pid=$!

  # 等待 sim 就绪（话题服务创建）
  for _ in $(seq 1 50); do
    grep -q "已就绪" "$run_dir/sim.log" && break
    if ! kill -0 "$sim_pid" 2>/dev/null; then
      echo "[e2e][run $i] sim 启动失败，日志尾部："; tail -5 "$run_dir/sim.log"; return 1
    fi
    sleep 0.2
  done

  echo "[e2e][run $i] 启动 void"
  # 等 sim 跑 3s（PD 启动瞬态结束）再起 void：瞬态中纯 IMU 传播
  # + 测量未建立会累积初始漂移（实测 0-2s 偏差 ~0.2m 直接进 ATE）
  sleep 3
  nohup env RUST_LOG=info cargo run --release -p void > "$run_dir/void.log" 2>&1 &
  void_pid=$!

  # recorder：前台同步采集 T 秒（sim/void 在后台跑）
  echo "[e2e][run $i] 采集 ${T}s（recorder 前台，sim/void 后台）"
  uv run python - "$T" "$run_dir" <<'PY'
"""订阅 Firefly/GroundTruth 与 Firefly/VoidOdom，记录轨迹到文件。"""
import sys
import time

import iceoryx2 as iox2
import numpy as np

from firefly_mujoco import OdomMessage, TraceContext

T = float(sys.argv[1])
WORK = sys.argv[2]

#: 与 Rust apps/void::VOID_ODOM_TOPIC 一致（本地常量，不扩 pubsub）
VOID_ODOM_TOPIC = "Firefly/VoidOdom"
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
    odom_sub = subscriber(node, VOID_ODOM_TOPIC)
    gt: list[tuple[float, np.ndarray]] = []
    odom: list[tuple[float, np.ndarray]] = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < T:
        while (s := gt_sub.receive()) is not None:
            m = s.payload().contents
            gt.append((m.timestamp, np.array([m.position_x, m.position_y, m.position_z])))
        while (s := odom_sub.receive()) is not None:
            m = s.payload().contents
            odom.append((m.timestamp, np.array([m.position_x, m.position_y, m.position_z])))
        time.sleep(0.005)
    print(f"[recorder] gt={len(gt)} odom={len(odom)} 样本")
    # 按时间戳去重（保留最后一条）+ 排序
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
    # 采集窗口终点：最后一条 GT 的时间戳（此后 sim 可能仍运行到被 SIGINT
    # 终止，但 recorder 已停止采集——ATE 只应覆盖采集窗口，避免停机段
    # 传感器中断造成的虚假误差进统计）
    with open(f"{WORK}/record_end.txt", "w") as f:
        f.write(str(gt[-1][0] if gt else 0.0))


if __name__ == "__main__":
    main()
PY
  rec_ec=$?

  # 优雅终止（SIGINT → WaitSet / Python KeyboardInterrupt 捕获 → 端口 Drop）
  # 注意：SIM_PID 是 `uv run` 包装进程，SIGINT 不转发给 Python——用
  # pkill -f 匹配实际进程（void 的 WaitSet 捕获 SIGINT 优雅退出）
  echo "[e2e][run $i] 停止 sim/void（SIGINT 优雅退出）"
  pkill -INT -f "firefly-sim --no-trace" 2>/dev/null
  pkill -INT -f "target/release/void" 2>/dev/null
  for _ in $(seq 1 50); do
    if ! pgrep -f "firefly-sim --no-trace" >/dev/null 2>&1 \
      && ! pgrep -f "target/release/void" >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
  # 兜底：仍未退出的（极慢帧处理）再给 3s
  for _ in $(seq 1 15); do
    if ! pgrep -f "firefly-sim --no-trace" >/dev/null 2>&1 \
      && ! pgrep -f "target/release/void" >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
  pkill -9 -f "firefly-sim --no-trace" 2>/dev/null
  pkill -9 -f "target/release/void" 2>/dev/null
  # 等 iceoryx 端口释放（SIGINT 后 Drop 需短暂时间）
  sleep 1

  # void 日志健康检查
  echo "=== [run $i] void 日志尾部 ==="
  tail -8 "$run_dir/void.log"
  echo "=== [run $i] sim 日志尾部 ==="
  tail -5 "$run_dir/sim.log"

  # ATE 计算（Umeyama 相似变换对齐）
  echo "=== [run $i] ATE 计算 ==="
  uv run python - "$run_dir" <<'PY'
"""离线对齐 GT 与 VoidOdom（Umeyama 相似变换），输出 ATE 统计与健康统计。"""
import sys

import numpy as np

WORK = sys.argv[1]

try:
    gt = np.load(f"{WORK}/gt.npy")
    gt_t = np.load(f"{WORK}/gt_t.npy")
    odom = np.load(f"{WORK}/odom.npy")
    odom_t = np.load(f"{WORK}/odom_t.npy")
except FileNotFoundError as e:
    print(f"[ate] 轨迹文件缺失：{e}（recorder 未采到数据）")
    sys.exit(1)

print(f"[ate] GT {len(gt)} 样本（t {gt_t.min():.2f}~{gt_t.max():.2f}），"
      f"VoidOdom {len(odom)} 样本（t {odom_t.min():.2f}~{odom_t.max():.2f}）")

if len(gt) < 10 or len(odom) < 10:
    print("[ate] 样本不足，无法对齐")
    sys.exit(1)

# 时间对齐：GT 为基准，估计按时间线性插值（取共同时间窗）
# 起评点后移 3s：void 启动期（GT 真值初始化 + 首帧收敛前）估计未收敛，
# 首点对齐锚在垃圾样本上会把启动偏置成整段常量误差（实测一次虚增到
# 2.47m）；后移只切启动瞬态，保留完整机动 + 返程（33 轮回放验证：
# 口径变更前后 PASS/FAIL  verdict 零翻转）
# 终点截到采集窗口（recorder 记录的最后一条 GT），排除 SIGINT 停机段
t0 = max(gt_t.min(), odom_t.min()) + 3.0
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

# 轨迹退化判断（悬停/短轨迹）：GT 与估计都近静止时对齐无意义，且
# 估计与 GT 同起点（t0 初始化，全局系一致），直接算逐点绝对误差。
gt_span = float(np.ptp(gt_w, axis=0).max())
odom_span = float(np.ptp(odom_w, axis=0).max())

# 健康统计：解析 void.log 的 perf-diag 行（深度/视觉收敛率）
print("=== void 健康统计（perf-diag）===")
try:
    with open(f"{WORK}/void.log", "r") as f:
        for line in f:
            if "[perf-diag]" in line:
                print(line.strip())
except FileNotFoundError:
    print("(无 void.log)")

if gt_span < 0.05 or odom_span < 0.05:
    err = np.linalg.norm(odom_w - gt_w, axis=1)
    ate_rms = float(np.sqrt(np.mean(err**2)))
    ate_mean = float(np.mean(err))
    ate_max = float(np.max(err))
    print(f"[ate] 轨迹退化（GT span={gt_span:.3f}m），直接绝对误差：")
    print(f"[ate] ATE-RMS  = {ate_rms:.4f} m")
    print(f"[ate] ATE-mean = {ate_mean:.4f} m")
    print(f"[ate] ATE-max  = {ate_max:.4f} m")
    ok = ate_rms < 0.3
    print(f"[ate] {'PASS' if ok else 'FAIL'}（阈值 ATE-RMS < 0.3m）")
else:
    # 首点对齐：odom 整体平移使首点与 GT 首点重合（仅移除初始位置偏置，
    # 保留绝对漂移/旋转/尺度误差）。另算 Umeyama 相似变换作对照——
    # 后者吸收全局平移/旋转/尺度，是"形状误差"，通常明显低于首点对齐。
    A = odom_w.T  # 3×N
    B = gt_w.T  # 3×N

    # 首点对齐（平移对齐）
    aligned = A - A[:, :1] + B[:, :1]
    err = np.linalg.norm(aligned - B, axis=0)
    ate_rms = float(np.sqrt(np.mean(err**2)))
    ate_mean = float(np.mean(err))
    ate_max = float(np.max(err))
    print(f"[ate] 首点对齐：")
    print(f"[ate] ATE-RMS  = {ate_rms:.4f} m")
    print(f"[ate] ATE-mean = {ate_mean:.4f} m")
    print(f"[ate] ATE-max  = {ate_max:.4f} m")

    # Umeyama 对照（相似变换：s·R·odom + t ≈ gt）
    mu_a = A.mean(axis=1, keepdims=True)
    mu_b = B.mean(axis=1, keepdims=True)
    aa = A - mu_a
    bb = B - mu_b
    h = aa @ bb.T
    u, s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    scale = np.trace(np.diag(s) @ r) / np.sum(aa * aa)
    t_vec = mu_b - scale * r @ mu_a
    aligned_u = scale * r @ A + t_vec
    err_u = np.linalg.norm(aligned_u - B, axis=0)
    print(f"[ate] 对照 Umeyama 对齐（s={scale:.4f}）：")
    print(f"[ate] Umeyama-RMS = {float(np.sqrt(np.mean(err_u**2))):.4f} m")

    ok = ate_rms < 0.3
    print(f"[ate] {'PASS' if ok else 'FAIL'}（阈值 ATE-RMS < 0.3m）")

with open(f"{WORK}/ate_meta.txt", "w") as f:
    f.write(f"{ate_rms} {ate_mean} {ate_max} {1 if ok else 0}\n")
sys.exit(0 if ok else 2)
PY
  ate_ec=$?

  # iceoryx 残留检查（幽灵端口 = 进程死后 node 目录里残留的 port_tag；
  # 服务注册文件是持久缓存，非幽灵）
  echo "=== [run $i] iceoryx 残留检查 ==="
  # 记录本次运行前已存在的幽灵（历史遗留），进程退出后新增的才算本次泄漏
  leftover=$(find /tmp/iceoryx2/nodes -name "*.port_tag" -newer "$run_dir/sim.log" 2>/dev/null | wc -l | tr -d ' ')
  echo "新增 port_tag 数：$leftover"
  if [ "$leftover" -gt 0 ]; then
    echo "[e2e][run $i] 警告：本次运行产生幽灵端口残留（进程未优雅退出），可清理："
    echo "      rm -rf /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state"
  fi

  echo "=== [run $i] 结束 (recorder=${rec_ec} ate=${ate_ec}) ==="
  [ "$rec_ec" -eq 0 ] && [ "$ate_ec" -eq 0 ] && [ "$leftover" -eq 0 ]
}

ALL_OK=1
for i in $(seq 1 "$RUNS"); do
  if ! run_one_round "$i"; then
    ALL_OK=0
  fi
done

# 汇总：逐轮明细 + mean±std（sim 噪声无种子，多轮统计为稳健性标准）
echo
echo "=========================================================="
echo "=== e2e 汇总（$RUNS 轮 × ${T}s，ATE 阈值 < 0.3m）==="
echo "=========================================================="
uv run python - "$WORK" "$RUNS" <<'PY'
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
if len(rms) == 1:
    std = 0.0
else:
    std = float(np.std(rms, ddof=1))
mean = float(np.mean(rms))
print(f"--- 汇总 ---")
print(f"ATE-RMS mean±std = {mean:.4f} ± {std:.4f} m（{len(rms)} 轮）")
print(f"最终判定：{'ALL PASS' if all_ok else 'SOME RUN FAILED'}（每轮 ATE-RMS < 0.3m）")
sys.exit(0 if all_ok else 1)
PY
SUM_EC=$?

echo
echo "=== e2e 全部结束（all_ok=${ALL_OK} summary=${SUM_EC}）==="
cleanup
[ "$ALL_OK" -eq 1 ] && [ "$SUM_EC" -eq 0 ]
