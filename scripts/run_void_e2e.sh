#!/usr/bin/env bash
# firefly-void 三进程 e2e 闭环验证：sim（MuJoCo 物理）→ void（DIVO 里程计）。
#
# 用法：scripts/run_void_e2e.sh [秒数]   （缺省 60s）
#
# 流程：
#   1. 后台起 `uv run firefly-sim --no-trace`（物理 + 传感器发布）
#   2. 后台起 `cargo run -p void`（估计，发布 Firefly/VoidOdom）
#   3. Python recorder 同步采集 GT 与 VoidOdom 到 /tmp（期间两进程在跑）
#   4. 到点优雅 kill（SIGINT → 端口 Drop，无 iceoryx 幽灵残留）
#   5. 离线对齐（Umeyama 相似变换）算 ATE，输出 RMS/mean/max 与健康统计
#
# 退出码：0 = 通过（ATE-RMS < 0.3m）；1 = 失败。

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
T=${1:-60}
WORK=/tmp/void_e2e
mkdir -p "$WORK"

cd "$ROOT"

# 清理可能的历史残留（进程全死后才安全；iceoryx2 服务注册文件是持久
# 缓存可复用，幽灵端口 = node 目录里的 port_tag 残留）
rm -rf /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state 2>/dev/null
pkill -9 -f "firefly-sim --no-trace" 2>/dev/null
pkill -9 -f "target/release/void" 2>/dev/null
sleep 1
rm -rf /tmp/iceoryx2/services /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state 2>/dev/null

echo "[e2e] 启动 sim（--no-trace，${T}s 闭环）"
nohup uv run firefly-sim --no-trace > "$WORK/sim.log" 2>&1 &
SIM_PID=$!

# 等待 sim 就绪（话题服务创建）
for _ in $(seq 1 50); do
  grep -q "已就绪" "$WORK/sim.log" && break
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "[e2e] sim 启动失败，日志尾部："; tail -5 "$WORK/sim.log"; exit 1
  fi
  sleep 0.2
done

echo "[e2e] 启动 void"
# 等 sim 跑 3s（PD 启动瞬态结束）再起 void：瞬态中纯 IMU 传播
# + 测量未建立会累积初始漂移（实测 0-2s 偏差 ~0.2m 直接进 ATE）
sleep 3
nohup env RUST_LOG=info cargo run --release -p void > "$WORK/void.log" 2>&1 &
VOID_PID=$!

# recorder：前台同步采集 T 秒（sim/void 在后台跑）
echo "[e2e] 采集 ${T}s（recorder 前台，sim/void 后台）"
uv run python - "$T" "$WORK" <<'PY'
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


if __name__ == "__main__":
    main()
PY
REC_EC=$?

# 优雅终止（SIGINT → WaitSet / Python KeyboardInterrupt 捕获 → 端口 Drop）
# 注意：SIM_PID 是 `uv run` 包装进程，SIGINT 不转发给 Python——用
# pkill -f 匹配实际进程（void 的 WaitSet 捕获 SIGINT 优雅退出）
echo "[e2e] 停止 sim/void（SIGINT 优雅退出）"
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
echo "=== void 日志尾部 ==="
tail -8 "$WORK/void.log"
echo "=== sim 日志尾部 ==="
tail -5 "$WORK/sim.log"

# ATE 计算（Umeyama 相似变换对齐）
echo "=== ATE 计算 ==="
uv run python - "$WORK" <<'PY'
"""离线对齐 GT 与 VoidOdom（Umeyama 相似变换），输出 ATE 统计。"""
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
t0 = max(gt_t.min(), odom_t.min()) + 0.1
t1 = min(gt_t.max(), odom_t.max()) - 0.1
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

# 轨迹退化判断（悬停/短轨迹）：GT 与估计都近静止时 Umeyama 的尺度
# 无定义（aa≈0 → scale=0 → ATE 恒 0）。此时估计与 GT 同起点（t0 初始化，
# 全局系一致），直接算逐点绝对误差。
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
    sys.exit(0 if ok else 2)

# Umeyama 相似变换：s·R·odom + t ≈ gt（吸收初始姿态/尺度误差）
A = odom_w.T  # 3×N
B = gt_w.T  # 3×N
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
aligned = scale * r @ A + t_vec
err = np.linalg.norm(aligned - B, axis=0)
ate_rms = float(np.sqrt(np.mean(err**2)))
ate_mean = float(np.mean(err))
ate_max = float(np.max(err))
print(f"[ate] Umeyama 对齐（s={scale:.4f}）：")
print(f"[ate] ATE-RMS  = {ate_rms:.4f} m")
print(f"[ate] ATE-mean = {ate_mean:.4f} m")
print(f"[ate] ATE-max  = {ate_max:.4f} m")

ok = ate_rms < 0.3
print(f"[ate] {'PASS' if ok else 'FAIL'}（阈值 ATE-RMS < 0.3m）")
sys.exit(0 if ok else 2)
PY
ATE_EC=$?

# iceoryx 残留检查（幽灵端口 = 进程死后 node 目录里残留的 port_tag；
# 服务注册文件是持久缓存，非幽灵）
echo "=== iceoryx 残留检查 ==="
# 记录本次运行前已存在的幽灵（历史遗留），进程退出后新增的才算本次泄漏
LEFTOVER=$(find /tmp/iceoryx2/nodes -name "*.port_tag" -newer "$WORK/sim.log" 2>/dev/null | wc -l | tr -d ' ')
echo "新增 port_tag 数：$LEFTOVER"
if [ "$LEFTOVER" -gt 0 ]; then
  echo "[e2e] 警告：本次运行产生幽灵端口残留（进程未优雅退出），可清理："
  echo "      rm -rf /tmp/iceoryx2/nodes/private/tmp/iox2*.shm_state"
fi

echo "=== e2e 结束 (recorder=${REC_EC} ate=${ATE_EC}) ==="
[ "$REC_EC" -eq 0 ] && [ "$ATE_EC" -eq 0 ] && [ "$LEFTOVER" -eq 0 ]
