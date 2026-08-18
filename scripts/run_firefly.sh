#!/usr/bin/env bash
# 一键启动 MuJoCo 双语言闭环（对照 AGENTS.md「运行」一节）。
#
# 用法：
#   scripts/run_firefly.sh [--no-viewer] [--save /path/task.rrd]
#
# 默认：起 rerun viewer（多进程共享 recording），随后按序后台拉起
#   sim → vio → firefly-demo。Ctrl-C 结束闭环并清理进程。
#
# 约定：每个 log 写到仓库根目录下 firefly-<名>.log。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

NO_VIEWER=0
SAVE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-viewer) NO_VIEWER=1 ;;
    --save) SAVE="$2"; shift ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
  shift
done

cleanup() {
  pkill -f "target/debug/firefly-demo" 2>/dev/null || true
  pkill -f "target/debug/vio" 2>/dev/null || true
  pkill -f "firefly-sim" 2>/dev/null || true
  pkill -f "[r]erun" 2>/dev/null || true
  echo "已清理闭环进程。"
}
trap cleanup EXIT INT TERM

echo "==> 清理旧进程/iceoryx ..."
pkill -f "target/debug/firefly-demo" 2>/dev/null || true
pkill -f "target/debug/vio" 2>/dev/null || true
pkill -f "firefly-sim" 2>/dev/null || true
pkill -f "[r]erun" 2>/dev/null || true
rm -rf /tmp/iceoryx2

if [[ $NO_VIEWER -eq 0 ]]; then
  if [[ -n "$SAVE" ]]; then
    echo "==> 启动 rerun viewer（保存到 $SAVE）..."
    rerun --save "$SAVE" > firefly-rerun.log 2>&1 &
  else
    echo "==> 启动 rerun viewer ..."
    rerun > firefly-rerun.log 2>&1 &
  fi
  sleep 3
fi

echo "==> 编译 vio/firefly-demo ..."
cargo build -p vio -p firefly-demo > /dev/null

echo "==> 启动 sim → vio → demo（日志: firefly-{sim,vio,demo}.log）..."
( uv run firefly-sim > firefly-sim.log 2>&1 ) &
( ./target/debug/vio > firefly-vio.log 2>&1 ) &
( env RUST_LOG=info ./target/debug/firefly-demo > firefly-demo.log 2>&1 ) &

echo "==> 闭环已启动。Ctrl-C 停止。查看："
echo "    viewer : http://127.0.0.1:9090  （状态: tail -f firefly-rerun.log）"
echo "    sim    : tail -f firefly-sim.log"
echo "    vio    : tail -f firefly-vio.log"
echo "    demo   : tail -f firefly-demo.log"
wait
