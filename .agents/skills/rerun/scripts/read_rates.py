"""从 .rrd 统计实体更新率（间隔分布），用于验证"实时层刷新够不够快"。

用法（仓库根目录）：
    uv run python .agents/skills/rerun/scripts/read_rates.py logs/run1.rrd plan/perceived gt/pose

输出每个实体的点数、时间跨度，以及间隔的中位/均值/p90/最大和 ≤150ms、>1s 占比。
"是否以目标频率在刷新"只看中位间隔；长尾（>1s）通常代表该实体本就无变化。
"""
import sys

import numpy as np
import pyarrow as pa
from rerun.chunk import RrdReader

DEFAULT_ENTITIES = ("gt/pose", "vio/odom")


def rates(reader, store, entity):
    ts = []
    for chunk in reader.stream(store=store):
        if chunk.entity_path.lstrip("/") != entity:
            continue
        tb = chunk.to_record_batch()
        if "sim_time" not in tb.column_names:
            continue
        ts += [t for t in tb.column("sim_time").cast(pa.int64()).to_pylist() if t is not None]
    return np.sort(np.unique(np.array(ts) / 1e9)) if ts else np.empty(0)


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    reader = RrdReader(sys.argv[1])
    store = reader.recordings()[0]
    for entity in sys.argv[2:] or list(DEFAULT_ENTITIES):
        ts = rates(reader, store, entity)
        if ts.size < 2:
            print(f"== {entity}: 数据不足（{ts.size} 点）")
            continue
        d = np.diff(ts)
        print(f"== {entity}: {ts.size} 点  t={ts[0]:.2f}~{ts[-1]:.2f}s")
        print(
            f"   间隔 中位 {np.median(d) * 1e3:.0f}ms 均值 {d.mean() * 1e3:.0f}ms "
            f"p90 {np.percentile(d, 90) * 1e3:.0f}ms 最大 {d.max() * 1e3:.0f}ms"
        )
        print(f"   ≤150ms {100 * np.mean(d <= 0.15):.0f}%   >1s {100 * np.mean(d > 1.0):.0f}%")


if __name__ == "__main__":
    main()
