"""从 .rrd 读取位姿实体（gt/pose、vio/odom）的原始数据。

用法（仓库根目录，rerun-sdk 是根项目 dev 依赖，直接 uv run）：
    uv run python .agents/skills/rerun/scripts/read_poses.py logs/run1.rrd
    uv run python .agents/skills/rerun/scripts/read_poses.py logs/run1.rrd vio/odom gt/pose  # 自定义实体

输出：每个实体先打一行统计，再逐行输出
    <sim_time_ns> x y z qx qy qz qw

坑（实测 2026-08）：
- chunk.entity_path 带前导 "/"，须 lstrip 再比较
- sim_time 列是 duration[ns]，须 cast(int64) 才能 to_pylist
- Transform3D:translation/quaternion 是 list<fixed_size_list<3|4>>，每行取 [0]
- 无 footer 的 rrd（viewer --save 被 SIGTERM 结束）回退线性扫描，store() 不可用
"""
import sys

import pyarrow as pa
from rerun.experimental import RrdReader

DEFAULT_ENTITIES = ("gt/pose", "vio/odom")


def read_pose(reader, store, entity):
    rows = []
    for chunk in reader.stream(store=store):
        if chunk.entity_path.lstrip("/") != entity:
            continue
        tb = chunk.to_record_batch()
        t = tb.column("sim_time").cast(pa.int64()).to_pylist()
        pos = tb.column("Transform3D:translation").to_pylist()
        quat = tb.column("Transform3D:quaternion").to_pylist()
        rows += [(tt, p[0], q[0]) for tt, p, q in zip(t, pos, quat) if p is not None]
    return sorted(rows)


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    entities = sys.argv[2:] or list(DEFAULT_ENTITIES)

    reader = RrdReader(path)
    store = reader.recordings()[0]
    for entity in entities:
        rows = read_pose(reader, store, entity)
        if not rows:
            print(f"== {entity}: 无数据", file=sys.stderr)
            continue
        t0, t1 = rows[0][0] / 1e9, rows[-1][0] / 1e9
        print(f"== {entity}: {len(rows)} 行  t={t0:.3f}~{t1:.3f}s")
        for t, p, q in rows:
            print(f"{t} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                  f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")


if __name__ == "__main__":
    main()
