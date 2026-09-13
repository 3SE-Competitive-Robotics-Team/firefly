"""碰撞聚合测试：贪心合并 + 体素实心 + 凹结构保留（合成网格，不依赖真件）。

约定：盒集合必须覆盖且不跨空腔——两个分离物块之间不得出现桥接盒，
否则 `MuJoCo` 里会撞到空气。
"""

from __future__ import annotations

import json

import numpy as np

from firefly_cad.collide import boxes_for_mesh, main, merge_boxes


def _cube(center: tuple[float, float, float], size: float):
    import trimesh

    mesh = trimesh.creation.box(extents=(size, size, size))
    mesh.apply_translation(center)
    return mesh


def test_merge_full_block_is_single_box() -> None:
    occ = np.ones((3, 4, 5), dtype=bool)
    boxes = merge_boxes(occ)
    assert boxes.shape == (1, 6)
    assert tuple(boxes[0]) == (0, 2, 0, 3, 0, 4)


def test_merge_empty_is_empty() -> None:
    assert merge_boxes(np.zeros((2, 2, 2), dtype=bool)).shape == (0, 6)


def test_collide_two_boxes_preserve_gap() -> None:
    # 两个 1m 立方，沿 x 相隔 3m：合并盒不得跨过中间空腔。
    import trimesh

    mesh = trimesh.util.concatenate(
        [_cube((0.0, 0.0, 0.5), 1.0), _cube((4.0, 0.0, 0.5), 1.0)]
    )
    boxes = boxes_for_mesh(mesh, res=0.25)
    assert len(boxes) >= 2
    for cx, _cy, _cz, hx, _hy, _hz in boxes:
        assert cx + hx <= 2.0 or cx - hx >= 2.0, f"盒跨空腔: {boxes}"


def test_collide_solid_volume_matches() -> None:
    import trimesh

    mesh = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    boxes = boxes_for_mesh(mesh, res=0.1)
    volume = sum(8.0 * hx * hy * hz for _cx, _cy, _cz, hx, hy, hz in boxes)
    # 体素化按半格外扩，实心体积略大于真值；量级必须对。
    assert 0.9 <= volume <= 2.0, volume


def test_collide_cli_roundtrip(tmp_path) -> None:
    import trimesh

    obj = tmp_path / "cube.obj"
    trimesh.creation.box(extents=(1.0, 1.0, 0.5)).export(obj)
    out = tmp_path / "collision.json"
    assert main([str(obj), "--out", str(out), "--res", "0.2"]) == 0
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["boxes"] and payload["res"] == 0.2
