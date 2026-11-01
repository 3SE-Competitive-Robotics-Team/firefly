"""Numerical/schema tests for the offline exporter; no viewer is started."""

import copy
import importlib.util
from pathlib import Path
import unittest

import numpy as np

SPEC = importlib.util.spec_from_file_location(
    "export_rrd", Path(__file__).parents[1] / "export_rrd.py"
)
EXPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXPORT)


def pose(x):
    return {"translation": [x, 0, 0], "rotation_xyzw": [0, 0, 0, 1]}


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.problem = {
            "format": "rootslam-global-ba-numerical-problem", "version": 1, "revision": 7,
            "keyframes": [{"map_key": 10, "pose": pose(0)}, {"map_key": 20, "pose": pose(1)}],
            "landmarks": [{"map_key": 3, "position": [2, 3, 4]}],
            "edges": [{"from": 10, "to": 20, "kind": "loop"}],
        }
        self.replay = {"revision": 7, "poses": [[20, pose(2)]], "points": [[3, [3, 4, 5]]]}

    def test_updates_preserve_gauge_and_do_not_mutate_input(self):
        original = copy.deepcopy(self.problem)
        before, after = EXPORT.snapshots(self.problem, self.replay)
        np.testing.assert_array_equal(after[0][10][0], [0, 0, 0])
        np.testing.assert_array_equal(before[0][20][0], [1, 0, 0])
        np.testing.assert_array_equal(after[0][20][0], [2, 0, 0])
        np.testing.assert_array_equal(after[1][3], [3, 4, 5])
        self.assertEqual(original, self.problem)

    def test_invalid_provenance_or_geometry_is_rejected(self):
        for change in (
            lambda p, r: r.update(revision=8),
            lambda p, r: r["poses"].append([99, pose(0)]),
            lambda p, r: r["poses"].append([20, pose(3)]),
            lambda p, r: p["edges"][0].update(to=99),
            lambda p, r: p["landmarks"][0].update(position=[float("nan"), 0, 0]),
            lambda p, r: r["poses"][0][1].update(rotation_xyzw=[0, 0, 0, 2]),
        ):
            problem, replay = copy.deepcopy(self.problem), copy.deepcopy(self.replay)
            change(problem, replay)
            with self.assertRaises(ValueError):
                EXPORT.snapshots(problem, replay)

    def test_collection_is_one_snapshot_row_and_edges_use_updated_positions(self):
        _, state = EXPORT.snapshots(self.problem, self.replay)
        chunks = {c.entity_path: c for c in EXPORT.scene_chunks("optimized", state, self.problem["edges"])}
        for chunk in chunks.values():
            self.assertTrue(chunk.is_static)
            self.assertEqual(chunk.num_rows, 1)
        edges = chunks["/rootslam/optimized/edges/loop"].to_record_batch()
        self.assertEqual(edges.column("LineStrips3D:strips").to_pylist(),
                         [[[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]]])
        axes = chunks["/rootslam/optimized/body_axes/0"].to_record_batch()
        axis = axes.column("LineStrips3D:strips").to_pylist()[0][0]
        np.testing.assert_allclose(axis, [[0, 0, 0], [0.1, 0, 0]], rtol=1e-7)

    def test_full_trajectory_requires_same_keyframe_poses(self):
        _, state = EXPORT.snapshots(self.problem, self.replay)
        rows = ["0.000000010 0 0 0 0 0 0 1", "0.000000015 1 0 0 0 0 0 1",
                "0.000000020 2 0 0 0 0 0 -1"]
        self.assertEqual(len(EXPORT.trajectory_rows(rows, state[0])), 3)
        for invalid in (rows[:-1], rows[::-1], rows + rows[-1:],
                        rows[:-1] + ["0.000000020 2.001 0 0 0 0 0 1"]):
            with self.assertRaises(ValueError):
                EXPORT.trajectory_rows(invalid, state[0])


if __name__ == "__main__":
    unittest.main()
