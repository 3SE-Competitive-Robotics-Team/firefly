"""Export RootSLAM numerical snapshots to Rerun, without starting a viewer.

Run with workspace dependencies: uv run python apps/rootslam/export_rrd.py --help.
This is an offline comparison of one frozen problem and its replay, not a live
SLAM recording. Images and online loop-detection events are not in these files.
"""

from __future__ import annotations

import argparse
from decimal import Decimal
import json
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from rerun.chunk import Chunk, LazyChunkStream


EDGE_COLORS = {
    "temporal": [140, 140, 150],
    "covisibility": [80, 140, 210],
    "spanning_tree": [230, 180, 60],
    "loop": [240, 70, 130],
}


def finite_vector(value, size):
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (size,) or not np.isfinite(vector).all():
        raise ValueError(f"Expected a finite {size}-component vector")
    return vector


def pose(value):
    translation = finite_vector(value["translation"], 3)
    q = finite_vector(value["rotation_xyzw"], 4)
    if abs(np.linalg.norm(q) - 1.0) > 1e-6:
        raise ValueError("Pose quaternion is not unit length")
    return translation, q


def unique_pairs(values, label):
    result = dict(values)
    if len(result) != len(values):
        raise ValueError(f"Duplicate {label} IDs")
    return result


def snapshots(problem, replay):
    """Validate provenance and reconstruct updates without changing the inputs."""
    if problem["format"] != "rootslam-global-ba-numerical-problem" or problem["version"] != 1:
        raise ValueError("Unsupported RootSLAM numerical problem format")
    if problem["revision"] != replay["revision"]:
        raise ValueError("Problem and replay revisions differ")
    poses = unique_pairs([(f["map_key"], pose(f["pose"])) for f in problem["keyframes"]], "pose")
    points = unique_pairs(
        [(p["map_key"], finite_vector(p["position"], 3)) for p in problem["landmarks"]], "point"
    )
    updates = unique_pairs([(i, pose(p)) for i, p in replay["poses"]], "pose update")
    point_updates = unique_pairs(
        [(i, finite_vector(p, 3)) for i, p in replay["points"]], "point update"
    )
    if not updates.keys() <= poses.keys() or not point_updates.keys() <= points.keys():
        raise ValueError("Replay contains an update outside the frozen problem")
    for edge in problem["edges"]:
        if edge["from"] not in poses or edge["to"] not in poses:
            raise ValueError("Graph edge references a missing keyframe")
        if edge["kind"] not in EDGE_COLORS:
            raise ValueError("Unknown graph edge kind")
    return (poses, points), (poses | updates, points | point_updates)


def static_chunk(entity, columns, instances):
    # This custom JSON schema has no Rerun reader. Each collection is one
    # immutable snapshot row, not one static row per landmark.
    return Chunk.from_columns(entity, [], columns.partition([instances]))


def trajectory_rows(lines, optimized_poses):
    """Require the full TUM trajectory to agree with every retained keyframe."""
    result = {}
    previous = None
    for line in lines:
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError("TUM rows must have timestamp, xyz and quaternion xyzw")
        timestamp = Decimal(fields[0]) * 1_000_000_000
        if not timestamp.is_finite() or timestamp != timestamp.to_integral_value():
            raise ValueError("TUM timestamp must be finite and exact to nanoseconds")
        timestamp = int(timestamp)
        if previous is not None and timestamp <= previous:
            raise ValueError("TUM timestamps must be strictly increasing")
        previous = timestamp
        result[timestamp] = pose({"translation": fields[1:4], "rotation_xyzw": fields[4:8]})
    for timestamp, (translation, quaternion) in optimized_poses.items():
        if timestamp not in result:
            raise ValueError("TUM trajectory is missing a retained keyframe")
        actual_translation, actual_quaternion = result[timestamp]
        # 1e-8 accommodates the original TUM writer's decimal serialization,
        # not differences between independent runs or optimizations.
        if (np.max(np.abs(actual_translation - translation)) > 1e-8 or
                min(np.max(np.abs(actual_quaternion - quaternion)),
                    np.max(np.abs(actual_quaternion + quaternion))) > 1e-8):
            raise ValueError("TUM trajectory and global result keyframe poses disagree")
    return result


def scene_chunks(name, state, edges):
    poses, points = state
    root = f"/rootslam/{name}"
    ids = sorted(poses)
    if points:
        point_ids = sorted(points)
        yield static_chunk(
            root + "/landmarks",
            rr.Points3D.columns(positions=[points[i] for i in point_ids],
                                colors=[[100, 190, 230]] * len(point_ids)), len(point_ids)
        )
    if ids:
        translations = [poses[i][0] for i in ids]
        yield static_chunk(root + "/keyframes", rr.Points3D.columns(
            positions=translations, colors=[[255, 180, 70]] * len(ids),
            labels=[str(i) for i in ids], show_labels=[False] * len(ids)), len(ids))
        if len(ids) > 1:
            yield static_chunk(root + "/keyframe_path", rr.LineStrips3D.columns(
                strips=[translations], colors=[[255, 180, 70]]), 1)
        # Body-to-world orientation: three colored 10 cm axes per keyframe.
        axes = [[], [], []]
        for translation, q in (poses[i] for i in ids):
            x, y, z, w = q
            rotation = np.array([
                [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
            ])
            for axis in range(3):
                axes[axis].append([translation, translation + 0.1 * rotation[:, axis]])
        for axis, color in enumerate(([230, 80, 80], [80, 230, 80], [80, 80, 230])):
            yield static_chunk(root + f"/body_axes/{axis}", rr.LineStrips3D.columns(
                strips=axes[axis], colors=[color] * len(ids)), len(ids))
    for kind, color in EDGE_COLORS.items():
        strips = [[poses[e["from"]][0], poses[e["to"]][0]]
                  for e in edges if e["kind"] == kind]
        if strips:
            yield static_chunk(root + "/edges/" + kind, rr.LineStrips3D.columns(
                strips=strips, colors=[color] * len(strips)), len(strips))


def export(problem_path: Path, replay_path: Path, output: Path, trajectory_path: Path | None = None):
    with problem_path.open() as stream:
        problem = json.load(stream)
    with replay_path.open() as stream:
        replay = json.load(stream)
    before, after = snapshots(problem, replay)
    if output.exists():
        raise FileExistsError(output)
    chunks = list(scene_chunks("input", before, problem["edges"]))
    chunks.extend(scene_chunks("optimized", after, problem["edges"]))
    trajectory = None
    if trajectory_path is not None:
        with trajectory_path.open() as stream:
            trajectory = trajectory_rows(stream, after[0])
        if len(trajectory) > 1:
            chunks.append(static_chunk("/rootslam/optimized/full_trajectory",
                rr.LineStrips3D.columns(strips=[[p[0] for p in trajectory.values()]],
                                       colors=[[100, 240, 130]]), 1))
    metadata = {
        "description": "Offline frozen global BA comparison; not live SLAM history",
        "problem": str(problem_path.resolve()), "replay": str(replay_path.resolve()),
        "revision": problem["revision"], "keyframes": len(before[0]),
        "landmarks": len(before[1]), "edge_counts": {
            k: sum(e["kind"] == k for e in problem["edges"]) for k in EDGE_COLORS},
        "solve": {k: replay[k] for k in (
            "initial_cost", "final_cost", "iterations", "accepted_steps", "converged",
            "linear_iterations", "peak_linear_storage_bytes")},
        "options": replay["options"],
        "trajectory": None if trajectory is None else {
            "path": str(trajectory_path.resolve()), "frames": len(trajectory),
            "retained_keyframes_verified": len(after[0]),
        },
        "notes": ["World coordinates in meters; body-to-world pose axes are 0.1 m",
                  "Keyframe path omits non-keyframes; it is not the full camera trajectory",
                  "Graph lines connect estimated poses; they are not raw measured displacements",
                  "No images, online event times or fisheye-to-pinhole approximations are fabricated"],
    }
    chunks.append(static_chunk("/rootslam/metadata", rr.TextDocument.columns(
        text=[json.dumps(metadata, indent=2)], media_type=["text/plain"]), 1))
    blueprint = rrb.Blueprint(rrb.Vertical(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/rootslam/input", name="Frozen input · meters"),
            rrb.Spatial3DView(origin="/rootslam/optimized", name="Global BA result · meters")),
        rrb.TextDocumentView(origin="/rootslam/metadata", name="Provenance and convergence"),
        row_shares=[4, 1]), collapse_panels=True)
    recording = rr.RecordingStream("firefly-rootslam-offline")
    recording.save(output, default_blueprint=blueprint)
    rr.send_chunks(LazyChunkStream.from_iter(chunks), recording=recording)
    recording.flush()
    recording.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--problem", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, help="Optional matching final TUM trajectory")
    args = parser.parse_args()
    export(args.problem, args.replay, args.output, args.trajectory)


if __name__ == "__main__":
    main()
