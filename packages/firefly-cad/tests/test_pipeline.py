"""独立几何契约与缓存失效测试，不依赖完整场地。"""

import json

import numpy as np
import pytest

from firefly_cad.map_export import collision_grid, export_map
from firefly_cad.pipeline import Options, stage, validate_assets


def test_build_failure_publishes_terminal_manifest(tmp_path, monkeypatch):
    from firefly_cad import pipeline
    from firefly_workspace import parse_scene
    source = tmp_path / "field.stp"
    source.write_text("input")
    monkeypatch.setattr(pipeline.shutil, "which", lambda _: "blender")
    monkeypatch.setattr(pipeline.subprocess, "check_output", lambda *a, **k: "Blender test\n")
    monkeypatch.setattr(pipeline, "version", lambda _: "test")
    def fail(*args):
        raise RuntimeError("tessellation failed")
    monkeypatch.setattr(pipeline, "tessellate", fail)
    output = tmp_path / "assets"
    with pytest.raises(RuntimeError, match="tessellation failed"):
        pipeline.build(source, parse_scene("rmuc2027@1.0.0"), output,
                       output / "map.ffmap", Options(), "blender")
    manifest = json.loads((output / "asset_manifest.json").read_text())
    assert manifest["status"] == "failed"
    assert manifest["reason"] == "tessellation failed"
    assert manifest["source_sha256"] == pipeline.digest(source)


def test_map_preserves_negative_origin_gap_and_volume(tmp_path):
    report = {"res": 0.5, "boxes": [[-2., 1., 0., 0.5, 1., 0.5], [2., 1., 0., 0.5, 1., 0.5]]}
    path = tmp_path / "map.ffmap"
    result = export_map(report, path)
    centers = np.loadtxt(path, skiprows=5)
    assert result["occupied_voxels"] == 32
    assert result["occupied_volume_m3"] == 4.0
    assert not np.any(np.abs(centers[:, 0]) < 1.5)
    assert np.allclose(centers.min(axis=0), [-2.25, 0.25, -0.25])
    assert np.allclose(centers.max(axis=0), [2.25, 1.75, 0.25])


@pytest.mark.parametrize("report", [
    {"res": 0, "boxes": [[0, 0, 0, 1, 1, 1]]},
    {"res": 0.5, "boxes": []},
    {"res": 0.5, "boxes": [[0, 0, 0, -1, 1, 1]]},
    {"res": 0.5, "boxes": [[0, 0, 0, 1, 1, 1], [0, 0, 0, 1, 1, 1]]},
    {"res": 0.5, "boxes": [[0, 0, 0, 1, 1, 1], [3.1, 0, 0, 1, 1, 1]]},
])
def test_map_rejects_invalid_or_ambiguous_geometry(report):
    with pytest.raises(ValueError):
        collision_grid(report)


def test_stage_checks_inputs_and_output_content_and_failure(tmp_path):
    output = tmp_path / "result"
    cache = tmp_path / "cache"
    calls = []
    def action():
        calls.append(1)
        output.write_text("correct")
    stage("test", cache, {"version": 1}, [output], action)
    stage("test", cache, {"version": 1}, [output], action)
    assert len(calls) == 1
    output.write_text("corrupt")
    stage("test", cache, {"version": 1}, [output], action)
    stage("test", cache, {"version": 2}, [output], action)
    assert len(calls) == 3
    def fail():
        raise RuntimeError("stage failed")
    with pytest.raises(RuntimeError):
        stage("test", cache, {"version": 3}, [output], fail)
    assert json.loads((cache / "test.json").read_text())["inputs"] == {"version": 2}


def test_validation_rejects_visual_coordinate_drift(tmp_path):
    import trimesh
    mesh = trimesh.creation.box(extents=[1, 1, 1])
    raw, visual, collision, ffmap = [tmp_path / p for p in ["raw.npz", "field.glb", "collision.json", "map.ffmap"]]
    np.savez(raw, vertices=mesh.vertices)
    mesh.export(visual)
    report = {"res": 0.5, "boxes": [[0, 0, 0, 0.5, 0.5, 0.5]]}
    collision.write_text(json.dumps(report))
    export_map(report, ffmap)
    assert validate_assets(raw, visual, collision, ffmap)["geometry_consistency"] == "passed"
    ffmap.write_text(ffmap.read_text().replace("RESOLUTION 0.5", "RESOLUTION 0.25"))
    with pytest.raises(ValueError, match="resolutions differ"):
        validate_assets(raw, visual, collision, ffmap)
    export_map(report, ffmap)
    mesh.apply_translation([0, 1, 0])
    mesh.export(visual)
    with pytest.raises(ValueError, match="GLB coordinate mismatch"):
        validate_assets(raw, visual, collision, ffmap)


def test_options_reject_nan():
    with pytest.raises(ValueError):
        Options(resolution_m=float("nan")).validate()
