"""离线建库必须拒绝跨时刻标签，并隔离不可飞相机位置。"""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from firefly_mujoco import GrayImageMessage, DepthImageMessage

spec = importlib.util.spec_from_file_location("capture", Path(__file__).resolve().parents[2] / "scripts/collect_vision_map.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def test_same_stamp_required():
    left, depth = GrayImageMessage(), DepthImageMessage()
    left.width = depth.width = 320
    left.height = depth.height = 240
    left.timestamp = 1.
    depth.timestamp = 1.1
    with pytest.raises(ValueError, match="timestamp mismatch"):
        capture.paired_frame(left, depth, 1.)
    depth.timestamp = 1.
    image, distance, quality = capture.paired_frame(left, depth, 1.)
    assert image.shape == distance.shape == (240, 320)
    assert quality["valid_depth_fraction"] == 0
    left.width = 319
    with pytest.raises(ValueError, match="dimensions"):
        capture.paired_frame(left, depth, 1.)


def test_grid_excludes_collision_and_has_unit_quaternions():
    boxes = np.array([[0., 0., 0., 1., 1., 1.]])
    poses = list(capture.grid_poses(boxes, 1., [0., 2.], [0., 90.], [-2., 2.], [-2., 2.]))
    assert poses
    for position, quaternion in poses:
        assert not np.all(np.abs(position) <= [1.15, 1.15, 1.15])
        assert np.linalg.norm(quaternion) == pytest.approx(1.)


def test_binary_frame_contract(tmp_path):
    import struct
    path = tmp_path / "frame.bin"
    capture.write_frame(path, np.zeros((240, 320)), np.ones((240, 320)), [1., 2., 3.], [0., 0., 0., 1.], 5.)
    data = path.read_bytes()
    assert len(data) == 16 + 320 * 240 * 5 + 64
    assert struct.unpack("<3d4dd", data[-64:]) == (1., 2., 3., 0., 0., 0., 1., 5.)


def test_capture_cache_rejects_missing_modified_or_unlisted_frames(tmp_path):
    frame = tmp_path / "frame_00000.bin"
    frame.write_bytes(b"pixels and depth")
    assets = {"renderer": "version1"}
    manifest = {"status": "complete", "requested": 1, "assets": assets,
                "frames": [{"index": 0, "accepted": True, "sha256": capture.digest(frame)}]}
    path = tmp_path / "capture_manifest.json"
    path.write_text(json.dumps(manifest))
    assert capture.reusable_capture(tmp_path, assets)
    assert not capture.reusable_capture(tmp_path, {"renderer": "version2"})
    frame.write_bytes(b"corrupted pixels")
    assert not capture.reusable_capture(tmp_path, assets)
    frame.unlink()
    assert not capture.reusable_capture(tmp_path, assets)
    frame.write_bytes(b"pixels and depth")
    (tmp_path / "frame_00001.bin").write_bytes(b"unexpected")
    assert not capture.reusable_capture(tmp_path, assets)
    manifest["frames"] = [{"index": 0, "accepted": False}]
    path.write_text(json.dumps(manifest))
    assert not capture.reusable_capture(tmp_path, assets)


def test_capture_retry_preserves_incomplete_attempt(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[2] / "scripts"))
    from prepare_rmuc import capture_directory
    assets = {"renderer": "version1"}
    first = capture_directory(tmp_path, assets)
    first.mkdir()
    frame = first / "frame_00000.bin"
    frame.write_bytes(b"unfinished")
    second = capture_directory(tmp_path, assets)
    assert second != first and not second.exists()
    assert frame.read_bytes() == b"unfinished"
    manifest = {"status": "complete", "requested": 1, "assets": assets,
                "frames": [{"index": 0, "accepted": True, "sha256": capture.digest(frame)}]}
    (first / "capture_manifest.json").write_text(json.dumps(manifest))
    assert capture_directory(tmp_path, assets) == first
    assert capture_directory(tmp_path, {"renderer": "version2"}) != first
