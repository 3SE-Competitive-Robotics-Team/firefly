"""真实 RMUC 资产的 provenance 与物理接触验收；须显式启用。"""
import hashlib
import json
import os
from pathlib import Path

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.skipif(os.environ.get("FIREFLY_RUN_RMUC_ASSETS") != "1", reason="requires built RMUC assets")


def test_asset_provenance():
    manifest = json.loads((ROOT / "models/rmuc2026/asset_manifest.json").read_text())
    assert manifest["status"] == "passed"
    for name, expected in manifest["artifacts"].items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == expected, name


def test_spawn_contact_and_passive_rest():
    from firefly_mujoco.scene import build_scene, drone_pad
    model = mujoco.MjModel.from_xml_string(build_scene())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert all(data.contact[i].dist >= -0.002 for i in range(data.ncon)), "spawn penetrates field"
    start = np.array(drone_pad())
    for _ in range(400):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert np.linalg.norm(data.qpos[:2] - start[:2]) < 0.02
    assert abs(data.qpos[2] - start[2]) < 0.05, "pad surface inconsistent with collision asset"
    assert np.linalg.norm(data.qvel) < 0.02
