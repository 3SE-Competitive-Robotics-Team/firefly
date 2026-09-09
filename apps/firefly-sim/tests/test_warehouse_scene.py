"""仓库场景回归（需 `models/warehouse/structure.obj` 资产，见 scene.py）。

CI 无资产（`models/` 已 ignore）时按 `pytest.importorskip` 语义跳过——
资产缺失是环境问题，不是回归失败（`scene._select_scene_xml` 同样回退
`boxes` 并警告，对照该函数注释）。
"""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
import numpy as np
import pytest

_ASSET = (
    Path(__file__).resolve().parent.parent.parent.parent.parent
    / "models"
    / "warehouse"
    / "structure.obj"
)
warehouse = pytest.mark.skipif(
    not _ASSET.is_file(),
    reason=f"仓库资产缺失（{_ASSET}），回退 boxes 场景（见 scene._select_scene_xml）",
)

pytestmark = warehouse

from firefly_mujoco.env import DroneEnv
from firefly_mujoco.scene import SCENE_XML


def test_visual_mesh_does_not_collide():
    m = mujoco.MjModel.from_xml_string(SCENE_XML)
    assert m.geom_contype[0] == 0
    assert m.geom_conaffinity[0] == 0


def test_spawn_is_contact_free():
    m = mujoco.MjModel.from_xml_string(SCENE_XML)
    d = mujoco.MjData(m)
    d.qpos[:3] = [2, 0, 1]
    d.qpos[3:] = [1, 0, 0, 0]
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    mujoco.mj_collision(m, d)
    assert d.ncon == 0


@pytest.mark.skipif(
    os.environ.get("MUJOCO_GL") == "disable",
    reason="无 GL 上下文（CI 无头）时跳过渲染器依赖用例",
)
def test_hover_holds_two_seconds():
    env = DroneEnv()
    env.reset(np.array([2.0, 0.0, 1.0]), np.array([0.0, 0.0, 0.0, 1.0]))
    ref = np.array([2.0, 0.0, 1.0])
    for _ in range(400):
        env.apply_pd(ref, np.zeros(3))
        env.step()
    p = env.data.body("drone").xpos
    assert np.isfinite(env.data.qpos).all()
    assert abs(p[2] - 1.0) < 0.05
