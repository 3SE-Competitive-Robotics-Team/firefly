"""仓库场景回归：视觉 mesh 只渲染不碰撞，出生点无接触、悬停保持。"""

from __future__ import annotations

import mujoco
import numpy as np

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
