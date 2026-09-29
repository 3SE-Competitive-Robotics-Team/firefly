"""RMUC 资产边界与物理执行器契约；几何夹具仅供单元测试。"""

import json

import numpy as np
import pytest

from firefly_mujoco import DroneEnv
from firefly_mujoco import scene


def test_scene_config_requires_rmuc(tmp_path):
    config = tmp_path / "scene.toml"
    config.write_text('scene = "rmuc2026"\n')
    assert scene.load_scene_name(config) == "rmuc2026"
    config.write_text('scene = "unsupported"\n')
    with pytest.raises(ValueError, match="仅支持场景"):
        scene.load_scene_name(config)
    config.unlink()
    with pytest.raises(FileNotFoundError):
        scene.load_scene_name(config)


def test_collision_asset_is_required(tmp_path, monkeypatch):
    monkeypatch.setattr(scene, "_RMUC_DIR", tmp_path)
    with pytest.raises(FileNotFoundError, match="RMUC 碰撞资产缺失"):
        scene.build_scene()
    (tmp_path / "rmuc2026_collision.json").write_text('{"boxes": []}')
    with pytest.raises(ValueError, match="RMUC 碰撞资产为空"):
        scene.build_scene()


def test_physics_and_imu_without_renderer(tmp_path, monkeypatch):
    monkeypatch.setattr(scene, "_RMUC_DIR", tmp_path)
    # 测试台阶顶面与停机坪同高，足够覆盖机体；不表示完整 RMUC 场地。
    (tmp_path / "rmuc2026_collision.json").write_text(json.dumps({
        "boxes": [[-13.0, 0.0, 0.1875, 0.5, 0.5, 0.1875]],
    }))
    env = DroneEnv(gyro_noise=0.0, accel_noise=0.0)
    np.testing.assert_allclose(env.state()[0], scene.drone_pad())
    assert env.data.ncon == 0
    assert env.model.nu == 4
    assert env.model.ncam == 0
    assert env.mass == pytest.approx(0.219)
    assert np.all(env.inertia_body() > 0)
    env.apply_motor_thrusts(np.zeros(4))
    for _ in range(400):
        env.step()
    gyro, accel = env.imu()
    np.testing.assert_allclose(gyro, 0.0, atol=1e-5)
    np.testing.assert_allclose(accel, [0.0, 0.0, 9.81], atol=1e-3)
    assert env.state()[0][2] > scene.PAD[2]
    env.apply_motor_thrusts(np.full(4, env.airframe()["max_thrust_per_motor"]))
    for _ in range(40):
        env.step()
    assert env.state()[1][2] > 0.5
