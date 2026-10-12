"""RMUC 资产边界与物理执行器契约；几何夹具仅供单元测试。"""

import json
from functools import partial

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
    assert scene.load_scene_name(config) == scene.SCENE_NAME


def test_collision_asset_is_required(tmp_path):
    with pytest.raises(FileNotFoundError, match="RMUC 碰撞资产缺失"):
        scene.build_scene(asset_dir=tmp_path)
    (tmp_path / "rmuc2026_collision.json").write_text('{"boxes": []}')
    with pytest.raises(ValueError, match="RMUC 碰撞资产为空"):
        scene.build_scene(asset_dir=tmp_path)


@pytest.mark.parametrize("rate", [200, 1000])
def test_physics_and_imu_without_renderer(tmp_path, rate):
    # 测试台阶顶面与停机坪同高，足够覆盖机体；不表示完整 RMUC 场地。
    (tmp_path / "rmuc2026_collision.json").write_text(json.dumps({
        "boxes": [[-13.0, 0.0, 0.1875, 0.5, 0.5, 0.1875]],
    }))
    env = DroneEnv(scene_factory=partial(scene.build_scene, asset_dir=tmp_path), timestep=1.0 / rate, gyro_density=0.0, accel_density=0.0)
    np.testing.assert_allclose(env.state()[0], scene.drone_pad())
    assert env.data.ncon == 0
    assert env.model.nu == 4
    assert env.model.ncam == 0
    assert env.mass == pytest.approx(0.219)
    assert np.all(env.inertia_body() > 0)
    env.apply_motor_thrusts(np.zeros(4))
    for _ in range(2 * rate):
        env.step()
    gyro, accel = env.imu()
    np.testing.assert_allclose(gyro, 0.0, atol=1e-5)
    np.testing.assert_allclose(accel, [0.0, 0.0, 9.81], atol=1e-3)
    assert env.state()[0][2] > scene.PAD[2]
    env.apply_motor_thrusts(np.full(4, env.airframe()["max_thrust_per_motor"]))
    for _ in range(round(0.2 * rate)):
        env.step()
    assert env.state()[1][2] > 0.5


def test_sensor_snapshot_uses_integrated_state(tmp_path):
    (tmp_path / "rmuc2026_collision.json").write_text(json.dumps({
        "boxes": [[-13.0, 0.0, 0.1875, 0.5, 0.5, 0.1875]],
    }))
    env = DroneEnv(scene_factory=partial(scene.build_scene, asset_dir=tmp_path), gyro_density=0.0, accel_density=0.0)
    origin = np.array([0.0, 0.0, 5.0])
    env.reset(origin, np.array([0.0, 0.0, 0.0, 1.0]))
    env.data.qvel[:3] = [1.0, 0.0, 0.0]
    env.data.qvel[3:6] = [0.0, 0.0, 0.4]
    for _ in range(3):
        env.step()
        position, velocity, quaternion, rate = env.state()
        np.testing.assert_allclose(position, env.data.qpos[:3], atol=1e-12)
        np.testing.assert_allclose(quaternion, env.data.qpos[[4, 5, 6, 3]], atol=1e-12)
        np.testing.assert_allclose(position[:2], [env.time, 0.0], atol=1e-12)
        np.testing.assert_allclose(velocity[2], -9.81 * env.time, atol=1e-12)
        gyro, accel = env.imu()
        np.testing.assert_allclose(gyro, rate, atol=1e-12)
        np.testing.assert_allclose(accel, 0.0, atol=1e-10)
        gt_position, gt_quaternion, gt_velocity = env.gt_pose()
        np.testing.assert_array_equal(gt_position, position)
        np.testing.assert_array_equal(gt_quaternion, quaternion)
        np.testing.assert_array_equal(gt_velocity, velocity)


@pytest.mark.parametrize("imu_rate", [100, 1000])
def test_imu_noise_preserves_continuous_density(tmp_path, imu_rate):
    (tmp_path / "rmuc2026_collision.json").write_text(json.dumps({
        "boxes": [[-13.0, 0.0, 0.1875, 0.5, 0.5, 0.1875]],
    }))
    env = DroneEnv(scene_factory=partial(scene.build_scene, asset_dir=tmp_path),
                   imu_period=1.0 / imu_rate, rng=np.random.default_rng(20260930))
    samples = np.array([np.concatenate(env.imu()) for _ in range(20000)])
    variance = samples.var(axis=0, ddof=1)
    # 与 VIO 的 imu_density_preserves_calibrated_integral_variance_across_rates
    # 使用相同的标定积分方差契约；部署 TOML 只包含差异覆盖。
    density_squared = np.array([4e-8] * 3 + [4e-6] * 3)
    # 由实际噪声样本估计每秒积分方差：N·Var(sample)·Δt²。
    np.testing.assert_allclose(variance / imu_rate, density_squared, rtol=0.03)
