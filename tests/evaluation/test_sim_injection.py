"""验收入口显式选择环境与发布策略，不启动 IPC 或物理循环。"""
from unittest.mock import create_autospec

import pytest

from firefly_evaluation import disturbance_sim, mission_sim
from firefly_mujoco import DroneEnv, build_scene, load_scene_name
from firefly_sim.main import decode_config, main as run_sim


@pytest.mark.parametrize(("entry", "environment"), [
    (mission_sim.main, mission_sim.ObservedEnv),
    (disturbance_sim.main, disturbance_sim.DisturbedEnv),
])
def test_acceptance_injects_environment_and_disables_plant_state(entry, environment):
    runner = create_autospec(run_sim, spec_set=True)
    entry(runner=runner)
    runner.assert_called_once_with(["--no-trace"], env_factory=environment, publish_plant_state=False)


@pytest.mark.parametrize("scene_name", ["rmuc2027@1.0.0"])
def test_sim_uses_injected_factory_with_configured_rates_and_scene(tmp_path, scene_name):
    class FactoryReached(Exception):
        pass

    config = tmp_path / "sim.toml"
    config.write_text("[rates]\nphysics = 200.0\nimu = 100.0")
    factory = create_autospec(DroneEnv, spec_set=True, side_effect=FactoryReached)
    scene_loader = create_autospec(load_scene_name, spec_set=True, return_value=scene_name)
    with pytest.raises(FactoryReached):
        run_sim(["--no-trace", "--config", str(config)], env_factory=factory,
                publish_plant_state=False, scene_loader=scene_loader)
    scene_loader.assert_called_once_with()
    factory.assert_called_once()
    args = factory.call_args.kwargs
    assert set(args) == {"timestep", "imu_period", "scene_factory", "rng"}
    assert args["timestep"] == .005
    assert args["imu_period"] == .01
    assert args["scene_factory"].func is build_scene
    assert args["scene_factory"].args == ()
    assert args["scene_factory"].keywords == {"name": scene_name}


@pytest.mark.parametrize("config", [
    {"ratess": {}}, {"rates": {"physic": 1000.}}, {"rates": 1000.},
    {"start": [1., 2.]}, {"start": [0., 0., float("nan")]},
])
def test_invalid_sim_config_fails_before_environment_creation(config):
    with pytest.raises(ValueError):
        decode_config(config)


@pytest.mark.parametrize("contents", ['scnee = "rmuc2027@1.0.0"', 'scene = 2027'])
def test_scene_config_rejects_unknown_keys_and_invalid_types(tmp_path, contents):
    path = tmp_path / "scene.toml"
    path.write_text(contents)
    with pytest.raises(ValueError):
        load_scene_name(path)
