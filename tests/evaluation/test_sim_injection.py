"""验收入口显式选择环境与发布策略，不启动 IPC 或物理循环。"""
from unittest.mock import create_autospec

import pytest

from firefly_evaluation import disturbance_sim, mission_sim
from firefly_mujoco import DroneEnv
from firefly_sim.main import main as run_sim


@pytest.mark.parametrize(("entry", "environment"), [
    (mission_sim.main, mission_sim.ObservedEnv),
    (disturbance_sim.main, disturbance_sim.DisturbedEnv),
])
def test_acceptance_injects_environment_and_disables_plant_state(entry, environment):
    runner = create_autospec(run_sim, spec_set=True)
    entry(runner=runner)
    runner.assert_called_once_with(["--no-trace"], env_factory=environment, publish_plant_state=False)


def test_sim_uses_injected_factory_with_configured_rates(tmp_path):
    class FactoryReached(Exception):
        pass

    config = tmp_path / "sim.toml"
    config.write_text("[rates]\nphysics = 200.0\nimu = 100.0")
    factory = create_autospec(DroneEnv, spec_set=True, side_effect=FactoryReached)
    with pytest.raises(FactoryReached):
        run_sim(["--no-trace", "--config", str(config)], env_factory=factory,
                publish_plant_state=False)
    factory.assert_called_once_with(timestep=.005, imu_period=.01)
