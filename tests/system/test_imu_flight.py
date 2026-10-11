"""独立姿态链路的 release 起降、持续悬停与视觉失联验收；原始证据只写 RRD。

uv run pytest tests/system/test_imu_flight.py -q -k imu_hover
需要图形会话、RMUC 资产和 release vio/fc/render/ffctl；运行前须停止其他闭环进程。
"""
from dataclasses import asdict

import pytest

from mission import Mission, Options
from hover_metrics import HoverLimits, score_hover


@pytest.mark.parametrize("case", ["imu_landing", "imu_vio_loss", "imu_hover"])
def test_imu_flight(case, flight_directory, flight_report):
    directory = flight_directory
    directory.mkdir(parents=True)
    options = Options()
    options.validate()
    mission = Mission(directory, case, options)
    phases = [("initialization", mission.initialized), ("takeoff", mission.takeoff), ("hover", mission.hover)]
    limits = HoverLimits()
    if case == "imu_hover":
        def sustained_hover():
            end = mission.t + limits.seconds
            mission.wait(lambda: mission.t >= end, sim=limits.seconds + 1.)
            return {"seconds": limits.seconds}
        phases.append(("sustained_hover", sustained_hover))
    phases.extend([("landing", mission.landing)] if case != "imu_vio_loss" else [
        ("estimator_fallback", mission.estimator_fallback), ("degraded_support", mission.degraded_support),
    ])
    result = mission.run_phases(phases, ("render", "vio", "fc"))
    if case == "imu_hover":
        score_hover(result, options, limits)
    if case == "imu_vio_loss":
        result["capability_limit"] = "Attitude support only; no guaranteed position hold, altitude hold or landing."
    flight_report(result, options, **({"hover_limits": asdict(limits)} if case == "imu_hover" else {}))
