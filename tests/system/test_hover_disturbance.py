"""RMUC 十轮水平外力恢复验收；需要图形会话与场景资产。

uv run pytest tests/system/test_hover_disturbance.py -q
"""
from dataclasses import asdict

from firefly_evaluation.disturbance import run_disturbance
from firefly_evaluation.disturbance_metrics import DisturbanceLimits
from firefly_evaluation.hover_metrics import HoverLimits
from firefly_evaluation.mission import Options


def test_hover_disturbance(flight_directory, flight_report):
    flight_directory.mkdir(parents=True)
    # 恢复半径必须小于要求的真实偏离量，不能把扰动全程都在容差内记作恢复。
    options = Options(hover_xy_error_m=.2)
    limits, hover_limits = DisturbanceLimits(), HoverLimits()
    result, forces = run_disturbance(flight_directory, options, limits, hover_limits)
    flight_report(result, options, disturbance_limits=asdict(limits),
                  hover_limits=asdict(hover_limits), forces_world_n=forces.tolist())
