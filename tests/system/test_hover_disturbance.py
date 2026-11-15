"""RMUC 十轮水平外力恢复验收；需要图形会话与场景资产。

uv run pytest tests/system/test_hover_disturbance.py -q
"""
from firefly_evaluation.disturbance import run_disturbance
from firefly_evaluation.disturbance_metrics import DisturbanceLimits
from firefly_evaluation.hover_metrics import HoverLimits
from firefly_evaluation.mission import Options


def test_hover_disturbance(flight_environment):
    # 恢复半径必须小于要求的真实偏离量，不能把扰动全程都在容差内记作恢复。
    options = Options(hover_xy_error_m=.2)
    limits, hover_limits = DisturbanceLimits(), HoverLimits()
    result, _ = run_disturbance(options, limits, hover_limits)
    if result.get("interrupted"):
        raise KeyboardInterrupt
    assert result["status"] == "passed", result
