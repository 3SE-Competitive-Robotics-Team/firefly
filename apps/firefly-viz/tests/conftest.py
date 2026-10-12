"""录制边界替身：成员受 Protocol 限制，方法签名受 autospec 校验。"""
from unittest.mock import create_autospec

import pytest

from firefly_viz.recording import MatchedPoints, RecordingOutputs


@pytest.fixture
def recordings_factory():
    def create(*, archive, live=None, matches=None):
        outputs = create_autospec(RecordingOutputs, instance=True, spec_set=True)
        outputs.archive = archive
        outputs.live = live
        outputs.matches = matches if matches is not None else create_autospec(
            MatchedPoints, instance=True, spec_set=True)
        return outputs
    return create
