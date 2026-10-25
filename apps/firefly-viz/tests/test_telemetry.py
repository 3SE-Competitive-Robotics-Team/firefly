"""曲线显式命名，同时保持验收读取的数值数组顺序。"""

import pytest
from rerun.chunk import RrdReader

from firefly_viz.config import Options
from firefly_viz.recording import Recordings, at
from firefly_viz.telemetry import log_scalars, scalar_labels


@pytest.mark.parametrize('entity,values,fields', [
    ('fc/standby_eskf/quaternion_xyzw', [0., 0., 0., 1.], ['x', 'y', 'z', 'w']),
    ('fc/debug/position_error_odom', [.1, -.2, .3], ['x_m', 'y_m', 'z_m']),
    ('viz/queues', [2., 3., 10., 1.], ['telemetry_pending', 'media_pending', 'telemetry_high_water', 'dropped_frames']),
    ('render/capture/stages_s', [.01, .02, .03, .04], ['submit_s', 'readback_s', 'assemble_s', 'queue_s']),
    ('acceptance/imu_health', [1., 1., .15], ['attitude_available', 'vio_fresh', 'posterior_age_s']),
])
def test_recorded_series_have_semantic_names_and_unchanged_values(tmp_path, entity, values, fields):
    path = tmp_path / 'series.rrd'
    outputs = Recordings(path, None, False, Options())
    try:
        at(outputs.archive, 1.)
        log_scalars(outputs.archive, entity, values)
    finally:
        outputs.close()
    batches = [chunk.to_record_batch() for chunk in RrdReader(path).stream()
               if chunk.entity_path.lstrip('/') == entity]
    assert [row for batch in batches for row in batch.column('SeriesLines:names').to_pylist()] == [
        [f'{entity}/{field}' for field in fields]]
    assert [row for batch in batches for row in batch.column('Scalars:scalars').to_pylist()] == [values]


def test_unnamed_multichannel_diagnostics_require_a_semantic_schema():
    with pytest.raises(ValueError, match='semantic names'):
        scalar_labels('new/measurement', 3)
    assert scalar_labels('new/measurement', 1) == ['new/measurement']
