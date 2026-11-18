"""Native resource release on owner silence and fresh-session reactivation."""
import os
from pathlib import Path
import time

import iceoryx2 as iox2
from firefly_mujoco.activity import ActivityLease, LANDING
from firefly_mujoco.messages import AttitudeMessage, DownImageMessage, ScalarMessage, TraceContext
from firefly_workspace.process import ManagedProcess, isolated_run

ROOT = Path(__file__).resolve().parents[2]


def test_owner_silence_releases_inference_and_can_wake_again():
    with isolated_run(ROOT):
        recorder = ManagedProcess([str(ROOT / "target/release/mimir")], cwd=ROOT, env=os.environ.copy())
        landing = None
        try:
            recording = recorder.wait_recording()
            iox2.set_log_level(iox2.LogLevel.Error)
            node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)

            def port(topic, payload, *, capacity=2, publishers=2, publish=False):
                service = node.service_builder(iox2.ServiceName.new("Firefly/" + topic)).publish_subscribe(payload).user_header(TraceContext).subscriber_max_buffer_size(capacity).max_publishers(publishers).open_or_create()
                return service.publisher_builder().create() if publish else service.subscriber_builder().buffer_size(capacity).create()

            attitude = port("VioAttitude", AttitudeMessage, publish=True)
            images = port("CameraDown", DownImageMessage, publish=True)
            scalars = port("VizScalars", ScalarMessage, capacity=1024, publishers=8)
            lease = ActivityLease(node)
            landing = ManagedProcess([str(ROOT / "target/release/landing"), "--simulation"], cwd=ROOT, env=os.environ.copy())
            beginning = time.monotonic()
            next_image, rows = 0., []

            def run_for(seconds, renew=False):
                nonlocal next_image
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    assert landing.process.poll() is None, " | ".join(landing.tail)
                    stamp = time.monotonic() - beginning
                    sample = AttitudeMessage(timestamp=stamp, posterior_time=stamp, session=7)
                    sample.quat_xyzw[3] = 1.
                    attitude.loan_uninit().write_payload(sample).send()
                    if stamp >= next_image:
                        image = DownImageMessage(timestamp=stamp, sensor_id=3, width=352, height=288)
                        images.loan_uninit().write_payload(image).send()
                        next_image = stamp + 1/15
                    if renew:
                        lease.pump(stamp)
                    while (sample := scalars.receive()) is not None:
                        message = sample.payload().contents
                        if bytes(message.entity[:message.entity_len]) == b"landing/runtime":
                            rows.append((message.timestamp, list(message.scalars)))
                    time.sleep(.01)

            run_for(.5)
            lease.set(LANDING, .5)
            run_for(2.5, renew=True)
            assert rows[-1][1][0] == 1 and rows[-1][1][1] >= 3, rows
            run_for(3.)  # The owner stops renewing; the process remains alive.
            assert rows[-1][1][0] == 0, rows
            count = rows[-1][1][1]
            run_for(1.2)
            assert rows[-1][1][0] == 0 and rows[-1][1][1] == count, rows
            lease.set(LANDING, time.monotonic() - beginning)
            run_for(2.5, renew=True)
            assert rows[-1][1][0] == 1 and rows[-1][1][1] > count, rows
            lease.set(0, time.monotonic() - beginning)
            run_for(.5)
            assert rows[-1][1][0] == 0, rows
        finally:
            if landing is not None:
                assert landing.stop() == 0
            assert recorder.stop() == 0
        from rerun.chunk import RrdReader
        reader = RrdReader(recording)
        assert reader.store(store=reader.recordings()[0]).schema() is not None
