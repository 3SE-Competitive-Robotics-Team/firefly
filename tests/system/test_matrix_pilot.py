"""Exercise the hardware pilot binary against a PTY and the real IPC stick topic."""
import binascii
import math
import os
from pathlib import Path
import pty
import select
import struct
import tempfile
import time

import iceoryx2 as iox2
from firefly_ipc.messages import (
    IMU_SERVICE_MAX, STICKS_BUFFER_SIZE, STICKS_TOPIC, TOPIC_IMU, ImuMessage,
    SticksMessage, TraceContext,
)
from firefly_workspace.process import ManagedProcess, isolated_run
from test_mimir import recording, read

ROOT = Path(__file__).resolve().parents[2]


def packet(payload):
    return payload + struct.pack("<H", binascii.crc_hqx(payload, 0))


def test_pilot_duplex_and_stale_sticks():
    work = ROOT / "target" / "pilot-protocol-tests"
    work.mkdir(exist_ok=True)
    with isolated_run(ROOT), tempfile.TemporaryDirectory(dir=work) as tmp:
        master, slave = pty.openpty()
        recorder, run = recording()
        try:
            config = Path(tmp) / "pilot.toml"
            config.write_text(f'device = "{os.ttyname(slave)}"\n')
            node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
            service = (node.service_builder(iox2.ServiceName.new(STICKS_TOPIC))
                       .publish_subscribe(SticksMessage).user_header(TraceContext)
                       .subscriber_max_buffer_size(STICKS_BUFFER_SIZE).open_or_create())
            port = service.publisher_builder().create()
            imu_service = (node.service_builder(iox2.ServiceName.new(TOPIC_IMU))
                           .publish_subscribe(ImuMessage).user_header(TraceContext)
                           .subscriber_max_buffer_size(IMU_SERVICE_MAX).open_or_create())
            imu = imu_service.subscriber_builder().buffer_size(IMU_SERVICE_MAX).create()
            imu_ids = []

            def drain_imu():
                while (sample := imu.receive()) is not None:
                    imu_ids.append(round(sample.payload().contents.linear_acceleration_x / .00980665))

            app = ManagedProcess([str(ROOT / "target/release/pilot"), "--config", str(config)],
                                 cwd=ROOT, env=os.environ.copy())
            try:
                request = bytearray()
                deadline = time.monotonic() + 3
                while len(request) < 9 and time.monotonic() < deadline:
                    if select.select([master], [], [], .1)[0]:
                        request.extend(os.read(master, 512))
                assert request == bytes.fromhex("24583c00013f0000ab"), list(app.tail)
                ack = bytes.fromhex("24583e00013f0000ab")
                os.write(master, b"FIold" + ack[:4])
                os.write(master, ack[4:])
                start = time.monotonic()
                received = bytearray()
                slots = []
                imu_sent = 0
                while (elapsed := time.monotonic() - start) < .8:
                    stamp = time.clock_gettime(time.CLOCK_MONOTONIC)
                    batch = bytearray()
                    while imu_sent <= int(elapsed * 1000):
                        batch.extend(packet(struct.pack("<2sI6h", b"FI", imu_sent * 1000,
                                                        imu_sent, 0, 1000, 0, 0, 0)))
                        imu_sent += 1
                    if batch:
                        os.write(master, batch)
                    drain_imu()
                    slot = int(elapsed * 10)
                    if slot > 0 and (not slots or slot > slots[-1]):
                        slots.append(slot)
                        status = packet(struct.pack(
                            "<2sII3h4H4H2HI2B2I4BHhHhHh", b"FT", int(elapsed * 1e6), slot,
                            -120, 45, 920, 1250, 1500, 1750, 1000, 1001, 1002, 1003, 1004,
                            1, 7, 64, 4, 5, 42000, 41000, 1, 1, 1, 63 if slot > 1 else 0,
                            375, -12, 23, 39, 320, 1275,
                        ))
                        os.write(master, status[:13])
                        os.write(master, status[13:])
                    if .1 < elapsed < .3:
                        msg = SticksMessage(state_time=stamp)
                        msg.channels[:] = [1250, 1500, 1750, 1000]
                        port.loan_uninit().write_payload(msg).send()
                    elif .4 < elapsed < .6:
                        msg = SticksMessage(state_time=stamp - .3)
                        msg.channels[:] = [1900, 1600, 1750, 1900]
                        port.loan_uninit().write_payload(msg).send()
                    if select.select([master], [], [], .005)[0]:
                        received.extend(os.read(master, 512))
                heartbeat = packet(b"FH")
                expected = packet(struct.pack("<2s4H", b"FC", 1250, 1500, 1750, 1000))
                assert received.count(heartbeat) >= 2
                assert received.count(expected) >= 2
                assert received.replace(heartbeat, b"").replace(expected, b"") == b""
                deadline = time.monotonic() + 1
                while len(imu_ids) < imu_sent and time.monotonic() < deadline:
                    drain_imu()
                    time.sleep(.001)
                assert imu_ids == list(range(imu_sent))
            finally:
                app.stop()
        finally:
            os.close(slave)
            os.close(master)
            assert recorder.stop() == 0, list(recorder.tail)
        rows, logs = read(run)
        stamps = [stamp for stamp, _ in rows["matrix/link"]]
        assert [values[0] for _, values in rows["matrix/link"]] == slots
        expected = {
            "matrix/attitude": [-12., 4.5, 92.],
            "matrix/control/sticks": [1250, 1500, 1750, 1000],
            "matrix/control/motors": [1001, 1002, 1003, 1004],
            "matrix/control/state": [1, 1, 1, 1],
            "matrix/faults/state": [64, 4],
            "matrix/faults/last_disarm": [5, 42000],
            "matrix/faults/last_failsafe": [1, 41000],
            "matrix/power/state": [1, 1],
            "matrix/power/meter": [3.75, -.12, 23],
            "matrix/power/extrema": [3.2, 12.75],
            "matrix/temperature": [39],
        }
        for group, values in expected.items():
            assert [stamp for stamp, _ in rows[group]] == stamps
            assert all(row == values for _, row in rows[group][1:])
        assert math.isnan(rows["matrix/temperature"][0][1][0])
        assert "recording complete" in logs
