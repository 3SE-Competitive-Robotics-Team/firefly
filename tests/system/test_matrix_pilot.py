"""Exercise the hardware pilot binary against a PTY and the real IPC stick topic."""
import binascii
import os
from pathlib import Path
import pty
import select
import struct
import tempfile
import time

import iceoryx2 as iox2
from firefly_ipc.messages import STICKS_BUFFER_SIZE, STICKS_TOPIC, SticksMessage, TraceContext
from firefly_workspace.process import ManagedProcess, isolated_run

ROOT = Path(__file__).resolve().parents[2]


def packet(payload):
    return payload + struct.pack("<H", binascii.crc_hqx(payload, 0))


def test_pilot_duplex_and_stale_sticks():
    work = ROOT / "target" / "pilot-protocol-tests"
    work.mkdir(exist_ok=True)
    with isolated_run(ROOT), tempfile.TemporaryDirectory(dir=work) as tmp:
        master, slave = pty.openpty()
        try:
            config = Path(tmp) / "pilot.toml"
            config.write_text(f'device = "{os.ttyname(slave)}"\n')
            node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
            service = (node.service_builder(iox2.ServiceName.new(STICKS_TOPIC))
                       .publish_subscribe(SticksMessage).user_header(TraceContext)
                       .subscriber_max_buffer_size(STICKS_BUFFER_SIZE).open_or_create())
            port = service.publisher_builder().create()
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
                while (elapsed := time.monotonic() - start) < .8:
                    stamp = time.clock_gettime(time.CLOCK_MONOTONIC)
                    sample = packet(struct.pack("<2sI6h", b"FI", int(elapsed * 1e6),
                                                0, 0, 1000, 0, 0, 0))
                    os.write(master, sample)
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
                assert received.count(expected) >= 1
                assert received.replace(heartbeat, b"").replace(expected, b"") == b""
            finally:
                app.stop()
        finally:
            os.close(slave)
            os.close(master)
