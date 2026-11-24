"""Open a Firefly USB session and print one IMU sample per second."""

import argparse
import binascii
import fcntl
import os
import select
import struct
import termios
import time
import tty

ENTER = bytes.fromhex("24 58 3c 00 01 3f 00 00 ab")
ACK = bytes.fromhex("24 58 3e 00 01 3f 00 00 ab")
HEARTBEAT = bytes.fromhex("46 48 a6 6e")


def read_usb(path):
    with open(path, "r+b", buffering=0) as port:
        fd = port.fileno()
        fcntl.ioctl(fd, termios.TIOCEXCL)
        tty.setraw(fd)
        port.write(ENTER)
        data = bytearray()
        deadline = time.monotonic() + 2
        while len(data) < len(ACK):
            if not select.select([fd], [], [], max(0, deadline - time.monotonic()))[0]:
                raise TimeoutError("USB handshake timed out")
            chunk = os.read(fd, len(ACK) - len(data))
            if not chunk:
                raise OSError("USB disconnected during handshake")
            data.extend(chunk)
        if data != ACK:
            raise ValueError(f"Flight controller did not accept ENTER: {data.hex(' ')}")

        data.clear()
        heartbeat_at = report_at = 0.0
        while True:
            now = time.monotonic()
            if now >= heartbeat_at:
                port.write(HEARTBEAT)
                heartbeat_at = now + 0.25
            if not select.select([fd], [], [], max(0, heartbeat_at - time.monotonic()))[0]:
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                raise OSError("USB disconnected")
            data.extend(chunk)
            while len(data) >= 2:
                size = {b"FI": 20, b"FT": 68}.get(bytes(data[:2]))
                if size is None:
                    del data[:1]
                    continue
                if len(data) < size:
                    break
                if binascii.crc_hqx(data[:size-2], 0) != int.from_bytes(data[size-2:size], "little"):
                    del data[:1]
                    continue
                if size == 20 and now >= report_at:
                    us, ax, ay, az, gx, gy, gz = struct.unpack_from("<I6h", data, 2)
                    print(f"{us} us  acc(mg) {ax} {ay} {az}  gyro(0.1 deg/s) {gx} {gy} {gz}", flush=True)
                    report_at = now + 1
                del data[:size]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device", help="USB CDC device, e.g. /dev/ttyACM0")
    try:
        read_usb(parser.parse_args().device)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError) as error:
        parser.exit(1, f"{error}\n")
