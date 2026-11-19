"""Exercise the release binary's systemd notification protocol in an isolated Linux guest."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time


def main(binary):
    address = f"\0firefly-watchdog-{os.getpid()}"
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as manager:
        manager.bind(address)
        manager.settimeout(10)
        with subprocess.Popen([binary], env={**os.environ, "NOTIFY_SOCKET": "@" + address[1:],
                                              "WATCHDOG_USEC": "600000"}) as daemon:
            try:
                assert manager.recv(256).strip() == b"READY=1"
                assert manager.recv(256).strip() == b"WATCHDOG=1"
                first = time.monotonic()
                assert manager.recv(256).strip() == b"WATCHDOG=1"
                interval = time.monotonic() - first
                assert .24 < interval < .6, interval
                daemon.send_signal(signal.SIGSTOP)
                manager.settimeout(.7)
                try:
                    manager.recv(256)
                except TimeoutError:
                    pass
                else:
                    raise AssertionError("watchdog notification while main loop is stopped")
                daemon.send_signal(signal.SIGCONT)
                assert manager.recv(256).strip() == b"WATCHDOG=1"
                print(f"READY, interval={interval:.3f}s, stopped >0.7s, resumed")
            finally:
                daemon.send_signal(signal.SIGCONT)
                daemon.terminate()
                daemon.wait(5)


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve())
