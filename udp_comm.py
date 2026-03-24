from __future__ import annotations

import socket
import time

from config import UdpConfig
from transport import TeleopCommand, TransportController


class UdpController(TransportController):
    def __init__(self, config: UdpConfig) -> None:
        self.config = config
        self._socket: socket.socket | None = None
        self._sequence = 0
        self._last_send_time = 0.0
        self._last_error = ""

    @property
    def connected(self) -> bool:
        return self._socket is not None

    @property
    def last_error(self) -> str:
        return self._last_error

    def connect(self) -> None:
        if not self.config.enabled:
            return
        host = (self.config.host or "").strip()
        if not host or host.upper() == "SET_ESP32_IP":
            self._last_error = "UDP host is not configured. Set udp.host in config/calibration.json."
            raise RuntimeError(self._last_error)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(self.config.timeout_s)
            if self.config.local_port > 0:
                sock.bind(("", self.config.local_port))
            self._socket = sock
            self._last_error = ""
        except Exception as exc:
            self._socket = None
            self._last_error = str(exc)
            raise RuntimeError(
                f"Could not open UDP transport to {self.config.host}:{self.config.port}: {exc}"
            ) from exc

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None

    def send(self, command: TeleopCommand, force: bool = False) -> bool:
        if self._socket is None:
            return False
        now = time.perf_counter()
        min_interval = 1.0 / max(self.config.write_hz, 1e-3)
        if not force and (now - self._last_send_time) < min_interval:
            return False
        self._sequence += 1
        command.sequence = self._sequence
        try:
            self._socket.sendto(command.encode(), (self.config.host, self.config.port))
            self._last_send_time = now
            self._last_error = ""
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False
