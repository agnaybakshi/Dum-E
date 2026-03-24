from __future__ import annotations

import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - runtime dependency
    serial = None
    list_ports = None

from config import SerialConfig
from transport import TeleopCommand, TransportController


class SerialController(TransportController):
    def __init__(self, config: SerialConfig) -> None:
        self.config = config
        self._port = None
        self._sequence = 0
        self._last_send_time = 0.0
        self._last_error = ""

    @property
    def connected(self) -> bool:
        return self._port is not None

    @property
    def last_error(self) -> str:
        return self._last_error

    def connect(self) -> None:
        if not self.config.enabled:
            return
        if serial is None:
            raise RuntimeError("pyserial is not installed. Install requirements.txt first.")
        if not hasattr(serial, "Serial"):
            raise RuntimeError(
                "Imported the wrong 'serial' package. Uninstall 'serial' and install 'pyserial'."
            )
        attempts = max(1, self.config.open_retries)
        last_exc: Exception | None = None
        for attempt in range(attempts):
            try:
                self._port = serial.Serial(
                    self.config.port,
                    self.config.baudrate,
                    timeout=self.config.timeout_s,
                    write_timeout=self.config.timeout_s,
                )
                time.sleep(self.config.warmup_s)
                self._port.reset_input_buffer()
                self._port.reset_output_buffer()
                self._last_error = ""
                return
            except Exception as exc:
                self._port = None
                last_exc = exc
                self._last_error = str(exc)
                if attempt < attempts - 1:
                    time.sleep(self.config.retry_delay_s)

        available_ports = ""
        if list_ports is not None:
            try:
                ports = sorted(port.device for port in list_ports.comports())
                available_ports = ", ".join(ports) if ports else "none"
            except Exception:
                available_ports = "unknown"
        details = (
            f"Could not open serial port {self.config.port}: {last_exc}. "
            "This usually means another program still owns the port "
            "(Arduino IDE, Serial Monitor, Serial Plotter, another Python process), "
            "or Windows has not released it yet after upload. "
        )
        if available_ports:
            details += f"Detected ports: {available_ports}. "
        details += (
            "Close Arduino IDE completely, wait a few seconds, unplug/replug the board if needed, "
            "then try again."
        )
        raise RuntimeError(details) from last_exc

    def close(self) -> None:
        if self._port is not None:
            try:
                self._port.close()
            except Exception:
                pass
            self._port = None

    def send(self, command: TeleopCommand, force: bool = False) -> bool:
        if self._port is None:
            return False
        now = time.perf_counter()
        min_interval = 1.0 / max(self.config.write_hz, 1e-3)
        if not force and (now - self._last_send_time) < min_interval:
            return False
        self._sequence += 1
        command.sequence = self._sequence
        try:
            self._port.write(command.encode())
            self._last_send_time = now
            self._last_error = ""
            return True
        except Exception as exc:
            self._last_error = str(exc)
            self.close()
            return False
