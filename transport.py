from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from config import AppConfig


@dataclass
class TeleopCommand:
    mode: str
    base_command: float
    lower_deg: float
    middle_deg: float
    upper_deg: float
    gripper_open: float
    sequence: int = 0

    def encode(self) -> bytes:
        line = (
            f"T,{self.sequence},{self.mode},"
            f"{self.base_command:.4f},{self.lower_deg:.3f},{self.middle_deg:.3f},"
            f"{self.upper_deg:.3f},{self.gripper_open:.3f}\n"
        )
        return line.encode("ascii")


class TransportController(ABC):
    @property
    @abstractmethod
    def connected(self) -> bool:
        raise NotImplementedError

    @property
    @abstractmethod
    def last_error(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def send(self, command: TeleopCommand, force: bool = False) -> bool:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError


def build_transport(config: AppConfig) -> TransportController:
    transport_kind = config.transport.kind.lower()
    if transport_kind == "udp":
        from udp_comm import UdpController

        return UdpController(config.udp)

    if transport_kind == "serial":
        from serial_comm import SerialController

        return SerialController(config.serial)

    raise ValueError(f"Unsupported transport kind: {config.transport.kind}")
