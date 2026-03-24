from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def apply_signed_deadband(value: float, deadband: float) -> float:
    magnitude = abs(value)
    if magnitude <= deadband:
        return 0.0
    scaled = (magnitude - deadband) / max(1e-6, 1.0 - deadband)
    return scaled if value >= 0.0 else -scaled


@dataclass
class LowPassFilter:
    alpha: float
    value: float | None = None

    def update(self, measurement: float) -> float:
        if self.value is None:
            self.value = measurement
        else:
            self.value += self.alpha * (measurement - self.value)
        return self.value

    def reset(self, measurement: float | None = None) -> None:
        self.value = measurement


@dataclass
class SlewRateLimiter:
    rate_per_second: float
    value: float | None = None

    def update(self, target: float, dt: float, rate_scale: float = 1.0) -> float:
        if self.value is None:
            self.value = target
            return target
        max_step = self.rate_per_second * max(0.0, rate_scale) * max(1e-3, dt)
        delta = clamp(target - self.value, -max_step, max_step)
        self.value += delta
        return self.value

    def reset(self, value: float | None = None) -> None:
        self.value = value
