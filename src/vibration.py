from collections import deque
from math import sqrt
from typing import Optional


class VibrationAnalyzer:
    def __init__(self, window_size: int = 100):
        self.samples = deque(maxlen=window_size)

    def add_sample(self, ax: Optional[float], ay: Optional[float], az: Optional[float]) -> None:
        if ax is None or ay is None or az is None:
            return
        self.samples.append((ax, ay, az))

    def rms(self) -> Optional[float]:
        if len(self.samples) < 10:
            return None

        avg_x = sum(s[0] for s in self.samples) / len(self.samples)
        avg_y = sum(s[1] for s in self.samples) / len(self.samples)
        avg_z = sum(s[2] for s in self.samples) / len(self.samples)

        total = 0.0
        for ax, ay, az in self.samples:
            dx = ax - avg_x
            dy = ay - avg_y
            dz = az - avg_z
            total += dx * dx + dy * dy + dz * dz

        return sqrt(total / len(self.samples))
