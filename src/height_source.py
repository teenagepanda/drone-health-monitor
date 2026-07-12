from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class HeightReading:
    height_m: float
    source: str
    timestamp: float


class HeightProvider:
    def __init__(self, vehicle=None, fallback_height_m: float = 0.25) -> None:
        self.vehicle = vehicle
        self.fallback_height_m = fallback_height_m
        self._last_valid: Optional[HeightReading] = None

    def read(self) -> HeightReading:
        if self.vehicle is not None:
            message = self.vehicle.recv_match(
                type=["DISTANCE_SENSOR", "GLOBAL_POSITION_INT"],
                blocking=False,
            )
            if message is not None:
                message_type = message.get_type()

                if message_type == "DISTANCE_SENSOR":
                    current_cm = getattr(message, "current_distance", 0)
                    if current_cm and current_cm > 0:
                        reading = HeightReading(
                            height_m=current_cm / 100.0,
                            source="DISTANCE_SENSOR",
                            timestamp=time.time(),
                        )
                        self._last_valid = reading
                        return reading

                if message_type == "GLOBAL_POSITION_INT":
                    relative_alt_mm = getattr(message, "relative_alt", 0)
                    if relative_alt_mm and relative_alt_mm > 0:
                        reading = HeightReading(
                            height_m=relative_alt_mm / 1000.0,
                            source="GLOBAL_POSITION_INT",
                            timestamp=time.time(),
                        )
                        self._last_valid = reading
                        return reading

        if (
            self._last_valid is not None
            and time.time() - self._last_valid.timestamp < 1.0
        ):
            return self._last_valid

        return HeightReading(
            height_m=self.fallback_height_m,
            source="MANUAL_FALLBACK",
            timestamp=time.time(),
        )
