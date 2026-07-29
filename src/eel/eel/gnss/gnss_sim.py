from typing import Protocol

# Fixed simulated position near test/mission coordinates (Gröndal area).
SIM_LAT = 59.309395
SIM_LON = 17.974279


class GnssSource(Protocol):
    def get_current_position(self) -> tuple[float | None, float | None]:
        pass


class GnssSimulator:
    def get_current_position(self) -> tuple[float | None, float | None]:
        return SIM_LAT, SIM_LON
