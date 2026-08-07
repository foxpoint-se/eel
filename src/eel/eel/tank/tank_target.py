from dataclasses import dataclass
from typing import Literal, Optional

TankStopReason = Literal["floor_reached", "ceiling_reached", "target_reached"]


@dataclass(frozen=True)
class TankTargetConfig:
    """Normalized tank level range and stop margins.

    edge_margin: how close to floor/ceiling counts as "at the limit"
    target_half_band: ± band around target that counts as "close enough"
    """

    level_floor: float
    level_ceiling: float
    edge_margin: float
    target_half_band: float


DEFAULT_TANK_TARGET_CONFIG = TankTargetConfig(
    level_floor=0.0,
    level_ceiling=1.0,
    edge_margin=0.02,
    target_half_band=0.01,
)


def is_within_accepted_target_boundaries(
    current_level: Optional[float],
    target_level: Optional[float],
    config: TankTargetConfig = DEFAULT_TANK_TARGET_CONFIG,
) -> bool:
    if current_level is None or target_level is None:
        return False
    half_band = config.target_half_band
    return (target_level - half_band) <= current_level <= (target_level + half_band)


def is_at_floor(current_level: float, config: TankTargetConfig = DEFAULT_TANK_TARGET_CONFIG) -> bool:
    return current_level <= (config.level_floor + config.edge_margin)


def is_at_ceiling(current_level: float, config: TankTargetConfig = DEFAULT_TANK_TARGET_CONFIG) -> bool:
    return current_level >= (config.level_ceiling - config.edge_margin)


def tank_stop_reason(
    level_average: float,
    target_level: float,
    config: TankTargetConfig = DEFAULT_TANK_TARGET_CONFIG,
) -> TankStopReason | None:
    if is_at_floor(level_average, config) and target_level <= level_average:
        return "floor_reached"
    if is_at_ceiling(level_average, config) and target_level >= level_average:
        return "ceiling_reached"
    if is_within_accepted_target_boundaries(level_average, target_level, config):
        return "target_reached"
    return None


class RunningAverage:
    def __init__(self, size: int) -> None:
        if size < 1:
            raise ValueError("RunningAverage size must be at least 1")
        self.size = size
        self.samples = [0.0] * size
        self.index = 0

    def add_sample(self, value: float) -> None:
        self.samples[self.index] = value
        self.index = (self.index + 1) % self.size

    def get_average(self) -> float:
        return sum(self.samples) / self.size
