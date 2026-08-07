from dataclasses import dataclass
from time import monotonic
from typing import Callable, Optional


@dataclass
class PidState:
    cumulative_error: float = 0.0
    last_error: float = 0.0


@dataclass(frozen=True)
class PidGains:
    kP: float
    kI: float
    kD: float


@dataclass(frozen=True)
class OutputLimits:
    min: float
    max: float


def clamp_output(value: float, limits: OutputLimits | None) -> float:
    if limits is None:
        return value
    return max(limits.min, min(limits.max, value))


def _validate_output_limits(output_min: float, output_max: float) -> None:
    if output_min > output_max:
        raise ValueError(f"output_min ({output_min}) must be <= output_max ({output_max})")


def pid_step(
    state: PidState,
    measured: float,
    setpoint: float,
    gains: PidGains,
    dt: float,
    limits: OutputLimits | None = None,
) -> tuple[float, PidState]:
    error = setpoint - measured

    cumulative_error = state.cumulative_error + error * dt
    p = gains.kP * error
    i = cumulative_error * gains.kI

    rate_of_error = (error - state.last_error) / dt if dt > 0 else 0.0
    d = rate_of_error * gains.kD

    raw = p + i + d
    output = clamp_output(raw, limits)

    if limits is not None and output != raw and gains.kI != 0 and dt > 0:
        cumulative_error += (output - raw) / gains.kI

    return output, PidState(cumulative_error=cumulative_error, last_error=error)


class PidController:
    def __init__(
        self,
        set_point: float,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        output_min: float | None = None,
        output_max: float | None = None,
        on_log_error: Optional[Callable[[float], None]] = None,
    ) -> None:
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.set_point = set_point
        if output_min is not None and output_max is not None:
            _validate_output_limits(output_min, output_max)
        self.output_min = output_min
        self.output_max = output_max
        self.on_log_error = on_log_error
        self._state = PidState()
        self._last_computed_at: float | None = None

    def update_set_point(self, value: float) -> None:
        self.set_point = value

    def update_output_limits(self, output_min: float, output_max: float) -> None:
        _validate_output_limits(output_min, output_max)
        self.output_min = output_min
        self.output_max = output_max

    def reset_cumulative_error(self) -> None:
        self._state.cumulative_error = 0.0

    def _output_limits(self) -> OutputLimits | None:
        if self.output_min is None or self.output_max is None:
            return None
        return OutputLimits(self.output_min, self.output_max)

    def compute(self, system_current_value: float, now: float | None = None) -> float:
        if now is None:
            now = monotonic()

        if self._last_computed_at is None:
            self._last_computed_at = now
            dt = 0.0
        else:
            dt = now - self._last_computed_at
            self._last_computed_at = now

        error = self.set_point - system_current_value
        if self.on_log_error:
            self.on_log_error(error)

        output, self._state = pid_step(
            self._state,
            system_current_value,
            self.set_point,
            PidGains(self.kP, self.kI, self.kD),
            dt,
            self._output_limits(),
        )
        return output
