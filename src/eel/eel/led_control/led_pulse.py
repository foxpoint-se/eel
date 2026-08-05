from typing import Literal

PulseAction = Literal["on", "off"]
PulseStep = tuple[PulseAction, float]


def plan_pulse_steps(nof_pulses: int, pulse_time: float) -> list[PulseStep]:
    """Build on/off steps matching the old blocking sequence() timing.

    For each pulse: turn on, wait pulse_time, turn off (no gap before the next on).
    """
    if nof_pulses <= 0 or pulse_time < 0:
        return []

    steps: list[PulseStep] = []
    for _ in range(nof_pulses):
        steps.append(("on", pulse_time))
        steps.append(("off", 0.0))
    return steps
