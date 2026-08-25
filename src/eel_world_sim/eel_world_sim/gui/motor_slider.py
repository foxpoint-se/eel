"""Map world-sim GUI motor slider (-100..100) to motor cmd (-1..1)."""

MOTOR_SLIDER_MAX = 100.0
MOTOR_SLIDER_DEADZONE = 5.0


def motor_slider_to_cmd(
    slider: float,
    *,
    deadzone: float = MOTOR_SLIDER_DEADZONE,
    max_slider: float = MOTOR_SLIDER_MAX,
) -> float:
    if abs(slider) <= deadzone:
        return 0.0
    sign = 1.0 if slider > 0.0 else -1.0
    magnitude = (abs(slider) - deadzone) / (max_slider - deadzone)
    return sign * min(magnitude, 1.0)
