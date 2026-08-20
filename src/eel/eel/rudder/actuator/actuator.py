from .types import ServoOptions

default_x_options: ServoOptions = {
    "pin": 13,
    "min_pulse_width": 0.81 / 1000,
    "max_pulse_width": 2.2 / 1000,
    "flip_direction": True,
    "cap_min": -0.75,
    "cap_max": 0.75,
    "offset": -0.20,  # Offset value should be as max_cap > value < min_cap
}
default_y_options: ServoOptions = {
    "pin": 19,
    "min_pulse_width": 0.81 / 1000,
    "max_pulse_width": 2.2 / 1000,
    "flip_direction": False,
    "cap_min": -0.75,
    "cap_max": 0.75,
    "offset": 0.0,  # Offset value should be as max_cap > value < min_cap
}
