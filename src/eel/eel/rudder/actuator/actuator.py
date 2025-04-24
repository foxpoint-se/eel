from typing import TypedDict, cast

from .types import ServoOptions
from .xy_rudder_sim import XYRudderSim
from .xy_rudder_with_servos import XYRudderWithServos
from .xy_rudder_base import XYRudder


class ActuatorOptions(TypedDict):
    simulate: bool
    pigpiod_host: str


default_x_options: ServoOptions = {
    "pin": 13,
    "min_pulse_width": 0.81 / 1000,
    "max_pulse_width": 2.2 / 1000,
    "flip_direction": True,
    "cap_min": -0.75,
    "cap_max": 0.75,
    "offset": 0.0  # Offset value should be as max_cap > value < min_cap
}
default_y_options: ServoOptions = {
    "pin": 19,
    "min_pulse_width": 0.81 / 1000,
    "max_pulse_width": 2.2 / 1000,
    "flip_direction": False,
    "cap_min": -0.75,
    "cap_max": 0.75,
    "offset": 0.0  # Offset value should be as max_cap > value < min_cap
}


def get_xy_rudder(options: ActuatorOptions) -> XYRudder:
    if options["simulate"]:
        return XYRudderSim(
            x_options=default_x_options,
            y_options=default_y_options,)
    
    options = cast(ActuatorOptions, options)
    
    return XYRudderWithServos(
        x_options=default_x_options,
        y_options=default_y_options,
        pigpiod_host=options["pigpiod_host"],
    )
