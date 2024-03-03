from .tank_base import Tank


def create_tank(
    simulate: bool,
    motor_pin: int,
    direction_pin: int,
    floor: float,
    ceiling: float,
    channel: int,
) -> Tank:
    if simulate:
        from .sim_tank import SimTank

        return SimTank()
    else:
        from .real_tank import RealTank

        return RealTank(
            motor_pin=motor_pin,
            direction_pin=direction_pin,
            floor=floor,
            ceiling=ceiling,
            channel=channel,
        )
