from .tank import Tank

NEUTRAL_LEVEL = 0.5


def create_sim_tank() -> Tank:
    from .simulation import SimPump, SimSensor, SimState

    state = SimState(NEUTRAL_LEVEL)
    pump = SimPump(state)
    sensor = SimSensor(state)
    return Tank(pump, sensor)


def create_hardware_tank(
    motor_pin: int, direction_pin: int, floor: float, ceiling: float, channel: int
) -> Tank:
    from .hardware import HardwarePump, HardwareSensor

    pump = HardwarePump(motor_pin, direction_pin)
    sensor = HardwareSensor(floor, ceiling, channel)
    return Tank(pump, sensor)
