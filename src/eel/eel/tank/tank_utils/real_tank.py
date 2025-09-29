from .tank_base import Tank
from .real_pump import RealPump
from .real_distance_sensor import RealDistanceSensor


class RealTank(Tank):
    def __init__(
        self,
        motor_pin: int,
        direction_pin: int,
        floor: float,
        ceiling: float,
        channel: int,
    ) -> None:
        super().__init__()
        self.pump = RealPump(motor_pin=motor_pin, direction_pin=direction_pin)
        self.distance_sensor = RealDistanceSensor(
            floor=floor,
            ceiling=ceiling,
            channel=channel,
        )

    def shutdown(self) -> None:
        super().shutdown()
        self.pump.stop()

    def get_level(self) -> float:
        return self.distance_sensor.get_level()

    def run_motor(self, value) -> None:
        self.pump.run_motor(value)

    def stop(self) -> None:
        super().stop()
        self.pump.stop()
