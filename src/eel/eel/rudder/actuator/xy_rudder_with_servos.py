from .types import ServoOptions, Vector2d
from .xy_rudder_base import XYRudder


class XYRudderWithServos(XYRudder):
    def __init__(
        self,
        x_options: ServoOptions,
        y_options: ServoOptions,
        pigpiod_host: str = "localhost",
    ) -> None:
        from .general_servo import RudderServo

        self.x_servo = RudderServo(
            options=x_options,
            pigpiod_host=pigpiod_host,
        )

        self.y_servo = RudderServo(
            options=y_options,
            pigpiod_host=pigpiod_host,
        )

    def set_rudder(self, value: Vector2d) -> None:
        self.x_servo.set_value(value=value["x"])
        self.y_servo.set_value(value=value["y"])

    def shutdown(self) -> None:
        self.x_servo.detach()
        self.y_servo.detach()
