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

    def set_x_rudder_offset_value(self, value: float) -> None:
        self.x_servo.set_offset_value(value)
    
    def get_x_rudder_offset_value(self):
        return self.x_servo.get_offset_value()

    def get_x_rudder_cap_min_value(self):
        return self.x_servo.get_cap_min_value()
    
    def get_x_rudder_cap_max_value(self):
        return self.x_servo.get_cap_max_value()
    
    def set_y_rudder_offset_value(self, value: float) -> None:
        self.y_servo.set_offset_value(value)

    def get_y_rudder_offset_value(self):
        return self.y_servo.get_offset_value()

    def get_y_rudder_cap_min_value(self):
        return self.y_servo.get_cap_min_value()
    
    def get_y_rudder_cap_max_value(self):
        return self.y_servo.get_cap_max_value()    

    def shutdown(self) -> None:
        self.x_servo.detach()
        self.y_servo.detach()
