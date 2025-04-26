from .types import ServoOptions, Vector2d
from .xy_rudder_base import XYRudder


class XYRudderSim(XYRudder):
    def __init__(self, x_options: ServoOptions, y_options: ServoOptions) -> None:
        
        self.x_servo_offset = x_options.get("offset")
        self.x_servo_cap_min = x_options.get("cap_min")
        self.x_servo_cap_max = x_options.get("cap_max")


        self.y_servo_offset = y_options.get("offset")
        self.y_servo_cap_min = y_options.get("cap_min")
        self.y_servo_cap_max = y_options.get("cap_max")

    def set_rudder(self, value: Vector2d) -> None:
        pass

    def set_x_rudder_offset_value(self, value: float) -> None:
        self.x_servo_offset = value
    
    def get_x_rudder_offset_value(self):
        return self.x_servo_offset

    def get_x_rudder_cap_min_value(self):
        return self.x_servo_cap_min
    
    def get_x_rudder_cap_max_value(self):
        return self.x_servo_cap_max
    
    def set_y_rudder_offset_value(self, value: float) -> None:
        self.y_servo_offset = value

    def get_y_rudder_offset_value(self):
        return self.y_servo_offset

    def get_y_rudder_cap_min_value(self):
        return self.y_servo_cap_min
    
    def get_y_rudder_cap_max_value(self):
        return self.y_servo_cap_max

    def shutdown(self) -> None:
        pass
