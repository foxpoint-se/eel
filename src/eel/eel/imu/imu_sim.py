from std_msgs.msg import Float32
from ..utils.translate import translate_from_range_to_range

RUDDER_TOPIC = "rudder"


class ImuSimulator:
    def __init__(self, parent_node=None) -> None:
        self.current_rudder_angle = float(0)
        self.current_heading = float(0)
        self.rudder_subscription = parent_node.create_subscription(
            Float32, RUDDER_TOPIC, self.handle_rudder_msg, 10
        )
        self.imu_updater = parent_node.create_timer(1.0, self.update_imu)

    def handle_rudder_msg(self, msg):
        self.current_rudder_angle = msg.data * 90

    def update_imu(self):
        self.current_heading += translate_from_range_to_range(
            self.current_rudder_angle, -90.0, 90.0, -10.0, 10.0
        )

    def get_calibration_status(self):
        sys = 3
        gyro = 3
        accel = 3
        mag = 3

        return sys, gyro, accel, mag

    def get_heading(self):
        return self.current_heading

    def get_is_calibrated(self):
        return True
