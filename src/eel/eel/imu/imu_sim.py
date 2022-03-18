from std_msgs.msg import Float32
from ..utils.translate import translate_from_range_to_range

RUDDER_TOPIC = "rudder"
MOTOR_TOPIC = "motor"

# hertz (updates per second)
UPDATE_FREQUENCY = 1

# hertz (degrees per second)
ANGULAR_VELOCITY = 10.0

# divided by UPDATE_FREQUENCY, since faster updates will otherwise make it turn faster
RUDDER_EXTREME = ANGULAR_VELOCITY / UPDATE_FREQUENCY


class ImuSimulator:
    def __init__(self, parent_node=None) -> None:
        self.current_rudder_angle = float(0)
        self.current_heading = float(0)
        self.speed = 0
        self.rudder_subscription = parent_node.create_subscription(
            Float32, RUDDER_TOPIC, self._handle_rudder_msg, 10
        )
        self.motor_subscription = parent_node.create_subscription(
            Float32, MOTOR_TOPIC, self._handle_motor_msg, 10
        )
        self.imu_updater = parent_node.create_timer(
            1.0 / UPDATE_FREQUENCY, self._update_imu
        )

    def _handle_rudder_msg(self, msg):
        self.current_rudder_angle = msg.data * 90

    def _handle_motor_msg(self, msg):
        self.speed = msg.data

    def _update_imu(self):
        if self.speed > 0:
            self.current_heading += translate_from_range_to_range(
                self.current_rudder_angle, -90.0, 90.0, -RUDDER_EXTREME, RUDDER_EXTREME
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
