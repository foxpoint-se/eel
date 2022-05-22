from time import time
from std_msgs.msg import Float32
from geopy import distance
from eel_interfaces.msg import ImuStatus
from ..utils.sim import LINEAR_VELOCITY
from ..utils.topics import MOTOR_CMD, IMU_STATUS


def calculate_position_delta(velocity_in_mps, time_in_s):
    return velocity_in_mps * time_in_s


class GnssSimulator:
    def __init__(self, parent_node=None) -> None:
        self.last_updated_at = time()
        self.speed = 0
        self.current_heading = float(0)
        self.current_position = {"lat": 59.309406850903784, "lon": 17.9742443561554}

        self.motor_subscription = parent_node.create_subscription(
            Float32, MOTOR_CMD, self._handle_motor_msg, 10
        )
        self.imu_subscription = parent_node.create_subscription(
            ImuStatus, IMU_STATUS, self._handle_imu_msg, 10
        )

        self.gnss_updater = parent_node.create_timer(
            1.0 / (parent_node.update_frequency * 2), self._update_position
        )

    def _handle_motor_msg(self, msg):
        self.speed = msg.data

    def _handle_imu_msg(self, msg):
        self.current_heading = msg.heading

    def _update_position(self):
        if self.speed > 0:
            now = time()
            time_delta = now - self.last_updated_at
            position_delta = calculate_position_delta(LINEAR_VELOCITY, time_delta)

            new_position = distance.distance(meters=position_delta).destination(
                (self.current_position["lat"], self.current_position["lon"]),
                bearing=self.current_heading,
            )
            self.current_position["lat"] = new_position.latitude
            self.current_position["lon"] = new_position.longitude

        self.last_updated_at = time()

    def get_current_position(self):
        return self.current_position["lat"], self.current_position["lon"]
