from std_msgs.msg import Float32
from geopy import distance
from eel_interfaces.msg import ImuStatus

MOTOR_TOPIC = "motor"
IMU_STATUS_TOPIC = "imu_status"


class GnssSimulator:
    def __init__(self, parent_node=None) -> None:
        self.motor_subscription = parent_node.create_subscription(
            Float32, MOTOR_TOPIC, self._handle_motor_msg, 10
        )
        self.imu_subscription = parent_node.create_subscription(
            ImuStatus, IMU_STATUS_TOPIC, self._handle_imu_msg, 10
        )

        self.position_updater = parent_node.create_timer(1.0, self._update_position)

        self.speed = 0
        self.current_heading = float(0)
        self.current_position = {"lat": 59.309406850903784, "lon": 17.9742443561554}

    def _handle_motor_msg(self, msg):
        self.speed = msg.data

    def _handle_imu_msg(self, msg):
        self.current_heading = msg.euler_heading

    def _update_position(self):
        if self.speed > 0:
            new_position = distance.distance(meters=3.0).destination(
                (self.current_position["lat"], self.current_position["lon"]),
                bearing=self.current_heading,
            )
            self.current_position["lat"] = new_position.latitude
            self.current_position["lon"] = new_position.longitude

    def get_current_position(self):
        return self.current_position["lat"], self.current_position["lon"]
