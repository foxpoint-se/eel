"""Pure tank I/O: distance sensor → level; pump_setpoint → GPIO pump."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from ..utils.constants import (
    DIRECTION_PIN_PARAM,
    DISTANCE_SENSOR_CHANNEL_PARAM,
    MOTOR_PIN_PARAM,
    STATUS_TOPIC_PARAM,
    TANK_CEILING_VALUE_PARAM,
    TANK_FLOOR_VALUE_PARAM,
)
from ..utils.node_runner import spin_node_until_shutdown
from .tank_topics import tank_level_topic, tank_pump_setpoint_topic
from .tank_utils.real_distance_sensor import RealDistanceSensor
from .tank_utils.real_pump import RealPump

PUBLISH_HZ = 10.0
TANK_CALIBRATION_UNSET = float("nan")


class TankHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("tank_hardware", parameter_overrides=[])
        self.declare_parameter(STATUS_TOPIC_PARAM, "")
        self.declare_parameter(MOTOR_PIN_PARAM, -1)
        self.declare_parameter(DIRECTION_PIN_PARAM, -1)
        self.declare_parameter(DISTANCE_SENSOR_CHANNEL_PARAM, -1)
        self.declare_parameter(TANK_FLOOR_VALUE_PARAM, TANK_CALIBRATION_UNSET)
        self.declare_parameter(TANK_CEILING_VALUE_PARAM, TANK_CALIBRATION_UNSET)

        status_topic = str(self.get_parameter(STATUS_TOPIC_PARAM).get_parameter_value().string_value)
        if not status_topic:
            raise TypeError(f"Missing topic argument ({STATUS_TOPIC_PARAM})")

        motor_pin = int(self.get_parameter(MOTOR_PIN_PARAM).get_parameter_value().integer_value)
        direction_pin = int(self.get_parameter(DIRECTION_PIN_PARAM).get_parameter_value().integer_value)
        channel = int(self.get_parameter(DISTANCE_SENSOR_CHANNEL_PARAM).get_parameter_value().integer_value)
        floor_value = float(self.get_parameter(TANK_FLOOR_VALUE_PARAM).get_parameter_value().double_value)
        ceiling_value = float(self.get_parameter(TANK_CEILING_VALUE_PARAM).get_parameter_value().double_value)
        if math.isnan(floor_value) or math.isnan(ceiling_value):
            raise TypeError(f"Missing calibration parameters ({TANK_FLOOR_VALUE_PARAM}, {TANK_CEILING_VALUE_PARAM})")

        level_topic = tank_level_topic(status_topic)
        pump_topic = tank_pump_setpoint_topic(status_topic)

        self._pump = RealPump(motor_pin=motor_pin, direction_pin=direction_pin)
        self._distance = RealDistanceSensor(floor=floor_value, ceiling=ceiling_value, channel=channel)
        self._level_pub = self.create_publisher(Float32, level_topic, 10)
        self.create_subscription(Float32, pump_topic, self._handle_pump, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_level)

        self.get_logger().info(
            f"Tank hardware started. level={level_topic} pump={pump_topic} "
            f"motor_pin={motor_pin} direction_pin={direction_pin} channel={channel}"
        )

    def shutdown(self) -> None:
        self._pump.stop()

    def _handle_pump(self, msg: Float32) -> None:
        value = float(msg.data)
        if value == 0.0:
            self._pump.stop()
        else:
            self._pump.run_motor(value)

    def _publish_level(self) -> None:
        msg = Float32()
        msg.data = float(self._distance.get_level())
        self._level_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TankHardwareNode()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
