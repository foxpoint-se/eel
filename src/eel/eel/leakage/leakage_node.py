#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import LEAKAGE_STATUS
from .leakage_source import LeakageSource


class Leakage(Node):
    def __init__(self) -> None:
        super().__init__("leakage_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.update_frequency = 1
        self.publisher = self.create_publisher(Bool, LEAKAGE_STATUS, 10)

        sensor: LeakageSource
        if not self.should_simulate:
            from .leakage_sensor import LeakageSensor

            sensor = LeakageSensor()
        else:
            from .leakage_sim import LeakageSimulator

            sensor = LeakageSimulator()

        self.sensor = sensor

        self.poller = self.create_timer(1.0 / self.update_frequency, self.read_and_publish_sensor_value)
        self.get_logger().info(f"{'Simulate' if self.should_simulate else ''} Leakage node started.")

    def read_and_publish_sensor_value(self) -> None:
        msg = Bool()
        msg.data = self.sensor.read_sensor()
        self.publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Leakage()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
