#!/usr/bin/env python3
import math
from time import time
from typing import Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import ImuStatus, PressureStatus

from ..utils.constants import SIMULATE_PARAM
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import IMU_STATUS, PRESSURE_STATUS
from .pressure_source import PressureSource

PUBLISH_FREQUENCY = 5
DEPTH_MOVEMENT_TOLERANCE = 0.2  # meters


def calculate_center_depth(main_depth: float, pitch_deg: float, displacement: float = 0.375) -> float:
    pitch_rad = math.radians(pitch_deg)
    return main_depth - (displacement * math.sin(pitch_rad))


def get_depth_velocity(
    depth: float,
    previous_depth: float | None,
    now: float,
    previous_depth_at: float | None,
) -> float:
    if previous_depth is None or previous_depth_at is None:
        return 0.0
    depth_delta = depth - previous_depth
    time_delta = now - previous_depth_at
    velocity = depth_delta / time_delta
    return velocity


def get_pressure_sensor(should_simulate: bool, parent_node: Node, serial_port: str) -> PressureSource:
    if should_simulate:
        from .pressure_sim import PressureSensorSimulator

        return PressureSensorSimulator(parent_node=parent_node)
    else:
        from .pressure_sensor import PressureSensor

        return PressureSensor(parent_node=parent_node, serial_port=serial_port)


# example usage: ros2 run eel pressure
class PressureNode(Node):
    def __init__(self) -> None:
        super().__init__("pressure_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)

        # At the time of writing, Ålen uses /dev/ttyUSB1 (which is why it's the default)
        # and we don't really know which one Tvålen should use.
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value

        self.sensor = get_pressure_sensor(self.should_simulate, self, serial_port)

        self.current_pitch = 0.0

        self.last_depth_reading: float | None = None
        self.last_depth_at: float | None = None

        self.create_subscription(ImuStatus, IMU_STATUS, self.handle_imu_msg, 10)

        self.publisher = self.create_publisher(PressureStatus, PRESSURE_STATUS, 10)
        self.updater = self.create_timer(1.0 / PUBLISH_FREQUENCY, self.publish_status)

        self.get_logger().info("{}Pressure node started.".format("SIMULATE " if self.should_simulate else ""))

    def handle_imu_msg(self, msg: ImuStatus) -> None:
        self.current_pitch = msg.pitch

    def publish_status(self) -> None:
        depth_reading = self.sensor.get_current_depth()

        if depth_reading is not None:
            current_depth = calculate_center_depth(depth_reading, self.current_pitch)

            now = time()

            depth_velocity = get_depth_velocity(current_depth, self.last_depth_reading, now, self.last_depth_at)

            msg = PressureStatus()
            msg.depth = current_depth
            msg.depth_velocity = depth_velocity

            self.last_depth_reading = current_depth
            self.last_depth_at = now
            self.publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PressureNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
