#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
from time import time
from eel_interfaces.msg import PressureStatus
from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import PRESSURE_STATUS, IMU_STATUS
from .pressure_source import PressureSource
from eel_interfaces.msg import ImuStatus

PUBLISH_FREQUENCY = 5
DEPTH_MOVEMENT_TOLERANCE = 0.1  # meters


def calculate_center_depth(main_depth, pitch_deg, displacement=0.375):
    pitch_rad = math.radians(pitch_deg)
    return main_depth - (displacement * math.sin(pitch_rad))


def get_depth_velocity(depth, previous_depth, now, previous_depth_at):
    if previous_depth is None or previous_depth_at is None:
        return 0.0
    depth_delta = depth - previous_depth
    time_delta = now - previous_depth_at
    velocity = depth_delta / time_delta
    return velocity


def get_pressure_sensor(should_simulate: bool, parent_node: Node) -> PressureSource:
    if should_simulate:
        from .pressure_sim import PressureSensorSimulator

        return PressureSensorSimulator(parent_node=parent_node)
    else:
        from .pressure_sensor import PressureSensor

        return PressureSensor(parent_node=parent_node)


def validate_depth(current_depth, last_depth_reading):
    if last_depth_reading is None:
        return current_depth

    if abs(current_depth - last_depth_reading) < DEPTH_MOVEMENT_TOLERANCE:
        return current_depth

    return last_depth_reading


# example usage: ros2 run eel pressure
class PressureNode(Node):
    def __init__(self):
        super().__init__("pressure_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)

        self.sensor = get_pressure_sensor(self.should_simulate, self)

        self.current_pitch = 0.0

        self.last_depth_reading = None
        self.last_depth_at = None

        self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_msg, qos_profile=qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            PressureStatus, PRESSURE_STATUS, qos_profile=qos_profile_sensor_data
        )

        self.updater = self.create_timer(1.0 / PUBLISH_FREQUENCY, self.publish_status)

        self.get_logger().info(
            "{}Pressure node started.".format(
                "SIMULATE " if self.should_simulate else ""
            )
        )

    def handle_imu_msg(self, msg: ImuStatus):
        self.current_pitch = msg.pitch

    def publish_status(self):
        try:
            depth_reading = self.sensor.get_current_depth()
            if depth_reading:
                current_depth = calculate_center_depth(
                    depth_reading, self.current_pitch
                )
                validated_depth = validate_depth(current_depth, self.last_depth_reading)

                now = time()

                depth_velocity = get_depth_velocity(
                    validated_depth, self.last_depth_reading, now, self.last_depth_at
                )

                msg = PressureStatus()
                msg.depth = validated_depth
                msg.depth_velocity = depth_velocity

                self.last_depth_reading = validated_depth
                self.last_depth_at = now
                self.publisher.publish(msg)
        except (OSError, IOError) as err:
            self.get_logger().error(str(err))


def main(args=None):
    rclpy.init(args=args)
    node = PressureNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
