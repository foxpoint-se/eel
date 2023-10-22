#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import sys
import math
from eel_interfaces.msg import ImuStatus
from .general_servo import RudderServo
from .rudder_sim import RudderSimulator
from ..utils.topics import (
    RUDDER_HORIZONTAL_CMD,
    RUDDER_HORIZONTAL_STATUS,
    RUDDER_VERTICAL_CMD,
    RUDDER_VERTICAL_STATUS,
    RUDDER_STATUS,
    IMU_STATUS,
)
from ..utils.constants import SIMULATE_PARAM
from ..utils.utils import clamp


SIM_CALIBRATION_TERM = 0.0
CALIBRATION_TERM = 0.0


class Rudder(Node):
    def __init__(self):
        super().__init__("rudder_node")

        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.rudder_calibration_term = (
            SIM_CALIBRATION_TERM if self.should_simulate else CALIBRATION_TERM
        )
        self.horizontal_cmd_subscription = self.create_subscription(
            Float32, RUDDER_HORIZONTAL_CMD, self.handle_horizontal_msg, 10
        )
        self.vertical_cmd_subscription = self.create_subscription(
            Float32, RUDDER_VERTICAL_CMD, self.handle_vertical_msg, 10
        )
        self.horizontal_status_publisher = self.create_publisher(
            Float32, RUDDER_HORIZONTAL_STATUS, 10
        )
        self.vertical_status_publisher = self.create_publisher(
            Float32, RUDDER_VERTICAL_STATUS, 10
        )
        self.rudder_status_publisher = self.create_publisher(
            Vector3, RUDDER_STATUS, 10
        )

        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self._handle_imu_msg, 10
        )

        self.current_roll = 0.0
        self.current_vertical_control = 0.0
        self.current_horizontal_control = 0.0

        if self.should_simulate:
            simulator = RudderSimulator()
            self.horizontal_detach = simulator.horizontal_detach
            self.horizontal_set_value = simulator.horizontal_set_value
            self.vertical_detach = simulator.vertical_detach
            self.vertical_set_value = simulator.vertical_set_value

        if not self.should_simulate:
            horizontal_servo = RudderServo(
                pin=13,
                min_pulse_width=0.81 / 1000,
                max_pulse_width=2.2 / 1000,
                flip_direction=True,
                cap_min=-0.75,
                cap_max=0.75,
            )
            self.horizontal_detach = horizontal_servo.detach
            self.horizontal_set_value = horizontal_servo.set_value

            vertical_servo = RudderServo(
                pin=19,
                min_pulse_width=0.81 / 1000,
                max_pulse_width=2.2 / 1000,
                flip_direction=False,
                cap_min=-0.75,
                cap_max=0.75,
            )
            self.vertical_detach = vertical_servo.detach
            self.vertical_set_value = vertical_servo.set_value

        self.get_logger().info(
            "{}Rudder node started.".format(
                "SIMULATE " if self.should_simulate else ""
            )
        )

    def shutdown(self):
        self.get_logger().info("Rudder node shutting down...")
        self.horizontal_detach()
        self.vertical_detach()

    def _handle_imu_msg(self, msg):
        self.current_roll = msg.roll

        self.calc_and_send(
            self.current_horizontal_control,
            self.current_vertical_control,
            self.current_roll,
        )

    def handle_horizontal_msg(self, msg):
        new_horizontal = clamp(msg.data, -1, 1)
        self.current_horizontal_control = new_horizontal
        status_msg = Float32()
        status_msg.data = float(new_horizontal)
        self.horizontal_status_publisher.publish(status_msg)

        self.calc_and_send(
            new_horizontal, self.current_vertical_control, self.current_roll
        )

    def calc_and_send(self, x, y, roll_degrees):
        roll_radians = math.radians(roll_degrees)
        rotation = -roll_radians
        x2 = math.cos(rotation) * x - math.sin(rotation) * y
        y2 = math.sin(rotation) * x + math.cos(rotation) * y

        x2 = float(clamp(x2, -1, 1))
        y2 = float(clamp(y2, -1, 1))

        # x2 = x
        # y2 = y

        self.horizontal_set_value(x2)
        self.vertical_set_value(y2)

        status_vector_msg = Vector3()
        status_vector_msg.x = x2
        status_vector_msg.y = y2
        self.rudder_status_publisher.publish(status_vector_msg)

    def handle_vertical_msg(self, msg):
        new_vertical = msg.data
        new_vertical = clamp(new_vertical, -1, 1)

        self.current_vertical_control = new_vertical

        self.calc_and_send(
            self.current_horizontal_control, new_vertical, self.current_roll
        )

        status_msg = Float32()
        status_msg.data = float(new_vertical)
        self.vertical_status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Rudder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
