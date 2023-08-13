#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
import sys
from .general_servo import RudderServo
from .rudder_sim import RudderSimulator
from ..utils.topics import RUDDER_HORIZONTAL_CMD, RUDDER_HORIZONTAL_STATUS, RUDDER_VERTICAL_CMD, RUDDER_VERTICAL_STATUS
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
        # self.rudder_cmd_subscription = self.create_subscription(
        #     Float32, RUDDER_CMD, self.handle_rudder_msg, 10
        # )
        self.horizontal_cmd_subscription = self.create_subscription(
            Float32, RUDDER_HORIZONTAL_CMD, self.handle_horizontal_msg, 10
        )
        self.vertical_cmd_subscription = self.create_subscription(
            Float32, RUDDER_VERTICAL_CMD, self.handle_vertical_msg, 10
        )
        self.horizontal_status_publisher = self.create_publisher(Float32, RUDDER_HORIZONTAL_STATUS, 10)
        self.vertical_status_publisher = self.create_publisher(Float32, RUDDER_VERTICAL_STATUS, 10)

        if self.should_simulate:
            # simulator = RudderSimulator()
            # self.detach = simulator.detach
            # self.set_value = simulator.set_value
            raise Exception("not implemented")
        if not self.should_simulate:
            horizontal_servo = RudderServo(
                pin=13,
                min_pulse_width=0.81 / 1000,
                max_pulse_width=2.2 / 1000,
                flip_direction=True,
                cap_min = -0.8,
                cap_max = 0.8,
            )
            self.horizontal_detach = horizontal_servo.detach
            self.horizontal_set_value = horizontal_servo.set_value
            
            vertical_servo = RudderServo(
                pin=19,
                min_pulse_width=0.81 / 1000,
                max_pulse_width=2.2 / 1000,
                flip_direction=False,
                cap_min = -0.8,
                cap_max = 0.8,
            )
            self.vertical_detach = vertical_servo.detach
            self.vertical_set_value = vertical_servo.set_value

        self.get_logger().info(
            "{}Rudder node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def shutdown(self):
        self.get_logger().info("Rudder node shutting down...")
        self.horizontal_detach()
        self.vertical_detach()

    def handle_horizontal_msg(self, msg):
        new_horizontal = msg.data + self.rudder_calibration_term
        new_horizontal = clamp(new_horizontal, -1, 1)
        self.horizontal_set_value(new_horizontal)
        status_msg = Float32()
        status_msg.data = float(new_horizontal)
        self.horizontal_status_publisher.publish(status_msg)

    def handle_vertical_msg(self, msg):
        new_vertical = msg.data
        new_vertical = clamp(new_vertical, -1, 1)
        self.vertical_set_value(new_vertical)
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
