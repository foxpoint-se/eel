#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
import sys
from .rudder_servo import RudderServo
from .rudder_sim import RudderSimulator


RUDDER_TOPIC = "rudder"
SIMULATE_PARAM = "simulate"


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    else:
        return value


class Rudder(Node):
    def __init__(self):
        super().__init__("rudder_node")

        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.rudder_subscription = self.create_subscription(
            Float32, RUDDER_TOPIC, self.handle_rudder_msg, 10
        )
        if self.should_simulate:
            simulator = RudderSimulator()
            self.detach = simulator.detach
            self.set_value = simulator.set_value
        if not self.should_simulate:
            servo = RudderServo()
            self.detach = servo.detach
            self.set_value = servo.set_value

        self.get_logger().info(
            "{}Rudder node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def shutdown(self):
        self.get_logger().info("Rudder node shutting down...")
        self.detach()

    def handle_rudder_msg(self, msg):
        rudder_value = clamp(msg.data, -1, 1)
        self.set_value(rudder_value)


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
