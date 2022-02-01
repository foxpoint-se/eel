#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
import sys
from .rudder_servo import RudderServo


RUDDER_TOPIC = "rudder"
SIMULATE_PARAM = "simulate"
FAKE_EEL_RUDDER_TOPIC = "/fake_eel_rudder"


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
            self.fake_eel_angular_publisher = self.create_publisher(
                Float32, FAKE_EEL_RUDDER_TOPIC, 10
            )
        if not self.should_simulate:
            self.servo = RudderServo()

        self.get_logger().info(
            "{}Rudder node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def shutdown(self):
        self.get_logger().info("Rudder node shutting down...")
        if not self.should_simulate:
            self.servo.detach()

    def handle_rudder_msg(self, msg):
        rudder_value = clamp(msg.data, -1, 1)

        if self.should_simulate:
            fake_eel_msg = Float32()
            fake_eel_msg.data = rudder_value
            self.fake_eel_angular_publisher.publish(fake_eel_msg)

        else:
            self.servo.set_value(rudder_value)


def main(args=None):
    rclpy.init(args=args)
    node = Rudder()

    # TODO: remove? test if we need this
    rclpy.get_default_context().on_shutdown(node.shutdown)

    # TODO: remove probably
    # rclpy.spin(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.shutdown()
        rclpy.try_shutdown()

    # TODO: remove probably
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
