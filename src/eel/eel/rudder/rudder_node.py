#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
import sys
from .rudder_servo import RudderServo


RUDDER_TOPIC = "rudder"
SIMULATE_PARAM = "simulate"


class Rudder(Node):
    def __init__(self):
        super().__init__("rudder_node")

        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.rudder_subscription = self.create_subscription(
            Float32, RUDDER_TOPIC, self.handle_rudder_msg, 10
        )

        # TODO: pass in self.should_simulate
        self.servo = RudderServo(simulate=True, logger=self.get_logger())

        self.get_logger().info(
            "{}Rudder node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def shutdown(self):
        self.get_logger().info("Rudder node shutting down...")
        self.servo.detach()

    def handle_rudder_msg(self, msg):
        self.get_logger().info("RUDDER NODE SAYS {}".format(msg))

        new_value = msg.data
        if new_value >= -1 and new_value <= 1:
            self.servo.set_value(new_value)


def main(args=None):
    rclpy.init(args=args)
    node = Rudder()

    # TODO: remove? test if we need this
    rclpy.get_default_context().on_shutdown(node.shutdown)

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

    # rclpy.shutdown()


if __name__ == "__main__":
    main()
