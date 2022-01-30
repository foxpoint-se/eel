#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


RUDDER_TOPIC = "rudder"


class Rudder(Node):
    def __init__(self):
        super().__init__("rudder_node")

        self.rudder_subscription = self.create_subscription(
            Float32, RUDDER_TOPIC, self.handle_rudder_msg, 10
        )

        self.get_logger().info("Rudder node started.")

    def handle_rudder_msg(self, msg):
        self.get_logger().info("RUDDER NODE SAYS {}".format(msg))


def main(args=None):
    rclpy.init(args=args)
    node = Rudder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
