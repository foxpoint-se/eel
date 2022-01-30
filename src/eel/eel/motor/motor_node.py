#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


MOTOR_TOPIC = "motor"


class Motor(Node):
    def __init__(self):
        super().__init__("motor_node")

        self.motor_subscription = self.create_subscription(
            Float32, MOTOR_TOPIC, self.handle_motor_msg, 10
        )

    def handle_motor_msg(self, msg):
        self.get_logger().info("MOTOR NODE SAYS {}".format(msg))


def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
