#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from ..utils.topics import MOTOR_CMD


class Motor(Node):
    def __init__(self):
        super().__init__("mqtt_bridge_node")

        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)

        self.get_logger().info("MQTT bridge node started.")


def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
