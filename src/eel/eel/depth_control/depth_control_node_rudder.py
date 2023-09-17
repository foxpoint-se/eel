#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class DepthControlNode(Node):
    def __init__(self):
        super().__init__("depth_control_node")

        self.get_logger().info("Depth control node started!!")


def main(args=None):
    rclpy.init(args=args)
    node = DepthControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
