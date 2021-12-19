#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class GNSS(Node):
    def __init__(self):
        super().__init__("gnss")


def main(args=None):
    rclpy.init(args=args)
    node = GNSS()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
