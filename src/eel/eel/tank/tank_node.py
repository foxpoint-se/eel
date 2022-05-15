#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ..utils.constants import SIMULATE_PARAM


class TankNode(Node):
    def __init__(self):
        super().__init__("tank_node")

        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        if self.should_simulate:
            # simulator = RudderSimulator()
            pass
        if not self.should_simulate:
            # servo = RudderServo()
            pass

        self.get_logger().info(
            "{}Tank node started.".format("SIMULATE " if self.should_simulate else "")
        )


def main(args=None):
    rclpy.init(args=args)
    node = TankNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
